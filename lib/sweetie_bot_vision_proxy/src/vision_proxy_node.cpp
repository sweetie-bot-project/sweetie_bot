#include <ros/ros.h>
#include <std_msgs/String.h>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <boost/asio.hpp>
#include <boost/asio/ssl.hpp>
#include <boost/beast.hpp>
#include <boost/beast/ssl.hpp>
#include <boost/beast/websocket.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <json/json.h>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
namespace ssl = boost::asio::ssl;
using tcp = net::ip::tcp;

#pragma pack(push, 1)
struct FrameHeader {
    char magic[4];              // "MJPG"
    uint16_t version;           // 1
    uint16_t header_size;       // sizeof(FrameHeader)
    uint64_t frame_id;
    uint64_t capture_ts_ns;
    uint32_t width;
    uint32_t height;
    uint32_t jpeg_size;
    int32_t camera_rotation_deg;
    uint32_t flags;
};
#pragma pack(pop)

struct Detection {
    std::string cls;
    double confidence = 0.0;
    double x = 0.0;
    double y = 0.0;
    double w = 0.0;
    double h = 0.0;
};

struct VisionResult {
    uint64_t frame_id = 0;
    std::vector<Detection> objects;
};

static_assert(sizeof(FrameHeader) == 44, "FrameHeader must be 40 bytes");

struct Frame {
    uint64_t frame_id = 0;
    uint64_t capture_ts_ns = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<uint8_t> jpeg;
};

struct ParsedUrl {
    std::string scheme;
    std::string host;
    std::string port;
    std::string target;
};

static uint64_t ros_now_ns()
{
    return static_cast<uint64_t>(ros::Time::now().toNSec());
}

static ParsedUrl parse_url(const std::string& url)
{
    ParsedUrl out;

    auto pos = url.find("://");
    if (pos == std::string::npos) {
        throw std::runtime_error("bad url: missing scheme");
    }

    out.scheme = url.substr(0, pos);
    std::string rest = url.substr(pos + 3);

    auto slash = rest.find('/');
    std::string hostport = slash == std::string::npos ? rest : rest.substr(0, slash);
    out.target = slash == std::string::npos ? "/" : rest.substr(slash);

    auto colon = hostport.rfind(':');
    if (colon != std::string::npos) {
        out.host = hostport.substr(0, colon);
        out.port = hostport.substr(colon + 1);
    } else {
        out.host = hostport;
        out.port = (out.scheme == "wss") ? "443" : "80";
    }

    if (out.scheme != "wss") {
        throw std::runtime_error("this prototype supports only wss:// URLs");
    }

    if (out.host.empty() || out.target.empty()) {
        throw std::runtime_error("bad url");
    }

    return out;
}

static std::vector<uint8_t> build_frame_message(
    const Frame& frame,
    int32_t camera_rotation_deg)
{
    FrameHeader h;
    std::memcpy(h.magic, "MJPG", 4);
    h.version = 1;
    h.header_size = sizeof(FrameHeader);
    h.frame_id = frame.frame_id;
    h.capture_ts_ns = frame.capture_ts_ns;
    h.width = frame.width;
    h.height = frame.height;
    h.jpeg_size = static_cast<uint32_t>(frame.jpeg.size());
    h.camera_rotation_deg = camera_rotation_deg;
    h.flags = 0;

    std::vector<uint8_t> msg;
    msg.resize(sizeof(FrameHeader) + frame.jpeg.size());

    std::memcpy(msg.data(), &h, sizeof(FrameHeader));
    std::memcpy(msg.data() + sizeof(FrameHeader), frame.jpeg.data(), frame.jpeg.size());

    return msg;
}

class VisionProxyNode {
public:
    VisionProxyNode()
        : nh_()
        , pnh_("~")
    {
        pnh_.param<std::string>("pipeline", pipeline_desc_,
            "v4l2src device=/dev/video0 io-mode=mmap ! "
            "image/jpeg,width=640,height=480,framerate=30/1 ! "
            "appsink name=sink emit-signals=true sync=false drop=true max-buffers=1");

        pnh_.param<std::string>("server_url", server_url_, "wss://vision.example.com/ws");
        pnh_.param<std::string>("api_key", api_key_, "");
        pnh_.param<int>("width", width_, 640);
        pnh_.param<int>("height", height_, 480);
        pnh_.param<double>("max_send_fps", max_send_fps_, 10.0);
        pnh_.param<int>("ring_size", ring_size_, 60);
        pnh_.param<bool>("insecure_skip_verify", insecure_skip_verify_, false);
        pnh_.param<std::string>("image_frame_id", image_frame_id_, "camera");
        pnh_.param<int>("camera_rotation_deg", camera_rotation_deg_, 0);
        if (camera_rotation_deg_ != 0 &&
            camera_rotation_deg_ != 90 &&
            camera_rotation_deg_ != 180 &&
            camera_rotation_deg_ != 270)
        {
                ROS_WARN_STREAM("Unsupported camera_rotation_deg=" << camera_rotation_deg_
                             << ", using 0. Supported: 0, 90, 180, 270");
                camera_rotation_deg_ = 0;
        }

        result_pub_ = nh_.advertise<std_msgs::String>("/vision_proxy/result_json", 10);
        stats_pub_ = nh_.advertise<std_msgs::String>("/vision_proxy/stats_json", 10);
        image_pub_ = nh_.advertise<sensor_msgs::Image>("/image_raw", 1);

        min_send_period_ = ros::Duration(max_send_fps_ > 0.0 ? 1.0 / max_send_fps_ : 0.0);
    }

    ~VisionProxyNode()
    {
        stop();
    }

    bool start()
    {
        gst_init(nullptr, nullptr);

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_desc_.c_str(), &error);

        if (!pipeline_) {
            ROS_ERROR_STREAM("Failed to create GStreamer pipeline: "
                             << (error ? error->message : "unknown error"));
            if (error) {
                g_error_free(error);
            }
            return false;
        }

        if (error) {
            ROS_WARN_STREAM("GStreamer warning: " << error->message);
            g_error_free(error);
        }

        GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        if (!sink) {
            ROS_ERROR("GStreamer pipeline must contain appsink named 'sink'");
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            return false;
        }

        appsink_ = GST_APP_SINK(sink);

        gst_app_sink_set_emit_signals(appsink_, true);
        gst_app_sink_set_drop(appsink_, true);
        gst_app_sink_set_max_buffers(appsink_, 1);

        g_signal_connect(appsink_, "new-sample", G_CALLBACK(&VisionProxyNode::on_new_sample_static), this);

        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            ROS_ERROR("Failed to set GStreamer pipeline to PLAYING");
            gst_object_unref(sink);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            return false;
        }

        running_.store(true);
        ws_thread_ = std::thread(&VisionProxyNode::websocket_loop, this);

        ROS_INFO_STREAM("vision_proxy_node started");
        ROS_INFO_STREAM("pipeline: " << pipeline_desc_);
        ROS_INFO_STREAM("server_url: " << server_url_);
        return true;
    }

    void stop()
    {
        bool expected = true;
        if (!running_.compare_exchange_strong(expected, false)) {
            return;
        }

        frame_cv_.notify_all();

        if (ws_thread_.joinable()) {
            ws_thread_.join();
        }

        if (pipeline_) {
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
        }
    }

private:

    bool parse_result_json(const std::string& text, VisionResult& out)
    {
        Json::CharReaderBuilder builder;
        builder["collectComments"] = false;

        Json::Value root;
        std::string errors;

        std::unique_ptr<Json::CharReader> reader(builder.newCharReader());

        if (!reader->parse(text.data(), text.data() + text.size(), &root, &errors)) {
            ROS_WARN_STREAM("Failed to parse result JSON: " << errors);
            return false;
        }

        if (!root.isMember("frame_id")) {
            ROS_WARN("Result JSON has no frame_id");
            return false;
        }

        out.frame_id = root["frame_id"].asUInt64();

        const Json::Value objects = root["objects"];
        if (!objects.isArray()) {
            return true;
        }

        for (const auto& obj : objects) {
            Detection d;

            if (obj.isMember("class")) {
                d.cls = obj["class"].asString();
            } else {
                d.cls = "object";
            }

            if (obj.isMember("confidence")) {
                d.confidence = obj["confidence"].asDouble();
            }

            const Json::Value bbox = obj["bbox"];
            if (!bbox.isArray() || bbox.size() != 4) {
                continue;
            }

            d.x = bbox[0].asDouble();
            d.y = bbox[1].asDouble();
            d.w = bbox[2].asDouble();
            d.h = bbox[3].asDouble();

            out.objects.push_back(d);
        }

        return true;
    }

    cv::Mat rotate_image_if_needed(const cv::Mat& src)
    {
        if (camera_rotation_deg_ == 0) {
            return src;
        }

        cv::Mat dst;

        if (camera_rotation_deg_ == 90) {
            cv::rotate(src, dst, cv::ROTATE_90_CLOCKWISE);
        } else if (camera_rotation_deg_ == 180) {
            cv::rotate(src, dst, cv::ROTATE_180);
        } else if (camera_rotation_deg_ == 270) {
            cv::rotate(src, dst, cv::ROTATE_90_COUNTERCLOCKWISE);
        } else {
            dst = src;
        }

        return dst;
    }

    Detection rotate_detection_bbox(const Detection& in)
    {
        Detection d = in;

        const double x = in.x;
        const double y = in.y;
        const double w = in.w;
        const double h = in.h;

        if (camera_rotation_deg_ == 0) {
            return d;
        }

        if (camera_rotation_deg_ == 180) {
            d.x = 1.0 - x - w;
            d.y = 1.0 - y - h;
            d.w = w;
            d.h = h;
        } else if (camera_rotation_deg_ == 90) {
            // 90 degrees clockwise
            d.x = 1.0 - y - h;
            d.y = x;
            d.w = h;
            d.h = w;
        } else if (camera_rotation_deg_ == 270) {
            // 90 degrees counter-clockwise
            d.x = y;
            d.y = 1.0 - x - w;
            d.w = h;
            d.h = w;
        }

        return d;
    }


    static double clamp01(double v)
    {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }

    void draw_detections(cv::Mat& image, const std::vector<Detection>& objects)
    {
        const int img_w = image.cols;
        const int img_h = image.rows;

        for (const auto& raw_det : objects) {
            Detection det = rotate_detection_bbox(raw_det);

            double x = clamp01(det.x);
            double y = clamp01(det.y);
            double w = clamp01(det.w);
            double h = clamp01(det.h);

            int px = static_cast<int>(x * img_w);
            int py = static_cast<int>(y * img_h);
            int pw = static_cast<int>(w * img_w);
            int ph = static_cast<int>(h * img_h);

            if (pw <= 0 || ph <= 0) {
                continue;
            }

            cv::Rect rect(px, py, pw, ph);
            rect &= cv::Rect(0, 0, img_w, img_h);

            if (rect.width <= 0 || rect.height <= 0) {
                continue;
            }

            cv::rectangle(image, rect, cv::Scalar(0, 255, 0), 2);

            std::ostringstream label;
            label << det.cls << " " << std::fixed << std::setprecision(2) << det.confidence;

            int baseline = 0;
            cv::Size text_size = cv::getTextSize(
                label.str(),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                1,
                &baseline
            );

            int text_x = rect.x;
            int text_y = std::max(0, rect.y - 5);

            cv::rectangle(
                image,
                cv::Rect(
                    text_x,
                    std::max(0, text_y - text_size.height - baseline),
                    text_size.width + 4,
                    text_size.height + baseline + 4
                ),
                cv::Scalar(0, 255, 0),
                cv::FILLED
            );

            cv::putText(
                image,
                label.str(),
                cv::Point(text_x + 2, text_y - 2),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 0, 0),
                1,
                cv::LINE_AA
            );
        }
    }

    void publish_overlay_image(const std::string& result_json)
    {
        VisionResult result;
        if (!parse_result_json(result_json, result)) {
            return;
        }

        Frame frame;
        if (!get_frame_from_ring(result.frame_id, frame)) {
            ROS_WARN_STREAM_THROTTLE(1.0, "No frame in ring buffer for frame_id=" << result.frame_id);
            return;
        }

        cv::Mat jpeg_data(1, static_cast<int>(frame.jpeg.size()), CV_8UC1, frame.jpeg.data());
        cv::Mat image = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

        if (image.empty()) {
            ROS_WARN_STREAM("Failed to decode JPEG for frame_id=" << result.frame_id);
            return;
        }

        cv::Mat rotated = rotate_image_if_needed(image);

        // rotate_image_if_needed() may return the original Mat by value.
        // Make sure we can draw safely.
        if (!rotated.isContinuous()) {
            rotated = rotated.clone();
        }

        draw_detections(rotated, result.objects);

        std_msgs::Header header;
        header.stamp = ros::Time().fromNSec(frame.capture_ts_ns);
        header.frame_id = image_frame_id_;

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
            header,
            "bgr8",
            rotated
        ).toImageMsg();

        image_pub_.publish(msg);
    }

    static GstFlowReturn on_new_sample_static(GstAppSink* sink, gpointer user_data)
    {
        return static_cast<VisionProxyNode*>(user_data)->on_new_sample(sink);
    }

    GstFlowReturn on_new_sample(GstAppSink* sink)
    {
        GstSample* sample = gst_app_sink_pull_sample(sink);
        if (!sample) {
            return GST_FLOW_ERROR;
        }

        GstBuffer* buffer = gst_sample_get_buffer(sample);
        if (!buffer) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            return GST_FLOW_ERROR;
        }

        Frame frame;
        frame.frame_id = ++frame_counter_;
        frame.capture_ts_ns = ros_now_ns();
        frame.width = static_cast<uint32_t>(width_);
        frame.height = static_cast<uint32_t>(height_);
        frame.jpeg.assign(map.data, map.data + map.size);

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);

            latest_frame_ = frame;
            latest_frame_valid_ = true;

            ring_[frame.frame_id] = frame;
            order_.push_back(frame.frame_id);

            while (static_cast<int>(order_.size()) > ring_size_) {
                uint64_t old_id = order_.front();
                order_.pop_front();
                ring_.erase(old_id);
            }

            captured_frames_++;
        }

        frame_cv_.notify_one();
        return GST_FLOW_OK;
    }

    bool get_latest_frame_blocking(Frame& out)
    {
        std::unique_lock<std::mutex> lock(frame_mutex_);

        frame_cv_.wait(lock, [&]() {
            return !running_.load() || latest_frame_valid_;
        });

        if (!running_.load()) {
            return false;
        }

        out = latest_frame_;
        latest_frame_valid_ = false;
        return true;
    }

    bool frame_exists(uint64_t frame_id)
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        return ring_.find(frame_id) != ring_.end();
    }

    bool get_frame_from_ring(uint64_t frame_id, Frame& out)
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);

        auto it = ring_.find(frame_id);
        if (it == ring_.end()) {
            return false;
        }

        out = it->second;
        return true;
    }

    void publish_stats(
        const Frame& frame,
        uint64_t send_start_ns,
        uint64_t recv_done_ns,
        size_t result_size,
        bool frame_still_buffered)
    {
        std::ostringstream ss;
        ss << "{"
           << "\"type\":\"stats\","
           << "\"frame_id\":" << frame.frame_id << ","
           << "\"jpeg_size\":" << frame.jpeg.size() << ","
           << "\"send_to_result_latency_ms\":"
           << std::fixed << std::setprecision(3)
           << ((recv_done_ns - send_start_ns) / 1000000.0) << ","
           << "\"capture_to_result_latency_ms\":"
           << ((recv_done_ns - frame.capture_ts_ns) / 1000000.0) << ","
           << "\"result_size\":" << result_size << ","
           << "\"frame_still_buffered\":" << (frame_still_buffered ? "true" : "false") << ","
           << "\"captured_frames\":" << captured_frames_.load() << ","
           << "\"sent_frames\":" << sent_frames_.load() << ","
           << "\"received_results\":" << received_results_.load()
           << "}";

        std_msgs::String msg;
        msg.data = ss.str();
        stats_pub_.publish(msg);
    }

    void websocket_loop()
    {
        while (running_.load() && ros::ok()) {
            try {
                websocket_session();
            } catch (const std::exception& e) {
                ROS_ERROR_STREAM("WebSocket session error: " << e.what());
            }

            if (running_.load() && ros::ok()) {
                ROS_WARN("WebSocket reconnect in 2 seconds");
                ros::Duration(2.0).sleep();
            }
        }
    }

    void websocket_session()
    {
        ParsedUrl url = parse_url(server_url_);

        net::io_context ioc;
        ssl::context ctx(ssl::context::tlsv12_client);

        ctx.set_default_verify_paths();

        if (insecure_skip_verify_) {
            ctx.set_verify_mode(ssl::verify_none);
        } else {
            ctx.set_verify_mode(ssl::verify_peer);
        }

        tcp::resolver resolver(ioc);
        websocket::stream<beast::ssl_stream<tcp::socket>> ws(ioc, ctx);

        if (!SSL_set_tlsext_host_name(ws.next_layer().native_handle(), url.host.c_str())) {
            beast::error_code ec{
                static_cast<int>(::ERR_get_error()),
                net::error::get_ssl_category()
            };
            throw beast::system_error{ec};
        }

        auto const results = resolver.resolve(url.host, url.port);

	net::connect(beast::get_lowest_layer(ws), results.begin(), results.end());

        ws.next_layer().handshake(ssl::stream_base::client);

        ws.set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));

        ws.set_option(websocket::stream_base::decorator(
            [&](websocket::request_type& req) {
                req.set(beast::http::field::user_agent, "vision_proxy_node/0.1");
                if (!api_key_.empty()) {
                    req.set(beast::http::field::authorization, "Bearer " + api_key_);
                }
            }
        ));

        ws.handshake(url.host, url.target);

        ROS_INFO_STREAM("Connected to " << server_url_);

        ros::Time last_send_time(0);

        while (running_.load() && ros::ok()) {
            Frame frame;
            if (!get_latest_frame_blocking(frame)) {
                break;
            }

            if (min_send_period_.toSec() > 0.0) {
                ros::Time now = ros::Time::now();
                ros::Duration elapsed = now - last_send_time;
                if (elapsed < min_send_period_) {
                    (min_send_period_ - elapsed).sleep();
                }
            }

            auto binary_msg = build_frame_message(frame, camera_rotation_deg_);

            uint64_t send_start_ns = ros_now_ns();

            ws.binary(true);
            ws.write(net::buffer(binary_msg));
            sent_frames_++;
            last_send_time = ros::Time::now();

            // Application-level backpressure:
            // не отправляем следующий кадр, пока не получили ответ.
            beast::flat_buffer buffer;
            ws.read(buffer);

            uint64_t recv_done_ns = ros_now_ns();

            std::string result_json = beast::buffers_to_string(buffer.data());

            received_results_++;

            std_msgs::String msg;
            msg.data = result_json;
            result_pub_.publish(msg);

            // Publish /image_raw with overlay for RViz.
            publish_overlay_image(result_json);

            bool still_buffered = frame_exists(frame.frame_id);
            publish_stats(frame, send_start_ns, recv_done_ns, result_json.size(), still_buffered);

            ROS_DEBUG_STREAM("result: " << result_json);
        }

        beast::error_code ec;
        ws.close(websocket::close_code::normal, ec);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher result_pub_;
    ros::Publisher stats_pub_;
    ros::Publisher image_pub_;

    int camera_rotation_deg_ = 0;
    std::string image_frame_id_ = "camera";

    std::string pipeline_desc_;
    std::string server_url_;
    std::string api_key_;

    int width_ = 640;
    int height_ = 480;
    double max_send_fps_ = 10.0;
    int ring_size_ = 60;
    bool insecure_skip_verify_ = false;

    ros::Duration min_send_period_;

    GstElement* pipeline_ = nullptr;
    GstAppSink* appsink_ = nullptr;

    std::atomic<bool> running_{false};
    std::thread ws_thread_;

    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;

    Frame latest_frame_;
    bool latest_frame_valid_ = false;

    std::unordered_map<uint64_t, Frame> ring_;
    std::deque<uint64_t> order_;

    std::atomic<uint64_t> frame_counter_{0};
    std::atomic<uint64_t> captured_frames_{0};
    std::atomic<uint64_t> sent_frames_{0};
    std::atomic<uint64_t> received_results_{0};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_proxy_node");

    VisionProxyNode node;
    if (!node.start()) {
        return 1;
    }

    ros::spin();

    node.stop();
    return 0;
}
