// sweetie_bot_vision_proxy — the C++ front transport + visualizer of the vision federation.
//
// Restores the original prototype's design into the federation: gstreamer ingest -> monotonic
// frame_id -> cyclic ring buffer -> Boost.Beast WebSocket to the provider container (FrameHeader+JPEG
// up, VRES+bytes down) -> relay the provider bytes opaquely to the py3.10 tracker-fuser over a TCP
// socket (msgpack envelope) -> parse the fuser's flat-JSON tracked reply -> draw boxes+keypoints on the
// ring frame and publish /image_raw, plus /detections (DetectionArray) + /hmi/vision_skeletons
// (MarkerArray) + /vision_proxy/result_json. The gasket is gone — this node is the single ROS face.
//
// PHASE 1: a single LOCAL provider over plain ws (ws://127.0.0.1:8080). Multi-container (wss/depth) is
// a later extension. The forward msgpack envelope is hand-encoded (no msgpack-cxx dep); the python
// fuser unpacks it with stdlib msgpack.
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sweetie_bot_text_msgs/Detection.h>
#include <sweetie_bot_text_msgs/DetectionArray.h>
#include <cv_bridge/cv_bridge.h>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <json/json.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

// ---- wire framing (must byte-match perfusion/transport/protocol.py) ----
#pragma pack(push, 1)
struct FrameHeader {                  // uplink to the provider
    char magic[4];                    // "MJPG"
    uint16_t version;                 // 1
    uint16_t header_size;             // 44
    uint64_t frame_id;
    uint64_t capture_ts_ns;
    uint32_t width;
    uint32_t height;
    uint32_t jpeg_size;
    int32_t camera_rotation_deg;
    uint32_t flags;
};
struct VresHeader {                   // downlink from the provider: "VRES" + body
    char magic[4];                    // "VRES"
    uint16_t version;
    uint16_t header_size;             // 28
    uint64_t frame_id;
    uint32_t payload_len;
    uint32_t encoding;                // 0=json 1=msgpack
    uint32_t flags;
};
#pragma pack(pop)
static_assert(sizeof(FrameHeader) == 44, "FrameHeader must be 44 bytes");
static_assert(sizeof(VresHeader) == 28, "VresHeader must be 28 bytes");

struct Frame {
    uint64_t frame_id = 0;
    uint64_t capture_ts_ns = 0;
    std::vector<uint8_t> jpeg;
};

struct Kpt { double x = 0, y = 0, s = 0; };
struct TrackedDet {
    std::string type;
    int track_id = -1;
    double score = 0.0;
    double bbox[4] = {0, 0, 0, 0};                       // x1,y1,x2,y2 px
    std::vector<Kpt> kpts;                               // 2D px
    std::vector<std::array<double, 3>> kpts3d;           // metric, camera frame (optional)
    bool has_pos = false; double pos[3] = {0, 0, 0};     // position_3d
    bool has_box = false; double box[3] = {0, 0, 0};     // scale_3d
    std::string label;                                   // semantic_label
    std::vector<std::pair<std::string, std::string>> attrs;
};

// COCO-17 skeleton bones (keypoint index pairs).
static const int COCO_BONES[][2] = {
    {0,1},{0,2},{1,3},{2,4},{5,6},{5,7},{7,9},{6,8},{8,10},
    {5,11},{6,12},{11,12},{11,13},{13,15},{12,14},{14,16}};

// ---- hand-rolled msgpack (just the fixed envelope; python msgpack unpacks it) ----
static void mp_u8(std::vector<uint8_t>& b, uint8_t v) { b.push_back(v); }
static void mp_str(std::vector<uint8_t>& b, const char* s) {
    size_t n = std::strlen(s);
    mp_u8(b, 0xa0 | static_cast<uint8_t>(n));            // fixstr (n<=31)
    b.insert(b.end(), s, s + n);
}
static void mp_u64(std::vector<uint8_t>& b, uint64_t v) {
    mp_u8(b, 0xcf);
    for (int i = 7; i >= 0; --i) b.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xff));
}
static void mp_bin32(std::vector<uint8_t>& b, const uint8_t* data, uint32_t n) {
    mp_u8(b, 0xc6);
    for (int i = 3; i >= 0; --i) b.push_back(static_cast<uint8_t>((n >> (8 * i)) & 0xff));
    b.insert(b.end(), data, data + n);
}

class VisionProxyNode {
public:
    VisionProxyNode() : nh_(), pnh_("~") {
        pnh_.param<std::string>("pipeline", pipeline_,
            "udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! "
            "jpegparse ! appsink name=sink emit-signals=true sync=false drop=true max-buffers=1");
        pnh_.param<std::string>("provider_host", prov_host_, "127.0.0.1");
        pnh_.param<int>("provider_port", prov_port_, 8080);
        pnh_.param<std::string>("provider_target", prov_target_, "/");
        pnh_.param<std::string>("fuser_host", fuser_host_, "127.0.0.1");
        pnh_.param<int>("fuser_port", fuser_port_, 9100);
        pnh_.param<int>("ring_size", ring_size_, 60);
        pnh_.param<std::string>("image_topic", image_topic_, "/image_raw");
        pnh_.param<std::string>("image_frame_id", image_frame_id_, "camera_link_optical");
        pnh_.param<std::string>("detections_topic", det_topic_, "detections");
        pnh_.param<std::string>("skeleton_topic", skel_topic_, "/hmi/vision_skeletons");
        pnh_.param<int>("camera_rotation_deg", rot_deg_, 0);
        result_pub_ = nh_.advertise<std_msgs::String>("/vision_proxy/result_json", 10);
        image_pub_ = nh_.advertise<sensor_msgs::Image>(image_topic_, 1);
        det_pub_ = nh_.advertise<sweetie_bot_text_msgs::DetectionArray>(det_topic_, 5);
        if (!skel_topic_.empty())
            skel_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(skel_topic_, 5);
    }
    ~VisionProxyNode() { stop(); }

    bool start() {
        gst_init(nullptr, nullptr);
        GError* err = nullptr;
        pipeline_p_ = gst_parse_launch(pipeline_.c_str(), &err);
        if (!pipeline_p_) {
            ROS_ERROR_STREAM("gst pipeline failed: " << (err ? err->message : "?"));
            if (err) g_error_free(err);
            return false;
        }
        GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline_p_), "sink");
        if (!sink) { ROS_ERROR("pipeline needs appsink name=sink"); return false; }
        appsink_ = GST_APP_SINK(sink);
        gst_app_sink_set_emit_signals(appsink_, true);
        gst_app_sink_set_drop(appsink_, true);
        gst_app_sink_set_max_buffers(appsink_, 1);
        g_signal_connect(appsink_, "new-sample", G_CALLBACK(&VisionProxyNode::on_sample_static), this);
        if (gst_element_set_state(pipeline_p_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            ROS_ERROR("gst set PLAYING failed"); return false;
        }
        running_.store(true);
        worker_ = std::thread(&VisionProxyNode::worker_loop, this);
        ROS_INFO_STREAM("vision_proxy_node up: provider ws://" << prov_host_ << ":" << prov_port_
                        << prov_target_ << " fuser " << fuser_host_ << ":" << fuser_port_);
        return true;
    }

    void stop() {
        bool e = true;
        if (!running_.compare_exchange_strong(e, false)) return;
        frame_cv_.notify_all();
        if (worker_.joinable()) worker_.join();
        if (fuser_fd_ >= 0) { ::close(fuser_fd_); fuser_fd_ = -1; }
        if (pipeline_p_) { gst_element_set_state(pipeline_p_, GST_STATE_NULL);
                           gst_object_unref(pipeline_p_); pipeline_p_ = nullptr; }
    }

private:
    // ---- gstreamer ingest -> ring ----
    static GstFlowReturn on_sample_static(GstAppSink* s, gpointer u) {
        return static_cast<VisionProxyNode*>(u)->on_sample(s);
    }
    GstFlowReturn on_sample(GstAppSink* s) {
        GstSample* sample = gst_app_sink_pull_sample(s);
        if (!sample) return GST_FLOW_ERROR;
        GstBuffer* buf = gst_sample_get_buffer(sample);
        GstMapInfo map;
        if (!buf || !gst_buffer_map(buf, &map, GST_MAP_READ)) { gst_sample_unref(sample); return GST_FLOW_ERROR; }
        Frame f;
        f.frame_id = ++counter_;
        f.capture_ts_ns = static_cast<uint64_t>(ros::Time::now().toNSec());
        f.jpeg.assign(map.data, map.data + map.size);
        gst_buffer_unmap(buf, &map);
        gst_sample_unref(sample);
        {
            std::lock_guard<std::mutex> lk(mtx_);
            latest_ = f; latest_valid_ = true;
            ring_[f.frame_id] = f; order_.push_back(f.frame_id);
            while (static_cast<int>(order_.size()) > ring_size_) { ring_.erase(order_.front()); order_.pop_front(); }
        }
        frame_cv_.notify_one();
        return GST_FLOW_OK;
    }
    bool latest_blocking(Frame& out) {
        std::unique_lock<std::mutex> lk(mtx_);
        frame_cv_.wait(lk, [&]{ return !running_.load() || latest_valid_; });
        if (!running_.load()) return false;
        out = latest_; latest_valid_ = false; return true;
    }
    bool ring_get(uint64_t id, Frame& out) {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = ring_.find(id);
        if (it == ring_.end()) return false;
        out = it->second; return true;
    }

    // ---- fuser TCP socket (length-prefixed; request=msgpack envelope, reply=flat-json) ----
    bool fuser_connect() {
        if (fuser_fd_ >= 0) return true;
        int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) return false;
        sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons(static_cast<uint16_t>(fuser_port_));
        if (::inet_pton(AF_INET, fuser_host_.c_str(), &a.sin_addr) != 1) { ::close(fd); return false; }
        if (::connect(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) { ::close(fd); return false; }
        int one = 1; ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        fuser_fd_ = fd; return true;
    }
    bool send_all(const uint8_t* p, size_t n) {
        while (n) { ssize_t k = ::send(fuser_fd_, p, n, MSG_NOSIGNAL); if (k <= 0) return false; p += k; n -= k; }
        return true;
    }
    bool recv_all(uint8_t* p, size_t n) {
        while (n) { ssize_t k = ::recv(fuser_fd_, p, n, 0); if (k <= 0) return false; p += k; n -= k; }
        return true;
    }
    // send {frame_id, stamp_ns, provider_results:[body], jpeg}; receive flat-json string.
    bool fuser_roundtrip(const Frame& f, const std::vector<uint8_t>& body, std::string& reply) {
        if (!fuser_connect()) return false;
        std::vector<uint8_t> env;
        mp_u8(env, 0x84);                                 // fixmap(4)
        mp_str(env, "frame_id");  mp_u64(env, f.frame_id);
        mp_str(env, "stamp_ns");  mp_u64(env, f.capture_ts_ns);
        mp_str(env, "provider_results"); mp_u8(env, 0x91); // array(1)
        mp_bin32(env, body.data(), static_cast<uint32_t>(body.size()));
        mp_str(env, "jpeg"); mp_bin32(env, f.jpeg.data(), static_cast<uint32_t>(f.jpeg.size()));
        uint32_t len = htonl(static_cast<uint32_t>(env.size()));
        if (!send_all(reinterpret_cast<uint8_t*>(&len), 4) || !send_all(env.data(), env.size())) {
            ::close(fuser_fd_); fuser_fd_ = -1; return false;
        }
        uint32_t rlen_be = 0;
        if (!recv_all(reinterpret_cast<uint8_t*>(&rlen_be), 4)) { ::close(fuser_fd_); fuser_fd_ = -1; return false; }
        uint32_t rlen = ntohl(rlen_be);
        std::vector<uint8_t> rbuf(rlen);
        if (rlen && !recv_all(rbuf.data(), rlen)) { ::close(fuser_fd_); fuser_fd_ = -1; return false; }
        reply.assign(rbuf.begin(), rbuf.end());
        return true;
    }

    static void jarr3(const Json::Value& v, double* out, bool& has) {
        if (v.isArray() && v.size() >= 3) { for (int i = 0; i < 3; ++i) out[i] = v[i].asDouble(); has = true; }
    }
    bool parse_reply(const std::string& js, std::vector<TrackedDet>& out) {
        Json::CharReaderBuilder b; Json::Value root; std::string err;
        std::unique_ptr<Json::CharReader> rd(b.newCharReader());
        if (!rd->parse(js.data(), js.data() + js.size(), &root, &err)) {
            ROS_WARN_STREAM_THROTTLE(2.0, "reply JSON parse: " << err); return false;
        }
        const Json::Value& dets = root["detections"];
        if (!dets.isArray()) return true;
        for (const auto& d : dets) {
            TrackedDet t;
            t.type = d.get("entity_type", "object").asString();
            if (d.isMember("track_id") && d["track_id"].isIntegral()) t.track_id = d["track_id"].asInt();
            t.score = d.get("score", 0.0).asDouble();
            const Json::Value& bb = d["bbox_xyxy"];
            if (bb.isArray() && bb.size() == 4) for (int i = 0; i < 4; ++i) t.bbox[i] = bb[i].asDouble();
            const Json::Value& kp = d["keypoints"];
            if (kp.isArray()) for (const auto& k : kp) {
                Kpt p; if (k.isArray() && k.size() >= 2) { p.x = k[0].asDouble(); p.y = k[1].asDouble();
                        p.s = k.size() >= 3 ? k[2].asDouble() : 1.0; } t.kpts.push_back(p);
            }
            const Json::Value& kp3 = d["keypoints_3d"];
            if (kp3.isArray()) for (const auto& k : kp3) {
                std::array<double,3> a{0,0,0}; if (k.isArray() && k.size() >= 3)
                    for (int i = 0; i < 3; ++i) a[i] = k[i].asDouble(); t.kpts3d.push_back(a);
            }
            jarr3(d["position_3d"], t.pos, t.has_pos);
            jarr3(d["scale_3d"], t.box, t.has_box);
            t.label = d.get("semantic_label", "").asString();
            const Json::Value& at = d["attributes"];
            if (at.isObject()) for (const auto& k : at.getMemberNames()) {
                const Json::Value& vv = at[k];
                t.attrs.emplace_back(k, vv.isString() ? vv.asString() : vv.toStyledString());
            }
            out.push_back(t);
        }
        return true;
    }

    cv::Scalar color_for(const std::string& type) {
        if (type == "face") return cv::Scalar(0, 220, 0);
        if (type == "body") return cv::Scalar(255, 160, 0);
        if (type == "hand") return cv::Scalar(0, 200, 255);
        return cv::Scalar(200, 200, 200);
    }
    void draw(cv::Mat& img, const std::vector<TrackedDet>& dets) {
        for (const auto& d : dets) {
            cv::Scalar c = color_for(d.type);
            cv::Rect r(cv::Point(static_cast<int>(d.bbox[0]), static_cast<int>(d.bbox[1])),
                       cv::Point(static_cast<int>(d.bbox[2]), static_cast<int>(d.bbox[3])));
            r &= cv::Rect(0, 0, img.cols, img.rows);
            if (r.width > 0 && r.height > 0) cv::rectangle(img, r, c, 2);
            for (const auto& k : d.kpts) if (k.s > 0.1)
                cv::circle(img, cv::Point(static_cast<int>(k.x), static_cast<int>(k.y)), 3, c, -1);
            if (d.kpts.size() >= 17) for (const auto& bn : COCO_BONES) {
                const Kpt& a = d.kpts[bn[0]]; const Kpt& b = d.kpts[bn[1]];
                if (a.s > 0.1 && b.s > 0.1)
                    cv::line(img, cv::Point(static_cast<int>(a.x), static_cast<int>(a.y)),
                             cv::Point(static_cast<int>(b.x), static_cast<int>(b.y)), c, 2);
            }
            std::string lbl = d.type;
            if (d.track_id >= 0) lbl += "#" + std::to_string(d.track_id);
            for (const auto& kv : d.attrs) if (kv.first == "gesture") lbl += " " + kv.second;
            int base = 0; cv::Size ts = cv::getTextSize(lbl, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base);
            int tx = r.x, ty = std::max(0, r.y - 5);
            cv::rectangle(img, cv::Rect(tx, std::max(0, ty - ts.height - base), ts.width + 4, ts.height + base + 4), c, cv::FILLED);
            cv::putText(img, lbl, cv::Point(tx + 2, ty - 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1, cv::LINE_AA);
        }
    }

    void publish_detections(const std::vector<TrackedDet>& dets, const std_msgs::Header& hdr) {
        sweetie_bot_text_msgs::DetectionArray arr;
        for (const auto& d : dets) {
            sweetie_bot_text_msgs::Detection m;
            m.header = hdr;
            m.id = d.track_id >= 0 ? d.track_id : 0;
            m.label = d.label;
            m.type = d.type;
            m.score = static_cast<float>(d.score);
            for (const auto& kv : d.attrs) { m.attribute.push_back(kv.first); m.value.push_back(kv.second); }
            m.pose.orientation.w = 1.0;
            if (d.has_pos) { m.pose.position.x = d.pos[0]; m.pose.position.y = d.pos[1]; m.pose.position.z = d.pos[2]; }
            if (d.has_box) { m.box.x = d.box[0]; m.box.y = d.box[1]; m.box.z = d.box[2]; }
            arr.detections.push_back(m);
        }
        det_pub_.publish(arr);
    }

    void publish_skeletons(const std::vector<TrackedDet>& dets, const std_msgs::Header& hdr) {
        if (!skel_pub_) return;
        visualization_msgs::MarkerArray arr;
        int id = 0;
        for (const auto& d : dets) {
            if (d.kpts3d.size() < 17) continue;
            std_msgs::ColorRGBA col; col.r = 0.2f; col.g = 0.9f; col.b = 0.3f; col.a = 1.0f;
            visualization_msgs::Marker bones;
            bones.header = hdr; bones.ns = "vision_skeleton"; bones.id = id++;
            bones.type = visualization_msgs::Marker::LINE_LIST; bones.action = visualization_msgs::Marker::ADD;
            bones.scale.x = 0.02; bones.color = col; bones.lifetime = ros::Duration(0.5);
            for (const auto& bn : COCO_BONES) {
                geometry_msgs::Point pa, pb;
                pa.x = d.kpts3d[bn[0]][0]; pa.y = d.kpts3d[bn[0]][1]; pa.z = d.kpts3d[bn[0]][2];
                pb.x = d.kpts3d[bn[1]][0]; pb.y = d.kpts3d[bn[1]][1]; pb.z = d.kpts3d[bn[1]][2];
                bones.points.push_back(pa); bones.points.push_back(pb);
            }
            arr.markers.push_back(bones);
        }
        skel_pub_.publish(arr);
    }

    void worker_loop() {
        while (running_.load() && ros::ok()) {
            try { session(); }
            catch (const std::exception& e) { ROS_ERROR_STREAM("provider session error: " << e.what()); }
            if (running_.load() && ros::ok()) { ROS_WARN("provider reconnect in 2s"); ros::Duration(2.0).sleep(); }
        }
    }
    void session() {
        net::io_context ioc;
        tcp::resolver resolver(ioc);
        websocket::stream<tcp::socket> ws(ioc);
        auto results = resolver.resolve(prov_host_, std::to_string(prov_port_));
        net::connect(ws.next_layer(), results.begin(), results.end());
        ws.set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));
        ws.handshake(prov_host_, prov_target_);
        ws.binary(true);
        ROS_INFO_STREAM("connected to provider ws://" << prov_host_ << ":" << prov_port_);
        while (running_.load() && ros::ok()) {
            Frame f;
            if (!latest_blocking(f)) break;
            FrameHeader h{}; std::memcpy(h.magic, "MJPG", 4); h.version = 1; h.header_size = sizeof(FrameHeader);
            h.frame_id = f.frame_id; h.capture_ts_ns = f.capture_ts_ns; h.width = 0; h.height = 0;
            h.jpeg_size = static_cast<uint32_t>(f.jpeg.size()); h.camera_rotation_deg = rot_deg_; h.flags = 0;
            std::vector<uint8_t> msg(sizeof(FrameHeader) + f.jpeg.size());
            std::memcpy(msg.data(), &h, sizeof(FrameHeader));
            std::memcpy(msg.data() + sizeof(FrameHeader), f.jpeg.data(), f.jpeg.size());
            ws.write(net::buffer(msg));
            beast::flat_buffer rb; ws.read(rb);
            std::string raw = beast::buffers_to_string(rb.data());
            // strip VRES header -> opaque provider body bytes
            std::vector<uint8_t> body;
            uint64_t result_fid = f.frame_id;
            if (raw.size() >= sizeof(VresHeader) && std::memcmp(raw.data(), "VRES", 4) == 0) {
                VresHeader vh; std::memcpy(&vh, raw.data(), sizeof(VresHeader));
                result_fid = vh.frame_id;
                size_t off = vh.header_size ? vh.header_size : sizeof(VresHeader);
                if (off <= raw.size()) body.assign(raw.begin() + off, raw.end());
            } else {
                body.assign(raw.begin(), raw.end());
            }
            // relay to fuser, get tracked flat-json
            std::string reply;
            if (!fuser_roundtrip(f, body, reply)) {
                ROS_WARN_STREAM_THROTTLE(2.0, "fuser roundtrip failed (frame " << f.frame_id << ")");
                continue;
            }
            std_msgs::String rj; rj.data = reply; result_pub_.publish(rj);
            std::vector<TrackedDet> dets;
            parse_reply(reply, dets);
            std_msgs::Header hdr; hdr.stamp = ros::Time().fromNSec(f.capture_ts_ns); hdr.frame_id = image_frame_id_;
            // draw on the ring frame matched by frame_id (async-tolerant)
            Frame draw_frame = f;
            if (result_fid != f.frame_id) ring_get(result_fid, draw_frame);
            cv::Mat raw_img = cv::imdecode(cv::Mat(1, static_cast<int>(draw_frame.jpeg.size()), CV_8UC1,
                                                   draw_frame.jpeg.data()), cv::IMREAD_COLOR);
            if (!raw_img.empty()) {
                cv::Mat img = raw_img.clone();
                draw(img, dets);
                image_pub_.publish(cv_bridge::CvImage(hdr, "bgr8", img).toImageMsg());
            }
            publish_detections(dets, hdr);
            publish_skeletons(dets, hdr);
        }
        beast::error_code ec; ws.close(websocket::close_code::normal, ec);
    }

    ros::NodeHandle nh_, pnh_;
    ros::Publisher result_pub_, image_pub_, det_pub_, skel_pub_;
    std::string pipeline_, prov_host_, prov_target_, fuser_host_, image_topic_, image_frame_id_, det_topic_, skel_topic_;
    int prov_port_ = 8080, fuser_port_ = 9100, ring_size_ = 60, rot_deg_ = 0;
    GstElement* pipeline_p_ = nullptr; GstAppSink* appsink_ = nullptr;
    std::atomic<bool> running_{false};
    std::thread worker_;
    std::mutex mtx_; std::condition_variable frame_cv_;
    Frame latest_; bool latest_valid_ = false;
    std::unordered_map<uint64_t, Frame> ring_; std::deque<uint64_t> order_;
    std::atomic<uint64_t> counter_{0};
    int fuser_fd_ = -1;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_proxy_node");
    VisionProxyNode node;
    if (!node.start()) return 1;
    ros::spin();
    node.stop();
    return 0;
}
