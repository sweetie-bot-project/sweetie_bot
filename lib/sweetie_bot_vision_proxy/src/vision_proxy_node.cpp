// sweetie_bot_vision_proxy — the C++ front transport + visualizer of the vision federation.
//
// gstreamer ingest -> monotonic frame_id -> cyclic ring buffer -> WebSocket to the provider
// container(s) (FrameHeader+JPEG up, VRES+bytes down) -> relay the provider bytes opaquely to the
// py3.10 tracker-fuser over a TCP socket (msgpack envelope) -> parse the fuser's flat-JSON tracked
// reply -> draw boxes+keypoints on the ring frame and publish /image_raw, plus /detections
// (DetectionArray, now carrying keypoints_3d) + /vision_proxy/result_json. PURE DATA bridge: rviz
// MarkerArray viz (skeleton/gaze) lives in the hmi package (sweetie_bot_rviz_interactions). Single ROS face.
//
// PHASE 2: MULTI-container WS client — `ws` to the LOCAL container (loopback) + `wss` (TLS) to an
// optional REMOTE container; each frame is fanned to all enabled providers and their result bodies are
// merged by the fuser (which from_wire's each). Forward msgpack envelope hand-encoded (no msgpack-cxx).
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <sweetie_bot_text_msgs/Detection.h>
#include <sweetie_bot_text_msgs/DetectionArray.h>
#include <cv_bridge/cv_bridge.h>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <boost/asio.hpp>
#include <boost/asio/ssl.hpp>
#include <boost/beast.hpp>
#include <boost/beast/ssl.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/websocket/ssl.hpp>
#include <boost/asio/ssl/rfc2818_verification.hpp>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <cstddef>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/freetype.hpp>
#include <json/json.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
namespace ssl = boost::asio::ssl;
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

// strip the VRES header from a provider reply -> opaque body bytes (returns frame_id, 0 if no header).
static uint64_t strip_vres(const std::string& raw, std::vector<uint8_t>& body) {
    if (raw.size() >= sizeof(VresHeader) && std::memcmp(raw.data(), "VRES", 4) == 0) {
        VresHeader vh; std::memcpy(&vh, raw.data(), sizeof(VresHeader));
        size_t off = vh.header_size ? vh.header_size : sizeof(VresHeader);
        if (off <= raw.size()) body.assign(raw.begin() + off, raw.end());
        return vh.frame_id;
    }
    body.assign(raw.begin(), raw.end());
    return 0;
}

// ---- provider connections: ws (local) and wss (remote). Lazy (re)connect; query = write+read+strip.
//
// P21 hardening. Beast's websocket stream timeout option applies to ASYNC operations only, and
// asio's synchronous recv retries EAGAIN on a blocking socket with an unbounded poll — so plain
// sync write/read can wedge FOREVER on a half-open link (server thread died without FIN,
// container pause, conntrack drop) with no exception, no log, no reconnect: the exact live
// incident (perception blind, /detections empty, not one error line). Therefore every I/O op
// runs ASYNC with a wall-clock bound (run_for + hard-close on overrun), failures are LOUD
// (unthrottled DOWN/RECOVERED transitions, throttled repeats), reconnects use a bounded
// 0.5->3 s backoff, and a reply whose echoed frame_id mismatches the uplink counts as a dead
// (desynced) link. A wedged-but-open socket is torn down, never reused. ----

// Run one async op to completion within timeout_s. start(fin) must begin the op and arrange for
// fin(ec) from its completion handler; on deadline overrun abort() must hard-close the transport
// (which completes the pending op with operation_aborted), after which the io_context is drained.
// Throws on timeout or op error.
template <class Start, class Abort>
static void run_bounded(net::io_context& ioc, double timeout_s, const char* what,
                        Start&& start, Abort&& abort) {
    bool done = false; beast::error_code ec;
    auto fin = [&done, &ec](beast::error_code e) { done = true; ec = e; };
    start(fin);
    ioc.restart();
    ioc.run_for(std::chrono::milliseconds(static_cast<long>(timeout_s * 1000)));
    if (!done) {
        abort();
        ioc.restart();
        ioc.run();
        char b[96];
        std::snprintf(b, sizeof(b), "%s timed out (%.1fs)", what, timeout_s);
        throw std::runtime_error(b);
    }
    if (ec) throw std::runtime_error(std::string(what) + ": " + ec.message());
}

// uplink frame_id straight off the wire message (the provider server echoes it in the VRES reply).
static uint64_t frame_id_of(const std::vector<uint8_t>& msg) {
    uint64_t fid = 0;
    if (msg.size() >= sizeof(FrameHeader) && std::memcmp(msg.data(), "MJPG", 4) == 0)
        std::memcpy(&fid, msg.data() + offsetof(FrameHeader, frame_id), sizeof(fid));
    return fid;
}

struct ProviderConn {
    virtual ~ProviderConn() {}
    virtual std::string name() const = 0;
    virtual bool query(const std::vector<uint8_t>& frame_msg, std::vector<uint8_t>& body) = 0;
    bool healthy() const { return healthy_; }
    // watchdog escalation: tear down even an apparently-open link and retry NOW (skip backoff).
    void force_reset() { teardown(); next_attempt_ = std::chrono::steady_clock::time_point{}; }
    // mark this link CRITICAL (the local provider): failure logs at ERROR grade and NEVER goes
    // silent -- run-real is blind without it. Optional remotes stay quiet-after-a-few (default).
    void set_critical(bool c) { critical_ = c; }
protected:
    virtual void teardown() = 0;
    // fail-fast gate: while in post-failure backoff, don't touch the network at all.
    bool in_backoff() const { return std::chrono::steady_clock::now() < next_attempt_; }
    void note_ok() {
        if (!healthy_ && fails_ > 0) {                 // failed -> healthy transition: ONE line
            double down = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - down_since_).count();
            char b[80];
            std::snprintf(b, sizeof(b), "(re)established after %.1fs (%d failed attempts)", down, fails_);
            ROS_INFO_STREAM(name() << " provider link " << b);
        }
        healthy_ = true; fails_ = 0; backoff_s_ = 0.5;
    }
    void note_fail(const std::string& what) {
        teardown();
        auto now = std::chrono::steady_clock::now();
        ++fails_;
        // Per-link severity. The LOCAL critical link (run-real is blind without it) logs at
        // ERROR grade and NEVER goes silent: a throttled ERROR keeps repeating until it returns.
        // An OPTIONAL remote logs a few loud WARNs, announces it is going quiet ONCE, then retries
        // silently at a capped interval (degradation, not a fatal outage).
        if (fails_ == 1) {                                  // healthy -> dead transition, unthrottled
            down_since_ = now;
            if (critical_) ROS_ERROR_STREAM(name() << " LOCAL provider link DOWN: " << what);
            else           ROS_WARN_STREAM (name() << " provider link DOWN: " << what);
        } else if (critical_) {                             // critical: loud forever, but throttled
            ROS_ERROR_STREAM_THROTTLE(loud_repeat_s_, name() << " LOCAL provider link still down (retry in "
                                      << backoff_s_ << "s): " << what);
        } else if (fails_ <= noisy_fails_) {                // optional remote: a few loud attempts
            ROS_WARN_STREAM(name() << " provider link still down (attempt " << fails_
                            << ", retry in " << backoff_s_ << "s): " << what);
        } else if (fails_ == noisy_fails_ + 1) {            // ...then announce going quiet, ONCE
            ROS_WARN_STREAM(name() << " provider link still down after " << noisy_fails_
                            << " attempts -- retrying quietly every " << quiet_backoff_s_ << "s");
        }                                                   // fails_ > noisy_fails_+1: silent, keep retrying
        healthy_ = false;
        next_attempt_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(backoff_s_));
        double cap = critical_ ? loud_cap_s_
                   : (fails_ > noisy_fails_ ? quiet_backoff_s_ : loud_cap_s_);
        backoff_s_ = std::min(backoff_s_ * 2.0, cap);
    }
    bool healthy_ = false; int fails_ = 0;
    double backoff_s_ = 0.5;
    // logging policy. critical_ (local): ERROR + never silent, fast reconnect (loud_cap_s_).
    // else (remote): quiet after noisy_fails_ loud WARNs, capped retry at quiet_backoff_s_.
    // loud_repeat_s_ = throttle window for the critical link's repeated ERROR.
    bool critical_ = false;
    int noisy_fails_ = 5;
    double loud_cap_s_ = 3.0;
    double quiet_backoff_s_ = 30.0;
    double loud_repeat_s_ = 15.0;
    std::chrono::steady_clock::time_point down_since_{}, next_attempt_{};
};

class WsProvider : public ProviderConn {
public:
    WsProvider(std::string h, int p, std::string t, std::string tok = "", double io_timeout_s = 5.0)
        : host_(std::move(h)), target_(std::move(t)), token_(std::move(tok)), port_(p),
          io_timeout_s_(io_timeout_s) {}
    std::string name() const override { return "ws://" + host_ + ":" + std::to_string(port_); }
    bool query(const std::vector<uint8_t>& msg, std::vector<uint8_t>& body) override {
        if (!ws_ && in_backoff()) return false;            // fail fast between reconnect attempts
        try {
            if (!ws_) connect();
            auto abort = [&] { beast::error_code ig; ws_->next_layer().close(ig); };
            run_bounded(*ioc_, io_timeout_s_, "write", [&](auto fin) {
                ws_->async_write(net::buffer(msg),
                                 [fin](beast::error_code ec, std::size_t) { fin(ec); }); }, abort);
            beast::flat_buffer rb;
            run_bounded(*ioc_, io_timeout_s_, "read", [&](auto fin) {
                ws_->async_read(rb,
                                [fin](beast::error_code ec, std::size_t) { fin(ec); }); }, abort);
            uint64_t got = strip_vres(beast::buffers_to_string(rb.data()), body);
            uint64_t sent = frame_id_of(msg);
            if (got && sent && got != sent)    // stale echo = desynced link: dead despite "working"
                throw std::runtime_error("desynced reply (sent frame " + std::to_string(sent) +
                                         ", got " + std::to_string(got) + ")");
            note_ok();
            return true;
        } catch (const std::exception& e) {
            note_fail(e.what());
            return false;
        }
    }
private:
    void teardown() override { ws_.reset(); ioc_.reset(); }
    void connect() {
        ioc_ = std::make_unique<net::io_context>();
        tcp::resolver res(*ioc_);
        auto r = res.resolve(host_, std::to_string(port_));   // sync: loopback/LAN, effectively instant
        ws_ = std::make_unique<websocket::stream<tcp::socket>>(*ioc_);
        auto abort = [&] { beast::error_code ig; ws_->next_layer().close(ig); };
        run_bounded(*ioc_, io_timeout_s_, "connect", [&](auto fin) {
            net::async_connect(ws_->next_layer(), r.begin(), r.end(),
                               [fin](beast::error_code ec, tcp::resolver::iterator) { fin(ec); }); },
            abort);
        std::string tok = token_;
        ws_->set_option(websocket::stream_base::decorator([tok](websocket::request_type& req) {
            req.set(beast::http::field::user_agent, "vision_proxy_node");
            if (!tok.empty()) req.set(beast::http::field::authorization, "Bearer " + tok);
        }));
        run_bounded(*ioc_, io_timeout_s_, "ws handshake", [&](auto fin) {
            ws_->async_handshake(host_, target_, [fin](beast::error_code ec) { fin(ec); }); }, abort);
        ws_->binary(true);
        ROS_INFO_STREAM("connected " << name() << (token_.empty() ? "" : " (auth)"));
    }
    std::string host_, target_, token_; int port_;
    double io_timeout_s_;
    std::unique_ptr<net::io_context> ioc_;
    std::unique_ptr<websocket::stream<tcp::socket>> ws_;
};

class WssProvider : public ProviderConn {
public:
    WssProvider(std::string h, int p, std::string t, bool insec, std::string tok,
                double io_timeout_s = 30.0)
        : host_(std::move(h)), target_(std::move(t)), token_(std::move(tok)), port_(p),
          insecure_(insec), io_timeout_s_(io_timeout_s) {}
    std::string name() const override { return "wss://" + host_ + ":" + std::to_string(port_); }
    bool query(const std::vector<uint8_t>& msg, std::vector<uint8_t>& body) override {
        if (!ws_ && in_backoff()) return false;            // fail fast between reconnect attempts
        try {
            if (!ws_) connect();
            auto abort = [&] { beast::error_code ig; beast::get_lowest_layer(*ws_).close(ig); };
            run_bounded(*ioc_, io_timeout_s_, "write", [&](auto fin) {
                ws_->async_write(net::buffer(msg),
                                 [fin](beast::error_code ec, std::size_t) { fin(ec); }); }, abort);
            beast::flat_buffer rb;
            run_bounded(*ioc_, io_timeout_s_, "read", [&](auto fin) {
                ws_->async_read(rb,
                                [fin](beast::error_code ec, std::size_t) { fin(ec); }); }, abort);
            uint64_t got = strip_vres(beast::buffers_to_string(rb.data()), body);
            uint64_t sent = frame_id_of(msg);
            if (got && sent && got != sent)    // stale echo = desynced link: dead despite "working"
                throw std::runtime_error("desynced reply (sent frame " + std::to_string(sent) +
                                         ", got " + std::to_string(got) + ")");
            note_ok();
            return true;
        } catch (const std::exception& e) {
            note_fail(e.what());
            return false;
        }
    }
private:
    void teardown() override { ws_.reset(); ctx_.reset(); ioc_.reset(); }
    void connect() {
        ioc_ = std::make_unique<net::io_context>();
        ctx_ = std::make_unique<ssl::context>(ssl::context::tlsv12_client);
        ctx_->set_default_verify_paths();
        ctx_->set_verify_mode(insecure_ ? ssl::verify_none : ssl::verify_peer);
        if (!insecure_) ctx_->set_verify_callback(ssl::rfc2818_verification(host_));
        tcp::resolver res(*ioc_);
        ws_ = std::make_unique<websocket::stream<beast::ssl_stream<tcp::socket>>>(*ioc_, *ctx_);
        if (!SSL_set_tlsext_host_name(ws_->next_layer().native_handle(), host_.c_str()))
            throw beast::system_error(beast::error_code(
                static_cast<int>(::ERR_get_error()), net::error::get_ssl_category()));
        auto r = res.resolve(host_, std::to_string(port_));
        auto abort = [&] { beast::error_code ig; beast::get_lowest_layer(*ws_).close(ig); };
        run_bounded(*ioc_, io_timeout_s_, "connect", [&](auto fin) {
            net::async_connect(beast::get_lowest_layer(*ws_), r.begin(), r.end(),
                               [fin](beast::error_code ec, tcp::resolver::iterator) { fin(ec); }); },
            abort);
        run_bounded(*ioc_, io_timeout_s_, "tls handshake", [&](auto fin) {
            ws_->next_layer().async_handshake(ssl::stream_base::client,
                                              [fin](beast::error_code ec) { fin(ec); }); }, abort);
        std::string tok = token_;
        ws_->set_option(websocket::stream_base::decorator([tok](websocket::request_type& req) {
            req.set(beast::http::field::user_agent, "vision_proxy_node");
            if (!tok.empty()) req.set(beast::http::field::authorization, "Bearer " + tok);
        }));
        run_bounded(*ioc_, io_timeout_s_, "ws handshake", [&](auto fin) {
            ws_->async_handshake(host_, target_, [fin](beast::error_code ec) { fin(ec); }); }, abort);
        ws_->binary(true);
        ROS_INFO_STREAM("connected " << name() << (insecure_ ? " (insecure)" : ""));
    }
    std::string host_, target_, token_; int port_; bool insecure_;
    double io_timeout_s_;
    std::unique_ptr<net::io_context> ioc_;
    std::unique_ptr<ssl::context> ctx_;
    std::unique_ptr<websocket::stream<beast::ssl_stream<tcp::socket>>> ws_;
};

// Wraps a (slow / unreliable) remote ProviderConn in its OWN thread: latest-frame-in / latest-result-out,
// so the fast local loop NEVER blocks on it. The main loop submit()s the current frame and reads the
// remote's most-recent fresh result (within max_age_ms); a slow/down remote just yields nothing that
// frame and self-heals in its own thread. Mirrors the native AsyncDepthClient's best-effort tier.
class RemoteWorker {
public:
    RemoteWorker(std::unique_ptr<ProviderConn> conn, double max_age_ms)
        : conn_(std::move(conn)), max_age_ms_(max_age_ms) {
        th_ = std::thread(&RemoteWorker::loop, this);
    }
    ~RemoteWorker() { stop(); }
    std::string name() const { return conn_->name(); }
    void submit(const std::vector<uint8_t>& msg) {
        { std::lock_guard<std::mutex> lk(m_); in_ = msg; have_in_ = true; }
        cv_.notify_one();
    }
    bool latest(std::vector<uint8_t>& out) {
        std::lock_guard<std::mutex> lk(m_);
        if (!have_out_) return false;
        double age = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - out_time_).count();
        if (age > max_age_ms_) return false;          // too stale -> skip this frame
        out = out_;
        return true;
    }
    void stop() {
        { std::lock_guard<std::mutex> lk(m_); running_ = false; }
        cv_.notify_all();
        if (th_.joinable()) th_.join();
    }
private:
    void loop() {
        while (true) {
            std::vector<uint8_t> msg;
            {
                std::unique_lock<std::mutex> lk(m_);
                cv_.wait(lk, [&]{ return !running_ || have_in_; });
                if (!running_) return;
                msg = in_; have_in_ = false;          // take the LATEST submitted frame (drop intermediates)
            }
            std::vector<uint8_t> body;
            if (conn_->query(msg, body)) {            // blocking on ITS own stream (may be slow) — off the fast loop
                std::lock_guard<std::mutex> lk(m_);
                out_ = std::move(body); have_out_ = true;
                out_time_ = std::chrono::steady_clock::now();
            }
        }
    }
    std::unique_ptr<ProviderConn> conn_;
    double max_age_ms_;
    std::thread th_;
    std::mutex m_; std::condition_variable cv_;
    std::vector<uint8_t> in_, out_;
    bool have_in_ = false, have_out_ = false, running_ = true;
    std::chrono::steady_clock::time_point out_time_;
};

class VisionProxyNode {
public:
    VisionProxyNode() : nh_(), pnh_("~") {
        pnh_.param<std::string>("pipeline", pipeline_,
            "udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! "
            "jpegparse ! appsink name=sink emit-signals=true sync=false drop=true max-buffers=1");
        pnh_.param<std::string>("provider_host", prov_host_, "127.0.0.1");
        pnh_.param<int>("provider_port", prov_port_, 8080);
        pnh_.param<std::string>("provider_target", prov_target_, "/");
        // optional REMOTE provider over wss (empty host -> local-only)
        pnh_.param<std::string>("remote_host", remote_host_, "");
        pnh_.param<int>("remote_port", remote_port_, 8443);
        pnh_.param<std::string>("remote_target", remote_target_, "/");
        pnh_.param<bool>("remote_insecure", remote_insecure_, true);
        pnh_.param<std::string>("remote_token", remote_token_, "");
        // The Bearer token is a SECRET: never stored in config. If the rosparam is empty, draw
        // it from $VISION_API_KEY in the environment (the same key the server validates against).
        if (remote_token_.empty()) {
            if (const char* k = std::getenv("VISION_API_KEY")) remote_token_ = k;
        }
        // VARIABLE NUMBER of remote provider containers: a `remotes` list param (XmlRpc array of
        // {host, port, tls, insecure, target, token_env}). Each becomes its own async RemoteWorker, so
        // the system scales to N containers (local + second laptop + server + ...) with no code change.
        XmlRpc::XmlRpcValue rlist;
        if (pnh_.getParam("remotes", rlist) && rlist.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < rlist.size(); ++i) {
                XmlRpc::XmlRpcValue& e = rlist[i];
                if (!e.hasMember("host")) continue;
                RemoteSpec rs;
                rs.host = static_cast<std::string>(e["host"]);
                if (rs.host.empty()) continue;
                rs.port = e.hasMember("port") ? static_cast<int>(e["port"]) : 443;
                rs.target = e.hasMember("target") ? static_cast<std::string>(e["target"]) : std::string("/");
                rs.tls = e.hasMember("tls") ? static_cast<bool>(e["tls"]) : true;
                rs.insecure = e.hasMember("insecure") ? static_cast<bool>(e["insecure"]) : false;
                std::string tenv = e.hasMember("token_env") ? static_cast<std::string>(e["token_env"])
                                                            : std::string("VISION_API_KEY");
                const char* tv = std::getenv(tenv.c_str());
                rs.token = tv ? std::string(tv) : std::string();
                remote_specs_.push_back(rs);
            }
        }
        // Back-compat: the legacy single remote_host param becomes one (wss) remote if no list is given.
        if (remote_specs_.empty() && !remote_host_.empty())
            remote_specs_.push_back(RemoteSpec{remote_host_, remote_target_, remote_token_,
                                               remote_port_, true, remote_insecure_});
        ROS_INFO_STREAM("vision_proxy: " << remote_specs_.size() << " remote container(s)");
        pnh_.param<double>("remote_max_staleness_ms", remote_max_staleness_ms_, 1500.0);
        // P21 local-link hardening: every local I/O op is wall-clock-bounded; the watchdog forces
        // a reconnect cycle (and an ERROR log) when nothing succeeds for too long.
        pnh_.param<double>("provider_io_timeout", provider_io_timeout_, 2.0);
        pnh_.param<double>("provider_watchdog", provider_watchdog_, 10.0);
        // provider_critical: the LOCAL link is FATAL on the real robot (run-real is blind without
        // it) -> ERROR-grade, never-silent logging. Default false so sim/noreal (no local
        // container) keeps a clean log. vision.launch wires this to run_real.
        pnh_.param<bool>("provider_critical", provider_critical_, false);
        pnh_.param<std::string>("fuser_host", fuser_host_, "127.0.0.1");
        pnh_.param<int>("fuser_port", fuser_port_, 9100);
        pnh_.param<int>("ring_size", ring_size_, 60);
        pnh_.param<std::string>("image_topic", image_topic_, "/image_raw");
        // Pre-draw pixels for dataset consumers (the data recorder records this topic): image_topic
        // keeps the HUD/boxes for the operator view, clean_image_topic carries the same frame
        // BEFORE draw() mutates it. Published lazily (only while subscribed).
        pnh_.param<std::string>("clean_image_topic", clean_image_topic_, "/vision_proxy/image_clean");
        pnh_.param<std::string>("image_frame_id", image_frame_id_, "camera_link_optical");
        pnh_.param<std::string>("detections_topic", det_topic_, "detections");
        // cyber HUD font via opencv_freetype; empty/missing path -> Hershey fallback
        std::string hud_font;
        pnh_.param<std::string>("hud_font", hud_font, "");
        if (!hud_font.empty()) {
            try {
                ft2_ = cv::freetype::createFreeType2();
                ft2_->loadFontData(hud_font, 0);
                ft_ready_ = true;
                ROS_INFO_STREAM("HUD font: " << hud_font);
            } catch (const std::exception& e) {
                ROS_WARN_STREAM("HUD font load failed (" << hud_font << "): " << e.what() << " -> Hershey");
            }
        }
        pnh_.param<int>("camera_rotation_deg", rot_deg_, 0);
        // The proto3 robot's camera_link_optical TF is non-standard (x-left, y-up vs REP-103
        // x-right, y-down), so the core's clean REP-103 output renders upside-down + mirrored. Negate
        // x,y of every published 3D position to match the robot frame (the old gasket did the same).
        pnh_.param<bool>("flip_optical_xy", flip_optical_xy_, true);
        // Map core entity types to the SOAR taxonomy at the ROS boundary (vision.soar keys a
        // person on 'human' and the plushie on 'pony'; the core/wire keep their raw types so the
        // dataset rig & caches are unaffected). Overridable via the ~type_map rosparam.
        type_map_ = {{"face", "human"}, {"pony_body", "pony"}};
        XmlRpc::XmlRpcValue tmap;
        if (pnh_.getParam("type_map", tmap) && tmap.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            type_map_.clear();
            for (auto it = tmap.begin(); it != tmap.end(); ++it)
                if (it->second.getType() == XmlRpc::XmlRpcValue::TypeString)
                    type_map_[it->first] = static_cast<std::string>(it->second);
        }
        result_pub_ = nh_.advertise<std_msgs::String>("/vision_proxy/result_json", 10);
        image_pub_ = nh_.advertise<sensor_msgs::Image>(image_topic_, 1);
        clean_pub_ = nh_.advertise<sensor_msgs::Image>(clean_image_topic_, 1);
        det_pub_ = nh_.advertise<sweetie_bot_text_msgs::DetectionArray>(det_topic_, 5);
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
        ROS_INFO_STREAM("vision_proxy_node up: local ws://" << prov_host_ << ":" << prov_port_
                        << (remote_host_.empty() ? "" : (" + remote wss://" + remote_host_ + ":" +
                            std::to_string(remote_port_))) << " fuser " << fuser_host_ << ":" << fuser_port_);
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
    // send {frame_id, stamp_ns, provider_results:[body...], jpeg}; receive flat-json string.
    bool fuser_roundtrip(const Frame& f, const std::vector<std::vector<uint8_t>>& bodies, std::string& reply) {
        if (!fuser_connect()) return false;
        std::vector<uint8_t> env;
        mp_u8(env, 0x84);                                 // fixmap(4)
        mp_str(env, "frame_id");  mp_u64(env, f.frame_id);
        mp_str(env, "stamp_ns");  mp_u64(env, f.capture_ts_ns);
        mp_str(env, "provider_results");
        mp_u8(env, 0x90 | static_cast<uint8_t>(bodies.size() & 0x0f));   // fixarray (n<=15)
        for (const auto& b : bodies) mp_bin32(env, b.data(), static_cast<uint32_t>(b.size()));
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
    // measured-color debug tint: BGR for each name the perfusion colorname vocabulary can emit,
    // so a box is drawn in the RECOGNIZED color (operator eyeballs the recognizer per object).
    bool named_color(const std::string& name, cv::Scalar& out) {
        static const std::unordered_map<std::string, cv::Scalar> M = {
            {"red", {0, 0, 230}},          {"orange", {0, 140, 255}},
            {"yellow", {0, 220, 255}},     {"green", {0, 200, 0}},
            {"cyan", {255, 255, 0}},       {"blue", {255, 80, 0}},
            {"purple", {200, 0, 160}},     {"pink", {180, 105, 255}},
            {"brown", {19, 69, 139}},      {"lavender", {230, 162, 200}},
            {"white", {255, 255, 255}},    {"gray", {128, 128, 128}},
            {"black", {50, 50, 50}}};
        auto it = M.find(name);
        if (it == M.end()) return false;
        out = it->second;
        return true;
    }
    void draw(cv::Mat& img, const std::vector<TrackedDet>& dets, uint64_t cap_ts_ns) {
        // TOP-LEFT HUD: rolling FPS (publish cadence) + pipeline latency (ingest -> draw).
        uint64_t now_wall = ros::WallTime::now().toNSec();
        if (last_draw_ns_ != 0) {
            double dt = (now_wall - last_draw_ns_) * 1e-9;
            if (dt > 1e-6) {
                double inst = 1.0 / dt;
                hud_fps_ = (hud_fps_ > 0.0) ? (0.9 * hud_fps_ + 0.1 * inst) : inst;
            }
        }
        last_draw_ns_ = now_wall;
        double lat_ms = (static_cast<double>(ros::Time::now().toNSec())
                         - static_cast<double>(cap_ts_ns)) * 1e-6;
        // refresh the displayed numbers at most ~2 Hz (every 500 ms) to reduce flicker
        if (hud_last_update_ns_ == 0 || (now_wall - hud_last_update_ns_) >= 500000000ULL) {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "%.1f fps  %.0f ms", hud_fps_, lat_ms);
            hud_text_ = buf;
            if (local_down_secs_ > 0.5) {   // P21: the stream must not look fine while blind
                char lbuf[32];
                std::snprintf(lbuf, sizeof(lbuf), "  NO LOCAL %.0fs", local_down_secs_);
                hud_text_ += lbuf;
            }
            hud_last_update_ns_ = now_wall;
        }
        if (!hud_text_.empty()) {
            const int fh = 20;                 // font height (px)
            const int xoff = 5, yoff = 5;
            int hb = 0;
            cv::Size hs = ft_ready_ ? ft2_->getTextSize(hud_text_, fh, -1, &hb)
                                    : cv::getTextSize(hud_text_, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &hb);
            cv::Rect hud_rect(4, 4, hs.width + 2 * xoff + 4, hs.height + hb + 2 * yoff + 4);
            hud_rect &= cv::Rect(0, 0, img.cols, img.rows);
            if (hud_rect.width > 0 && hud_rect.height > 0) {
                cv::Mat roi = img(hud_rect);
                // glyph mask in the cyber font (Hershey fallback). BINARY: threshold away any AA gray.
                cv::Mat mask;
                if (ft_ready_) {
                    cv::Mat m3 = cv::Mat::zeros(roi.size(), CV_8UC3);
                    ft2_->putText(m3, hud_text_, cv::Point(xoff, yoff), fh, cv::Scalar(255, 255, 255), -1, cv::LINE_AA, false);
                    cv::cvtColor(m3, mask, cv::COLOR_BGR2GRAY);
                } else {
                    mask = cv::Mat::zeros(roi.size(), CV_8UC1);
                    cv::putText(mask, hud_text_, cv::Point(xoff, hs.height + yoff), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                cv::Scalar(255), 2, cv::LINE_8);
                }
                cv::threshold(mask, mask, 127, 255, cv::THRESH_BINARY);
                // EMBOLDEN: the TTF has no bold weight, so thicken the strokes by dilating the mask.
                cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
                // PER-CHARACTER color (not per-pixel): each glyph is uniformly black or white, picked
                // by the mean background brightness under that character's ink (monospace -> equal cells).
                cv::Mat gray; cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
                int n = static_cast<int>(hud_text_.size());
                double cellw = static_cast<double>(hs.width) / std::max(1, n);
                for (int i = 0; i < n; ++i) {
                    int cx0 = xoff + static_cast<int>(std::lround(i * cellw));
                    int cx1 = xoff + static_cast<int>(std::lround((i + 1) * cellw));
                    cx0 = std::max(0, std::min(cx0, roi.cols));
                    cx1 = std::max(0, std::min(cx1, roi.cols));
                    if (cx1 <= cx0) continue;
                    cv::Rect cell(cx0, 0, cx1 - cx0, roi.rows);
                    cv::Mat cm = mask(cell);
                    if (cv::countNonZero(cm) == 0) continue;        // space / empty cell
                    double bg = cv::mean(gray(cell), cm)[0];        // bg brightness under this glyph
                    roi(cell).setTo(bg > 127 ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255), cm);
                }
            }
        }
        for (const auto& d : dets) {
            cv::Scalar c = color_for(d.type);
            std::string measured;   // "color" attribute -> tint box + append name to the label
            for (const auto& kv : d.attrs)
                if (kv.first == "color" && named_color(kv.second, c)) measured = kv.second;
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
            if (!measured.empty()) lbl += " " + measured;
            int base = 0; cv::Size ts = cv::getTextSize(lbl, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base);
            int tx = r.x, ty = std::max(0, r.y - 5);
            cv::rectangle(img, cv::Rect(tx, std::max(0, ty - ts.height - base), ts.width + 4, ts.height + base + 4), c, cv::FILLED);
            cv::putText(img, lbl, cv::Point(tx + 2, ty - 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1, cv::LINE_AA);
        }
    }

    // REP-103 core position -> robot camera_link_optical convention (negate x,y if the frame is the
    // non-standard proto3 one). Applied to every published 3D position so /detections (-> SWM) and the
    // hmi marker node (skeleton/gaze) are all consistent and render right-side-up.
    inline double fx_(double x) const { return flip_optical_xy_ ? -x : x; }
    inline double fy_(double y) const { return flip_optical_xy_ ? -y : y; }

    void publish_detections(const std::vector<TrackedDet>& dets, const std_msgs::Header& hdr) {
        sweetie_bot_text_msgs::DetectionArray arr;   // DetectionArray has no header; each Detection carries one
        for (const auto& d : dets) {
            sweetie_bot_text_msgs::Detection m;
            m.header = hdr;
            m.id = d.track_id >= 0 ? d.track_id : 0;
            auto tm = type_map_.find(d.type);
            m.type = (tm != type_map_.end()) ? tm->second : d.type;
            // SOAR's vision rules require a non-None label; synthesize one from type + track id
            m.label = !d.label.empty() ? d.label : (m.type + "_" + std::to_string(m.id));
            m.score = static_cast<float>(d.score);
            for (const auto& kv : d.attrs) { m.attribute.push_back(kv.first); m.value.push_back(kv.second); }
            m.pose.orientation.w = 1.0;
            if (d.has_pos) { m.pose.position.x = fx_(d.pos[0]); m.pose.position.y = fy_(d.pos[1]); m.pose.position.z = d.pos[2]; }
            if (d.has_box) { m.box.x = d.box[0]; m.box.y = d.box[1]; m.box.z = d.box[2]; }   // extent, not a position
            // skeleton keypoints travel typed on the Detection now; the hmi marker node draws them.
            if (!d.kpts3d.empty()) {
                m.keypoint_layout = "coco17";
                for (const auto& k : d.kpts3d) {
                    geometry_msgs::Point p;
                    p.x = fx_(k[0]); p.y = fy_(k[1]); p.z = k[2];
                    m.keypoints_3d.push_back(p);
                }
            }
            arr.detections.push_back(m);
        }
        det_pub_.publish(arr);
    }

    void worker_loop() {
        WsProvider local(prov_host_, prov_port_, prov_target_, "",  // local = synchronous FAST path;
                         provider_io_timeout_);                       // every op wall-clock-bounded
        local.set_critical(provider_critical_);   // real robot: ERROR-grade, never-silent local logs
        std::vector<std::unique_ptr<RemoteWorker>> remotes;          // remotes = async best-effort, off the loop
        for (const auto& rs : remote_specs_) {                       // one async worker per remote container
            std::unique_ptr<ProviderConn> conn;
            // Remotes get a generous bound (heavy nets + WAN): still finite, so a wedged remote
            // worker thread also self-heals instead of blocking forever.
            if (rs.tls)
                conn = std::make_unique<WssProvider>(rs.host, rs.port, rs.target, rs.insecure,
                                                     rs.token, 30.0);
            else
                conn = std::make_unique<WsProvider>(rs.host, rs.port, rs.target, rs.token, 30.0);
            remotes.push_back(std::make_unique<RemoteWorker>(std::move(conn), remote_max_staleness_ms_));
        }
        auto last_local_ok = std::chrono::steady_clock::now();
        while (running_.load() && ros::ok()) {
            Frame f;
            if (!latest_blocking(f)) break;
            FrameHeader h{}; std::memcpy(h.magic, "MJPG", 4); h.version = 1; h.header_size = sizeof(FrameHeader);
            h.frame_id = f.frame_id; h.capture_ts_ns = f.capture_ts_ns; h.width = 0; h.height = 0;
            h.jpeg_size = static_cast<uint32_t>(f.jpeg.size()); h.camera_rotation_deg = rot_deg_; h.flags = 0;
            std::vector<uint8_t> msg(sizeof(FrameHeader) + f.jpeg.size());
            std::memcpy(msg.data(), &h, sizeof(FrameHeader));
            std::memcpy(msg.data() + sizeof(FrameHeader), f.jpeg.data(), f.jpeg.size());
            // LOCAL: synchronous (the fast critical path). REMOTES: hand off the latest frame to their
            // worker threads and attach whatever fresh result they already have — NEVER blocking here.
            std::vector<std::vector<uint8_t>> bodies;
            std::vector<uint8_t> lb;
            bool local_ok = local.query(msg, lb);
            if (local_ok) {
                bodies.push_back(std::move(lb));
                last_local_ok = std::chrono::steady_clock::now();
            }
            local_down_secs_ = local_ok ? 0.0 : std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_local_ok).count();
            // liveness watchdog: no successful LOCAL reply for provider_watchdog_ seconds while
            // frames flow -> force a full teardown (covers any residual wedged-but-open mode) and
            // escalate to ERROR so the outage can't be missed.
            if (!local_ok && local_down_secs_ > provider_watchdog_) {
                ROS_ERROR_STREAM_THROTTLE(10.0, "local provider " << local.name()
                    << ": no successful reply for " << static_cast<int>(local_down_secs_)
                    << "s while frames flow; forcing reconnect");
                local.force_reset();
            }
            for (auto& r : remotes) {
                r->submit(msg);
                std::vector<uint8_t> rb;
                if (r->latest(rb)) bodies.push_back(std::move(rb));
            }
            if (bodies.empty()) {
                // No provider answered. Keep the operator-facing stream alive AND SAYING SO —
                // during the P21 incident the demo stream looked healthy while perception was
                // blind. The HUD renders the dead-link state onto the raw camera frame.
                cv::Mat raw = cv::imdecode(cv::Mat(1, static_cast<int>(f.jpeg.size()), CV_8UC1,
                                                   f.jpeg.data()), cv::IMREAD_COLOR);
                if (!raw.empty()) {
                    std_msgs::Header hdr;
                    hdr.stamp = ros::Time().fromNSec(f.capture_ts_ns);
                    hdr.frame_id = image_frame_id_;
                    // clean copy leaves before draw() mutates raw in place (toImageMsg copies)
                    if (clean_pub_.getNumSubscribers() > 0)
                        clean_pub_.publish(cv_bridge::CvImage(hdr, "bgr8", raw).toImageMsg());
                    draw(raw, std::vector<TrackedDet>(), f.capture_ts_ns);
                    image_pub_.publish(cv_bridge::CvImage(hdr, "bgr8", raw).toImageMsg());
                }
                ros::Duration(0.05).sleep();
                continue;
            }
            std::string reply;
            if (!fuser_roundtrip(f, bodies, reply)) {
                ROS_WARN_STREAM_THROTTLE(2.0, "fuser roundtrip failed (frame " << f.frame_id << ")");
                continue;
            }
            std_msgs::String rj; rj.data = reply; result_pub_.publish(rj);
            std::vector<TrackedDet> dets;
            parse_reply(reply, dets);
            std_msgs::Header hdr; hdr.stamp = ros::Time().fromNSec(f.capture_ts_ns); hdr.frame_id = image_frame_id_;
            cv::Mat raw_img = cv::imdecode(cv::Mat(1, static_cast<int>(f.jpeg.size()), CV_8UC1, f.jpeg.data()),
                                           cv::IMREAD_COLOR);
            if (!raw_img.empty()) {
                if (clean_pub_.getNumSubscribers() > 0)
                    clean_pub_.publish(cv_bridge::CvImage(hdr, "bgr8", raw_img).toImageMsg());
                cv::Mat img = raw_img.clone();
                draw(img, dets, f.capture_ts_ns);
                image_pub_.publish(cv_bridge::CvImage(hdr, "bgr8", img).toImageMsg());
            }
            publish_detections(dets, hdr);
        }
        for (auto& r : remotes) r->stop();
    }

    ros::NodeHandle nh_, pnh_;
    ros::Publisher result_pub_, image_pub_, clean_pub_, det_pub_;
    struct RemoteSpec { std::string host, target, token; int port = 443; bool tls = true; bool insecure = false; };
    std::vector<RemoteSpec> remote_specs_;
    std::string pipeline_, prov_host_, prov_target_, remote_host_, remote_target_, remote_token_,
                fuser_host_, image_topic_, clean_image_topic_, image_frame_id_, det_topic_;
    std::map<std::string, std::string> type_map_;
    int prov_port_ = 8080, remote_port_ = 8443, fuser_port_ = 9100, ring_size_ = 60, rot_deg_ = 0;
    bool remote_insecure_ = true;
    bool flip_optical_xy_ = true;
    double hud_fps_ = 0.0;
    uint64_t last_draw_ns_ = 0;
    std::string hud_text_;
    uint64_t hud_last_update_ns_ = 0;
    cv::Ptr<cv::freetype::FreeType2> ft2_;
    bool ft_ready_ = false;
    double remote_max_staleness_ms_ = 1500.0;
    double provider_io_timeout_ = 2.0;   // per-op wall-clock bound on the local provider link (s)
    double provider_watchdog_ = 10.0;    // no-local-success escalation threshold (s)
    bool provider_critical_ = false;     // local link fatal (run-real): ERROR + never-silent logs
    double local_down_secs_ = 0.0;       // seconds since last successful local reply (worker thread)
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
