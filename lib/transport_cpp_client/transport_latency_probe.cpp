// transport_latency_probe.cpp
//
// ROS-agnostic C++ transport BENCHMARK PROBE for the Sweetie Bot vision system. It exercises the
// production transport decisions -- encrypted wss, the exact binary FrameHeader+JPEG uplink, and
// 1-frame-in-flight backpressure -- and measures true end-to-end round-trip latency against the real
// server. It is NOT the production native client: the downlink is schema-blind (it reads only the VRES
// result header / frame_id and treats the msgpack PerceptionFrame body as opaque), so it does not
// decode detections, has no fusion/sink handoff, and does not reconnect/run-forever.
//
// The production native client (msgpack decode -> native FusionDriver handoff, reconnect, run-forever)
// is FUTURE WORK and should live alongside this in lib/transport_cpp_client (see README.md). The wire
// framing it must speak is perfusion/transport/protocol.py in the vision repo (FrameHeader / VRES).
//
// Build deps: Boost.Beast + OpenSSL + GStreamer (no OpenCV/jsoncpp). Frame source: a GStreamer
// pipeline producing image/jpeg (e.g. streaming an mp4), or a JPEG file/dir.

#include <boost/asio.hpp>
#include <boost/asio/ssl.hpp>
#include <boost/beast.hpp>
#include <boost/beast/ssl.hpp>
#include <boost/beast/websocket.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>
#include <sys/stat.h>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
namespace ssl = boost::asio::ssl;
using tcp = net::ip::tcp;

#pragma pack(push, 1)
struct FrameHeader {
    char magic[4]; uint16_t version; uint16_t header_size;
    uint64_t frame_id; uint64_t capture_ts_ns;
    uint32_t width; uint32_t height; uint32_t jpeg_size;
    int32_t camera_rotation_deg; uint32_t flags;
};
struct ResultHeader {
    char magic[4]; uint16_t version; uint16_t header_size;
    uint64_t frame_id; uint32_t payload_len; uint32_t encoding; uint32_t flags;
};
#pragma pack(pop)
static_assert(sizeof(FrameHeader) == 44, "FrameHeader must be 44 bytes");
static_assert(sizeof(ResultHeader) == 28, "ResultHeader must be 28 bytes");

static std::atomic<bool> g_running{true};
static void on_signal(int) { g_running.store(false); }

static uint64_t now_ns() {
    using namespace std::chrono;
    return (uint64_t)duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
}

struct Frame {
    uint64_t frame_id = 0, capture_ts_ns = 0;
    uint32_t width = 0, height = 0;
    std::vector<uint8_t> jpeg;
};

struct Config {
    std::string server_url = "wss://127.0.0.1:8443/ws";
    std::string api_key, gst_pipeline, source;
    int width = 800, height = 600, rotation = 0;
    double max_send_fps = 0.0;        // 0 = as fast as backpressure allows
    uint64_t max_frames = 120, warmup = 10;
    bool insecure = false, show_help = false;
};

struct ParsedUrl { std::string scheme, host, port, target; };

static ParsedUrl parse_url(const std::string& url) {
    ParsedUrl o;
    auto p = url.find("://");
    if (p == std::string::npos) throw std::runtime_error("bad url");
    o.scheme = url.substr(0, p);
    std::string rest = url.substr(p + 3);
    auto s = rest.find('/');
    std::string hp = s == std::string::npos ? rest : rest.substr(0, s);
    o.target = s == std::string::npos ? "/" : rest.substr(s);
    auto c = hp.rfind(':');
    if (c != std::string::npos) { o.host = hp.substr(0, c); o.port = hp.substr(c + 1); }
    else { o.host = hp; o.port = (o.scheme == "wss") ? "443" : "80"; }
    if (o.scheme != "ws" && o.scheme != "wss") throw std::runtime_error("scheme must be ws/wss");
    return o;
}

static std::vector<uint8_t> build_frame_message(const Frame& f, int32_t rot) {
    FrameHeader h{};
    std::memcpy(h.magic, "MJPG", 4);
    h.version = 1; h.header_size = sizeof(FrameHeader);
    h.frame_id = f.frame_id; h.capture_ts_ns = f.capture_ts_ns;
    h.width = f.width; h.height = f.height;
    h.jpeg_size = (uint32_t)f.jpeg.size(); h.camera_rotation_deg = rot; h.flags = 0;
    std::vector<uint8_t> m(sizeof(FrameHeader) + f.jpeg.size());
    std::memcpy(m.data(), &h, sizeof(FrameHeader));
    std::memcpy(m.data() + sizeof(FrameHeader), f.jpeg.data(), f.jpeg.size());
    return m;
}

// ---- frame sources --------------------------------------------------------------------
struct IFrameSource { virtual ~IFrameSource() = default; virtual bool next(Frame&) = 0; };

static bool has_image_ext(const std::string& n) {
    auto p = n.rfind('.'); if (p == std::string::npos) return false;
    std::string e = n.substr(p + 1); std::transform(e.begin(), e.end(), e.begin(), ::tolower);
    return e == "jpg" || e == "jpeg" || e == "png";
}

class FileFrameSource : public IFrameSource {
public:
    FileFrameSource(const std::string& path, uint32_t w, uint32_t h) : w_(w), h_(h) {
        struct stat st{};
        if (::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
            if (DIR* d = ::opendir(path.c_str())) {
                while (dirent* e = ::readdir(d)) { std::string n = e->d_name; if (has_image_ext(n)) files_.push_back(path + "/" + n); }
                ::closedir(d);
            }
            std::sort(files_.begin(), files_.end());
        } else files_.push_back(path);
        if (files_.empty()) throw std::runtime_error("no images at " + path);
    }
    bool next(Frame& o) override {
        const std::string& p = files_[idx_++ % files_.size()];
        std::ifstream f(p, std::ios::binary);
        std::vector<uint8_t> b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        if (b.empty()) return false;
        o.frame_id = ++c_; o.capture_ts_ns = now_ns(); o.width = w_; o.height = h_; o.jpeg = std::move(b);
        return true;
    }
private:
    std::vector<std::string> files_; size_t idx_ = 0; uint64_t c_ = 0; uint32_t w_, h_;
};

class GstFrameSource : public IFrameSource {
public:
    GstFrameSource(const std::string& desc, uint32_t w, uint32_t h) : w_(w), h_(h) {
        gst_init(nullptr, nullptr);
        GError* err = nullptr;
        pipeline_ = gst_parse_launch(desc.c_str(), &err);
        if (!pipeline_) { std::string m = err ? err->message : "?"; if (err) g_error_free(err); throw std::runtime_error("gst: " + m); }
        if (err) g_error_free(err);
        GstElement* s = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
        if (!s) { gst_object_unref(pipeline_); throw std::runtime_error("pipeline needs appsink name=sink"); }
        appsink_ = GST_APP_SINK(s);
        gst_app_sink_set_drop(appsink_, FALSE);  // do not drop: we want every frame for a clean count
        gst_app_sink_set_max_buffers(appsink_, 4);
        if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
            throw std::runtime_error("failed to PLAY pipeline");
    }
    ~GstFrameSource() override {
        if (pipeline_) { gst_element_set_state(pipeline_, GST_STATE_NULL); if (appsink_) gst_object_unref(appsink_); gst_object_unref(pipeline_); }
    }
    bool next(Frame& o) override {
        GstSample* sample = nullptr;
        while (g_running.load()) {
            sample = gst_app_sink_try_pull_sample(appsink_, 200 * GST_MSECOND);
            if (sample) break;
            if (gst_app_sink_is_eos(appsink_)) return false;
        }
        if (!sample) return false;
        GstBuffer* buf = gst_sample_get_buffer(sample);
        GstMapInfo map;
        if (!buf || !gst_buffer_map(buf, &map, GST_MAP_READ)) { gst_sample_unref(sample); return false; }
        uint32_t w = w_, h = h_;
        if (GstCaps* caps = gst_sample_get_caps(sample)) {
            GstStructure* st = gst_caps_get_structure(caps, 0);
            int cw = 0, ch = 0;
            if (gst_structure_get_int(st, "width", &cw) && cw > 0) w = (uint32_t)cw;
            if (gst_structure_get_int(st, "height", &ch) && ch > 0) h = (uint32_t)ch;
        }
        o.frame_id = ++c_; o.capture_ts_ns = now_ns(); o.width = w; o.height = h;
        o.jpeg.assign(map.data, map.data + map.size);
        gst_buffer_unmap(buf, &map); gst_sample_unref(sample);
        return true;
    }
private:
    GstElement* pipeline_ = nullptr; GstAppSink* appsink_ = nullptr; uint64_t c_ = 0; uint32_t w_, h_;
};

// ---- latency stats --------------------------------------------------------------------
struct Stats {
    std::vector<double> rtt_ms;        // send -> result
    std::vector<double> cap_ms;        // capture -> result
    std::vector<size_t> payloads;
    double first_frame_ms = 0.0;       // includes server warmup (model load)
    double wall_s = 0.0;

    static double pct(std::vector<double> v, double p) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        double idx = p / 100.0 * (v.size() - 1);
        size_t lo = (size_t)std::floor(idx), hi = (size_t)std::ceil(idx);
        double frac = idx - lo;
        return v[lo] * (1 - frac) + v[hi] * frac;
    }
    static double mean(const std::vector<double>& v) {
        if (v.empty()) return 0.0; double s = 0; for (double x : v) s += x; return s / v.size();
    }
    void print() const {
        size_t n = rtt_ms.size();
        double mean_payload = 0; for (size_t p : payloads) mean_payload += p; if (!payloads.empty()) mean_payload /= payloads.size();
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "\n==== round-trip latency (after warmup discard) ====\n";
        std::cout << "frames measured : " << n << "\n";
        std::cout << "first frame (warmup, incl. model load): " << first_frame_ms << " ms\n";
        if (n) {
            std::cout << "send->result ms : min=" << pct(rtt_ms,0) << " p50=" << pct(rtt_ms,50)
                      << " mean=" << mean(rtt_ms) << " p90=" << pct(rtt_ms,90)
                      << " p95=" << pct(rtt_ms,95) << " p99=" << pct(rtt_ms,99)
                      << " max=" << pct(rtt_ms,100) << "\n";
            std::cout << "capture->result : p50=" << pct(cap_ms,50) << " mean=" << mean(cap_ms)
                      << " p95=" << pct(cap_ms,95) << "\n";
            std::cout << "effective fps   : " << (wall_s > 0 ? n / wall_s : 0) << "  (backpressure-governed)\n";
            std::cout << "result payload  : mean=" << mean_payload << " bytes (opaque msgpack PerceptionFrame)\n";
        }
    }
};

// ---- session --------------------------------------------------------------------------
template <class WS>
static void run_session(WS& ws, const Config& cfg, IFrameSource& src, Stats& stats) {
    const uint64_t period_ns = cfg.max_send_fps > 0 ? (uint64_t)(1e9 / cfg.max_send_fps) : 0;
    uint64_t last_send = 0, sent = 0;
    uint64_t t0 = now_ns();
    while (g_running.load()) {
        if (cfg.max_frames > 0 && sent >= cfg.max_frames) break;
        Frame f;
        if (!src.next(f)) break;
        if (period_ns && last_send) {
            uint64_t now = now_ns();
            if (now - last_send < period_ns) std::this_thread::sleep_for(std::chrono::nanoseconds(period_ns - (now - last_send)));
        }
        auto msg = build_frame_message(f, cfg.rotation);
        uint64_t t_send = now_ns();
        ws.binary(true);
        ws.write(net::buffer(msg));
        last_send = now_ns();

        beast::flat_buffer buf;
        ws.read(buf);                 // backpressure: wait for the result
        uint64_t t_recv = now_ns();
        std::string data = beast::buffers_to_string(buf.data());

        double rtt = (t_recv - t_send) / 1e6;
        double cap = (t_recv - f.capture_ts_ns) / 1e6;
        size_t payload = data.size();
        if (data.size() >= sizeof(ResultHeader) && std::memcmp(data.data(), "VRES", 4) == 0) {
            ResultHeader rh{}; std::memcpy(&rh, data.data(), sizeof(ResultHeader));
            payload = rh.payload_len;
        }
        ++sent;
        if (sent == 1) stats.first_frame_ms = rtt;
        if (sent > cfg.warmup) { stats.rtt_ms.push_back(rtt); stats.cap_ms.push_back(cap); stats.payloads.push_back(payload); }
        std::cout << "{\"frame\":" << sent << ",\"rtt_ms\":" << std::fixed << std::setprecision(2) << rtt
                  << ",\"payload\":" << payload << "}\n" << std::flush;
    }
    // measure wall time over the post-warmup window
    stats.wall_s = (now_ns() - t0) / 1e9;
    beast::error_code ec;
    ws.close(websocket::close_code::normal, ec);
}

static void decorate(const Config& cfg, websocket::request_type& req) {
    req.set(beast::http::field::user_agent, "sb_transport_latency_probe/1.0");
    if (!cfg.api_key.empty()) req.set(beast::http::field::authorization, "Bearer " + cfg.api_key);
}

static void run_plain(const Config& cfg, const ParsedUrl& u, IFrameSource& src, Stats& st) {
    net::io_context ioc; tcp::resolver res(ioc);
    websocket::stream<tcp::socket> ws(ioc);
    auto r = res.resolve(u.host, u.port);
    net::connect(ws.next_layer(), r.begin(), r.end());
    beast::get_lowest_layer(ws).set_option(tcp::no_delay(true));  // no Nagle/delayed-ACK stalls
    ws.set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));
    ws.set_option(websocket::stream_base::decorator([&](websocket::request_type& q){ decorate(cfg, q); }));
    ws.handshake(u.host, u.target);
    std::cerr << "connected (ws) to " << cfg.server_url << "\n";
    run_session(ws, cfg, src, st);
}

static void run_ssl(const Config& cfg, const ParsedUrl& u, IFrameSource& src, Stats& st) {
    net::io_context ioc;
    ssl::context ctx(ssl::context::tlsv12_client);
    ctx.set_default_verify_paths();
    ctx.set_verify_mode(cfg.insecure ? ssl::verify_none : ssl::verify_peer);
    tcp::resolver res(ioc);
    websocket::stream<beast::ssl_stream<tcp::socket>> ws(ioc, ctx);
    if (!SSL_set_tlsext_host_name(ws.next_layer().native_handle(), u.host.c_str())) {
        beast::error_code ec{(int)::ERR_get_error(), net::error::get_ssl_category()};
        throw beast::system_error{ec};
    }
    auto r = res.resolve(u.host, u.port);
    net::connect(beast::get_lowest_layer(ws), r.begin(), r.end());
    beast::get_lowest_layer(ws).set_option(tcp::no_delay(true));  // no Nagle/delayed-ACK stalls
    ws.next_layer().handshake(ssl::stream_base::client);
    ws.set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));
    ws.set_option(websocket::stream_base::decorator([&](websocket::request_type& q){ decorate(cfg, q); }));
    ws.handshake(u.host, u.target);
    std::cerr << "connected (wss, encrypted) to " << cfg.server_url << "\n";
    run_session(ws, cfg, src, st);
}

static void usage() {
    std::cerr <<
        "sb_transport_latency_probe -- ROS-agnostic encrypted (wss) transport benchmark probe\n\n"
        "Usage: transport_latency_probe (--gst-pipeline <desc> | --source <path>) [options]\n"
        "  --gst-pipeline <d>   GStreamer pipeline producing image/jpeg, ending 'appsink name=sink'\n"
        "  --source <path>      JPEG file or directory (looped)\n"
        "  --server-url <url>   wss://host:port/path (default wss://127.0.0.1:8443/ws) or ws://...\n"
        "  --insecure           skip TLS cert verification (self-signed server)\n"
        "  --api-key <k>        Bearer token\n"
        "  --max-frames <n>     frames to send (default 120)\n"
        "  --warmup <n>         discard first n frames from stats (default 10)\n"
        "  --max-fps <f>        cap send rate (default 0 = backpressure-bound)\n"
        "  --width/--height     frame metadata fallback (default 800x600)\n"
        "  --rotation <deg>     0|90|180|270 (default 0)\n";
}

static Config parse_args(int argc, char** argv) {
    Config c;
    auto need = [&](int& i) { if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + argv[i]); return std::string(argv[++i]); };
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--gst-pipeline") c.gst_pipeline = need(i);
        else if (a == "--source") c.source = need(i);
        else if (a == "--server-url") c.server_url = need(i);
        else if (a == "--api-key") c.api_key = need(i);
        else if (a == "--max-frames") c.max_frames = std::stoull(need(i));
        else if (a == "--warmup") c.warmup = std::stoull(need(i));
        else if (a == "--max-fps") c.max_send_fps = std::stod(need(i));
        else if (a == "--width") c.width = std::stoi(need(i));
        else if (a == "--height") c.height = std::stoi(need(i));
        else if (a == "--rotation") c.rotation = std::stoi(need(i));
        else if (a == "--insecure") c.insecure = true;
        else if (a == "-h" || a == "--help") c.show_help = true;
        else throw std::runtime_error("unknown arg: " + a);
    }
    return c;
}

int main(int argc, char** argv) {
    Config cfg;
    try { cfg = parse_args(argc, argv); }
    catch (const std::exception& e) { std::cerr << "arg error: " << e.what() << "\n\n"; usage(); return 2; }
    if (cfg.show_help) { usage(); return 0; }
    if (cfg.gst_pipeline.empty() && cfg.source.empty()) { std::cerr << "need --gst-pipeline or --source\n\n"; usage(); return 2; }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::unique_ptr<IFrameSource> src;
    ParsedUrl url;
    try {
        if (!cfg.gst_pipeline.empty()) { src = std::make_unique<GstFrameSource>(cfg.gst_pipeline, cfg.width, cfg.height); std::cerr << "source: GStreamer\n"; }
        else { src = std::make_unique<FileFrameSource>(cfg.source, cfg.width, cfg.height); std::cerr << "source: file\n"; }
        url = parse_url(cfg.server_url);
    } catch (const std::exception& e) { std::cerr << "setup error: " << e.what() << "\n"; return 1; }

    Stats stats;
    try {
        if (url.scheme == "wss") run_ssl(cfg, url, *src, stats);
        else run_plain(cfg, url, *src, stats);
    } catch (const std::exception& e) { std::cerr << "session error: " << e.what() << "\n"; }
    stats.print();
    return 0;
}
