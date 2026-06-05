# transport_cpp_client

Native (ROS-agnostic) C++ transport for the Sweetie Bot vision federation. This is the laptop-side
("brain") counterpart of the vision repo's Python transport server: it speaks the same wire framing
(`FrameHeader`+JPEG uplink / `VRES`+msgpack downlink — see `perfusion/transport/protocol.py` in the
vision repo), over `ws` (loopback) or `wss` (network), with `TCP_NODELAY` and 1-frame backpressure.

Moved here in the 2026-06 consolidation: native C++ transport belongs in the main `sweetie_bot` repo
(this `lib/`), not in the vision repo. It is a **standalone CMake project**, not a catkin package
(no `package.xml`, so `catkin_make` skips it).

## Build

```bash
cmake -S . -B build && cmake --build build -j
```

Deps: Boost.Beast + OpenSSL + GStreamer (no OpenCV / jsoncpp / ROS).

## Contents

- **`transport_latency_probe`** — a transport **benchmark probe**. It exercises the production transport
  decisions (wss, exact uplink framing, backpressure) and measures true end-to-end round-trip latency
  against the real server. It is **schema-blind on the downlink**: it reads only the `VRES` header
  (`frame_id`) and treats the msgpack `PerceptionFrame` body as opaque — so it does **not** decode
  detections, hand off to a fusion sink, or reconnect/run-forever.

  ```bash
  ./build/transport_latency_probe --server-url wss://127.0.0.1:8443/ws --insecure \
      --gst-pipeline '... ! jpegenc ! appsink name=sink'
  ```

## TODO — production native client

The real native client is **future work** and belongs in this package: decode the `PerceptionFrame`
(msgpack) on the downlink and hand it to the native `FusionDriver` (the one tracker), with reconnect
and run-forever semantics. It must match the wire framing in `perfusion/transport/protocol.py`. Until
it exists, the native fusion core uses the Python `ProviderClient` / `TransportClient`.
