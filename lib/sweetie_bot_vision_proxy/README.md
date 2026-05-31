# vision_proxy

`vision_proxy` is a ROS1 node for forwarding MJPEG camera frames from a robot-side video stream to a remote AI server and receiving inference results back as JSON.

The current prototype does not use WebRTC. Instead, the robot sends an RTP/JPEG stream over UDP to the control laptop. The laptop receives JPEG frames with GStreamer, forwards selected frames to the AI server over WebSocket Secure (`wss://`), receives JSON results back, and publishes both the raw result JSON and an RViz-friendly image with overlayed detections.

## Purpose

This prototype is intended for a setup where:

* the robot camera outputs MJPEG;
* the robot should not decode or transcode video;
* the control laptop should avoid heavy AI processing;
* the remote server performs or emulates AI inference;
* results are returned to the laptop as JSON;
* the laptop publishes an image with overlayed detections for RViz.

## Architecture

```text
                    Local robot network                         Internet / VPN / WAN
┌──────────────────────────────────────────┐                 ┌──────────────────────────────┐
│                  Robot                   │                 │          AI Server           │
│                                          │                 │                              │
│  /dev/video0                             │                 │  FastAPI / WebSocket server  │
│      │                                   │                 │                              │
│      │ MJPEG from camera                 │                 │  receives JPEG frame         │
│      ▼                                   │                 │  rotates image if requested  │
│  GStreamer RTP/JPEG sender               │                 │  runs / emulates inference   │
│                                          │                 │  returns JSON result         │
│  v4l2src                                 │                 │                              │
│    ! image/jpeg                          │                 └──────────────▲───────────────┘
│    ! jpegparse                           │                                │
│    ! rtpjpegpay                          │                                │
│    ! udpsink                             │                                │
└────────────────┬─────────────────────────┘                                │
                 │                                                          │
                 │ RTP/JPEG over UDP                                        │
                 │ robot -> laptop                                          │
                 │                                                          │
                 ▼                                                          │
┌──────────────────────────────────────────┐                                │
│             Control Laptop               │                                │
│                                          │                                │
│  ROS1 vision_proxy_node                  │                                │
│                                          │                                │
│  GStreamer RTP/JPEG receiver             │                                │
│                                          │                                │
│  udpsrc                                  │                                │
│    ! rtpjitterbuffer                     │                                │
│    ! rtpjpegdepay                        │                                │
│    ! jpegparse                           │                                │
│    ! appsink                             │                                │
│      │                                   │                                │
│      │ JPEG buffers                      │                                │
│      ▼                                   │                                │
│  frame_id + local ring buffer            │                                │
│      │                                   │                                │
│      │ Binary WebSocket frame message    │                                │
│      │ JPEG + frame_id + timestamps      │                                │
│      │ width/height + camera_rotation_deg│                                │
│      └────────────────────────── WSS request channel ─────────────────────▶
│                                          │                                │
│      ◀───────────────────────── WSS JSON result channel ──────────────────┘
│      │                                   │
│      │ JSON result:                      │
│      │   frame_id                        │
│      │   objects[]                       │
│      │   bbox[]                          │
│      │   confidence                      │
│      │                                   │
│      ▼                                   │
│  Parse JSON result                       │
│  Find original frame by frame_id         │
│  Decode JPEG locally for visualization   │
│  Rotate image if configured              │
│  Transform bbox coordinates if needed    │
│  Draw boxes and labels                   │
│      │                                   │
│      ├── /image_raw                      │
│      ├── /vision_proxy/stats_json        │
│      └── /vision_proxy/result_json       │
│                                          │
└──────────────────────────────────────────┘
```

## Data Flow

### 1. Robot to Laptop

The robot sends an RTP/JPEG stream over UDP.

```text
Robot camera
  -> MJPEG frame
  -> RTP/JPEG packets
  -> UDP port on the laptop
```

Example sender pipeline on the robot:

```bash
gst-launch-1.0 -v \
  v4l2src device=/dev/video0 do-timestamp=true io-mode=mmap ! \
  image/jpeg,width=800,height=600,framerate=30/1 ! \
  jpegparse ! \
  rtpjpegpay pt=26 ! \
  udpsink host=laptop port=5000 sync=false async=false buffer-size=1000000
```

Notes:

* `laptop` is resolved to control laptop ip address.
* `5000` is the UDP port listened to by `vision_proxy_node`.
* The robot-side stream remains MJPEG/JPEG.
* The robot does not decode, rotate, or re-encode the image.

### 2. Laptop GStreamer Receiver

`vision_proxy_node` receives RTP/JPEG using a configurable GStreamer pipeline.

Example receiver pipeline:

```bash
udpsrc port=4433 caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=JPEG,payload=26" ! \
rtpjitterbuffer latency=50 drop-on-latency=true ! \
rtpjpegdepay ! \
jpegparse ! \
appsink name=sink emit-signals=true sync=false drop=true max-buffers=1
```

The node consumes JPEG frames from `appsink`.

### 3. Laptop to AI Server

Each selected JPEG frame is wrapped into a binary WebSocket message.

Frame message layout:

```text
+----------------------+----------------------------------------+
| Field                | Description                            |
+----------------------+----------------------------------------+
| magic[4]             | "MJPG"                                 |
| version              | protocol version, currently 2          |
| header_size          | sizeof(FrameHeader)                    |
| frame_id             | monotonically increasing frame id      |
| capture_ts_ns        | ROS capture timestamp in nanoseconds   |
| width                | image width                            |
| height               | image height                           |
| jpeg_size            | size of JPEG payload in bytes          |
| camera_rotation_deg  | requested rotation before AI inference |
| flags                | reserved                               |
| jpeg bytes           | encoded JPEG frame                     |
+----------------------+----------------------------------------+
```

`camera_rotation_deg` tells the AI server how the image should be oriented before inference.

Supported values:

```text
0
90
180
270
```

The WebSocket URL is configured by the `server_url` ROS parameter.

Example:

```text
wss://vision.example.com/ws
```

The API key is passed in the WebSocket HTTP upgrade request:

```http
Authorization: Bearer <api_key>
```

### 4. AI Server to Laptop

The server returns JSON text messages over the same WebSocket connection.

This is the return channel.

Example response:

```json
{
  "type": "result",
  "frame_id": 123,
  "capture_ts_ns": 1710000000000000000,
  "camera_rotation_deg": 180,
  "server_recv_ts_ns": 1710000000010000000,
  "server_send_ts_ns": 1710000000060000000,
  "image": {
    "width": 800,
    "height": 600,
    "jpeg_size": 48123
  },
  "objects": [
    {
      "class": "test_object",
      "confidence": 0.99,
      "bbox": [0.25, 0.25, 0.50, 0.50]
    }
  ]
}
```

Bounding boxes use normalized coordinates:

```text
bbox = [x, y, w, h]
x, y, w, h are in range 0.0 .. 1.0
```

Important convention:

* The AI server should return bounding boxes in the coordinate system of the image it actually processed.
* If the server rotates the image by `camera_rotation_deg` before inference, the returned boxes should match the rotated image.
* The laptop should use the same rotation setting for RViz visualization so that boxes align with the displayed frame.

### 5. ROS Output

The node publishes:

```text
/vision_proxy/result_json   std_msgs/String
/vision_proxy/stats_json    std_msgs/String
/image_raw                  sensor_msgs/Image
```

`/image_raw` contains the JPEG frame decoded locally on the laptop, optionally rotated, with AI detections drawn on top.

## Backpressure Model

The prototype intentionally uses application-level backpressure:

```text
send frame N
wait for result N
take latest available frame
send frame M
wait for result M
...
```

This means:

* only one frame is in flight;
* the client does not build an infinite send queue;
* if the server pipeline is slow, effective send FPS drops automatically;
* old captured frames are skipped;
* the server receives a recent frame after finishing the previous one.

This is useful for diagnosing whether the bottleneck is the remote AI server.

## ROS Parameters

### `~pipeline`

GStreamer receiver pipeline.

It must end with:

```text
appsink name=sink
```

Default example:

```text
udpsrc port=4433 caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=JPEG,payload=26" ! rtpjitterbuffer latency=50 drop-on-latency=true ! rtpjpegdepay ! jpegparse ! appsink name=sink emit-signals=true sync=false drop=true max-buffers=1
```

### `~server_url`

WebSocket server URL.

Example:

```text
wss://vision.example.com/ws
```

### `~api_key`

Bearer token sent during the WebSocket handshake.

### `~width`

Image width used in frame metadata.

Example:

```text
800
```

### `~height`

Image height used in frame metadata.

Example:

```text
600
```

### `~max_send_fps`

Maximum frame send rate to the AI server.

Example:

```text
10.0
```

This is not the camera FPS. The robot may send 30 FPS locally, while the node sends only 10 FPS or less to the AI server.

### `~ring_size`

Number of recent frames stored on the laptop.

Example:

```text
60
```

The ring buffer is used to find the original frame when the server returns a result for a specific `frame_id`.

### `~camera_rotation_deg`

Image orientation metadata sent to the AI server and applied to local visualization.

Supported values:

```text
0
90
180
270
```

Example:

```text
180
```

The original JPEG is sent as-is. The AI server receives `camera_rotation_deg` and may rotate the image before inference. The laptop also uses this value when publishing `/image_raw`.

### `~image_frame_id`

ROS frame id used in `/image_raw`.

Example:

```text
camera
```

### `~insecure_skip_verify`

Disable TLS certificate verification.

Example:

```text
true
```

Use only for local testing with self-signed certificates.

## Build Dependencies

Install required packages:

```bash
sudo apt update
sudo apt install -y \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libssl-dev \
  libboost-system-dev \
  libboost-thread-dev \
  libopencv-dev \
  libjsoncpp-dev \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-image-transport
```

## Run the Test AI Server

```bash
cd server
python3 -m venv venv
. venv/bin/activate
pip install fastapi "uvicorn[standard]"

export VISION_API_KEY=test-key
export VISION_PROCESSING_DELAY_SEC=0.05

python3 server.py
```

Health check:

```bash
curl http://127.0.0.1:8080/health
```

Expected output:

```text
ok
```

## Run the Robot RTP/JPEG Sender

On the robot:

```bash
gst-launch-1.0 -v \
  v4l2src device=/dev/video0 do-timestamp=true io-mode=mmap ! \
  image/jpeg,width=800,height=600,framerate=30/1 ! \
  jpegparse ! \
  rtpjpegpay pt=26 ! \
  udpsink host=laptop port=5000 sync=false async=false buffer-size=1000000
```

## Run `vision_proxy_node`

On the laptop:

```bash
rosrun vision_proxy vision_proxy_node \
  _server_url:=wss://vision.example.com/ws \
  _api_key:=test-key \
  _insecure_skip_verify:=false \
  _width:=800 \
  _height:=600 \
  _max_send_fps:=10.0 \
  _ring_size:=60 \
  _camera_rotation_deg:=180 \
  _image_frame_id:=camera \
  _pipeline:='udpsrc port=5000 caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=JPEG,payload=26" ! rtpjitterbuffer latency=50 drop-on-latency=true ! rtpjpegdepay ! jpegparse ! appsink name=sink emit-signals=true sync=false drop=true max-buffers=1'
```

