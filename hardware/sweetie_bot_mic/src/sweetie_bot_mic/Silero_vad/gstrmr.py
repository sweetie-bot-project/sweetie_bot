import logging
import socket
import struct
import threading
import time

# Log formatting setup
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# Create formatter
formatter = logging.Formatter('%(asctime)s.%(msecs)03d [gstrmr] %(levelname)s: %(message)s', datefmt='%M:%S')

# Create file handler
file_handler = logging.FileHandler('gstrmr.log')
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

# Disable console output
logger.propagate = False

# Socket settings
HOST = '192.168.3.186'  # Server address
HOST = '127.0.0.1'
PORT = 1234             # TCP connection port
RECONNECT_DELAY = 5     # Delay before reconnecting in seconds

# Global variable for socket
sock = None
is_connected = False

def create_socket():
    """Create a new socket"""
    global sock
    try:
        if sock is not None:
            sock.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(10)  # Socket operation timeout
        return True
    except Exception as e:
        logger.error(f"Error creating socket: {e}")
        return False

def connect_to_server():
    """Connect to server with retries"""
    global sock, is_connected
    while True:
        try:
            if not create_socket():
                time.sleep(RECONNECT_DELAY)
                continue

            logger.info(f"Attempting to connect to {HOST}:{PORT}")
            sock.connect((HOST, PORT))
            is_connected = True
            logger.info("Successfully connected to server")
            return True
        except Exception as e:
            logger.error(f"Connection error: {e}")
            is_connected = False
            time.sleep(RECONNECT_DELAY)

def ensure_connection():
    """Check and restore connection if necessary"""
    global is_connected
    if not is_connected:
        return connect_to_server()
    return True

def send_data(data):
    """Send data with error handling"""
    global is_connected
    try:
        if not ensure_connection():
            return False
        sock.send(data)
        return True
    except Exception as e:
        logger.error(f"Error sending data: {e}")
        is_connected = False
        return False

def receive_confidence():
    """Receive confidence value with error handling"""
    global is_connected
    while True:
        try:
            if not ensure_connection():
                time.sleep(RECONNECT_DELAY)
                continue

            # Receive 4 bytes (size of float32)
            data = sock.recv(4)
            if not data:
                logger.warning("Received empty data, reconnecting...")
                is_connected = False
                continue

            # Unpack confidence value
            confidence = struct.unpack('f', data)[0]
            return confidence
        except socket.timeout:
            logger.warning("Timeout while receiving data")
            continue
        except Exception as e:
            logger.error(f"Error receiving confidence: {e}")
            is_connected = False
            time.sleep(RECONNECT_DELAY)

try:
    import gi
    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    from gi.repository import Gst, GstApp, GObject
except ModuleNotFoundError as e:
    logger.error(f"Exception: {e}")
    logger.error(
        "In order to use GStreamer, you need to `pip install pipecat-ai[gstreamer]`. Also, you need to install GStreamer in your system."
    )
    raise Exception(f"Missing module: {e}")

def appsink_audio_new_sample(appsink: GstApp.AppSink):
    buffer = appsink.pull_sample().get_buffer()
    (_, info) = buffer.map(Gst.MapFlags.READ)
    
    try:
        # Send audio data
        if send_data(info.data):
            # Receive confidence value
            confidence = receive_confidence()
            if confidence is not None:
                if confidence > 0.6:
                    logger.info(f"Voice detected, confidence: {confidence:.6f}")
                elif confidence < 0.4:
                    logger.info(f"No voice, confidence: {confidence:.6f}")
                else:
                    logger.info(f"Pause, confidence: {confidence:.6f}")
            
    except Exception as e:
        logger.error(f"Error processing data: {e}")

    buffer.unmap(info)
    return Gst.FlowReturn.OK

# Initialize Gstreamer
Gst.init(None)

# Create pipeline
pipeline = Gst.Pipeline.new("rtp_pipeline")

# Create elements
udpsrc = Gst.ElementFactory.make("udpsrc", "udpsrc")
udpsrc.set_property("port", 5004)
udpsrc.set_property("caps", Gst.caps_from_string("application/x-rtp"))
queue = Gst.ElementFactory.make("queue", "queue")
queue.set_property("max-size-bytes", 1024)
rtppcmudepay = Gst.ElementFactory.make("rtppcmudepay", "rtppcmudepay")
mulawdec = Gst.ElementFactory.make("mulawdec", "mulawdec")
audioconvert = Gst.ElementFactory.make("audioconvert", "audioconvert")
audiobuffersplit = Gst.ElementFactory.make("audiobuffersplit", "audiobuffersplit")
audiobuffersplit.set_property("output-buffer-duration", Gst.Fraction(8, 125))
audiobuffersplit.set_property("strict-buffer-size", True)

appsink_audio = Gst.ElementFactory.make("appsink", None)
appsink_audio.set_property("emit-signals", True)
appsink_audio.set_property("sync", False)
appsink_audio.connect("new-sample", appsink_audio_new_sample)

# Add elements to pipeline
pipeline.add(udpsrc)
pipeline.add(queue)
pipeline.add(rtppcmudepay)
pipeline.add(mulawdec)
pipeline.add(audioconvert)
pipeline.add(audiobuffersplit)
pipeline.add(appsink_audio)

# Link elements
udpsrc.link(queue)
queue.link(rtppcmudepay)
rtppcmudepay.link(mulawdec)
mulawdec.link(audioconvert)
audioconvert.link(audiobuffersplit)
audiobuffersplit.link(appsink_audio)

# Start pipeline
pipeline.set_state(Gst.State.PLAYING)

# Start the GTK main loop
GObject.MainLoop().run()


