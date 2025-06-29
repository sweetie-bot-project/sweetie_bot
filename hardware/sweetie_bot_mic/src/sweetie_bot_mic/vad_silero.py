import rospy
import socket
import struct
import numpy as np
from .vad import VoiceActivityDetector,VoiceActivity

class VoiceActivityDetectorSilero_vad(VoiceActivityDetector):
    ''' By silero_vad ROS topic message as speech indicator. '''
    _detector_type = 'silero_vad'
 
    def __init__(self, **kwargs):
        rospy.loginfo(f"VAD type: silero_vad.")
        rospy.loginfo(kwargs)
        self.HOST = kwargs.get('receive_ip', '127.0.0.1') # Server address
        self.PORT = kwargs.get('receive_port', 1234)   # Port for TCP connection
        self.threshold_voiced = kwargs.get('treshhold_voiced', 0.6)
        self.threshold_unvoiced = kwargs.get('treshhold_unvoiced', 0.4)
        self.sock = None # Global variable for socket
        self.is_connected = False 
        self.create_socket()
              
    def create_socket(self):
        """Creating a new socket"""
        try:
            if self.sock is not None:
                self.sock.close()
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(10)  # Timeout on socket operation
            return True
        except Exception as e:
            rospy.logerr(f"Error creating socket: {e}")
            return False
   
    def connect_to_server(self):
        """Connecting to the server with repeated attempts"""
        rospy.loginfo("Connecting to Silero")
        try:
            while True:
                if not self.create_socket():
                    continue
                else:break

            rospy.loginfo(f"Trying to connect to Silero server {self.HOST}:{self.PORT}")
            self.sock.connect((self.HOST, self.PORT))
            self.is_connected = True
            rospy.loginfo("Success connection to Silero")
            return True
        except Exception as e:
            rospy.logerr(f"Error connecting to Silero: {e}")
            self.is_connected = False

    def ensure_connection(self):
        """Fix connection to Silero"""
        if not self.is_connected:
            return self.connect_to_server()
        return True

    def send_data(self, data):
        """Sending audio_data to Silero"""
        try:
            if not self.ensure_connection():
                return False
            self.sock.send(data)
            return True
        except Exception as e:
            rospy.logerr(f"Error sending audio_data to Silero: {e}")
            self.is_connected = False
            return False

    def receive_confidence(self):
        """reciving confidence metric from silero"""
        try:
            while True:
                if not self.ensure_connection():
                    continue
                else:break

            # We get 4 bytes (size float32)
            data = self.sock.recv(4)
            if not data:
                rospy.logwarn("Empty data Silero")
                self.is_connected = False

            confidence = struct.unpack('f', data)[0]
            return confidence
        except socket.timeout:
            rospy.logerr("Socket timeout error ")
            # continue
            pass
        except Exception as e:
            rospy.logerr(f"Error while reciving confidence metric from Silero: {e}")
            self.is_connected = False
            
    def update(self, audio_data:np.ndarray):
        # Using only last frame
        audio_data_buf = audio_data.tobytes()
        length = len(audio_data_buf)
        if length <= 1024:
            rospy.logwarn("Silero: audio_data array length is not 4096 ")
            return VoiceActivity.UNVOICED, 0
        if length != 0 and length % 1024 == 0:
            if not self.ensure_connection():
                rospy.logwarn("Silero: connection is not available, skipping audio fragment")
                return VoiceActivity.UNVOICED, 0
            if not self.send_data(audio_data_buf):
                rospy.logwarn("Silero: Failed to send audio_data to Silero")
                return VoiceActivity.UNVOICED, 0
            cycles = int(length / 1024)
            confidence = []
            for part in range(cycles):
                conf = self.receive_confidence()
                if conf is not None:
                    confidence.append(conf)
            if confidence:
                mx = max(confidence)
                if mx > self.threshold_voiced:
                    rospy.logdebug(f"Silero: Voice detected, confidence: {mx:.6f}")
                    return VoiceActivity.VOICED, mx
                else:
                    rospy.logdebug(f"Silero: No voice, confidence: {mx:.6f}")
                    return VoiceActivity.UNVOICED, mx
            else:
                rospy.logwarn("Silero: No confidence values received from Silero")
                return VoiceActivity.UNVOICED, 0
        else:
            rospy.logwarn("Silero: audio_data array length is not 4096 ")
            return VoiceActivity.UNVOICED, 0  

    def _on_speech_indicator(self, msg):
        self._speech_indicator = msg.data