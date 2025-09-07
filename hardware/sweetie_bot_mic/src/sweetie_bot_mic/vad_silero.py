import rospy
import socket
import struct
import numpy as np
from .vad import VoiceActivityDetector,VoiceActivity

class VoiceActivityDetectorSilero_vad(VoiceActivityDetector):
    ''' By silero_vad ROS topic message as speech indicator. '''
    _detector_type = 'silero_vad'
    DATA_CHUNK_SIZE = 1024
 
    def __init__(self, **kwargs):
        rospy.loginfo(f"VAD type: silero_vad.")
        rospy.loginfo(kwargs)
        self.server_address = kwargs.get('receive_ip', '127.0.0.1') # Server address
        self.server_port = kwargs.get('receive_port', 1234)   # Port for TCP connection
        self.threshold_voiced = kwargs.get('treshhold_voiced', 0.6)
        self.timeout = 1.0
        self.sock = None # Global variable for socket
        self.is_connected = False 
        self.connect_to_server()
              
    def connect_to_server(self):
        """Connecting to the server with repeated attempts"""
        rospy.loginfo("Connecting to Silero")
        try:
            # close old socket if any present
            if self.sock is not None:
                self.sock.close()
            # create socket
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)  # Timeout on socket operation
            # connect to sever
            rospy.loginfo(f"Trying to connect to Silero server {self.server_address}:{self.server_port}")
            self.sock.connect((self.server_address, self.server_port))
            rospy.loginfo("Success connection to Silero")
            return True
        except Exception as e:
            rospy.logerr(f"Error connecting to Silero: {e}")
            self.disconnect_from_server()
            return False
        
    def disconnect_from_server(self):
        if self.sock is not None:
            self.sock.close()
        self.sock = None
        
    def is_server_connected(self):
        return self.sock is not None
            
    def update(self, audio_data : np.ndarray):
        # convert arrya to raw data
        audio_data_buf = audio_data.tobytes()
        length = len(audio_data_buf)
        # check size
        if length == 0 or length % self.DATA_CHUNK_SIZE != 0:
            rospy.logwarn("Silero: audio_data array length is divided by 1024 or empty")
            return VoiceActivity.UNVOICED, 0.0
        # check if server is connected
        if not self.is_server_connected():
            success = self.connect_to_server()
            if not success:
                # failed to connect
                return VoiceActivity.UNVOICED, 0.0
        try:
            # send data to server
            self.sock.send(audio_data_buf)
            # receive confidence
            confidence_data = self.sock.recv(4*(length//self.DATA_CHUNK_SIZE))
            if len(confidence_data) == 0:
                raise ConnectionAbortedError()
            # decode confidence
            confidence = np.frombuffer(confidence_data, np.float32)
            mx = confidence.max()
            if mx > self.threshold_voiced:
                rospy.logdebug(f"Silero: Voice detected, confidence: {mx:.6f}")
                return VoiceActivity.VOICED, mx
            else:
                rospy.logdebug(f"Silero: No voice, confidence: {mx:.6f}")
                return VoiceActivity.UNVOICED, mx
        except (ConnectionError, TimeoutError) as e:
            self.disconnect_from_server()
            rospy.logwarn("Silero: error during echange with server.")
            return VoiceActivity.UNVOICED, 0.0
        
