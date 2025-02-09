#!/usr/bin/env python3
import struct
import sys
import threading
import numpy as np

import rospy
import tf
import PyKDL
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

from sweetie_bot_mic.cfg import microphoneConfig

from sweetie_bot_text_msgs.msg import SoundEvent, Detection, DetectionArray, TextCommand
from sweetie_bot_text_msgs.srv import Transcribe, TranscribeRequest, TranscribeResponse
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Bool, ColorRGBA
from geometry_msgs.msg import Vector3 
from std_srvs.srv import Trigger, TriggerResponse

from .gstreamer import GstreamerAudioSource
from .doa_estimator import DOAEstimator

class HearingNode:

    def __init__(self):
        rospy.init_node("microphone")
        #
        # Node parameters
        #

        # common parameters
        sample_rate = rospy.get_param("~sample_rate", 16000)
        update_rate = rospy.get_param("~update_rate", 4)
        n_channels = rospy.get_param('~n_channels', 2)
        gstreamer_pipeline = rospy.get_param('~gstreamer_pipeline', 'alsasrc' )
        self._main_channel = rospy.get_param('~main_channel', 0)
        self._mic_frame_id = rospy.get_param("~microphone_frame_id", "microphone_link")
        self._stationary_frame_id = rospy.get_param("~stationary_frame_id", "base_link_path")
        self._enable_translate = rospy.get_param("~enable_translate", True)
        # DOA estimator configuration
        doa_config = rospy.get_param("~doa", {'type': 'none'})
        # detections and marker publication parameters
        self._detections_are_published = rospy.get_param("~detections/publish_detections", True)
        self._markers_are_published = rospy.get_param("~detections/publish_markers", True)
        self._detections_distance = rospy.get_param("~detections/distance", 2.0)
        self._detections_z_position = rospy.get_param("~/detections/2d_z_position", 1.0)
        self._detections_fix_mic_inclination = rospy.get_param("~detections/2d_fix_mic_inclination", True)
        # debug related parameters
        self._debug_plot = rospy.get_param("~debug_plot", True)
        self._intensity_histogram_forget_factor = rospy.get_param("~intensity_histogram_forget_factor", 0.999)
        self._intensity_histogram_step = rospy.get_param("~intensity_histogram_step", 100.0)

        #
        # Direaction of arrival estmation
        #
        self.doa_estimator = DOAEstimator(sample_rate = sample_rate, **doa_config) 

        #
        # Hardware interfaces
        #

        # audio interface
        buffer_size = int(2**np.round(np.log2(sample_rate/update_rate)))
        rospy.loginfo(f'audio: actual update rate {sample_rate/buffer_size} Hz (requested {update_rate} Hz), buffer size {buffer_size} samples, main channel {self._main_channel}.')
        self.gstreamer_audio = GstreamerAudioSource(pipeline=gstreamer_pipeline, sample_rate=sample_rate, buffer_size=buffer_size, n_channels = n_channels,  
                                              on_audio = self.on_audio, on_error = self.on_gstreamer_error, on_warning = None)

        #
        # Node state 
        # 

        self._speech_audio_buffer = bytearray()
        self._external_speech_indicator = False
        self._disable_hearing = False
        self._intesity_histogram = np.ones(shape=(10,))
        self._intesity_histogram_lock = threading.Lock()
        self._s_speech_source_direction = PyKDL.Vector()
        self._speech_source_intensity = 0.0

        #
        # Messages buffers
        #

        # sound event message
        self._sound_event = SoundEvent()
        self._sound_event.header.frame_id = self._mic_frame_id
        self._sound_event.doa_azimuth = self.doa_estimator.grid_azimuth
        self._sound_event.doa_colatitude = self.doa_estimator.grid_colatitude

        # create object detection  message
        self._detection_sound_msg = Detection(header = self._sound_event.header, id = 0, label = 'sound', type = 'sound')
        self._detection_speech_msg = Detection(header = self._sound_event.header, id = 0, label = 'speech', type = 'speech')

        # create visualization marker message (temporary)
        self._marker_sound_msg = Marker(header = self._sound_event.header, ns = 'microphone', 
                                        id = 0, type = Marker.SPHERE, action = Marker.ADD, lifetime = rospy.Duration(1.0), 
                                        scale = Vector3(0.2, 0.2, 0.2), color = ColorRGBA(0.0, 1.0, 0.0, 0.5),
                                        pose = self._detection_sound_msg.pose)
        self._marker_speech_msg = Marker(header = self._sound_event.header, ns = 'microphone', 
                                        id = 1, type = Marker.SPHERE, action = Marker.ADD, lifetime = rospy.Duration(1.0), 
                                        scale = Vector3(0.3, 0.3, 0.3), color = ColorRGBA(1.0, 0.0, 0.0, 0.5),
                                        pose = self._detection_speech_msg.pose)

        # debug plot messages
        if self._debug_plot:
            # import plot message library
            from sweetie_bot_plot.msg import Plot, Subplot, Curve
            # create plot buffer
            self._plot_msg = Plot(title=rospy.get_name(), action=Plot.UPDATE)
            intensity_splt = Subplot(title = 'Sound intensity histogram', xlabel = 'intensity', ylabel= 'count')
            intensity_splt.curves.append( Curve(name = 'statistics', type = Curve.HISTOGRAM) )
            intensity_splt.curves.append( Curve(name = 'now', type = Curve.HISTOGRAM) )
            self._plot_msg.subplots.append( intensity_splt )
            if self.doa_estimator.dim == 2:
                direction_splt = Subplot(title = 'Azimuth Diagram', xlabel = 'azimuth', ylabel= 'likehood')
                direction_splt.curves.append( Curve(type = Curve.LINE) )
                self._plot_msg.subplots.append( direction_splt )
        
        #
        # Node interface
        #

        # dynamic parameters
        self.dyn_paramters = DynamicReconfigureServer(microphoneConfig, self.on_parameters_update)
        # listen: mic button is pressed
        self.sub_speech_indicator = rospy.Subscriber("mic_button", Bool, self.on_speech_indicator)
        # listen: robot speech event to avoid self-listening
        self.sub_hearing_disable = rospy.Subscriber("mouth", TextCommand, self.on_hearing_disable)
        # advertise
        self.pub_sound_event = rospy.Publisher("sound_event", SoundEvent , queue_size=10)
        self.pub_detections = rospy.Publisher("detections", DetectionArray, queue_size=1)
        self.pub_markers = rospy.Publisher("hmi/detections", MarkerArray, queue_size=1)
        if self._debug_plot:
            self.pub_plot = rospy.Publisher("plot", Plot, queue_size=1)
            self.srv_statistics_reset = rospy.Service('~reset_statistics', Trigger, self.on_reset_statistics)
        # transcribe service
        rospy.wait_for_service('/transcribe_api', 5.0)
        self._transcribe_service_call = rospy.ServiceProxy("/transcribe_api", Transcribe, persistent=True)
        # voice log
        self.voice_log = rospy.Publisher('voice_log', TextCommand, queue_size=1)
        # tf
        self.tf_listener = tf.TransformListener()

        #
        # start audio processing
        #
        self.gstreamer_audio.start()

    def on_gstreamer_error(self, msg):
        rospy.signal_shutdown('gstreamer error') 

    def on_parameters_update(self, config, level):
        accepted_config = {}
        for param, value in config.items():
            if param in ('sound_intensity_threshold', 'intensity_normalization_divider'):
                setattr(self, '_'+param, value)
                accepted_config[param] = value
        return accepted_config

    def on_reset_statistics(self, request):
        # reset histogramm
        with self._intesity_histogram_lock:
            self._intesity_histogram = np.ones(shape=(10,))
        return TriggerResponse(success = True, message='Histogram is cleared.')

    def on_hearing_disable(self, msg):
        self._disable_hearing = msg.data
        if self._disable_hearing: 
            self.gstreamer_audio.stop()
        else:
            self.gstreamer_audio.start()

    def on_speech_indicator(self, msg):
        self._external_speech_indicator = msg.data

    def is_speaking(self):
        return self._external_speech_indicator and not self._disable_hearing

    def buf_to_wav(self, data):
        # prepare wav data
        wav_data = bytearray()
        wav_data += struct.pack('<4sL4s4sLHHLLHH4sL', b'RIFF', 
                36 + len(data), b'WAVE', b'fmt ', 16,
                1, # no compression
                1, # n channels
                self.gstreamer_audio.rate, # framerate,
                1 * self.gstreamer_audio.rate * self.gstreamer_audio.sample_width,
                1 * self.gstreamer_audio.sample_width,
                self.gstreamer_audio.sample_width * 8, 
                b'data', len(data))
        wav_data += data

        return wav_data

    def publish_detections(self, sound_flags, doa_direction, intensity):
        # transform between microphone frame to stationary frame
        try:
            trans, rot = self.tf_listener.lookupTransform(self._mic_frame_id, self._stationary_frame_id, rospy.Time(0))
        except tf.Exception as e:
            rospy.logerr('tf request failed: %s', str(e))
            # failsafe      values
            trans = (0, 0, 0)
            rot = (0, 0, 0, 1.0)

        m_T_s = PyKDL.Frame(PyKDL.Rotation.Quaternion(*rot), PyKDL.Vector(*trans))
        # for 2D problem use only Z-axis rotation
        if self.doa_estimator.dim == 2:
            rot = np.array(rot)
            rot[0:2] = 0.0
            rot /= np.linalg.norm(rot)
        # microphone -> stationary
        s_R_m = PyKDL.Rotation.Quaternion(*rot).Inverse()
        # transform doa to stationary
        s_doa_direction = s_R_m * doa_direction

        # average speech source direction
        if sound_flags & SoundEvent.SPEECH_DETECTING:
            self._s_speech_source_direction += s_doa_direction * intensity
            self._speech_source_intensity += intensity
        else:
            self._s_speech_source_direction = PyKDL.Vector()
            self._speech_source_intensity = 0.0

        # form and publish messages
        if self._detections_are_published or self._markers_are_published:
            # modify only detection messsages
            # visualization marker shares the same header and pose
            detections_msg = DetectionArray()
            markers_msg = MarkerArray()
            # sound object
            if sound_flags & SoundEvent.SOUND_DETECTING:
                pos = self._detection_sound_msg.pose.position
                if self.doa_estimator.dim == 2:
                    s_object_pos = s_doa_direction * self._detections_distance
                    s_object_pos.z(self._detections_z_position)
                    pos.x, pos.y, pos.z = m_T_s * s_object_pos
                else:
                    pos.x, pos.y, pos.z = doa_direction * self._detections_distance
                detections_msg.detections.append(self._detection_sound_msg)
                self._marker_sound_msg.color.a = min(intensity / self._intensity_normalization_divider, 1.0)
                markers_msg.markers.append(self._marker_sound_msg)
            # speech object
            if sound_flags & SoundEvent.SPEECH_DETECTING:
                pos = self._detection_speech_msg.pose.position
                pos.x, pos.y, pos.z = self._s_speech_source_direction * (self._detections_distance / self._speech_source_intensity)
                s_object_pos = self._s_speech_source_direction * (self._detections_distance / self._speech_source_intensity)
                if self.doa_estimator.dim == 2:
                    s_object_pos.z(self._detections_z_position)
                pos.x, pos.y, pos.z = m_T_s * s_object_pos
                detections_msg.detections.append(self._detection_speech_msg)
                self._marker_speech_msg.color.a = min(intensity / self._intensity_normalization_divider, 1.0)
                markers_msg.markers.append(self._marker_speech_msg)
            # publish messages
            if len(detections_msg.detections) > 0:
                if self._detections_are_published:
                    self.pub_detections.publish(detections_msg)
                if self._markers_are_published:
                    self.pub_markers.publish(markers_msg)

    def on_audio(self, audio_data):
        # calculate sound rms
        main_channel_data = audio_data[:, self._main_channel]
        intensity = np.sqrt(np.sum(main_channel_data.astype(np.float32)**2) / len(main_channel_data)) 

        # doa estimation
        doa_direction, doa_values = self.doa_estimator.update(audio_data)

        # detect if speech is heard
        is_speaking = self.is_speaking();
        # add data to speech buffer
        if is_speaking:
            self._speech_audio_buffer.extend(main_channel_data.tobytes())

        # form speech event
        self._sound_event.header.stamp = rospy.Time.now()
        self._sound_event.intensity = intensity / self._intensity_normalization_divider
        self._sound_event.doa = Vector3(*doa_direction)
        self._sound_event.doa_values = doa_values
        sound_flags = 0
        if is_speaking:
            sound_flags |= SoundEvent.SPEECH_DETECTING
        if intensity > self._sound_intensity_threshold:
            sound_flags |= SoundEvent.SOUND_DETECTING
        self._sound_event.sound_flags = sound_flags

        # decode speech
        self._sound_event.text = ''
        self._sound_event.language = ''
        if  not is_speaking and len(self._speech_audio_buffer) != 0:
            # new speech fragment is received
            # transcribe it to test
            transcribe_req = TranscribeRequest(data=self.buf_to_wav(self._speech_audio_buffer))
            result = self._transcribe_service_call(transcribe_req)
            if result is not None:
                self._sound_event.sound_flags |= SoundEvent.SPEECH_DECODED
                self._sound_event.text = result.text
                self._sound_event.language = result.language
            # clear buffer
            self._speech_audio_buffer.clear()

        # publish result
        self.pub_sound_event.publish(self._sound_event)

        # publush object detections 
        if doa_direction != PyKDL.Vector.Zero():
            self.publish_detections(sound_flags, doa_direction, intensity)

        # publish debug plot
        if self._debug_plot:
            # DOA histogramm messagwe
            if self.doa_estimator.dim == 2:
                doa = self._plot_msg.subplots[1].curves[0]
                doa.x = self._sound_event.doa_azimuth 
                doa.y = self._sound_event.doa_values
            # intensity histogram update
            intensity_index = int(intensity // self._intensity_histogram_step)
            with self._intesity_histogram_lock:
                # resize bins array if necessary
                if intensity_index >= len(self._intesity_histogram):
                    new_size = int(1.3 * intensity_index)
                    self._intesity_histogram.resize((new_size,))
                # update
                self._intesity_histogram *= self._intensity_histogram_forget_factor
                self._intesity_histogram[intensity_index] += 1
                # check current measurement agnist histogram
                intensity_speech_sum = self._intesity_histogram.sum()
                p_speech = self._intesity_histogram[intensity_index] / intensity_speech_sum
            # inrtensity histogramm plot message
            stat = self._plot_msg.subplots[0].curves[0]
            stat.x = np.arange(0, len(self._intesity_histogram)+1) * self._intensity_histogram_step
            stat.y = self._intesity_histogram / intensity_speech_sum
            now = self._plot_msg.subplots[0].curves[1]
            now.x = np.array([intensity_index, intensity_index + 1]) * self._intensity_histogram_step
            now.y = [ p_speech ]
            # publish result
            self.pub_plot.publish(self._plot_msg)

    def finalize(self):
        # stop gstreamer
        self.gstreamer_audio.close()

def main():
    try:
        node = HearingNode()
    except Exception as e:
        rospy.logerr(str(e))
        raise
    rospy.spin()
    node.finalize()
