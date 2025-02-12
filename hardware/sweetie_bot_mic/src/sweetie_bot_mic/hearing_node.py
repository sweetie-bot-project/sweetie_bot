#!/usr/bin/env python3
import struct
import sys
from threading import Thread, Lock
from queue import Queue

import numpy as np

import rospy
import tf
import PyKDL
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

from sweetie_bot_mic.cfg import microphoneConfig

from sweetie_bot_text_msgs.msg import SoundEvent, Detection, DetectionArray, TextCommand
from sweetie_bot_text_msgs.srv import Transcribe, TranscribeRequest, TranscribeResponse
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3 
from std_srvs.srv import Trigger, TriggerResponse

from .gstreamer import GstreamerAudioSource
from .doa_estimator import DOAEstimator
from .vad import VoiceActivity, VoiceActivityDetector

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
        # DOA estimator and VAD configuration
        doa_config = rospy.get_param("~doa", {'type': 'none'})
        vad_config = rospy.get_param("~vad", {'type': 'none'})
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
        # Direaction of arrival estimator and voice activity detector
        #
        self.doa_estimator = DOAEstimator(sample_rate = sample_rate, **doa_config) 
        self.vad_detector = VoiceActivityDetector(**vad_config)

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
        self._disable_hearing = False
        self._intesity_histogram = np.ones(shape=(10,))
        self._intesity_histogram_lock = Lock()
        self._processing_queue = Queue()

        #
        # Messages buffers
        #

        header = Header(frame_id = self._mic_frame_id)
        # create object detection  message
        self._detection_sound_msg = Detection(header = header, id = 0, label = 'sound', type = 'sound')
        self._detection_speech_msg = Detection(header = header, id = 0, label = 'speech', type = 'speech')

        # create visualization marker message (temporary)
        self._marker_sound_msg = Marker(header = header, ns = 'microphone', 
                                        id = 0, type = Marker.SPHERE, action = Marker.ADD, lifetime = rospy.Duration(1.0), 
                                        scale = Vector3(0.2, 0.2, 0.2), color = ColorRGBA(0.0, 1.0, 0.0, 0.5),
                                        pose = self._detection_sound_msg.pose)
        self._marker_speech_msg = Marker(header = header, ns = 'microphone', 
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
        self._transcribe_thread = Thread(target = self.on_transcribe)
        # tf
        self.tf_listener = tf.TransformListener()

        #
        # start audio processing
        #
        self.gstreamer_audio.start()
        self._transcribe_thread.start()

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

    def _publish_detections(self, sound_event, s_sound_direction, s_speech_direction, s_T_m):
        sound_flags = sound_event.sound_flags
        intensity = sound_event.intensity
        speech_probability = sound_event.speech_probability
        # form and publish messages
        if self._detections_are_published or self._markers_are_published:
            # modify only detection messsages
            # visualization marker shares the same header and pose
            detections_msg = DetectionArray()
            markers_msg = MarkerArray()
            # sound object
            if sound_flags & SoundEvent.SOUND_DETECTING:
                pos = self._detection_sound_msg.pose.position
                s_object_pos = s_sound_direction * self._detections_distance
                if self.doa_estimator.dim == 2:
                    s_object_pos.z(self._detections_z_position)
                pos.x, pos.y, pos.z = s_T_m.Inverse(s_object_pos)
                detections_msg.detections.append(self._detection_sound_msg)
                self._marker_sound_msg.color.a = min(intensity / self._intensity_normalization_divider, 1.0)
                markers_msg.markers.append(self._marker_sound_msg)
            # speech object
            if sound_flags & SoundEvent.SPEECH_DETECTING:
                pos = self._detection_speech_msg.pose.position
                s_object_pos = s_speech_direction * self._detections_distance
                if self.doa_estimator.dim == 2:
                    s_object_pos.z(self._detections_z_position)
                pos.x, pos.y, pos.z = s_T_m.Inverse(s_object_pos)
                detections_msg.detections.append(self._detection_speech_msg)
                self._marker_speech_msg.color.a = min(speech_probability * intensity / self._intensity_normalization_divider, 1.0)
                markers_msg.markers.append(self._marker_speech_msg)
            # publish messages
            if len(detections_msg.detections) > 0:
                if self._detections_are_published:
                    self.pub_detections.publish(detections_msg)
                if self._markers_are_published:
                    self.pub_markers.publish(markers_msg)

    def _get_transforms_from_mic_to_stationary(self):
        # get transform from microphone to stationary frame
        try:
            trans, rot = self.tf_listener.lookupTransform(self._stationary_frame_id, self._mic_frame_id, rospy.Time(0))
        except tf.Exception as e:
            rospy.logerr('tf request failed: %s', str(e))
            # failsafe      values
            trans = (0, 0, 0)
            rot = (0, 0, 0, 1.0)
        # represent it as KDL frame
        s_T_m = PyKDL.Frame(PyKDL.Rotation.Quaternion(*rot), PyKDL.Vector(*trans))
        # calculate projection to horizontal plane for 2D problems
        if self.doa_estimator.dim == 2 and self._detections_fix_mic_inclination:
            rot = np.array(rot)
            rot[0:2] = 0.0
            rot /= np.linalg.norm(rot)
        # microphone -> stationary
        s_PT_m = PyKDL.Frame(PyKDL.Rotation.Quaternion(*rot), PyKDL.Vector(*trans))
        # return result 
        return s_T_m, s_PT_m

    def on_audio(self, audio_data):
        # calculate sound rms
        main_channel_data = audio_data[:, self._main_channel]
        intensity = np.sqrt(np.sum(main_channel_data.astype(np.float32)**2) / len(main_channel_data)) 

        # doa estimation
        m_sound_direction, doa_values = self.doa_estimator.update(audio_data)

        # transform doa to stationary
        s_T_m, s_PT_m = self._get_transforms_from_mic_to_stationary()
        s_sound_direction = s_PT_m.M * m_sound_direction # use projectied transform to be correct for 2D estimate

        # vad detector
        vad_result, speech_probability, speech_audio_buffer, s_speech_direction  = self.vad_detector.update(main_channel_data, s_sound_direction, intensity)

        # sound event message
        sound_event = SoundEvent(header = Header(frame_id = self._mic_frame_id, stamp = rospy.Time.now()), 
                                 intensity = intensity / self._intensity_normalization_divider,
                                 speech_probability = speech_probability,
                                 doa = Vector3(*m_sound_direction),
                                 doa_azimuth = self.doa_estimator.grid_azimuth, 
                                 doa_colatitude = self.doa_estimator.grid_colatitude,
                                 doa_values = doa_values)
        # sound flags
        sound_flags = 0
        if intensity > self._sound_intensity_threshold:
            sound_flags |= SoundEvent.SOUND_DETECTING
        if vad_result in (VoiceActivity.VOICED, VoiceActivity.UNVOICED):
            sound_flags |= SoundEvent.SPEECH_DETECTING
        elif vad_result == VoiceActivity.VOICED_END:
            sound_flags |= SoundEvent.SPEECH_DECODED
            speech_audio_buffer = self.buf_to_wav(speech_audio_buffer) # create copy
        sound_event.sound_flags = sound_flags

        # pass data for futher processing
        self._processing_queue.put_nowait( (sound_event, speech_audio_buffer) )

        # publish object detections 
        if m_sound_direction != PyKDL.Vector.Zero():
            self._publish_detections(sound_event, s_sound_direction, s_speech_direction, s_T_m)

        # publish debug plot
        if self._debug_plot:
            # DOA histogramm messagwe
            if self.doa_estimator.dim == 2:
                doa = self._plot_msg.subplots[1].curves[0]
                doa.x = self.doa_estimator.grid_azimuth 
                doa.y = doa_values
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

    def on_transcribe(self):
        while True:
            # get availabe frame (block until data is available)
            sound_event, audio_wav = self._processing_queue.get()
            # check shutting down 
            if sound_event is None:
                break
            # check if audio data for transcription is supplied
            if audio_wav is not None:
                # new speech fragment is received
                transcribe_req = TranscribeRequest(data = audio_wav)
                result = self._transcribe_service_call(transcribe_req)
                if result is not None:
                    sound_event.sound_flags |= SoundEvent.SPEECH_DECODED
                    sound_event.text = result.text
                    sound_event.language = result.language
                else:
                    sound_event.text = ''
                    sound_event.language = ''
            # publish result
            self.pub_sound_event.publish(sound_event)

    def finalize(self):
        # stop gstreamer
        self.gstreamer_audio.close()
        # stop transcribe thread
        self._processing_queue.put_nowait( (None, None) )  # None sound_event indicates shutdown

def main():
    try:
        node = HearingNode()
    except Exception as e:
        rospy.logerr(str(e))
        raise
    rospy.spin()
    node.finalize()
