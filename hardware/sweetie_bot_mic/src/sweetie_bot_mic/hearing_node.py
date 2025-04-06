#!/usr/bin/env python3
import struct
import sys
from threading import Thread, Lock
from queue import Queue, Full as QueueFullError

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
        # speech source tracking and end of speech detection paramteres
        self._speech_timeout = rospy.get_param('~speech_timeout', 0.5)
        self._speech_direction_tolerance = rospy.get_param('~speech_direction_tolerance', 60.0)
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
        self._intensity_plot_duration = rospy.get_param("~intensity_plot_duration", 8.0)

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
        # external block flag
        self._disable_hearing = False
        # histogram and plot
        self._plot_start_timestamp = rospy.Time.now()
        self._intesity_histogram = np.ones(shape=(10,))
        self._intesity_histogram_lock = Lock()
        # speech detection tracker
        self._speech_is_tracked = False
        self._s_speech_direction = PyKDL.Vector()
        self._speech_direction_intensity = 0.0
        self._speech_buffer = bytearray()
        self._speech_unvoiced_duration = 0.0
        # transcribe request queue
        self._processing_queue = Queue(maxsize = 50)

        #
        # Messages buffers
        #

        header = Header(frame_id = self._mic_frame_id)
        # create object detection  message
        self._msg_header = header
        self._detection_sound_msg = Detection(header = header, id = 0, label = 'sound', type = 'sound')
        self._detection_sound_msg.pose.orientation.w = 1.0
        self._detection_speech_msg = Detection(header = header, id = 0, label = 'speech', type = 'speech')
        self._detection_speech_msg.pose.orientation.w = 1.0

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
            xlength = self._intensity_plot_duration
            # import plot message library
            from sweetie_bot_plot.msg import Plot, Subplot, Curve
            # create plot buffer
            intensity_histogram_subplot = Subplot(title = 'Sound intensity histogram', xlabel = 'intensity', ylabel= 'count',
                                                  curves = [ Curve(name = 'statistics', type = Curve.HISTOGRAM), Curve(name = 'now', type = Curve.HISTOGRAM)])
            intensity_subplot = Subplot(title = 'Sound intensity', xlabel = 't', ylabel= 'intensity',
                                        curves = [ Curve(name = 'intensity', type = Curve.LINE_APPEND, xlength = xlength, style='-', x = [ 0.0,], y = [0.0,]), 
                                                   Curve(name = 'threshold', type = Curve.LINE, x = [ -xlength, 0.0], y = [0.0, 0.0]),  
                                                   Curve(name = 'sound flag', type = Curve.LINE_APPEND, xlength = xlength, x = [ 0.0,], y = [0.0,]) ])
            speech_likehood_subplot = Subplot(title = 'Speech likehood', xlabel = 't', ylabel= 'likehood',
                                              curves = [ Curve(name = 'likehood', type = Curve.LINE_APPEND, xlength = xlength, style='-', x = [ 0.0,], y = [0.0,]), 
                                                         Curve(name = 'VOICED/UNVOICED', type = Curve.LINE_APPEND, xlength = xlength, x = [ 0.0,], y = [0.0,]),
                                                         Curve(name = 'speech flag', type = Curve.LINE_APPEND, xlength = xlength, x = [ 0.0,], y = [0.0,]) ])
            self._plot_msg = Plot(title=rospy.get_name(), action=Plot.UPDATE, subplots = [ intensity_histogram_subplot, intensity_subplot, speech_likehood_subplot ])
        
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
        self._plot_start_timestamp = rospy.Time.now()
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

    def _average_speech_direction(self, doa_direction, intensity):
        ''' Average speech source direction using intensity as weight. '''
        self._speech_direction += doa_direction * intensity
        self._speech_direction_intensity += intensity

    def _reset_speech_direction(self):
        ''' Reset averaged direction. '''
        self._speech_direction = PyKDL.Vector()
        self._speech_direction_intensity = 0.0

    @property
    def speech_direction(self):
        ''' Get curernt averaged direction to speech source. '''
        if self._speech_direction_intensity > 0.0:
            return (1.0/self._speech_direction_intensity) * self._speech_direction
        else:
            return PyKDL.Vector()

    def _publish_detections(self, sound_event, s_sound_direction, s_speech_direction, s_T_m):
        # form and publish messages
        if self._detections_are_published or self._markers_are_published:
            sound_flags = sound_event.sound_flags
            intensity = sound_event.intensity
            speech_probability = sound_event.speech_probability
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
                self._marker_sound_msg.color.a = min(intensity, 1.0)
                markers_msg.markers.append(self._marker_sound_msg)
            # speech object
            if sound_flags & SoundEvent.SPEECH_DETECTING:
                pos = self._detection_speech_msg.pose.position
                s_object_pos = s_speech_direction * self._detections_distance
                if self.doa_estimator.dim == 2:
                    s_object_pos.z(self._detections_z_position)
                pos.x, pos.y, pos.z = s_T_m.Inverse(s_object_pos)
                detections_msg.detections.append(self._detection_speech_msg)
                self._marker_speech_msg.color.a = min(speech_probability * intensity, 1.0)
                markers_msg.markers.append(self._marker_speech_msg)
            # publish messages
            if len(detections_msg.detections) > 0:
                # update headers (all messges share common header)
                self._msg_header.stamp = sound_event.header.stamp
                # publication
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

    def _track_speech(self, audio_data, intensity, s_sound_direction, vad_result, vad_likehood):
        # check if first VOICED fragment is arrived 
        if not self._speech_is_tracked and vad_result == VoiceActivity.VOICED:
            # reset state
            self._speech_buffer.clear()
            self._speech_unvoiced_duration = 0.0
            PyKDL.SetToZero(self._s_speech_direction)
            self._speech_direction_intensity = 0.0
            # new state
            self._speech_is_tracked = True

        # track current speech source
        if self._speech_is_tracked:
            # update state 
            if vad_result in (VoiceActivity.VOICED, VoiceActivity.UNVOICED):
                # process frame
                self._speech_buffer.extend(audio_data.tobytes())
                weight = intensity * vad_likehood
                self._s_speech_direction += weight * s_sound_direction
                self._speech_direction_intensity += weight
            # reset timeut on VOICED fragment
            if vad_result == VoiceActivity.VOICED:
                self._speech_unvoiced_duration = 0.0
            # check timeout on UNVOICED
            if vad_result == VoiceActivity.UNVOICED:
                self._speech_unvoiced_duration += len(audio_data) / self.gstreamer_audio.rate
                if self._speech_unvoiced_duration > self._speech_timeout:
                    # end of speech
                    self._speech_is_tracked = False
                    return SoundEvent.SPEECH_DECODED
            # check external triger
            if vad_result == VoiceActivity.SILENCE:
                # end of speech
                self._speech_is_tracked = False
                return SoundEvent.SPEECH_DECODED
            # continue speech tracking
            return SoundEvent.SPEECH_DETECTING
        else:
            # speech is not tracked
            return 0

    def on_audio(self, audio_data):
        timestamp = rospy.Time.now()

        # calculate sound rms
        main_channel_data = audio_data[:, self._main_channel]
        intensity = np.sqrt(np.sum(main_channel_data.astype(np.float32)**2) / len(main_channel_data)) 

        # doa estimation (microphone frame)
        m_sound_direction, doa_values = self.doa_estimator.update(audio_data)

        # vad detector
        vad_result, vad_likehood = self.vad_detector.update(main_channel_data)

        # transform DOA data to stationary frame
        s_T_m, s_PT_m = self._get_transforms_from_mic_to_stationary()
        s_sound_direction = s_PT_m.M * m_sound_direction # use projectied transform to be correct for 2D estimate

        # speech tracker
        speech_flags = self._track_speech(main_channel_data, intensity, s_sound_direction, vad_result, vad_likehood)

        # sound event message
        sound_event = SoundEvent(header = Header(frame_id = self._mic_frame_id, stamp = timestamp), 
                                 intensity = intensity / self._intensity_normalization_divider,
                                 speech_probability = vad_likehood,
                                 doa = Vector3(*m_sound_direction),
                                 doa_azimuth = self.doa_estimator.grid_azimuth, 
                                 doa_colatitude = self.doa_estimator.grid_colatitude,
                                 doa_values = doa_values)
        # sound flags
        sound_flags = 0
        if intensity > self._sound_intensity_threshold:
            sound_flags |= SoundEvent.SOUND_DETECTING
        sound_flags |= speech_flags
        sound_event.sound_flags = sound_flags

        # copy speech audio data fi it is necessary
        if sound_flags & SoundEvent.SPEECH_DECODED:
            speech_audio_buffer = self.buf_to_wav(self._speech_buffer) # create copy
        else:
            speech_audio_buffer = None

        # pass data for futher processing
        try:
            self._processing_queue.put_nowait( (sound_event, speech_audio_buffer) )
        except QueueFullError:
            rospy.logwarn('HearingNode: SoundEvent queue is full. Drop sound event. Maybe speech to text transcription is taking to long.')

        # publish object detections 
        if m_sound_direction != PyKDL.Vector.Zero():
            s_speech_direction = self._s_speech_direction / self._speech_direction_intensity
            self._publish_detections(sound_event, s_sound_direction, s_speech_direction, s_T_m)

        # publish debug plot
        if self._debug_plot:
            stat_plot = self._plot_msg.subplots[0].curves[0]
            now_plot = self._plot_msg.subplots[0].curves[1]
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
            # intensity histogramm subplot message update
            if len(stat_plot.x) != len(self._intesity_histogram)+1:
                stat_plot.x = np.arange(0, len(self._intesity_histogram)+1) * self._intensity_histogram_step
            stat_plot.y = self._intesity_histogram / intensity_speech_sum
            now_plot.x = np.array([intensity_index, intensity_index + 1]) * self._intensity_histogram_step
            now_plot.y = [ p_speech ]
            # intensity subplot
            t = (timestamp - self._plot_start_timestamp).to_sec()
            T = len(main_channel_data)/self.gstreamer_audio.rate
            intensity_subplot = self._plot_msg.subplots[1]
            intensity_subplot.curves[0].x[0] = t
            intensity_subplot.curves[0].y[0] = intensity
            #intensity_subplot.curves[0].x = np.linspace(0.0, T, 256, endpoint=False) + np.round(t/T)*T
            #intensity_subplot.curves[0].y = main_channel_data.reshape(256, len(main_channel_data)//256).sum(axis=1)
            intensity_subplot.curves[1].x[:] = t - self._intensity_plot_duration, t
            intensity_subplot.curves[1].y[:] = self._sound_intensity_threshold, self._sound_intensity_threshold
            intensity_subplot.curves[2].x[0] = t
            intensity_subplot.curves[2].y[0] = 2 * self._sound_intensity_threshold if sound_flags & SoundEvent.SOUND_DETECTING else 0.0
            # speech likehood subplot
            speech_likehood_subplot = self._plot_msg.subplots[2]
            speech_likehood_subplot.curves[0].x[0] = t
            speech_likehood_subplot.curves[0].y[0] = vad_likehood
            speech_likehood_subplot.curves[1].x[0] = t
            speech_likehood_subplot.curves[1].y[0] = 0.9 if vad_result == VoiceActivity.VOICED else (0.1 if vad_result == VoiceActivity.UNVOICED else 0.0)
            speech_likehood_subplot.curves[2].x[0] = t
            speech_likehood_subplot.curves[2].y[0] = 1.0 if sound_flags & SoundEvent.SPEECH_DETECTING else 0.0
            # add submodules plots
            del self._plot_msg.subplots[3:]
            self._plot_msg.subplots.extend( self.doa_estimator.debug_plots(t) )
            self._plot_msg.subplots.extend( self.vad_detector.debug_plots(t) )
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
            # debug speech file
            #if self._debug_plot and audio_wav is not None:
                #with open('/tmp/speech.wav', 'wb') as fd:
                    #fd.write(audio_wav)

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
