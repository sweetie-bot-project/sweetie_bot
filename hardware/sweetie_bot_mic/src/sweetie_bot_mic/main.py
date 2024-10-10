#!/usr/bin/env python3
import struct
import threading
import numpy as np

import rospy
import tf
import PyKDL
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

from sweetie_bot_mic.cfg import microphoneConfig

from sweetie_bot_text_msgs.msg import SoundEvent, TextCommand, Detection, DetectionArray
from sweetie_bot_text_msgs.srv import Transcribe, TranscribeRequest, TranscribeResponse
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Bool, ColorRGBA
from geometry_msgs.msg import Vector3 
from std_srvs.srv import Trigger, TriggerResponse

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import pyroomacoustics as pra

class GstreamerAudio(object):
    _gstreamer_is_initialized = False

    FINALIZED = Gst.State.NULL
    PAUSED = Gst.State.READY
    RUNNING = Gst.State.PLAYING
            
    def __init__(self, on_audio, pipeline = None, sample_rate = 16000, n_channels=1, buffer_size = 1024, selected_channels = None, on_warning = None, on_error = None):
        self.sample_width = 2
        self.rate = sample_rate
        self.channels = selected_channels
        self.on_audio = on_audio
        self.on_warning = on_warning
        self.on_error = on_error
        # init gstreamer
        if not self._gstreamer_is_initialized:
            Gst.init(None)
            self._gstreamer_is_initialized = True
        # process parameters
        self.available_channels = n_channels
        if selected_channels is not None:
            self.selected_channels = selected_channels
        else:
            self.selected_channels = [range(0, self.available_channels)]
        # create gstreamer pipeline and configure it
        pipeline_list = []
        pipeline_list.append(pipeline)
        pipeline_list.extend([
            f'audio/x-raw,format=S16LE,channels={n_channels},rate={sample_rate},layout=interleaved',
            f'audiobuffersplit output-buffer-size={buffer_size*n_channels*self.sample_width} strict-buffer-size=true',
            f'appsink name=appsink-microphone-node max-buffers=10 drop=true emit-signals=true'
        ])
        pipeline_string = str.join(' ! ', pipeline_list)
        rospy.loginfo('gstreamer pipeline: \n  %s', pipeline_string.replace('!', '!\n  '))
        self._gstreamer_pipeline = Gst.parse_launch(pipeline_string)
        # add bus calback
        self._gstreamer_bus = self._gstreamer_pipeline.get_bus()
        self._gstreamer_bus.set_sync_handler(self._on_event, None)
        # event processing thread
        self._event_thread = threading.Thread(target =self._event_loop)
        self._event_cond = threading.Condition()
        self._event_buffer = []
        # add sample callback
        gstreamer_sink = self._gstreamer_pipeline.get_by_name('appsink-microphone-node')
        gstreamer_sink.connect("new-sample", self._on_buffer, None)
        # start pipeline
        self._gstreamer_pipeline.set_state(Gst.State.READY)
        self._event_thread.start()

    def _event_loop(self):
        while True:
            # wait until message is available
            with self._event_cond:
                while  len(self._event_buffer) == 0:
                    self._event_cond.wait()
                # get message
                msg = self._event_buffer.pop(0)
            
            # process message
            if msg.type == Gst.MessageType.WARNING:
                if self.on_warning is not None:
                # call on warn callback
                    self.on_warning(self._event_buffer)
                # log it
                rospy.logwarn('GstreamerAudio:  %s (%s)' % msg.parse_warning())

            elif msg.type == Gst.MessageType.ERROR:
                # call on error callback
                if self.on_error is not None:
                    self.on_error(self._event_buffer)
                # stop pipeline
                self._gstreamer_pipeline.set_state(Gst.State.NULL)
                rospy.logerr('GstreamerAudio: pipeline is stopped due to error: %s (%s).' % msg.parse_error())

            elif msg.type == Gst.MessageType.EOS:
                # exit event loop and thread
                return

    def _on_event(self, bus, message, userdata):
        if message.type in (Gst.MessageType.ERROR, Gst.MessageType.WARNING):
            # add message to buffer and notify event loop
            with self._event_cond:
                self._event_buffer.append(message)
                self._event_cond.notify()
        # drop alll messages
        return Gst.BusSyncReply.DROP

    def _on_buffer(self, sink, data):
        while True:
            # get gstreamer sample
            sample = sink.emit('try-pull-sample', 0.0)
            if sample is None:
                break
            # get buffer
            buffer = sample.get_buffer()
            if buffer is None:
                continue
            try:
                # bet buffer and map memory 
                success, buffer_map = buffer.map(Gst.MapFlags.READ)
                if not success:
                    continue
                # construct numpy object over buffer
                data = np.frombuffer(buffer_map.data, dtype=np.int16)
                chunk_per_channel = len(data) // self.available_channels
                data_channels = np.reshape(data, (chunk_per_channel, self.available_channels))
                # select channels
                # TODO: eliminate copy
                # TODO:  as_strided(np.frombuffer(a.data, dtype=a.dtype, offset=a.dtype.type().nbytes*1), shape=(a.shape[0],2), strides=a.strides, writeable=False)
                selected_channels = data_channels[:, self.channels]
                # invoke callback
                self.on_audio(selected_channels)
            finally:
                buffer.unmap(buffer_map)

        return Gst.FlowReturn.OK

    def __del__(self):
        if hasattr(self, '_gstreamer_pipeline'):
            # join thread
            if hasattr(self, '_event_thread'):
                with self._event_cond:
                    self._event_buffer.append(Gst.Message.new_eos())
                    self._event_cond.notify()
                self._event_thread.join()
            # stop pipeline
            self._gstreamer_pipeline.set_state(Gst.State.NULL)

    def start(self):
        self._gstreamer_pipeline.set_state(Gst.State.PLAYING)

    def stop(self):
        self._gstreamer_pipeline.set_state(Gst.State.READY)

    def finalize(self):
        # stop event loop
        with self._event_cond:
            self._event_buffer.append(Gst.Message.new_eos())
            self._event_cond.notify()
        self._event_thread.join()
        # stop pipeline
        self._gstreamer_pipeline.set_state(Gst.State.NULL)

    def get_state(self):
        return self.gstreamer_pipeline.current_state

class RespeakerNode(object):

    def __init__(self):
        rospy.init_node("microphone")
        #rospy.on_shutdown(self.on_shutdown)
        #
        # Common parameters
        #
        sample_rate = rospy.get_param("~sample_rate", 16000)
        update_rate = rospy.get_param("~update_rate", 4)
        n_channels = rospy.get_param('~n_channels', 2)
        gstreamer_pipeline = rospy.get_param('~gstreamer_pipeline', 'alsasrc' )
        main_channel = rospy.get_param('~main_channel', 0)
        self._mic_frame_id = rospy.get_param("~microphone_frame_id", "microphone_link")
        self._stationary_frame_id = rospy.get_param("~stationary_frame_id", "base_link_path")
        call_llm = rospy.get_param("~call_llm", False)
        self.debug_plot = rospy.get_param("~debug_plot", True)
        self.enable_translate = rospy.get_param("~enable_translate", True)
        self._intensity_histogram_forget_factor = rospy.get_param("~intensity_histogram_forget_factor", 0.999)
        self._intensity_histogram_step = rospy.get_param("~intensity_histogram_step", 100.0)

        #
        # Speech-to-text services
        #
        rospy.wait_for_service('/transcribe_api', 5.0)
        self._transcribe_service_call = rospy.ServiceProxy("/transcribe_api", Transcribe, persistent=True)

        #
        # Direaction of arrival estmation algorithm
        #
        self.doa_algorithm = rospy.get_param("~doa/algorithm", 'none')
        self.doa_object_distance = rospy.get_param("~doa/object_distance", 2.0)
        self.doa_object_z_position = rospy.get_param("~doa/object_z_position", 1.0)
        channels = [main_channel]
        if self.doa_algorithm in pra.doa.algorithms:
            # get paramters
            mic_coords = rospy.get_param("~doa/mic_coords")
            if not isinstance(mic_coords, list) or any(not isinstance(c, (float, int)) for c in mic_coords) or len(mic_coords) % 3 != 0:
                raise RuntimeError('DOA estimator: \'mic_coords\' parameter must be specified and be list of float of format [ x1, y1, z1, x2, y2, ... xn, yn, zn ]')
            mic_coords = np.reshape(mic_coords, (len(mic_coords)//3, 3)).T
            mic_channels = rospy.get_param("~doa/mic_channels")
            if not isinstance(mic_channels, list) or any(not isinstance(c, int) for c in mic_channels) or mic_coords.shape[1] != len(mic_channels):
                raise RuntimeError('DOA estimator: \'mic_channels\' parameter must be specified and be list of int, it size must be equal to size of mic_coords divided by two.')
            channels.extend(mic_channels)
            nfft = rospy.get_param("~doa/nfft", 256)
            if not isinstance(nfft, int) or nfft < 0 or 2**int(np.log2(nfft)) != nfft:
                raise RuntimeError('DOA estimator: \'nfft\' parameter must be power of 2.')
            self.doa_nfft = nfft
            freq_range = rospy.get_param("~doa/freq_range", [80.0, 2000.0])
            if not isinstance(freq_range, list) or len(freq_range) != 2 or any(not isinstance(freq, (int, float)) for freq in freq_range):
                raise RuntimeError('DOA estimator: \'freq_range\' must be list with two frequencies: [min_freq, max_freq].')
            if freq_range[0] >= freq_range[1] or freq_range[0] < 0.0  or freq_range[1] > sample_rate/2.0:
                raise RuntimeError('DOA estimator: incorrect \'freq_range\' [min_freq, max_freq]: the following condition must hold 0.0 <= min_freq <= max_freq <= sample_rate/2.')
            self.freq_bin_range = np.round( np.array(freq_range) / (sample_rate/2.0) * nfft ).astype(np.int32)
            mode = rospy.get_param("~doa/mode", 'far')
            if not isinstance(mode, str):
                raise RuntimeError("DOA estimator: 'mode' parameter must be string.")
            # get grid type
            grid = rospy.get_param("~doa/grid_type", None)
            if grid in ('2d_circle', '3d_sphere'):
                # get grid size paramter
                colatitude = None
                azimuth = None
                if grid.startswith('2d'):
                    dim = 2
                else:
                    dim = 3
                n_grid = rospy.get_param("~doa/grid_size", 32 if dim == 2 else 32*16)
                if not isinstance(n_grid, int) or n_grid <= 0:
                    raise RuntimeError("DOA estimator: 'grid_size' parameter must be positive integer.")
            elif grid in ('2d_linspace', '3d_linspace'):
                # interpret colatitude and azimuth as linspace specification
                n_grid = None
                azimuth = rospy.get_param("~doa/azimuth", [-180, 180, 32])
                colatitude = rospy.get_param("~doa/colatitude", None)
                try:
                    azimuth = np.deg2rad( np.linspace(azimuth[0], azimuth[1], int(azimuth[2])) )
                    if colatitude is not None:
                        colatitude = np.deg2rad( np.linspace(colatitude[0], colatitude[1], int(colatitude[2])) )
                except (TypeError, ValueError, IndexError):
                    raise RuntimeError("DOA estimator: 'azimuth' and 'colatitude' paramters must be lists in format [start_angle, stop_angel, N]. 'azimuth' paramter must be specified.")
                # 2d or 3d
                if grid.startswith('2d'):
                    dim = 2
                    colatitude = None
                else:
                    dim = 3
                    if colatitude is None:
                        raise RuntimeError("DOA estimator: 'colatitude' parameter must be specified for '3d_lispace' grid mode.")
            else:
                raise RuntimeError("DOA estimator: 'grid_type' parameter must be specified and be string with one of following value: '2d_circle', '3d_sphere', '2d_linspace', '3d_linspace'.")
            # create estimator 
            alg = pra.doa.algorithms[self.doa_algorithm]
            self.doa_estimator = alg(L = mic_coords, fs = sample_rate, nfft = nfft, num_src = 1, mode = mode, dim = dim, n_grid = n_grid, azimuth = azimuth, colatitude = colatitude)
            self.stft = pra.transform.STFT(N = nfft, hop = nfft // 2, channels = len(mic_channels))
            # log info
            rospy.loginfo(f"DOA estimator '{self.doa_algorithm}': nfft {nfft}, freq range {freq_range}, grid type '{grid}', grid size {self.doa_estimator.grid.n_points}")
        elif self.doa_algorithm != 'none':
            raise RuntimeError(f'DOA estimator: unknown algorithms {self.doa_algorithm}')

        #
        # Hardware interfaces
        #

        # audio interface
        buffer_size = int(2**np.round(np.log2(sample_rate/update_rate)))
        rospy.loginfo(f'audio: actual update rate {sample_rate/buffer_size} Hz (requested {update_rate} Hz), buffer size {buffer_size} samples, selected channels {channels}.')
        self.gstreamer_audio = GstreamerAudio(pipeline=gstreamer_pipeline, sample_rate=sample_rate, buffer_size=buffer_size, 
                                              n_channels = n_channels, selected_channels=channels, 
                                              on_audio = self.on_audio, on_error = self.on_error, on_warning = self.on_error)

        #
        # Node state 
        # 

        self.speech_audio_buffer = bytearray()
        self._button_pressed = False
        self._robot_is_speaking = False
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
        if self.doa_algorithm != 'none' and self.doa_estimator is not None:
            # add grid paramters to SoundEvent
            self._sound_event.doa_azimuth = self.doa_estimator.grid.azimuth
            if self.doa_estimator.dim == 2:
                self._sound_event.doa_colatitude = []
            else:
                self._sound_event.doa_colatitude = self.doa_estimator.grid.colatitude

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

        # plot message
        if self.debug_plot:
            # import plot message library
            from sweetie_bot_plot.msg import Plot, Subplot, Curve
            # create plot buffer
            self._plot_msg = Plot(title=rospy.get_name(), action=Plot.UPDATE)
            intensity_splt = Subplot(title = 'Sound intensity histogram', xlabel = 'intensity', ylabel= 'count')
            intensity_splt.curves.append( Curve(name = 'statistics', type = Curve.HISTOGRAM) )
            intensity_splt.curves.append( Curve(name = 'now', type = Curve.HISTOGRAM) )
            self._plot_msg.subplots.append( intensity_splt )
            if self.doa_algorithm != 'none':
                spectrum_splt = Subplot(title = 'Spectrum', xlabel = 'freq')
                spectrum_splt.curves.append( Curve(type = Curve.LINE) )
                spectrum_splt.curves[0].x = np.arange(0, nfft/2+1) * sample_rate/nfft
                self._plot_msg.subplots.append( spectrum_splt )
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
        self.sub_button_event = rospy.Subscriber("mic_button", Bool, self.on_mic_button)
        # listen: robot speech event to avoid self-listening
        self.sub_mouth_event = rospy.Subscriber("mouth", TextCommand, self.on_mouth)
        # service call: language model
        if call_llm:
            rospy.wait_for_service('/input_text', 5.0)
            self.llm_service_client = rospy.ServiceProxy("/input_text", CompleteSimple, persistent=True)
        else:
            self.llm_service_client = None
        # advertise
        self.pub_sound_event = rospy.Publisher("sound_event", SoundEvent , queue_size=10)
        if self.doa_algorithm != 'none':
            self.pub_detections = rospy.Publisher("detections", DetectionArray, queue_size=1)
            self.pub_markers = rospy.Publisher("hmi/detections", MarkerArray, queue_size=1)
        if self.debug_plot:
            self.pub_plot = rospy.Publisher("plot", Plot, queue_size=1)
            self.srv_statistics_reset = rospy.Service('~reset_statistics', Trigger, self.on_reset_statistics)
        # voice log
        self.voice_log = rospy.Publisher('voice_log', TextCommand, queue_size=1)
        # tf
        self._tf_listener = tf.TransformListener()

        #
        # start audio processing
        #
        self.gstreamer_audio.start()

    def on_error(self, msg):
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

    def on_mouth(self, cmd):
        # If robot is speaking, everyone else must shut up!
        # jk! we just don't want the robot to talk to itself.
        if cmd.type == 'mouth/speech':
            if cmd.command == 'begin':
                self._robot_is_speaking = True
                rospy.logdebug("Robot is speaking. Disabling microphone")
                self.gstreamer_audio.stop()
            else:
                self._robot_is_speaking = False
                rospy.logdebug("Robot is not speaking. Enabling microphone")
                self.gstreamer_audio.start()

    def on_mic_button(self, msg):
        self._button_pressed = msg.data

    def finalize(self):
        self.gstreamer_audio.finalize()

    def is_speaking(self):
        return self._button_pressed and not self._robot_is_speaking

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


    def on_audio(self, channels_data):
        # set current status
        is_speaking = self.is_speaking();

        # add data to speech buffer
        main_channel = channels_data[:,0]
        if is_speaking:
            self.speech_audio_buffer.extend(main_channel.tobytes())

        # calculate sound rms
        intensity = np.sqrt(np.sum(main_channel.astype(np.float32)**2) / len(main_channel)) 

        # sound intensity profile estimation (only is debug plot is enabled)
        if self.debug_plot:
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
            # debug plot
            stat = self._plot_msg.subplots[0].curves[0]
            stat.x = np.arange(0, len(self._intesity_histogram)+1) * self._intensity_histogram_step
            stat.y = self._intesity_histogram / intensity_speech_sum
            now = self._plot_msg.subplots[0].curves[1]
            now.x = np.array([intensity_index, intensity_index + 1]) * self._intensity_histogram_step
            now.y = [ p_speech ]

        # doa estimation
        doa_dim = 3
        doa_direction = PyKDL.Vector()
        if self.doa_algorithm != 'none':
            # perform fft on raw mic channels
            nfft = self.doa_nfft
            X = self.stft.analysis(channels_data[:, 1:].astype(np.float32))
            # estimate direction
            self.doa_estimator.locate_sources(np.swapaxes(X, 0, 2), freq_bins=np.arange(*self.freq_bin_range)) 
            # calculate peak direction
            sources_azimuth = self.doa_estimator.azimuth_recon
            sources_colatitude = self.doa_estimator.colatitude_recon
            if sources_azimuth is not None and len(sources_azimuth) > 0:
                azimuth = sources_azimuth[0]
            else:
                azimuth = 0.0
            if sources_colatitude is not None and len(sources_colatitude) > 0:
                colatitude = sources_colatitude[0]
            else:
                colatitude = np.pi/2.0
            doa_direction =  PyKDL.Vector(np.cos(azimuth)*np.sin(colatitude), np.sin(azimuth)*np.sin(colatitude), np.cos(colatitude))
            doa_dim = self.doa_estimator.dim

            # copy raw DOA data
            doa_values_sum = np.sum(self.doa_estimator.grid.values)
            self._sound_event.doa_values = self.doa_estimator.grid.values / doa_values_sum

            # debug plot
            if self.debug_plot:
                # copy spectrum
                spectum = self._plot_msg.subplots[1].curves[0]
                spectum.y = np.abs(X[0, :, :].sum(axis=1))
                if self.doa_estimator.dim == 2:
                    # copy DOA to debug 
                    doa = self._plot_msg.subplots[2].curves[0]
                    doa.x = self.doa_estimator.grid.azimuth 
                    doa.y = self._sound_event.doa_values

        # transform between microphone frame to stationary frame
        try:
            trans, rot = self._tf_listener.lookupTransform(self._mic_frame_id, self._stationary_frame_id, rospy.Time(0))
        except tf.Exception as e:
            rospy.logerr('tf request failed: %s', str(e))
            # failsafe values
            trans = (0, 0, 0)
            rot = (0, 0, 0, 1.0)

        m_T_s = PyKDL.Frame(PyKDL.Rotation.Quaternion(*rot), PyKDL.Vector(*trans))
        # for 2D problem use only Z-axis rotation
        if doa_dim == 2:
            rot = np.array(rot)
            rot[0:2] = 0.0
            rot /= np.linalg.norm(rot)
        # microphone -> stationary
        s_R_m = PyKDL.Rotation.Quaternion(*rot).Inverse()
        # transform doa to stationary
        s_doa_direction = s_R_m * doa_direction

        # average speech source direction
        if is_speaking:
            self._s_speech_source_direction += s_doa_direction * intensity
            self._speech_source_intensity += intensity
        else:
            self._s_speech_source_direction = PyKDL.Vector()
            self._speech_source_intensity = 0.0

        # form speech event
        self._sound_event.header.stamp = rospy.Time.now()
        self._sound_event.doa = Vector3(*doa_direction)
        self._sound_event.intensity = intensity / self._intensity_normalization_divider
        sound_flags = 0
        if is_speaking:
            sound_flags |= SoundEvent.SPEECH_DETECTING
        if intensity > self._sound_intensity_threshold:
            sound_flags |= SoundEvent.SOUND_DETECTING
        self._sound_event.sound_flags = sound_flags

        # decode speech
        self._sound_event.text = ''
        self._sound_event.language = ''
        if not is_speaking and len(self.speech_audio_buffer) != 0:
            # new speech fragment is received
            # transcribe it to test
            transcribe_req = TranscribeRequest(data=self.buf_to_wav(self.speech_audio_buffer))
            result = self._transcribe_service_call(transcribe_req)
            if result is not None:
                self._sound_event.sound_flags |= SoundEvent.SPEECH_DECODED
                self._sound_event.text = result.text
                self._sound_event.language = result.language
            # clear buffer
            self.speech_audio_buffer.clear()

        # publish result
        self.pub_sound_event.publish(self._sound_event)

        # publush object detections and markers if someone is speeching or sound is heard
        if self.doa_algorithm != 'none':
            detections_msg = DetectionArray()
            markers_msg = MarkerArray()
            # modify only detection messsages
            # visualization marker shares the same header and pose
            if sound_flags & SoundEvent.SOUND_DETECTING:
                pos = self._detection_sound_msg.pose.position
                if doa_dim == 2:
                    s_object_pos = s_doa_direction * self.doa_object_distance
                    s_object_pos.z(self.doa_object_z_position)
                    pos.x, pos.y, pos.z = m_T_s * s_object_pos
                else:
                    pos.x, pos.y, pos.z = doa_direction * self.doa_object_distance
                detections_msg.detections.append(self._detection_sound_msg)
                self._marker_sound_msg.color.a = min(intensity / self._intensity_normalization_divider, 1.0)
                markers_msg.markers.append(self._marker_sound_msg)
            if sound_flags & SoundEvent.SPEECH_DETECTING:
                pos = self._detection_speech_msg.pose.position
                pos.x, pos.y, pos.z = self._s_speech_source_direction * (self.doa_object_distance / self._speech_source_intensity)
                s_object_pos = self._s_speech_source_direction * (self.doa_object_distance / self._speech_source_intensity)
                if doa_dim == 2:
                    s_object_pos.z(self.doa_object_z_position)
                pos.x, pos.y, pos.z = m_T_s * s_object_pos
                detections_msg.detections.append(self._detection_speech_msg)
                self._marker_speech_msg.color.a = min(intensity / self._intensity_normalization_divider, 1.0)
                markers_msg.markers.append(self._marker_speech_msg)
            if len(detections_msg.detections) > 0:
                self.pub_detections.publish(detections_msg)
                self.pub_markers.publish(markers_msg)

        # publish debug plot
        if self.debug_plot:
            self.pub_plot.publish(self._plot_msg)


def main():
    try:
        node = RespeakerNode()
    except Exception as e:
        rospy.logerr(str(e))
        raise
    rospy.spin()
    node.finalize()
