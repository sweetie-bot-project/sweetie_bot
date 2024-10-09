#!/usr/bin/env python3
import os
import sys
import time
from contextlib import contextmanager
import usb.core
import usb.util
import pyaudio
import numpy as np
import struct
import threading

import rospy
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

from sweetie_bot_mic.cfg import microphoneConfig

from sweetie_bot_text_msgs.msg import SoundEvent, TextCommand, Detection, DetectionArray
from sweetie_bot_text_msgs.srv import Transcribe, TranscribeRequest, TranscribeResponse
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Bool, ColorRGBA
from geometry_msgs.msg import Vector3
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Bool

import pyroomacoustics as pra

# suppress error messages from ALSA
# https://stackoverflow.com/questions/7088672/pyaudio-working-but-spits-out-error-messages-each-time
# https://stackoverflow.com/questions/36956083/how-can-the-terminal-output-of-executables-run-by-python-functions-be-silenced-i
@contextmanager
def ignore_stderr(enable=True):
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield


class RespeakerInterface(object):
    VENDOR_ID = 0x2886
    PRODUCT_ID = 0x0018
    TIMEOUT = 100000

    # Partly copied from https://github.com/respeaker/usb_4_mic_array
    # parameter list
    # name: (id, offset, type, max, min , r/w, info)
    PARAMETERS = {
        'AECFREEZEONOFF': (18, 7, 'int', 1, 0, 'rw', 'Adaptive Echo Canceler updates inhibit.', '0 = Adaptation enabled', '1 = Freeze adaptation, filter only'),
        'AECNORM': (18, 19, 'float', 16, 0.25, 'rw', 'Limit on norm of AEC filter coefficients'),
        'AECPATHCHANGE': (18, 25, 'int', 1, 0, 'ro', 'AEC Path Change Detection.', '0 = false (no path change detected)', '1 = true (path change detected)'),
        'RT60': (18, 26, 'float', 0.9, 0.25, 'ro', 'Current RT60 estimate in seconds'),
        'HPFONOFF': (18, 27, 'int', 3, 0, 'rw', 'High-pass Filter on microphone signals.', '0 = OFF', '1 = ON - 70 Hz cut-off', '2 = ON - 125 Hz cut-off', '3 = ON - 180 Hz cut-off'),
        'RT60ONOFF': (18, 28, 'int', 1, 0, 'rw', 'RT60 Estimation for AES. 0 = OFF 1 = ON'),
        'AECSILENCELEVEL': (18, 30, 'float', 1, 1e-09, 'rw', 'Threshold for signal detection in AEC [-inf .. 0] dBov (Default: -80dBov = 10log10(1x10-8))'),
        'AECSILENCEMODE': (18, 31, 'int', 1, 0, 'ro', 'AEC far-end silence detection status. ', '0 = false (signal detected) ', '1 = true (silence detected)'),
        'AGCONOFF': (19, 0, 'int', 1, 0, 'rw', 'Automatic Gain Control. ', '0 = OFF ', '1 = ON'),
        'AGCMAXGAIN': (19, 1, 'float', 1000, 1, 'rw', 'Maximum AGC gain factor. ', '[0 .. 60] dB (default 30dB = 20log10(31.6))'),
        'AGCDESIREDLEVEL': (19, 2, 'float', 0.99, 1e-08, 'rw', 'Target power level of the output signal. ', '[-inf .. 0] dBov (default: -23dBov = 10log10(0.005))'),
        'AGCGAIN': (19, 3, 'float', 1000, 1, 'rw', 'Current AGC gain factor. ', '[0 .. 60] dB (default: 0.0dB = 20log10(1.0))'),
        'AGCTIME': (19, 4, 'float', 1, 0.1, 'rw', 'Ramps-up / down time-constant in seconds.'),
        'CNIONOFF': (19, 5, 'int', 1, 0, 'rw', 'Comfort Noise Insertion.', '0 = OFF', '1 = ON'),
        'FREEZEONOFF': (19, 6, 'int', 1, 0, 'rw', 'Adaptive beamformer updates.', '0 = Adaptation enabled', '1 = Freeze adaptation, filter only'),
        'STATNOISEONOFF': (19, 8, 'int', 1, 0, 'rw', 'Stationary noise suppression.', '0 = OFF', '1 = ON'),
        'GAMMA_NS': (19, 9, 'float', 3, 0, 'rw', 'Over-subtraction factor of stationary noise. min .. max attenuation'),
        'MIN_NS': (19, 10, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression.', '[-inf .. 0] dB (default: -16dB = 20log10(0.15))'),
        'NONSTATNOISEONOFF': (19, 11, 'int', 1, 0, 'rw', 'Non-stationary noise suppression.', '0 = OFF', '1 = ON'),
        'GAMMA_NN': (19, 12, 'float', 3, 0, 'rw', 'Over-subtraction factor of non- stationary noise. min .. max attenuation'),
        'MIN_NN': (19, 13, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression.', '[-inf .. 0] dB (default: -10dB = 20log10(0.3))'),
        'ECHOONOFF': (19, 14, 'int', 1, 0, 'rw', 'Echo suppression.', '0 = OFF', '1 = ON'),
        'GAMMA_E': (19, 15, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (direct and early components). min .. max attenuation'),
        'GAMMA_ETAIL': (19, 16, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (tail components). min .. max attenuation'),
        'GAMMA_ENL': (19, 17, 'float', 5, 0, 'rw', 'Over-subtraction factor of non-linear echo. min .. max attenuation'),
        'NLATTENONOFF': (19, 18, 'int', 1, 0, 'rw', 'Non-Linear echo attenuation.', '0 = OFF', '1 = ON'),
        'NLAEC_MODE': (19, 20, 'int', 2, 0, 'rw', 'Non-Linear AEC training mode.', '0 = OFF', '1 = ON - phase 1', '2 = ON - phase 2'),
        'SPEECHDETECTED': (19, 22, 'int', 1, 0, 'ro', 'Speech detection status.', '0 = false (no speech detected)', '1 = true (speech detected)'),
        'FSBUPDATED': (19, 23, 'int', 1, 0, 'ro', 'FSB Update Decision.', '0 = false (FSB was not updated)', '1 = true (FSB was updated)'),
        'FSBPATHCHANGE': (19, 24, 'int', 1, 0, 'ro', 'FSB Path Change Detection.', '0 = false (no path change detected)', '1 = true (path change detected)'),
        'TRANSIENTONOFF': (19, 29, 'int', 1, 0, 'rw', 'Transient echo suppression.', '0 = OFF', '1 = ON'),
        'VOICEACTIVITY': (19, 32, 'int', 1, 0, 'ro', 'VAD voice activity status.', '0 = false (no voice activity)', '1 = true (voice activity)'),
        'STATNOISEONOFF_SR': (19, 33, 'int', 1, 0, 'rw', 'Stationary noise suppression for ASR.', '0 = OFF', '1 = ON'),
        'NONSTATNOISEONOFF_SR': (19, 34, 'int', 1, 0, 'rw', 'Non-stationary noise suppression for ASR.', '0 = OFF', '1 = ON'),
        'GAMMA_NS_SR': (19, 35, 'float', 3, 0, 'rw', 'Over-subtraction factor of stationary noise for ASR. ', '[0.0 .. 3.0] (default: 1.0)'),
        'GAMMA_NN_SR': (19, 36, 'float', 3, 0, 'rw', 'Over-subtraction factor of non-stationary noise for ASR. ', '[0.0 .. 3.0] (default: 1.1)'),
        'MIN_NS_SR': (19, 37, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression for ASR.', '[-inf .. 0] dB (default: -16dB = 20log10(0.15))'),
        'MIN_NN_SR': (19, 38, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression for ASR.', '[-inf .. 0] dB (default: -10dB = 20log10(0.3))'),
        'GAMMAVAD_SR': (19, 39, 'float', 1000, 0, 'rw', 'Set the threshold for voice activity detection.', '[-inf .. 60] dB (default: 3.5dB 20log10(1.5))'),
        # 'KEYWORDDETECT': (20, 0, 'int', 1, 0, 'ro', 'Keyword detected. Current value so needs polling.'),
        'DOAANGLE': (21, 0, 'int', 359, 0, 'ro', 'DOA angle. Current value. Orientation depends on build configuration.')
    }

    def __init__(self):
        self.dev = usb.core.find(idVendor=self.VENDOR_ID,
                                 idProduct=self.PRODUCT_ID)
        if not self.dev:
            raise RuntimeError("Failed to find Respeaker device")
        rospy.loginfo("Initializing Respeaker device")
        # TODO: check if this code can be deleted,
        # self.dev.reset()
        # time.sleep(10)  # it will take 10 seconds to re-recognize as audio device
        rospy.loginfo("Respeaker device initialized (Version: %s)" % self.version)

    def __del__(self):
        try:
            self.close()
        except:
            pass
        finally:
            self.dev = None

    def write(self, name, value):
        # get parameter information    
        data = self.PARAMETERS.get(name)
        if data is None:
            raise KeyError(f'Unknown ReSpeaker parameter "{name}".')
        # check if paramter is wirtable
        if data[5] == 'ro':
            raise ValueError('{} is read-only'.format(name))
        # get id and encode payload
        id = data[0]
        # 4 bytes offset, 4 bytes value, 4 bytes type
        try:
            if data[2] == 'int':
                payload = struct.pack(b'iii', data[1], int(value), 1)
            else:
                payload = struct.pack(b'ifi', data[1], float(value), 0)
        except ValueError:
            raise ValueError(f'Parameter "{name}" have incompatible value: {value}')
        # send command
        self.dev.ctrl_transfer(
            usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0, id, payload, self.TIMEOUT)

    def read(self, name):
        # get parameter information    
        data = self.PARAMETERS.get(name)
        if data is None:
            raise KeyError(f'Unknown parameter name "{name}".')
        # get parameter id and process it type
        id = data[0]
        cmd = 0x80 | data[1]
        if data[2] == 'int':
            cmd |= 0x40
        length = 8
        # request device
        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmd, id, length, self.TIMEOUT)
        # uanpack and decode result
        response = struct.unpack(b'ii', response)
        if data[2] == 'int':
            result = response[0]
        else:
            result = response[0] * (2.**response[1])
        return result

    def is_voice(self):
        return self.read('VOICEACTIVITY')

    def direction(self):
        return self.read('DOAANGLE')

    @property
    def version(self):
        return self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0x80, 0, 1, self.TIMEOUT)[0]

    def close(self):
        """
        close the interface
        """
        usb.util.dispose_resources(self.dev)

class RespeakerAudio(object):

    def __init__(self, on_audio, mic_name = '', sample_rate = 16000, buffer_size = 1024, channels=None, suppress_error=True):
        self.on_audio = on_audio
        with ignore_stderr(enable=suppress_error):
            self.pyaudio = pyaudio.PyAudio()
        self.available_channels = None
        self.channels = channels
        self.device_index = None
        self.rate = sample_rate
        self.sample_width = 2
        self.bit_depth = 16

        # find device
        count = self.pyaudio.get_device_count()
        rospy.logdebug(f"{count} audio devices found")
        for i in range(count):
            info = self.pyaudio.get_device_info_by_index(i)
            name = info["name"]
            chan = info["maxInputChannels"]
            rospy.logdebug(f" - {i}: {name} ({chan} channels)")
            if name.startswith(mic_name):
                self.available_channels = chan
                self.device_index = i
                rospy.loginfo(f"Found pyaudio device id={i} name='{name}' (channels: {chan})")
                break

        if self.device_index is None:
            msg = f"Failed to find pyaudio device by name '{mic_name}'. Available devices:\n"
            for i in range(count):
                info = self.pyaudio.get_device_info_by_index(i)
                msg += f" - '{info['name']}' (input channels: {info['maxInputChannels']}, output channels: {info['maxOutputChannels']})\n"
            raise RuntimeError(msg)

        # selected channels
        if self.channels is None:
            self.channels = range(self.available_channels)
        elif any(c < 0 or c >= self.available_channels for c in self.channels):
            raise RuntimeError("Invalid channels {list(self.channels)}. (Available channels are {self.available_channels}).")
        rospy.loginfo(f"Using channels {list(self.channels)}")

        self.stream = self.pyaudio.open(
            input=True, start=False,
            format=pyaudio.paInt16,
            channels=self.available_channels,
            rate=self.rate,
            frames_per_buffer=int(buffer_size),
            stream_callback=self.stream_callback,
            input_device_index=self.device_index,
        )

    def __del__(self):
        try:
            self.stop()
            self.stream.close()
        except:
            pass
        finally:
            self.stream = None
        try:
            self.pyaudio.terminate()
        except:
            pass

    def stream_callback(self, in_data, frame_count, time_info, status):
        # split channel
        data = np.frombuffer(in_data, dtype=np.int16)
        chunk_per_channel = len(data) // self.available_channels
        data_channels = np.reshape(data, (chunk_per_channel, self.available_channels))
        # select channels
        selected_channels = data_channels[:, self.channels]
        # invoke callback
        self.on_audio(selected_channels)
        # continue function
        return None, pyaudio.paContinue

    def start(self):
        if self.stream.is_stopped():
            self.stream.start_stream()

    def stop(self):
        if self.stream.is_active():
            self.stream.stop_stream()

class RespeakerNode(object):

    def __init__(self):
        # shutdown hook
        rospy.on_shutdown(self.on_shutdown)
        #
        # Common parameters
        #
        mic_name = rospy.get_param("~mic_name", "ReSpeaker 4 Mic Array (UAC1.0)")
        sample_rate = rospy.get_param("~sample_rate", 16000)
        update_rate = rospy.get_param("~update_rate", 4)
        main_channel = rospy.get_param('~main_channel', 0)
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        call_llm = rospy.get_param("~call_llm", False)
        self.debug_plot = rospy.get_param("~debug_plot", True)
        self.enable_translate = rospy.get_param("~enable_translate", True)
        suppress_pyaudio_error = rospy.get_param("~suppress_pyaudio_error", True)
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
        channels = [main_channel]
        if self.doa_algorithm == 'respeaker':
            self.doa_estimator = None
            rospy.loginfo(f'DOA estimator: respeaker')
        elif self.doa_algorithm in pra.doa.algorithms:
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
        self.respeaker_audio = RespeakerAudio(self.on_audio, mic_name, sample_rate=sample_rate, buffer_size=buffer_size, channels=channels, suppress_error=suppress_pyaudio_error)

        # respeaker interface
        try:
            self.respeaker = RespeakerInterface()
            params = rospy.get_param("~respeaker", {})
            for k, v in params:
                self.respeaker.write(k, v)
        except RuntimeError:
            # continur without respeaker
            rospy.logwarn('respeaker device is not detected')
            self.respeaker = None
        # check if we can continue without respeaker
        if self.respeaker is None and self.doa_algorithm == 'respeaker':
            raise RuntimeError('DOA estimation algorithm "respeaker" is not available: Respeaker device is not found')

        #
        # Node state 
        # 

        self.speech_audio_buffer = bytearray()
        self._button_pressed = False
        self._robot_is_speaking = False
        self._intesity_histogram = np.ones(shape=(10,))
        self._intesity_histogram_lock = threading.Lock()
        self._speech_source_direction = np.zeros(shape=(3,))
        self._speech_source_intensity = 0.0

        #
        # Messages buffers
        #

        # sound event message
        self._sound_event = SoundEvent()
        self._sound_event.header.frame_id = self.frame_id
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
            self._plot_msg = Plot(title=mic_name, action=Plot.UPDATE)
            intensity_splt = Subplot(title = 'Sound intensity histogram', xlabel = 'intensity', ylabel= 'count')
            intensity_splt.curves.append( Curve(name = 'statistics', type = Curve.HISTOGRAM) )
            intensity_splt.curves.append( Curve(name = 'now', type = Curve.HISTOGRAM) )
            self._plot_msg.subplots.append( intensity_splt )
            if self.doa_algorithm not in ('none', 'respeaker'):
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

        #
        # start audio processing
        #
        self.respeaker_audio.start()

    def on_shutdown(self):
        try:
            self.respeaker_audio.stop()
            self.keyboard_listener.stop()
        except:
            pass
        finally:
            self.respeaker_audio = None
            self.respeaker = None

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
                self.respeaker_audio.stop()
            else:
                self._robot_is_speaking = False
                rospy.logdebug("Robot is not speaking. Enabling microphone")
                self.respeaker_audio.start()

    def on_mic_button(self, msg):
        self._button_pressed = msg.data

    def is_speaking(self):
        return self._button_pressed and not self._robot_is_speaking

    def buf_to_wav(self, data):
        # prepare wav data
        wav_data = bytearray()
        wav_data += struct.pack('<4sL4s4sLHHLLHH4sL', b'RIFF', 
                36 + len(data), b'WAVE', b'fmt ', 16,
                1, # no compression
                1, # n channels
                self.respeaker_audio.rate, # framerate,
                1 * self.respeaker_audio.rate * self.respeaker_audio.sample_width,
                1 * self.respeaker_audio.sample_width,
                self.respeaker_audio.sample_width * 8, 
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
        doa_direction = np.zeros(shape=(3,))
        if self.doa_algorithm == 'respeaker':
            # get direction from hardware
            doa_rad = np.deg2rad(self.respeaker.direction())
            doa_direction[:] = np.cos(doa_rad), np.sin(doa_rad), 0.0
        elif self.doa_algorithm != 'none':
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
            doa_direction = np.array( (np.cos(azimuth)*np.sin(colatitude), np.sin(azimuth)*np.sin(colatitude), np.cos(colatitude)) )

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

        # average speech source direction
        # TODO: use non-moving frame
        if is_speaking:
            self._speech_source_direction += doa_direction * intensity
            self._speech_source_intensity += intensity
        else:
            self._speech_source_direction[:] = 0.0
            self._speech_source_intensity = 0.0

        # form speech event
        self._sound_event.header.stamp = rospy.Time.now()
        self._sound_event.doa = Vector3( *doa_direction )
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
                pos.x, pos.y, pos.z = doa_direction * self.doa_object_distance
                detections_msg.detections.append(self._detection_sound_msg)
                self._marker_sound_msg.color.a = min(intensity / self._intensity_normalization_divider, 1.0)
                markers_msg.markers.append(self._marker_sound_msg)
            if sound_flags & SoundEvent.SPEECH_DETECTING:
                pos = self._detection_speech_msg.pose.position
                pos.x, pos.y, pos.z = self._speech_source_direction * (self.doa_object_distance / self._speech_source_intensity)
                detections_msg.detections.append(self._detection_speech_msg)
                self._marker_speech_msg.color.a = min(intensity / self._intensity_normalization_divider, 1.0)
                markers_msg.markers.append(self._marker_speech_msg)
            if len(detections_msg.detections) > 0:
                self.pub_detections.publish(detections_msg)
                self.pub_markers.publish(markers_msg)

        # publish debug plot
        if self.debug_plot:
            self.pub_plot.publish(self._plot_msg)

        # print intensity
        #if is_speaking:
        #    print(f'Intesity: {intensity}')

def main():
    rospy.init_node("respeaker_node")
    try:
        n = RespeakerNode()
    except Exception as e:
        rospy.logerr(str(e))
        raise
    rospy.spin()
