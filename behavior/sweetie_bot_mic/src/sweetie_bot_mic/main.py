#!/usr/bin/env python3
import os
import sys
import time
from contextlib import contextmanager
import usb.core
import usb.util
import pyaudio
from pynput import keyboard
from pynput.keyboard import Key
import requests
import numpy as np
import math
import struct

import rospy
from sweetie_bot_text_msgs.msg import SoundEvent, TextCommand, Detection, DetectionArray
from sweetie_bot_text_msgs.srv import CompleteSimple, CompleteSimpleRequest, CompleteSimpleResponse
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Bool, ColorRGBA
from geometry_msgs.msg import Vector3

import six
from google.cloud import translate_v2 as translate

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

    def __init__(self, on_audio, mic_name = '', buffer_rate = 10, channels=None, suppress_error=True):
        self.on_audio = on_audio
        with ignore_stderr(enable=suppress_error):
            self.pyaudio = pyaudio.PyAudio()
        self.available_channels = None
        self.channels = channels
        self.device_index = None
        self.rate = 16000
        self.sample_width = 2
        self.bit_depth = 16

        # find device
        count = self.pyaudio.get_device_count()
        rospy.logdebug("%d audio devices found" % count)
        for i in range(count):
            info = self.pyaudio.get_device_info_by_index(i)
            rospy.logdebug(info)
            name = info["name"]
            chan = info["maxInputChannels"]
            rospy.logdebug(" - %d: %s" % (i, name))
            if name.startswith(mic_name):
                self.available_channels = chan
                self.device_index = i
                rospy.loginfo("Found pyaudio device id=%d name=%s (channels: %d)" % (i, name, chan))
                break

        if self.device_index is None:
            rospy.logwarn("Failed to find pyaudio device by name (%s). Using default input" % mic_name)
            info = self.pyaudio.get_default_input_device_info()
            self.available_channels = info["maxInputChannels"]
            self.device_index = info["index"]

        if self.available_channels != 6:
            rospy.logwarn("%d channel is found for microphone" % self.available_channels)

        # whisper can only work with mono
        self.available_channels = 1

        if self.channels is None:
            self.channels = range(self.available_channels)
        else:
            self.channels = filter(lambda c: 0 <= c < self.available_channels, self.channels)
        if not self.channels:
            raise RuntimeError('Invalid channels %s. (Available channels are %s)' % (
                self.channels, self.available_channels))

        rospy.loginfo('Using channels %s' % self.channels)

        self.stream = self.pyaudio.open(
            input=True, start=False,
            format=pyaudio.paInt16,
            channels=self.available_channels,
            rate=self.rate,
            frames_per_buffer=int(self.rate/buffer_rate),
            stream_callback=self.stream_callback,
            input_device_index=self.device_index,
        )

    def __del__(self):
        self.stop()
        try:
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
        # TODO optimize
        data = np.frombuffer(in_data, dtype=np.int16)
        chunk_per_channel = len(data) // self.available_channels
        data_channels = np.reshape(data, (chunk_per_channel, self.available_channels))
        for chan in self.channels:
            chan_data = data_channels[:, chan]
            # invoke callback
            self.on_audio(chan_data.tobytes(), chan)
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
        # get parameters
        mic_name = rospy.get_param("~mic_name", "ReSpeaker 4 Mic Array (UAC1.0)")
        call_llm = rospy.get_param("~call_llm", False)
        self.enable_gtranslate = rospy.get_param("~enable_gtranslate", True)
        self.update_rate = rospy.get_param("~update_rate", 10.0)
        self.main_channel = rospy.get_param('~main_channel', 0)
        suppress_pyaudio_error = rospy.get_param("~suppress_pyaudio_error", True)
        servers = rospy.get_param("~transcribe_servers", {'0': "http://localhost:8577/"})
        self.transcribe_servers = [ servers[k] for k in sorted(servers) ]
        rospy.loginfo('urls: %s', self.transcribe_servers)
        keys_combo = rospy.get_param("~key_combination", ['ctrl', 'w'])
        self.respeaker_frame_id = rospy.get_param("~frame_id", "base_link")
        self.doa_object_distance = rospy.get_param("~doa_distance", 2.0)
        self.doa_angle_shift = rospy.get_param("~doa_angle_shift", 2.0)
        # respeaker interface
        try:
            self.respeaker = RespeakerInterface()
            params = rospy.get_param("~respeaker", {})
            for k, v in params:
                self.respeaker.write(k, v)
        except RuntimeError:
            rospy.logwarn('respeaker device is not detected')
            self.respeaker = None
        # audio interface
        self.respeaker_audio = RespeakerAudio(self.on_audio, mic_name, buffer_rate=self.update_rate, suppress_error=suppress_pyaudio_error)
        # buffers
        self.speech_audio_buffer = bytearray()
        self.sound_rms = 0.0
        self.sound_direction = 180.0
        # keyboard
        pynput_key_map = { 'ctrl': Key.ctrl, 'alt': Key.alt, 'space': Key.space, 'pause': Key.pause }
        self._keys_pressed = set()
        self._keys_combo = set()
        for key in keys_combo:
            if key in pynput_key_map:
                self._keys_combo.add( pynput_key_map[key] )
            elif len(key) == 1:
                self._keys_combo.add( keyboard.KeyCode.from_char(key) )
            else:
                raise KeyError('unknown key specification: %s' % key)
        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.keyboard_listener.start()
        rospy.loginfo("Listening for global hotkey  %s" % self._keys_combo) 
        # listen: mic button is pressed
        self._button_pressed = False
        self.sub_button_event = rospy.Subscriber("mic_button", Bool, self.on_mic_button)
        # listen: robot speech event to avoid self-listening
        self._robot_is_speeching = False
        self.sub_mouth_event = rospy.Subscriber("mouth", TextCommand, self.on_mouth)
        # service call: language model
        if call_llm:
            rospy.wait_for_service('/input_text', 5.0)
            self.llm_service_client = rospy.ServiceProxy("/input_text", CompleteSimple, persistent=True)
        else:
            self.llm_service_client = None
        # advertise
        self.pub_sound_event = rospy.Publisher("sound_event", SoundEvent , queue_size=10)
        self.pub_detections = rospy.Publisher("detections", DetectionArray, queue_size=1)
        self.pub_markers = rospy.Publisher("hmi/detections", MarkerArray, queue_size=1)
        # voice log
        self.voice_log = rospy.Publisher('voice_log', TextCommand, queue_size=1)
        # start
        self.respeaker_audio.start()
        self.info_timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate),
                                      self.on_timer)

    def on_shutdown(self):
        try:
            self.respeaker_audio.stop()
            self.keyboard_listener.stop()
        except:
            pass
        finally:
            self.respeaker_audio = None
            self.respeaker = None

    def on_mouth(self, cmd):
        # If robot is speeching everyone else must shurt up!
        # jk! we don't want the robot to talk to itself.
        if cmd.type == 'mouth/speech':
            if cmd.command == 'begin':
                self._robot_is_speeching = True
                rospy.loginfo("Robot is speaking. Disabling microphone")
                self.respeaker_audio.stop()
            else:
                self._robot_is_speeching = False
                rospy.loginfo("Robot is not speaking. Enabling microphone")
                self.respeaker_audio.start()

    def on_mic_button(self, msg):
        self._button_pressed = msg.data

    def on_key_press(self, key):
        if key in self._keys_combo:
            self._keys_pressed.add(key)

    def on_key_release(self, key):
        self._keys_pressed.discard(key)

    def is_speeching(self):
        combo_is_pressed = len(self._keys_pressed) == len(self._keys_combo)
        return (combo_is_pressed or self._button_pressed) and not self._robot_is_speeching

    def transcribe(self, data):
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
        # request transcribe server
        resp = None
        for url in self.transcribe_servers:
            rospy.logdebug("Send %d bytes to %s" % (len(data), url))
            try:
                resp = requests.post(url, files={'file': ('audio.wav', wav_data)})
                # check status
                if resp.status_code == 200:
                    break
                else:
                    rospy.logerr("%d %s" % (resp.status_code, resp.reason))
            except requests.ConnectionError as e:
                rospy.logwarn("Connection failed: %s" % e)
        # deacode server response
        if resp is None:
            rospy.logerr('Transcription failed! Cannot decode response')
            return None
        try:
            resp_decoded = resp.json()
        except:
            rospy.logerr('Transcription failed! Cannot decode response (%s)' % (resp))
            return None

        self.voice_log.publish('log/voice/in/'+resp_decoded['language'], resp_decoded['text'], '')

        if self.enable_gtranslate and resp_decoded['language'] !='en':
            resp_decoded['text'] = self.translate_text('en', resp_decoded['text'])

        rospy.loginfo('Transcription %s (%.2fs) [%s]: "%s"' % (resp_decoded['status'],
                                                               resp_decoded['transcribe_duration'],
                                                               resp_decoded['language'],
                                                               resp_decoded['text']))
        return resp_decoded

    def translate_text(self, target, text):
        """Translates text into the target language.

        Target must be an ISO 639-1 language code.
        See https://g.co/cloud/translate/v2/translate-reference#supported_languages
        """
        translate_client = translate.Client()

        if isinstance(text, six.binary_type):
            text = text.decode("utf-8")

        # Text can also be a sequence of strings, in which case this method
        # will return a sequence of results for each text.
        result = translate_client.translate(text, target_language=target)

        #print(u"Text: {}".format(result["input"]))
        #print(u"Translation: {}".format(result["translatedText"]))
        #print(u"Detected source language: {}".format(result["detectedSourceLanguage"]))

        return result["translatedText"]

    def on_audio(self, data, channel):
        if channel == self.main_channel:
            # calculate rms
            adata = np.frombuffer(data, np.int16).astype(np.float32) 
            self.sound_rms = np.sqrt(np.sum(adata**2) / len(adata)) 
            # get direction
            if self.respeaker is not None:
                self.sound_direction = self.respeaker.direction()
                if self.sound_rms > 1000:
                    print('direction: ', self.sound_direction, ', intensity: ', self.sound_rms)
            # add data to speech buffer
            if self.is_speeching():
                self.speech_audio_buffer.extend(data)

    def on_timer(self, event):
        # form speech event
        sound_event = SoundEvent()
        sound_event.header.stamp = event.current_real or rospy.Time.now()
        sound_event.header.frame_id = self.respeaker_frame_id

        # set current status
        is_speeching = self.is_speeching();
        if is_speeching:
            sound_event.sound_flags |= SoundEvent.SPEECH_DETECTING | SoundEvent.SOUND_DETECTING

        # add intensity and direction
        sound_event.intensity = self.sound_rms
        doa_rad = math.radians(self.sound_direction - 180.0)
        sound_event.direction.x = np.cos(doa_rad)
        sound_event.direction.y = np.sin(doa_rad)
        sound_event.direction.z = 0.0
       
        # decode speech
        if not self.is_speeching() and len(self.speech_audio_buffer) != 0:
            # new speech fragment is received
            result = self.transcribe(self.speech_audio_buffer)
            if result is not None:
                sound_event.sound_flags |= SoundEvent.SPEECH_DECODED
                sound_event.text = result['text']
                sound_event.language = result['language']
            # clear buffer
            self.speech_audio_buffer.clear()

        # publish result
        self.pub_sound_event.publish(sound_event)

        # publush object detection if speeching
        if is_speeching:
            # create detection object
            detection = Detection()
            detection.header = sound_event.header
            detection.id = 0
            detection.label = 'speech'
            detection.type = 'sound'
            detection.pose.position.x = sound_event.direction.x * self.doa_object_distance
            detection.pose.position.y = sound_event.direction.y * self.doa_object_distance
            detection.pose.position.z = sound_event.direction.z * self.doa_object_distance
            # publish it
            detections = DetectionArray()
            detections.detections.append(detection)
            self.pub_detections.publish(detections)
            # create visualization marker
            marker = Marker()
            marker.header = sound_event.header
            marker.ns = 'microphone'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = detection.pose
            marker.scale = Vector3(0.3, 0.3, 0.3)
            marker.lifetime = rospy.Duration(1.0)
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.5)
            # publish it
            markers = MarkerArray()
            markers.markers.append(marker)
            self.pub_markers.publish(markers)

        # pass text to language model: new speech is decoded and Complete service is available
        if sound_event.sound_flags & SoundEvent.SPEECH_DECODED and self.llm_service_client is not None:
            try:
                req = CompleteSimpleRequest()
                req.data.type = 'complete/request'
                req.data.command = sound_event.text
                req.data.options = sound_event.language
                start = time.time()
                ret = self.llm_service_client(req)
                duration = time.time() - start
                rospy.loginfo("Complete ok (%.2f): %s" % (duration, ret.data.command))
            except rospy.ServiceException as exc:
                rospy.logerr("Service did not process request: " + str(exc))

def main():
    rospy.init_node("respeaker_node")
    n = RespeakerNode()
    rospy.spin()
