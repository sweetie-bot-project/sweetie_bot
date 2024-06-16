#!/usr/bin/env python3
import os
import sys
import time
from contextlib import contextmanager
import pyaudio
from pynput import keyboard
from pynput.keyboard import Key
import numpy as np
import struct

import rospy
from sweetie_bot_text_msgs.msg import SoundEvent, TextCommand
from sweetie_bot_text_msgs.srv import Transcribe, TranscribeRequest, TranscribeResponse
from std_msgs.msg import Bool

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

class RespeakerAudio(object):

    def __init__(self, on_audio, channels=None, suppress_error=True):
        self.on_audio = on_audio
        self.mic_name = rospy.get_param("~mic_name", "ReSpeaker 4 Mic Array (UAC1.0)")
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
        rospy.logdebug(f"{count} audio devices found")
        for i in range(count):
            info = self.pyaudio.get_device_info_by_index(i)
            rospy.logdebug(info)
            name = info["name"]
            chan = info["maxInputChannels"]
            rospy.logdebug(f" - {i}: {name}")
            if name.startswith(self.mic_name):
                self.available_channels = chan
                self.device_index = i
                rospy.loginfo(f"Found pyaudio device id={i} name={name} (channels: {chan})")
                break

        if self.device_index is None:
            rospy.logwarn(f"Failed to find pyaudio device by name ({self.mic_name}). Using default input")
            info = self.pyaudio.get_default_input_device_info()
            self.available_channels = info["maxInputChannels"]
            self.device_index = info["index"]

        if self.available_channels != 6:
            rospy.logwarn(f"{self.available_channels} channel is found for microphone")

        # whisper can only work with mono
        self.available_channels = 1

        if self.channels is None:
            self.channels = range(self.available_channels)
        else:
            self.channels = filter(lambda c: 0 <= c < self.available_channels, self.channels)
        if not self.channels:
            raise RuntimeError(f"Invalid channels {self.channels}. (Available channels are {self.available_channels})")

        rospy.loginfo(f"Using channels {self.channels}")

        self.stream = self.pyaudio.open(
            input=True, start=False,
            format=pyaudio.paInt16,
            channels=self.available_channels,
            rate=self.rate,
            frames_per_buffer=1024,
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
        self.update_rate = rospy.get_param("~update_rate", 10.0)
        self.main_channel = rospy.get_param('~main_channel', 0)
        suppress_pyaudio_error = rospy.get_param("~suppress_pyaudio_error", False)
        keys_combo = rospy.get_param("~key_combination", ['ctrl', 'alt','w'])
        # audio interface
        self.respeaker_audio = RespeakerAudio(self.on_audio, suppress_error=suppress_pyaudio_error)
        self.speech_audio_buffer = bytearray()
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
                raise KeyError(f"unknown key specification: {key}")
        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.keyboard_listener.start()
        rospy.loginfo(f"Listening for global hotkey  {self._keys_combo}")
        # listen: mic button is pressed
        self._button_pressed = False
        self.sub_button_event = rospy.Subscriber("mic_button", Bool, self.on_mic_button)
        # define transcribe service node
        rospy.wait_for_service('/transcribe_api', 5.0)
        self.transcribe_service_client = rospy.ServiceProxy("/transcribe_api", Transcribe, persistent=True)
        # listen: robot speech event to avoid self-listening
        self._robot_is_speaking = False
        self.sub_mouth_event = rospy.Subscriber("mouth", TextCommand, self.on_mouth)
        # advertise
        self.pub_sound_event = rospy.Publisher("sound_event", SoundEvent, queue_size=10)
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

    def on_key_press(self, key):
        if key in self._keys_combo:
            self._keys_pressed.add(key)

    def on_key_release(self, key):
        self._keys_pressed.discard(key)

    def is_speaking(self):
        combo_is_pressed = len(self._keys_pressed) == len(self._keys_combo)
        return (combo_is_pressed or self._button_pressed) and not self._robot_is_speaking

    def on_audio(self, data, channel):
        if channel == self.main_channel:
            # store speech data
            if self.is_speaking():
                self.speech_audio_buffer.extend(data)

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

    def on_timer(self, event):
        # form speech event
        sound_event = SoundEvent()
        sound_event.header.stamp = event.current_real or rospy.Time.now()

        # set current status
        is_speaking = self.is_speaking();
        if is_speaking:
            sound_event.sound_flags |= SoundEvent.SPEECH_DETECTING | SoundEvent.SOUND_DETECTING
       
        # decode speech
        if not self.is_speaking() and len(self.speech_audio_buffer) != 0:
            # new speech fragment is received
            transcribe_req = TranscribeRequest(
                data=self.buf_to_wav(self.speech_audio_buffer),
            )
            result = self.transcribe_service_client(transcribe_req)
            if result is not None:
                sound_event.sound_flags |= SoundEvent.SPEECH_DECODED
                sound_event.text = result.text
                sound_event.language = result.language
            # clear buffer
            self.speech_audio_buffer.clear()

        # publish result
        self.pub_sound_event.publish(sound_event)

def main():
    rospy.init_node("microphone_node")
    n = RespeakerNode()
    rospy.spin()
