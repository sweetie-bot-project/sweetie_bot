#!/usr/bin/env python3
import os
import sys
from contextlib import contextmanager
import pyaudio
from pynput import keyboard
import requests
import numpy as np
import struct

import rospy
from sweetie_bot_text_msgs.msg import SoundEvent
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
        rospy.logdebug("%d audio devices found" % count)
        for i in range(count):
            info = self.pyaudio.get_device_info_by_index(i)
            rospy.logdebug(info)
            name = info["name"]
            chan = info["maxInputChannels"]
            rospy.logdebug(" - %d: %s" % (i, name))
            if name.startswith(self.mic_name):
                self.available_channels = chan
                self.device_index = i
                rospy.loginfo("Found pyaudio device id=%d name=%s (channels: %d)" % (i, name, chan))
                break

        if self.device_index is None:
            rospy.logwarn("Failed to find pyaudio device by name (%s). Using default input" % self.mic_name)
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

    # The key combination to check
    COMBINATION = {keyboard.Key.ctrl, keyboard.Key.alt, keyboard.KeyCode.from_char('w')}
    # The currently active modifiers
    current = set()

    def __init__(self):
        # shutdown hook
        rospy.on_shutdown(self.on_shutdown)
        # get parameters
        self.update_rate = rospy.get_param("~update_rate", 10.0)
        self.main_channel = rospy.get_param('~main_channel', 0)
        suppress_pyaudio_error = rospy.get_param("~suppress_pyaudio_error", True)
        self.transcribe_servers = rospy.get_param("~transcribe_servers", {'0': "http://localhost:8577/"})
        # audio interface
        self.respeaker_audio = RespeakerAudio(self.on_audio, suppress_error=suppress_pyaudio_error)
        self.speech_audio_buffer = bytearray()
        self._is_speeching = False
        self._robot_is_speeching = False
        # keyboard
        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.keyboard_listener.start()
        rospy.loginfo("Listening for global hotkey  %s" % self.COMBINATION) #, key=lambda item: item[1]))) # sorted(self.COMBINATION))
        # listen
        self.sub_button_event = rospy.Subscriber("mic_button", Bool, self.on_mic_button)
        self.sub_robot_is_speeching_event = rospy.Subscriber("robot_is_speeching", Bool, self.on_robot_is_speeching)
        # advertise
        self.pub_sound_event = rospy.Publisher("sound_event", SoundEvent , queue_size=10)
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

    def on_robot_is_speeching(self, msg):
        # If robot is speeching everyone else must shurt up!
        # jk! we don't want the robot to talk to itself.
        self._robot_is_speeching = msg.data
        if msg.data:
            rospy.loginfo("Robot is speeching. Disabling microphone")
            self.respeaker_audio.stop()
        else:
            rospy.loginfo("Robot is not speeching. Enabling microphone")
            self.respeaker_audio.start()

    def on_mic_button(self, msg):
        self._is_speeching = msg.data

    def on_key_press(self, key):
        if key in self.COMBINATION:
            self.current.add(key)
            if all(k in self.current for k in self.COMBINATION):
                self._is_speeching = True

    def on_key_release(self, key):
        if key in self.current:
            try:
                self.current.remove(key)
            except KeyError:
                pass
            if self._is_speeching and not all(k in self.current for k in self.COMBINATION):
                self._is_speeching = False

    def is_speeching(self):
        return self._is_speeching

    def transcribe(self, data):
        resp = None
        urls = self.transcribe_servers
        for n in sorted(urls):
            rospy.logdebug("Send %d bytes to %s" % (len(data), urls[n]))

            header = struct.pack('<4sL4s4sLHHLLHH4sL', b'RIFF', 
                    36 + len(data), b'WAVE', b'fmt ', 16,
                    1, # no compression
                    1, # n channels
                    self.respeaker_audio.rate, # framerate,
                    1 * self.respeaker_audio.rate * self.respeaker_audio.sample_width,
                    1 * self.respeaker_audio.sample_width,
                    self.respeaker_audio.sample_width * 8, 
                    b'data', len(data))

            try:
                resp = requests.post(urls[n], files={'file': ('audio.wav', header + data)})
                # check s
                if resp.status_code == 200:
                    break
                else:
                    rospy.logerr("%d %s" % (resp.status_code, resp.reason))
                    continue # next url
            except requests.ConnectionError as e:
                rospy.logwarn("Connection failed! %s" % e)

        if resp is None:
            rospy.logerr('Transcription failed! Cannot decode response')
            return None

        try:
            resp_decoded = resp.json()
        except:
            rospy.logerr('Transcription failed! Cannot decode response (%s)' % (resp))
            return None

        rospy.loginfo('Transcription %s (%.2fs) [%s]: "%s"' % (resp_decoded['status'],
                                                               resp_decoded['transcribe_duration'],
                                                               resp_decoded['language'],
                                                               resp_decoded['text']))
        return resp_decoded
    
    def on_audio(self, data, channel):
        if channel == self.main_channel:
            # store speech data
            if self.is_speeching():
                self.speech_audio_buffer.extend(data)

    def on_timer(self, event):
        # form speech event
        sound_event = SoundEvent()
        sound_event.header.stamp = event.current_real or rospy.Time.now()

        # set current status
        is_speeching = self.is_speeching();
        if is_speeching:
            sound_event.sound_flags |= SoundEvent.SPEECH_DETECTING | SoundEvent.SOUND_DETECTING
       
        # decode speech
        if not self.is_speeching() and len(self.speech_audio_buffer) != 0:
            # new speech fragment is received
            result = self.transcribe(self.speech_audio_buffer)
            if result is not None:
                sound_event.sound_flags |= SoundEvent.SPEECH_DECODED
                sound_event.text = result['text']
                sound_event.language = result['language']
            else:
                sound_event.text = ''
            # clear buffer
            self.speech_audio_buffer.clear()

        # publish result
        self.pub_sound_event.publish(sound_event)


def main():
    rospy.init_node("respeaker_node")
    n = RespeakerNode()
    rospy.spin()
