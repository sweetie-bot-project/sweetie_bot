#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, actionlib, roslib, rospkg, os, sys, random
from sound_play.libsoundplay import SoundClient

from sweetie_bot_text_msgs.msg import TextCommand
from sweetie_bot_text_msgs.msg import TextActionAction as TextAction
from sweetie_bot_text_msgs.msg import TextActionGoal, TextActionFeedback, TextActionResult

from rhvoice_wrapper import TTS

import gi

gi.require_version('Gst', '1.0')

from gi.repository import Gst


class TextCommandHandler:
    ''' 
        Handling TextCommand, which provide no way to detect when sound stop playing
    '''
    def __init__(self, sounds, playback_command, tts_backend = 'sound_play', voice_synthesizer = None):
        self.soundhandle = SoundClient()
        self.sounds = sounds
        self.playback_command = playback_command

        self.tts_backend = tts_backend
        self.voice_synthesizer = voice_synthesizer


    def command_callback(self, cmd):
        # Check command type
        if cmd.type == 'voice/play_wav':
            # Play specified sound file
            if cmd.command in self.sounds:
                filename = self.sounds[cmd.command]
                rospy.loginfo('Playing sound: {0} ({1}).'.format(cmd.command, filename))
                # Playback selection
                if not self.playback_command:
                    # Use sound client
                    snd = self.soundhandle.waveSound(filename)
                    snd.play()
                else:
                    # Use system command
                    os.system(self.playback_command + ' ' + filename)
            else:
                rospy.logerr('Unknown play_wav sound: ' + cmd.command)

        elif cmd.type == 'voice/say':
            # Invoke text-to-speech subsystem
            if self.tts_backend == 'sound_play':
                # Pass request to sound play server
                snd = self.soundhandle.voiceSound(cmd.command)
                snd.play()
            # Use RHVoice synthesizer
            elif self.tts_backend == 'rhvoice' or self.tts_backend == 'rhvoice_robotized':
                self.voice_synthesizer.generate_and_play(cmd.command)


class VoiceSynthesizer:
    ENGLISH_ALPHABET = 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNUPQRSTUVWXYZ'
    RUSSIAN_ALPHABET = 'абвгдеёжзийклмнопрстуфхцчшщьыъэюяАВБГДЕЁЖЗИЙКЛМНОПРСТУФХЦЧШЩЬЫЪЭЮЯ'

    ''' 
        Interfacing with RHVoice synthesizer and gstreamer pipeline providing desired sound of speech
    '''
    def __init__(self, gstreamer_pipeline_string, relative_volume = 1.0, relative_rate = 1.0, relative_pitch_russian = 1.0, relative_pitch_english = 1.0):
        self.relative_volume = relative_volume
        self.relative_rate   = relative_rate

        self.relative_pitch_russian = relative_pitch_russian
        self.relative_pitch_english = relative_pitch_english

        self.gstreamer_pipeline = Gst.parse_launch(gstreamer_pipeline_string)
        self.gstreamer_src = self.gstreamer_pipeline.get_by_name('source')

        # @Note: Important, as we will produce timestamped buffers
        self.gstreamer_src.set_property('format', Gst.Format.TIME)

        self.gstreamer_pipeline.set_state(Gst.State.PLAYING)

        self.rhvoice_tts = TTS(threads=2)

        self.rhvoice_tts.set_params(voice_profile='Anna+CLB', relative_rate=self.relative_rate, relative_volume=self.relative_volume)

    def generate_and_play(self, command_string):
        Gst.Event.new_flush_start()

        # TODO: Think about if language should be specified manually
        # Detect what language is used simply by the first letter
        spaceless_command_string = ' '.join(command_string.split())
        used_language = 'russian'
        for character in spaceless_command_string:
            if character in self.RUSSIAN_ALPHABET:
                used_language = 'russian'
                break
            elif character in self.ENGLISH_ALPHABET:
                used_language = 'english'
                break

        # Choose pitch correction depending on the language
        if used_language == 'russian':
            self.rhvoice_tts.set_params(relative_pitch=self.relative_pitch_russian)
        else:
            self.rhvoice_tts.set_params(relative_pitch=self.relative_pitch_english)

        # Get synthesized PCM sound from RHVoice server
        raw_sound = self.rhvoice_tts.get(command_string, format_='pcm')

        Gst.Event.new_flush_stop(True)

        # @Note: Pipeline must be restarted exactly like this. Otherwise it will not push buffer after EOS signal
        self.gstreamer_pipeline.set_state(Gst.State.READY)
        self.gstreamer_pipeline.set_state(Gst.State.PLAYING)

        # Allocate new buffer
        buf = Gst.Buffer.new_allocate(None, len(raw_sound), None)
        buf.fill(0, raw_sound)

        # We cannot know duration of generated audio from rhvoice wrapper API
        buf.duration = Gst.CLOCK_TIME_NONE
        buf.pts = Gst.CLOCK_TIME_NONE

        # Push buffer on appsrc
        gst_flow_return = self.gstreamer_src.emit('push-buffer', buf)

        # Propagate End Of Stream event downstream to the sink
        self.gstreamer_src.emit('end-of-stream')

        if gst_flow_return != Gst.FlowReturn.OK:
            rospy.logerr('Gstreamer error. Failed to process synthesized voice')


class TextActionServer:
    ''' 
        Handling TextAction, which detects end time of a sound (useful, for example, for FlexBe states)
    '''
    feedback = TextActionFeedback()
    result = TextActionResult()

    def __init__(self, name, voice_synthesizer):
        self.voice_synthesizer = voice_synthesizer

        self.action_name = name
        self.action_server = actionlib.SimpleActionServer(self.action_name, TextAction, execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()


    def execute_cb(self, goal):
        # @Note: Currently TextAction provide only voice/say message type for rhvoice synthesis
        if goal.command.type != 'voice/say':
            rospy.logerr('{}: unsupported command type for TextAction'.format(goal.command.type))
        else:

            action_finished = True

            # @Hack: Not very helpful status message at all
            self.feedback.status = 'speaking'

            bus = self.voice_synthesizer.gstreamer_pipeline.get_bus()

            # Discard all previous EOS messages on the pipeline's bus
            while bus.pop_filtered(Gst.MessageType.EOS) is not None:
                pass

            self.voice_synthesizer.generate_and_play(goal.command.command)

            while True:
                if self.action_server.is_preempt_requested():
                    rospy.loginfo('{}: Preempted'.format(self.action_name))
                    self.action_server.set_preempted()
                    action_finished = False
                    break

                self.action_server.publish_feedback(self.feedback)

                # Wait for the end of stream
                result = bus.poll(Gst.MessageType.EOS, 10 * Gst.MSECOND)
                if result is not None:
                    break

            if action_finished:
                self.result.error_code = 0
                self.result.error_string = ''

                rospy.loginfo('{}: Succeeded'.format(self.action_name))
                self.action_server.set_succeeded(self.result)


def file_dict(directory, ext):
    ''' Return list of files with given extension.

        Paramters:
            directory (string) --- directory name,
            ext (string) --- file extension,

        Return: list of strings.
    '''
    files = {} 
    try:
        for f in os.listdir(directory):
            basename, extension = os.path.splitext(f)
            if extension.endswith(ext):
                files[basename] = os.path.join(directory, f)
    except OSError as e:
        rospy.logwarn('Unable to list `%s` directory.' % directory)
        return []
    return files


def main():
    rospy.init_node('voice', anonymous = True)

    playback_command = rospy.get_param('~playback_command', None) 
    if playback_command != None and not isinstance(playback_command, str):
        rospy.logerr('"playback_command" parameter must be a string.')
        sys.exit(2)
    
    # TTS backend
    tts_backend = rospy.get_param('~tts_backend', 'sound_play') 
    if not isinstance(tts_backend, str) or tts_backend not in ('sound_play', 'rhvoice', 'rhvoice_robotized'):
        rospy.logerr('"tts_backend" parameter must be a "sound_play", "rhvoice", "rhvoice_robotized".')
        sys.exit(2)

    # Get language settings
    lang_prefixes = rospy.get_param('lang', 'ru,en')
    if not isinstance(lang_prefixes, str):
        rospy.logerr('"lang_prefixes" parameter must be a string.')
        sys.exit(2)

    lang_prefixes = str.split(lang_prefixes, ',')
    rospy.loginfo('Sound prefixes: ' + repr(lang_prefixes))
   
    # Get sounds location
    sound_packages = rospy.get_param('~sound_packages', [])
    if not isinstance(sound_packages, list):
        rospy.logerr('"sound_packages" parameter must contain a list of packages\' names.')
        sys.exit(2)

    sound_packages.append('sweetie_bot_voice')
    rospy.loginfo('Sound packages: ' + repr(sound_packages))

    # Get path to ladspa plugins directory
    ladspa_path = rospy.get_param('~ladspa_path', "~/.ladspa/")
    if not isinstance(ladspa_path, str):
        rospy.logerr('"ladspa_path" parameter must be a string')
        sys.exit(1)

    # Set mandatory enviroment variables for used gstreamer plugins
    os.environ['LADSPA_PATH'] = ladspa_path

    # Apply some dirty fix to make sure gstreamer will work
    os.system("rm -f ~/.cache/gstreamer-1.0/registry.x86_64.bin")

    # Initialize gstreamer
    voice_synthesizer = None
    if tts_backend == 'rhvoice' or tts_backend == 'rhvoice_robotized':
        Gst.init(None)

        rhvoice_relative_volume = rospy.get_param('~relative_volume', 1.0) 
        if not isinstance(rhvoice_relative_volume, float):
            rospy.logerr('"relative_volume" parameter must be a float')
            sys.exit(2)

        rhvoice_relative_rate = rospy.get_param('~relative_rate', 1.0) 
        if not isinstance(rhvoice_relative_rate, float):
            rospy.logerr('"rhvoice_relative_rate" parameter must be a float')
            sys.exit(2)

        rhvoice_relative_pitch_russian = rospy.get_param('~relative_pitch_russian', 1.0) 
        if not isinstance(rhvoice_relative_pitch_russian, float):
            rospy.logerr('"rhvoice_relative_pitch_russian" parameter must be a float')
            sys.exit(2)

        rhvoice_relative_pitch_english = rospy.get_param('~relative_pitch_english', 1.0) 
        if not isinstance(rhvoice_relative_pitch_english, float):
            rospy.logerr('"rhvoice_relative_pitch_english" parameter must be a float')
            sys.exit(2)

        pipeline_beginning = 'appsrc name=source ! audio/x-raw,format=S16LE,channels=1,rate=24000,layout=interleaved !'

        if tts_backend == 'rhvoice':
            auto_audio_sink_pipeline = pipeline_beginning + 'autoaudiosink'
        elif tts_backend == 'rhvoice_robotized':
            robotization_pipeline = rospy.get_param('~robotization_pipeline', 'autoaudiosink')
            if not isinstance(robotization_pipeline, str):
                rospy.logerr('"robotization_pipeline" parameter must be a string')
                sys.exit(2)

            auto_audio_sink_pipeline = pipeline_beginning + robotization_pipeline

        voice_synthesizer = VoiceSynthesizer(auto_audio_sink_pipeline, rhvoice_relative_volume, rhvoice_relative_rate, rhvoice_relative_pitch_russian, rhvoice_relative_pitch_english)

        TextActionServer('syn', voice_synthesizer)

    # List sound files
    sounds = {}
    rospack = rospkg.RosPack()
    for pkg in sound_packages:
        try:
            sounds_path = os.path.join(rospack.get_path(pkg), 'sounds')
        except rospkg.ResourceNotFound:
            rospy.logerr('Sound package `%s` is not found.' % pkg)
            continue

        for prefix in lang_prefixes:
            path = os.path.join(sounds_path, prefix)
   
            sounds.update(file_dict(path, 'wav'))
            sounds.update(file_dict(path, 'ogg'))

    # Initialize TextCommand handler
    text_command_subscriber = TextCommandHandler(sounds, playback_command, tts_backend, voice_synthesizer)
    rospy.Subscriber('control', TextCommand, text_command_subscriber.command_callback)

    rospy.loginfo('Registered sounds: ' + repr(sounds))
    rospy.loginfo('Registered %d sounds.' % len(sounds))
    rospy.loginfo('TTS backend: %s.' % tts_backend)

    rospy.sleep(1)
    text_command_subscriber.soundhandle.stopAll()

    rospy.spin()
