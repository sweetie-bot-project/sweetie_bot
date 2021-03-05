#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, roslib, rospkg, os, sys, random
from sound_play.libsoundplay import SoundClient

from sweetie_bot_text_msgs.msg import TextCommand

from rhvoice_wrapper import TTS

import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')

from gi.repository import Gst

Gst.init(None)

soundhandle = SoundClient()
sounds = {}
playback_command = None
tts_backend = None

rhvoice_relative_rate = 1.0
rhvoice_relative_volume = 1.0
gstreamer_pipeline = None
gstreamer_src = None

def commandCallback(cmd):
    global soundhandle, sounds, playback_command, tts_backend
    global gstreamer_pipeline, gstreamer_src, rhvoice_relative_rate, rhvoice_relative_volume
    # check command type
    if cmd.type == 'voice/play_wav':
        # play specified sound file
        if cmd.command in sounds:
            filename = sounds[cmd.command]
            rospy.loginfo('Playing sound: {0} ({1}).'.format(cmd.command, filename))
            # playback selection
            if not playback_command:
                # use sound client
                snd = soundhandle.waveSound(filename)
                snd.play()
            else:
                # use system command
                os.system(playback_command + " " + filename)
        else:
            rospy.logerr('Unknown play_wav sound: ' + cmd.command)

    elif cmd.type == 'voice/say':
        # invoke text-to-speech subsystem
        if tts_backend == 'sound_play':
            # pass request to sound play server
            snd = soundhandle.voiceSound(cmd.command)
            snd.play()
        # Use RHVoice synthesizer
        elif tts_backend == 'rhvoice' or tts_backend == 'rhvoice_robotized':
            Gst.Event.new_flush_start()

            # Get synthesized PCM sound from RHVoice server
            rhvoice_tts = TTS(threads=2)
            rhvoice_tts.set_params(voice_profile="anna+clb", relative_rate=rhvoice_relative_rate, relative_volume=rhvoice_relative_volume)
            raw_sound = rhvoice_tts.get(cmd.command, format_='pcm')

            Gst.Event.new_flush_stop(True)

            # @Note: Pipeline must be resetted exactly like this. Otherwise it will not flush
            gstreamer_pipeline.set_state(Gst.State.PAUSED)
            gstreamer_pipeline.set_state(Gst.State.PLAYING)
            
            # Allocate new buffer
            buf = Gst.Buffer.new_allocate(None, len(raw_sound), None)
            buf.fill(0, raw_sound)
            # Push buffer on appsrc
            gst_flow_return = gstreamer_src.emit('push-buffer', buf)

            if gst_flow_return != Gst.FlowReturn.OK:
                rospy.logerr('Gstreamer error. Failed to process synthesized voice')


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
    global soundhandle, sounds, playback_command, tts_backend
    global gstreamer_pipeline, gstreamer_src, rhvoice_relative_volume, rhvoice_relative_rate
    rospy.init_node('voice', anonymous = True)
    rospy.Subscriber('control', TextCommand, commandCallback)

    # Playback configuration
    playback_command = rospy.get_param('~playback_command', None) 
    if playback_command != None and not isinstance(playback_command, str):
        rospy.logerr('"playback_command" parameter must be a string.')
        sys.exit(2)
    
    # tts backend
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

    # Get path to java for robotize plugin
    java_home = rospy.get_param('~java_home', "/lib/jvm/java-8-openjdk-amd64/")
    if not isinstance(java_home, str):
        rospy.logerr('"java_home" parameter must be a string')
        sys.exit(1)

    # Set mandatory enviroment variables for used gstreamer plugins
    os.environ['LADSPA_PATH'] = ladspa_path
    os.environ['JAVA_HOME'] = java_home

    # Apply some dirty fix to make sure gstreamer will work
    os.system("rm -f ~/.cache/gstreamer-1.0/registry.x86_64.bin")

    # Initialize gstreamer
    if tts_backend == 'rhvoice' or tts_backend == 'rhvoice_robotized':
        rhvoice_relative_volume = rospy.get_param('~relative_volume', 1.0) 
        if not isinstance(rhvoice_relative_volume, float):
            rospy.logerr('"relative_volume" parameter must be a float')
            sys.exit(2)

        rhvoice_relative_rate = rospy.get_param('~relative_rate', 1.0) 
        if not isinstance(rhvoice_relative_rate, float):
            rospy.logerr('"rhvoice_relative_rate" parameter must be a float')
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

        gstreamer_pipeline = Gst.parse_launch(auto_audio_sink_pipeline)
        gstreamer_src = gstreamer_pipeline.get_by_name('source')
        gstreamer_pipeline.set_state(Gst.State.PLAYING)

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

    rospy.logdebug('Registered sounds: ' + repr(sounds))
    rospy.loginfo('Registered %d sounds.' % len(sounds))
    rospy.loginfo('TTS backend: %s.' % tts_backend)

    rospy.sleep(1)
    soundhandle.stopAll()

    rospy.spin()
