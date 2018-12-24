#!/usr/bin/env python
# -*- coding: utf-8 -*-

import string
import rospy, roslib, rospkg, os, sys, random
from sound_play.libsoundplay import SoundClient

from sweetie_bot_text_msgs.msg import TextCommand

soundhandle = SoundClient()
sounds = {}
playback_command = None

def commandCallback(cmd):
    global soundhandle, sounds;
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
        # use voice synthesis
        snd = soundhandle.voiceSound(cmd.command)
        snd.play()


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
    global soundhandle, sounds, playback_command
    rospy.init_node('voice', anonymous = True)
    rospy.Subscriber('control', TextCommand, commandCallback)

    # Playback configuration
    playback_command = rospy.get_param('~playback_command', None) 
    if playback_command != None and not isinstance(playback_command, str):
        rospy.logerr('"playback_command" parameter must be a string.')
        sys.exit(-1)

    # Get language settings
    lang_prefixes = rospy.get_param('lang', 'ru,en')
    if not isinstance(lang_prefixes, str):
        rospy.logerr('"lang_prefixes" parameter must be a string.')
        sys.exit(-1)

    lang_prefixes = string.split(lang_prefixes, ',')
    rospy.loginfo('Sound prefixes: ' + repr(lang_prefixes))
   
    # Get sounds location
    sound_packages = rospy.get_param('~sound_packages', [])
    if not isinstance(sound_packages, list):
        rospy.logerr('"sound_packages" parameter must contain a list of packages\' names.')
        sys.exit(-1)

    sound_packages.append('sweetie_bot_voice')
    rospy.loginfo('Sound packages: ' + repr(sound_packages))

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

    rospy.sleep(1)
    soundhandle.stopAll()

    rospy.spin()
