#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, roslib, rospkg, os, sys, random
from sound_play.libsoundplay import SoundClient

from std_msgs.msg import String
from sweetie_bot_text_msgs.msg import TextCommand

soundhandle = SoundClient()
sounds = {}

def commandCallback(cmd):
    global soundhandle, sounds;
    if cmd.type == 'voice/play_wav':
        if cmd.command in sounds:
            filename = sounds[cmd.command]
            rospy.loginfo('Playing sound: {0} ({1}).'.format(cmd.command, filename))
            snd = soundhandle.waveSound(filename)
            snd.play()
        else:
            rospy.logerr('Unknown play_wav sound: ' + cmd.command)


def file_dict(directory, ext):
    files = {} 
    try:
        for f in os.listdir(directory):
            basename, extension = os.path.splitext(f)
            print basename, ' ', extension
            if extension.endswith(ext):
                files[basename] = os.path.join(directory, f)
    except OSError as e:
        rospy.logerr('Unable to list `%s` directory.' % directory)
        return []
    return files


def main():
    global soundhandle, sounds;
    rospy.init_node('voice', anonymous = True)
    rospy.Subscriber('control', TextCommand, commandCallback)

    lang_prefix = rospy.get_param('lang', 'ru')

    rospack = rospkg.RosPack()
    path = os.path.join(rospack.get_path('sweetie_bot_voice'), 'sounds' , lang_prefix)
    rospy.loginfo("Sound files path: " + path)

    sounds = file_dict(path, 'wav')
    rospy.loginfo('Found sounds:\n' + repr(sounds))

    rospy.sleep(1)
    
    soundhandle.stopAll()

    rospy.spin()
