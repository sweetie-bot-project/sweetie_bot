#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, roslib, rospkg, os, sys, random
from sound_play.msg import SoundRequest
from std_msgs.msg import String

from sound_play.libsoundplay import SoundClient

soundhandle = SoundClient()
path = '';
sounds = [];

def sleep(t):
    try:
        rospy.sleep(t)
    except:
        pass

def chatterCallback(data):
    global soundhandle, path, sounds;
    rospy.loginfo(rospy.get_caller_id() + " [%s]", data.data)
    rospy.loginfo(path)
    snd = soundhandle.waveSound(path + data.data  + '.wav')
    snd.play()
    #rate.sleep()

def main():
    global soundhandle, path, sounds;
    rospy.init_node('voice', anonymous = True)
    rospy.Subscriber("voice", String, chatterCallback)

    rospack = rospkg.RosPack()
    path = rospack.get_path('sweetie_bot_voice') + '/sounds/'
    rospy.loginfo(path)

    sounds = rospy.get_param('~sounds', '[]')
    rospy.loginfo(sounds)

    rospy.sleep(1)
    
    soundhandle.stopAll()

    rospy.spin()
