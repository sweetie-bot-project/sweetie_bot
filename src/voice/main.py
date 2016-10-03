#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, roslib, rospkg, os, sys, random
from sound_play.msg import SoundRequest

from sound_play.libsoundplay import SoundClient

def sleep(t):
    try:
        rospy.sleep(t)
    except:
        pass

def main():
    rospy.init_node('voice', anonymous = True)

    rospack = rospkg.RosPack()
    path = rospack.get_path('sweetie_bot_voice') + '/sounds/'
    rospy.loginfo(path)

    sounds = rospy.get_param('~sounds', '[]')
    rospy.loginfo(sounds)

    soundhandle = SoundClient()

    rospy.sleep(1)
    
    soundhandle.stopAll()

    rate = rospy.Rate(0.05) # 10hz
    while not rospy.is_shutdown():
	choice = random.choice(sounds)
	rospy.loginfo(choice)
	s2 = soundhandle.waveSound(path + choice  + '.wav')
	rospy.loginfo("play")
	s2.play()
	rospy.loginfo("stop")
	rate.sleep()


    #print "This script will run continuously until you hit CTRL+C, testing various sound_node sound types."

    #print
    #print 'Try to play wave files that do not exist.'
    #soundhandle.playWave('17')
    #soundhandle.playWave('dummy')
        
    #print 'say'
    #soundhandle.say('Hello world!')
    #sleep(3)
    #    
    #print 'wave'
    #soundhandle.playWave(path + random.choice(sounds) + '.wav')
    #sleep(2)
        
    #print 'plugging'
    #soundhandle.play(SoundRequest.NEEDS_PLUGGING)
    #sleep(2)

    #print 'unplugging'
    #soundhandle.play(SoundRequest.NEEDS_UNPLUGGING)
    #sleep(2)

    #print 'plugging badly'
    #soundhandle.play(SoundRequest.NEEDS_PLUGGING_BADLY)
    #sleep(2)

    #print 'unplugging badly'
    #soundhandle.play(SoundRequest.NEEDS_UNPLUGGING_BADLY)
    #sleep(2)

    #s1 = soundhandle.builtinSound(SoundRequest.NEEDS_UNPLUGGING_BADLY)
    #s2 = soundhandle.waveSound(path + random.choice(sounds) + '.wav')
    #s3 = soundhandle.voiceSound("Testing the new A P I")

    #print "New API start voice"
    #s3.repeat()
    #sleep(3)

    #print "New API wave"
    #s2.play()
    #sleep(2)

    #print "New API builtin"
    #s1.play()
    #sleep(2)

    #print "New API stop"
    #s3.stop()
