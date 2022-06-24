import rospy

from sweetie_bot_text_msgs.msg import TextCommand
from sensor_msgs.msg import JointState
from led_msgs.srv import SetLEDs, SetLED
from threading import Timer
import time
import random

led_choice = [0, 128, 255]

class RepeatTimer(Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)


class BodyLanguageNode():
    def __init__(self):
        rospy.init_node('body_language', anonymous = True)
        self.mouth_timer = RepeatTimer(0.5, self.dummyfn)

        # register ROS interface
        rospy.Subscriber('mouth', TextCommand, self.mouth_cb)
        rospy.Subscriber('control', TextCommand, self.control_cb)
        out_topic = '/motion/controller/joint_state/out_joints_src_reset'
        out_topic = '/motion/controller/joint_state/in_joints_ref'
        self.pub = rospy.Publisher(out_topic, JointState, queue_size=1)
        self.horn = rospy.ServiceProxy('/led/set_leds', SetLEDs)
        self.msg = JointState()

    def dummyfn(self, m=0.32):
        self.msg.name = ['mouth_joint']
        self.msg.position = [-random.uniform(0.2,m)]
        self.pub.publish(self.msg)
        rospy.loginfo(self.msg)

    def control_cb(self, cmd):
        #rospy.loginfo(cmd)
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.seq += 1
        if cmd.type == 'eyes/emotion':
            led = SetLED()
            led.index=0
            led.r=random.choice(led_choice)
            led.g=random.choice(led_choice)
            led.b=random.choice(led_choice)
            leds = [ led ]

            if cmd.command == 'reset':
               self.msg.name = ['ear_l_joint', 'ear_r_joint']
               self.msg.position = [0.00, 0.00]
               self.horn(leds)
            else:
               self.msg.name = ['ear_l_joint', 'ear_r_joint']
               self.msg.position = [-random.uniform(0.0,1.1), -random.uniform(0.0,1.1)]

            rospy.loginfo(f"r={led.r}, g={led.g}, b={led.b}, R={self.msg.position[0]}, L={self.msg.position[1]}")
            self.horn(leds)
            self.pub.publish(self.msg)

    def mouth_cb(self, cmd):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.seq += 1
        if cmd.type == 'mouth/speech':
            if cmd.command == 'begin':
               self.msg.name = ['mouth_joint']
               self.msg.position = [-0.15]
               #if not self.mouth_timer.is_alive():
               #    self.mouth_timer.start()
            elif cmd.command == 'end':
               self.msg.name = ['mouth_joint']
               self.msg.position = [-0.0]
               #if self.mouth_timer.is_alive():
               #    self.mouth_timer.cancel()
            else:
               self.msg.name = []
               self.msg.position = []

            #rospy.loginfo(self.msg)
            rospy.loginfo(f"M={self.msg.position[0]}")
            self.pub.publish(self.msg)

def main():
    node = BodyLanguageNode()
    rospy.spin()

