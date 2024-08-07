import rospy

from sweetie_bot_text_msgs.msg import TextCommand
from sensor_msgs.msg import JointState
from threading import Timer
import time
import random


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
        self.msg = JointState()

    def dummyfn(self, m=0.32):
        self.msg.name = ['mouth_joint']
        self.msg.position = [-random.uniform(0.2,m)]
        self.pub.publish(self.msg)
        #rospy.loginfo(self.msg)

    def control_cb(self, cmd):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.seq += 1
        if cmd.type == 'eyes/emotion':
            if cmd.command == 'reset':
               self.msg.name = ['ear_l_joint', 'ear_r_joint']
               self.msg.position = [0.00, 0.00]
            else:
               self.msg.name = ['ear_l_joint', 'ear_r_joint']
               self.msg.position = [-random.uniform(0.0,1.1), -random.uniform(0.0,1.1)]

            rospy.logdebug(f'R={self.msg.position[0]}, L={self.msg.position[1]}')
            self.pub.publish(self.msg)

    def mouth_cb(self, cmd):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.seq += 1
        if cmd.type == 'mouth/speech':
            if cmd.command == 'begin':
               self.msg.name = ['mouth_joint']
               self.msg.position = [-0.15]
            elif cmd.command == 'end':
               self.msg.name = ['mouth_joint']
               self.msg.position = [-0.0]
            else:
               self.msg.name = []
               self.msg.position = []

            rospy.logdebug(f"M={self.msg.position[0]}")
            self.pub.publish(self.msg)


def main():
    node = BodyLanguageNode()
    rospy.spin()

