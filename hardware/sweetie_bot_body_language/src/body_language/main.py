import rospy

from sweetie_bot_text_msgs.msg import TextCommand
from sensor_msgs.msg import JointState

class BodyLanguageNode():
    def __init__(self):
        rospy.init_node('body_language', anonymous = True)

        # register ROS interface
        rospy.Subscriber('mouth', TextCommand, self.mouth_cb)
        out_topic = '/motion/controller/joint_state/in_joints_ref'
        out_topic = '/motion/controller/joint_state_head/in_joints_ref'
        self.pub = rospy.Publisher(out_topic, JointState, queue_size=10)
        self.msg = JointState()

    def mouth_cb(self, cmd):
        if cmd.type == 'mouth/speech':
            self.msg.header.stamp = rospy.Time.now()
            self.msg.header.seq += 1
            if cmd.command == 'begin':
               self.msg.name = ['mouth_joint']
               self.msg.position = [-0.2]
            elif cmd.command == 'end':
               self.msg.name = ['mouth_joint']
               self.msg.position = [-0.0]
            else:
               self.msg.name = []
               self.msg.position = []

            rospy.logerr(self.msg)
            self.pub.publish(self.msg)

def main():
    node = BodyLanguageNode()
    rospy.spin()

