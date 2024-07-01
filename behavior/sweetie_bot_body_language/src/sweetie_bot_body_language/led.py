import rospy

from led_msgs.srv import SetLEDs, SetLED

led_choice = [0, 128, 255]

class LEDNode:
    def __init__(self):
        rospy.init_node('led', anonymous = True)
        rospy.Subscriber('control', TextCommand, self.control_cb)
        self.horn = rospy.ServiceProxy('/led/set_leds', SetLEDs)

    def control_cb(self, cmd):
        # TODO: Make communication protocol less obscure
        if cmd.type == 'eyes/emotion':
            led = SetLED()
            led.index=0
            led.r=random.choice(led_choice)
            led.g=random.choice(led_choice)
            led.b=random.choice(led_choice)
            leds = [ led ]

            if cmd.command == 'reset':
               self.horn(leds)

            rospy.logdebug(f'r={led.r}, g={led.g}, b={led.b}')
            self.horn(leds)


def main():
    node = LEDNode()
    rospy.spin()
