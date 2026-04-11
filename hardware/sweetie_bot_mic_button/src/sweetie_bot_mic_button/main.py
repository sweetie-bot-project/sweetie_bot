#!/usr/bin/env python3
import rospy
import os
import sys
from pynput import keyboard
from pynput.keyboard import Key

from std_msgs.msg import Bool

class KeyboardListenerNode(object):

    def __init__(self):
        # shutdown hook
        rospy.on_shutdown(self.on_shutdown)
        rospy.init_node('keyboard_listener')
        self.pub = rospy.Publisher('mic_button', Bool, queue_size=10)
        self.last_published = False
        keys_combo = rospy.get_param("~key_combination", ['ctrl', 'alt', 'w'])
        enabled = rospy.get_param("~enable_global_hotkey", True)
        if enabled:
            # keyboard
            pynput_key_map = { 'ctrl': Key.ctrl, 'alt': Key.alt, 'shift':Key.shift, 'space': Key.space, 'pause': Key.pause }
            self._keys_pressed = set()
            self._keys_combo = set()
            for key in keys_combo:
                if key in pynput_key_map:
                    self._keys_combo.add( pynput_key_map[key] )
                elif len(key) == 1:
                    self._keys_combo.add( keyboard.KeyCode.from_char(key) )
                else:
                    raise KeyError(f"Unknown key specification: {key}")
            self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
            self.keyboard_listener.start()
            rospy.loginfo(f"Listening for global hotkey {keys_combo}")

    def publish_mic_button(self):
        combo_is_pressed = len(self._keys_pressed) == len(self._keys_combo)
        if combo_is_pressed:
            if not self.last_published:
                self.pub.publish(True)
                self.last_published = True
        else:
            if self.last_published:
                self.pub.publish(False)
                self.last_published = False

    def on_shutdown(self):
        try:
            self._keys_pressed = set()
            self.publish_mic_button()
            self.keyboard_listener.stop()
        except:
            pass

    def on_key_press(self, key):
        if key in self._keys_combo and not key in self._keys_pressed:
            self._keys_pressed.add(key)
            self.publish_mic_button()

    def on_key_release(self, key):
        if key in self._keys_combo and key in self._keys_pressed:
            self._keys_pressed.discard(key)
            self.publish_mic_button()

def main():
    try:
        n = KeyboardListenerNode()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
