#!/usr/bin/env python
import rospy
from roslib.message import check_type

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher

from sweetie_bot_text_msgs.msg import TextCommand

class TextCommandState(EventState):
    '''
    Send TextCommand message to the given topic.

    -- type         string          Type field of TextCommand.
    -- command      string          Command string.
    -- topic        string          Topic where to publish message.

    <= done 	                    Message is sent.  
    '''

    def __init__(self, type, command, topic = 'control'):
        super(TextCommandState, self).__init__(outcomes = ['done'])

        # Store state parameter for later use.
        self._topic = topic

        # Check and form message
        check_type('type', 'string', type)
        check_type('command', 'string', command)
        self._message = TextCommand(type = type, command = command)

        # create publisher
        self._publisher = ProxyPublisher({ topic: TextCommand })

        # error in enter hook
        self._error = False

    def on_enter(self, userdata):
        self._error = False

        try: 
            self._publisher.publish(self._topic, self._message)
        except Exception as e:
            Logger.logerr('Failed to send TextCommand message `' + self._topic + '`:\n%s' % str(e))
            self._error = True
            return

        Logger.loginfo('Send TextCommand `{0}`: type = "{1}" cmd = "{2}".'.format(self._topic, self._message.type, self._message.command))

    def execute(self, userdata):
        return 'done'

