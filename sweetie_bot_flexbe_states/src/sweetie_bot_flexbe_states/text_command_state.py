#!/usr/bin/env python
import rospy
from roslib.message import check_type

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher

from sweetie_bot_text_msgs.msg import TextCommand

class TextCommandState(EventState):
    '''
    Send TextCommand message to the given topic.

    -- topic        string          Topic where to publish message.
    -- type         string          Type field of TextCommand.
    -- command      string          Command string.

    <= done 	                    Message is sent.  
    <= failed 	                    Unable to send a message.

    '''

    def __init__(self, topic, type, command):
        super(TextCommandState, self).__init__(outcomes = ['done', 'failed'])

        # Store state parameter for later use.
        self._topic = topic

        # Check and form message
        check_type('type', 'string', type)
        check_type('command', 'string', command)
        self._message = TextCommand(type = type, command = command)

        # create publisher
        self._publisher = ProxyPublisher({ topic: TextCommand })

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        
        try: 
            self._publisher.publish(self._topic, self._message)
        except Exception as e:
            Logger.logwarn('Failed to send TextCommand message `' + self._topic + '`:\n%s' % str(e))
            return 'failed'


        Logger.loginfo('Sent TextCommand `{0}`: type = "{1}" cmd = "{2}".'.format(self._topic, self._message.type, self._message.command))
        return 'done'

