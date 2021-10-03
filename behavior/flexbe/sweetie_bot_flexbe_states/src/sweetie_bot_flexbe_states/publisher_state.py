#!/usr/bin/env python
import rospy
from roslib.message import check_type

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher

class PublisherState(EventState):
    '''
    Send message to the given topic.

    -- topic        string          Topic where to publish message.
    -- msg_type     datatype        Message type.
    -- value        dict            Message initializer. Dict is translated to argument list.

    <= done                         Message is sent.  
    <= failed                       Unable to send a message.

    '''

    def __init__(self, topic, msg_type, value):
        super(PublisherState, self).__init__(outcomes = ['done', 'failed'])

        # Store state parameter for later use.
        self._topic = topic

        # form message
        self._message = msg_type(**value)

        # create publisher
        self._publisher = ProxyPublisher({ topic: msg_type })

        # error in enter hook
        self._error = False

    def on_enter(self, userdata):
        try: 
            self._publisher.publish(self._topic, self._message)
        except Exception as e:
            Logger.logwarn('Failed to send message `' + self._topic + '`:\n%s' % str(e))
            self._error = True

        Logger.loginfo('Send message on topic `{0}`.'.format(self._topic))

    def execute(self, userdata):
        if self._error:
            return 'failed'

        return 'done'

