#!/usr/bin/env python
import rospy
import rostopic
import inspect
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxySubscriberCached

#Dirty hack to make CromeApp be able to see documentation
from flexbe_states.subscriber_state import SubscriberState as EventState

'''
Created on 14.10.2017

@author: disRecord
'''

class WaitForMessageState(EventState):
        '''
        Wait for the message on the given topic with specified properties.

        -- topic		string	  The topic on which should be listened.
        -- condition 		function	The condition to check message properties.
                                                                Has to expect one parameter which will be set to message and return a boolean.
        -- buffered		bool	    Use buffered connection to avoid message loss.
        -- clear		bool	    Drops last message on this topic on enter
                                                                in order to only handle message received since this state is active.

        #> message	      object	  Message which passed conditions check.

        <= received			   	Message was received. 
        <= unavailable			  The topic is not available when this state becomes actives.

        '''

#	def __init__(self, topic, condition, buffered = False, clear = False):
#               super(SubscriberState, self).__init__(outcomes=['received', 'unavailable'], output_keys=['message'])
	def __init__(self, topic, condition, buffered = False, clear = False):
		super(WaitForMessageState , self).__init__(topic = topic, blocking = False, clear = clear)

		self._buffered = buffered

		if not callable(condition):
			raise TypeError, 'Provided condition parameter is not callable object.'
		self._condition = condition


	def execute(self, userdata):

		if not self._connected:
			return 'unavailable'

		if not self._buffered:
			message = self._sub.get_last_msg(self._topic)
			self._sub.remove_last_msg(self._topic)
			if message and self._condition(message):
				userdata.message = message
				return 'received'
		else:
			while self._sub.has_buffered(self._topic):
				message = self._sub.get_from_buffer(self._topic)
				if self._condition(message):
					userdata.message = message
					return 'received'


	def on_enter(self, userdata):
		super(WaitForMessageState , self).on_enter(userdata)
		if self._buffered:
			self._sub.enable_buffer(self._topic)

	def on_exit(self, userdata):
		if self._buffered:
			self._sub.disable_buffer(self._topic)

