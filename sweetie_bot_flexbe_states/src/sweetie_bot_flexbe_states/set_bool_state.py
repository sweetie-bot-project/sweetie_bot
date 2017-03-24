#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from std_srvs.srv import SetBool


class SetBoolState(EventState):
    '''
    Issue call to std_srv.SetBool service. 

    -- service      string          Service name.
    -- value        bool            Argument for service call.

    #> success      bool            Request result as in std_srvs.srv.SetBoolResponce message.
    #> message      string          Request result as in std_srvs.srv.SetBoolResponce message.

    <= true 	                    True has been returned.		
    <= false                        False has been returned.
    <= failure 			    Service call has failed.

    '''

    def __init__(self, service, value):
        super(SetBoolState, self).__init__(outcomes = ['true', 'false', 'failure'], output_keys = ['success', 'message'])

        # Store state parameter for later use.
        self._value = value
        self._service = service

        # get service caller
        self._service_caller = ProxyServiceCaller({ service: SetBool })
        self._response = None

        # It may happen that the service caller fails to send the action goal.
        self._error = False
    
    def on_enter(self, userdata):
        userdata.success = None
        userdata.message = None

        try: 
            self._response = self._service_caller.call(self._service, self._value)
        except Exception as e:
            Logger.logwarn('Failed to call SetBool service `' + self._service + '`:\n%s' % str(e))
            self._error = True

        Logger.loginfo('Call SetBool: `{0}` (value={1}) result: {2} "{3}".'.format(self._service, self._value, self._response.success, self._response.message))


    def execute(self, userdata):
        if self._error:
            return 'failure'

        userdata.success = self._response.success
        userdata.message = self._response.message

        if self._response.success:
            return 'true'
        else:
            return 'false'


    def on_exit(self, userdata):
        pass

