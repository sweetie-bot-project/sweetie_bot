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

        # It may happen that the service caller fails to send the action goal.
        self._error = False

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Main purpose is to check state conditions and trigger a corresponding outcome.
        
        userdata.success = None
        userdata.message = None

        try: 
            response = self._service_caller.call(self._service, self._value)
        except Exception as e:
            Logger.logwarn('Failed to call SetBool service `' + self._service + '`:\n%s' % str(e))
            return 'failure'

        Logger.loginfo('Call SetBool: `{0}` (value={1}) result: {2} "{3}".'.format(self._service, self._value, response.success, response.message))

        userdata.success = response.success
        userdata.message = response.message

        if response.success:
            return 'true'
        else:
            return 'false'


    def on_exit(self, userdata):
        # This method is called when an outcome is returned and another state gets active.
        # It can be used to stop possibly running processes started by on_enter.
        pass

