#!/usr/bin/env python
from flexbe_core import EventState 
from flexbe_core import Logger
from flexbe_core.proxy import ProxyActionClient

from sweetie_bot_control_msgs.msg import SetOperationalAction, SetOperationalGoal, SetOperationalResult

class SetOperational(EventState):
    '''
    Make ceratin controller operational or non-operational.

    -- controller           string          Controller namespace.
    -- operational          bool            Activate/deactivate controller.
    -- resources            string          Resource list.
    -- sync                 bool            If set to true execution is blocked until controller operational.

    <= done                         Finished.
    <= failure                      Failed to change controller state.

    '''

    def __init__(self, controller = 'motion/controller/joint_state_head', operational = True, resources = [], sync = True):
        super(SetOperational, self).__init__(outcomes = ['done', 'failure'])

        # Store topic parameter for later use.
        self._controller = controller 
        self._operational = operational
        self._sync = sync
        self._resources = resources

        # create proxies
        self._action_client = ProxyActionClient({self._controller: SetOperationalAction})

        # check parameters
        if not isinstance(resources, list) or not all( [ isinstance(r, str) for r in resources] ):
            raise ValueError('SetOperational: resource must be list of strings')
        if not isinstance(operational, bool) or not isinstance(sync, bool):
            raise ValueError('SetOperational: sync and operational parameters must be bool.')
        
        # error in enter hook
        self._error = False
    
    def on_enter(self, userdata):
        self._error = False
        # activate/deactivate controller
        actiavtion_request = SetOperationalGoal()
        actiavtion_request.operational = self._operational
        actiavtion_request.resources = self._resources
        try:
            self._action_client.send_goal(self._controller, actiavtion_request)
        except Exception as e:
            Logger.logwarn('SetJointStateBase: Failed to send the SetOperational command:\n%s' % str(e))
            self._error = True

        Logger.loginfo('SetOperational: SetOperational(%s) request is sent.' % self._operational)

    def execute(self, userdata):
        # error in start hook
        if self._error:
            return 'failure'

        # async mode
        if not self._sync:
            return 'done'

        # check if controller is active
        if not self._action_client.is_active(self._controller):
            if self._operational:
                Logger.loginfo('SetOperational: controller was deactivated by external cause.')
            return 'done'

    def on_exit(self, userdata):
        if self._action_client.is_active(self._controller):
            try: 
                self._action_client.cancel(self._controller)
                Logger.loginfo('SetOperational: SetOperational(%s) cancel request is sent.' % self._operational)
            except Exception as e:
                Logger.logwarn('SetJointStateBase: failed to deactivate `' + self._controller + '` controller:\n%s' % str(e))


