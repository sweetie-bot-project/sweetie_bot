#!/usr/bin/env python
from flexbe_core import Logger
from internal.compound_action_base import CompoundActionBase as EventState

# Superclass is imported as EventState to allow to parse state definition FlexApp correctly.
class CompoundActionParamKey(EventState):
    '''
    Execute compound action stored on ROS parameter server. Parameter name is specified as state input key.
    
    Compound actions are stored as sweetie_bot_text_msgs.CompoundAction messages serialized in binary form.

    -- action_ns                        string    Namespace where messages are stored. 

    ># action_param                     string    ROS parameter which stores CompoundAction message.

    <= success 				All actions are executed successfully.
    <= unavailable                      Unable to load desired movement.
    <= invalid_pose 			One of the action failed. The cause of failure is inconsistent robot pose, i.e. invalid_pose or partial_movement.
    <= failure 				One of actions has failed.

    '''

    def __init__(self, action_ns = 'saved_msgs/compound_action'):
        super(CompoundActionParamKey, self).__init__(outcomes=['success', 'unavailable', 'invalid_pose', 'failure'], input_keys=['action_param'])

        self._outcome = None
        self._action_ns = action_ns

    def on_enter(self, userdata):
        self._outcome = None
        # Load CompoundAction from Parameter Server 
        try:
            compound_action_msg = self.load_compound_action_msg(self._action_ns, userdata.action_param)
            self._actions = self.parse_compound_action_msg(compound_action_msg)
        except Exception as e:
            Logger.logwarn('CompoundActionParamKey: unable to load message from `%s/%s` parameter:\n%s' % (self._action_ns, userdata.action_param, str(e)) )
            self._outcome = 'unavailable'
            return

        # proceed with actions execution

        # perform on_start()
        super(CompoundActionParamKey, self).on_start()
        # perform on_enter()
        super(CompoundActionParamKey, self).on_enter(userdata)

    def execute(self, userdata):
        if self._outcome:
            return self._outcome

        return super(CompoundActionParamKey, self).execute(userdata)

    def on_exit(self, userdata):
        # perform on_stop()
        super(CompoundActionParamKey, self).on_stop()
        # clear actions list
        self._actions = []
        
