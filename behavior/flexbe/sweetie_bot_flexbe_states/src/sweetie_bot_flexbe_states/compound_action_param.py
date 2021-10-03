#!/usr/bin/env python

from .internal.compound_action_base import CompoundActionBase as EventState

# Superclass is imported as EventState to allow to parse state definition FlexApp correctly.
class CompoundActionParam(EventState):
    '''
    Execute compound action stored on ROS parameter server. Parameter name is specified as state parameter.
    
    Compound actions are stored as sweetie_bot_text_msgs.CompoundAction messages serialized in binary form.

    -- action_param            string    ROS parameter which stores CompoundAction message.
    -- action_ns               string    Namespace where messages are stored. 

    <= success                 All actions are executed successfully.
    <= invalid_pose            One of the action failed. The cause of failure is inconsistent robot pose, i.e. invalid_pose or partial_movement.
    <= failure                 One of actions has failed.

    '''

    def __init__(self, action_param = '', action_ns = 'saved_msgs/compound_action'):
        # load stored message
        compound_action_msg = self.load_compound_action_msg(action_ns, action_param)
        # parse it and initialize flexbe state
        super(CompoundActionParam, self).__init__(compound_action_msg = compound_action_msg, outcomes=['success', 'invalid_pose', 'failure'])
