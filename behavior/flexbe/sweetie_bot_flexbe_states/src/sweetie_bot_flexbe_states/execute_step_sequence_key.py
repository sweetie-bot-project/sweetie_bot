#!/usr/bin/env python
from flexbe_core import Logger

from internal.execute_step_sequence_base import ExecuteStepSequenceBase as EventState

# Superclass is imported as EventState to allow to parse state definition FlexApp correctly.
class ExecuteStepSequenceKey(EventState):
    '''
    Execute stored sweetie_bot_control_msgs.msg.FollowStepSequence action. Goal is stored on ROS parameter server in serialized form.
    Parameter name is specified as state parameter.

    -- controller          string    Action server to execute action.
    -- trajectory_ns       string    Namespace where trajectores are stored. 

    ># trajectory_param    string    ROS parameter in trajectory_ns which stores FollowStepSequence message.

    <= success              Indicate that goal is achived.
    <= unavailable          Unable to load desired movement.
    <= partial_movement     Execution stopped in midway (path or goal tolerance error, obstacle, external cancel request).
    <= invalid_pose         Initial pose is invalid, movement cannot be started.
    <= failure              Any other failure. (Invalid joints, ActionServer unavailable and so on).

    '''

    def __init__(self, controller = 'motion/controller/step_sequence', trajectory_ns = 'saved_msgs/joint_trajectory'):
        # Declare outcomes and output keys
        # Actually constructor of ExecuteStepSequenceBase is called instead of constructor of EventState.
        super(ExecuteStepSequenceKey, self).__init__(controller = controller, outcomes = ['success', 'unavailable', 'partial_movement', 'invalid_pose', 'failure'], input_keys = ['trajectory_param'])

        self._trajectory_ns = trajectory_ns

    def on_enter(self, userdata):
        self._outcome = None
        # Load FollowStepSequenceGoal from Parameter Server
        try:
            goal = self.load_goal_msg(self._trajectory_ns, userdata.trajectory_param)
        except Exception as e:
            Logger.logwarn('ExecuteStepSequenceKey: unable to load trajectory from `%s/%s` parameter:\n%s' % (self._trajectory_ns, userdata.trajectory_param, str(e)) )
            self._outcome = 'unavailable'
            return

        # send loaded goal
        self.send_goal(goal)

