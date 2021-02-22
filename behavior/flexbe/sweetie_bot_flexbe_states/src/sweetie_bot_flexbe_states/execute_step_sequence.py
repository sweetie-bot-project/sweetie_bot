#!/usr/bin/env python
from .internal.execute_step_sequence_base import ExecuteStepSequenceBase as EventState 

# Superclass is imported as EventState to allow to parse state definition FlexApp correctly.
class ExecuteStepSequence(EventState):
    '''
    Execute stored sweetie_bot_control_msgs.msg.FollowStepSequence action. Goal is stored on ROS parameter server in serialized form.
    Parameter name is specified as state parameter.

    -- controller          string    Action server to execute action.
    -- trajectory_param    string    ROS parameter in trajectory_ns which stores FollowStepSequence message.
    -- trajectory_ns       string    Namespace where trajectores are stored. 

    <= success              Indicate that goal is achived.
    <= partial_movement     Execution stopped in midway (path or goal tolerance error, obstacle, external cancel request).
    <= invalid_pose         Initial pose is invalid, movement cannot be started.
    <= failure              Any other failure. (Invalid joints, ActionServer unavailable and so on).

    '''

    def __init__(self, controller = 'motion/controller/step_sequence', trajectory_param = '', trajectory_ns = 'saved_msgs/step_sequence'):
        # Declare outcomes and output keys
        # Actually constructor of ExecuteStepSequenceBase is called instead of constructor of EventState.
        # 

        super(ExecuteStepSequence, self).__init__(controller = controller, outcomes = ['success', 'partial_movement', 'invalid_pose', 'failure'])

        # Load FollowStepSequenceGoal from Parameter Server
        self._goal = self.load_goal_msg(trajectory_ns, trajectory_param)

    def on_enter(self, userdata):
        # send loaded goal
        self.send_goal(self._goal)

