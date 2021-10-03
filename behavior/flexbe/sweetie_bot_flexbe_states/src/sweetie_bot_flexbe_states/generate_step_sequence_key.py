#!/usr/bin/env python
from .internal.generate_step_sequence_base import GenerateStepSequenceBase as EventState
from flexbe_core import Logger

# Superclass is imported as EventState to allow to parse state definition FlexApp correctly.
class GenerateStepSequenceKey(EventState):
    '''
    Generate stored sweetie_bot_clop_generator::MoveBase action. Goal is stored on ROS parameter server in serialized form.
    This state receives ROS parameter name via input key.

    -- controller          string    Action server to execute action.
    -- trajectory_param    string    ROS parameter in trajectory_ns which stores MoveBase message. If is set to None corresponding key value is used.
    -- trajectory_ns       string    Namespace where trajectores are stored. 

    #> trajectory_param    string   ROS parameter in trajectory_ns which stores MoveBase message. 

    <= success             Motion have been planned and executed succesfully.
    <= solution_not_found   Planner is unable to find solution.
    <= partial_movement     Execution stopped in midway (path or goal tolerance error, obstacle, external cancel request).
    <= invalid_pose         Initial pose is invalid, movement cannot be started.
    <= failure              Any other failure. (Invalid joints, ActionServer unavailable and so on).

    '''

    def __init__(self, controller = 'clop_generator', trajectory_ns = 'saved_msgs/move_base'):
        # Declare outcomes and output keys
    super(GenerateStepSequenceKey, self).__init__(controller = controller, outcomes = ['success', 'solution_not_found', 'partial_movement', 'invalid_pose', 'failure'], input_keys = ['trajectory_param'])
    # Save parameter namespace
        self._trajectory_ns = trajectory_ns

    def on_enter(self, userdata):
        self._outcome = None
        # Load MoveBaseGoal from Parameter Server
        try:
            goal = self.load_goal_msg(self._trajectory_ns, userdata.trajectory_param)
        except Exception as e:
            Logger.logwarn('GenerateStepSequenceKey: unable to load trajectory from `%s/%s` parameter:\n%s' % (self._trajectory_ns, userdata.trajectory_param, str(e)) )
            self._outcome = 'failure'
            return

        # send loaded goal
        self.send_goal(goal)
