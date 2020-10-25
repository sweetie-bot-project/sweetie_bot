#!/usr/bin/env python
from .internal.generate_step_sequence_base import GenerateStepSequenceBase as EventState
from flexbe_core import Logger

# Superclass is imported as EventState to allow to parse state definition FlexApp correctly.
class GenerateStepSequence(EventState):
    '''
    Use saved sweetie_bot_clop_generator::MoveBase action to plan and execute movement. Action goal is stored on ROS parameter server in serialized form.
    Parameter name is specified with state parameter.

    -- controller          string    Action server to execute action.
    -- trajectory_param    string    ROS parameter in trajectory_ns which stores MoveBase message. If is set to None corresponding key value is used.
    -- trajectory_ns       string    Namespace where trajectores are stored. 

    <= success 		    Motion have been planned and executed succesfully.
    <= solution_not_found   Planner is unable to find solution.
    <= partial_movement     Execution stopped in midway (path or goal tolerance error, obstacle, external cancel request).
    <= invalid_pose         Initial pose is invalid, movement cannot be started.
    <= failure              Any other failure. (Invalid joints, ActionServer unavailable and so on).

    '''

    def __init__(self, controller = 'clop_generator', trajectory_param = None, trajectory_ns = 'saved_msgs/move_base'):
        # Declare outcomes and output keys
	super(GenerateStepSequence, self).__init__(controller = controller, outcomes = ['success', 'solution_not_found', 'partial_movement', 'invalid_pose', 'failure'])

        # Load MoveBaseGoal from Parameter Server if parameter is not supplied
	self._goal = self.loadGoalMsg(trajectory_ns, trajectory_param)

    def on_enter(self, userdata):
        # Send the goal
        self.send_goal(self._goal)


