#!/usr/bin/env python

from internal.compound_action_base import CompoundActionBase as EventState
from flexbe_core import Logger

from sweetie_bot_text_msgs.msg import TextCommand, CompoundAction as CompoundActionMsg, CompoundActionElement

# Superclass is imported as EventState to allow to parse state definition FlexApp correctly.
class CompoundAction(EventState):
    '''
    Compound action consists of several simple actions which are executed at specific time or after each other. 
    Simple action is described by type (<type>), command field (<cmd>) and start execution time label t (<t>). 

    Actions are translated into other flexbe states:

    * type 'motion/joint_trajectory' -> ExecuteJointTrajectory(trajectory_param=<cmd>);
    * type 'motion/step_sequence' -> ExecuteStepSequence(trajectory_param=<cmd>);
    * type 'set/joint_state' -> SetJointState(pose_param=<cmd>);
    * type 'generate/step_sequence' -> GenerateStepSequence(motion_param=<cmd>);
    * other strings -> TextCommand(type=<type>, cmd=<cmd>);
    * None -> no actions.

    -- t1			(int,float)			  Time label of the first action. Time label is a tuple which contains two values: previous simple action reference and time delay (seconds). So `(2, 0.5)` means that action is started with 0.5 seconds delay after second action is finished. Zero coresponds to CompoundAction state start time.
    -- type1		string				  Type of the first action. None means no action.  
    -- cmd1			string				  First action command.
    -- t2			(int,float)		          Time label of the second action.
    -- type2		string				  Type of the second action.
    -- cmd2			string				  Second action command.
    -- t3			(int,float)		          Time label of the third action.
    -- type3		string				  Type of the thrid action.
    -- cmd3			string				  Third action command.
    -- t4			(int,float)		          Time label of the third action.
    -- type4		string				  Type of the fourth action.
    -- cmd4			string				  Fourth action command.

    <= success 				All actions are executed successfully.
    <= invalid_pose 			One of the action failed. The cause of failure is inconsistent robot pose, i.e. invalid_pose or partial_movement.
    <= failure 				One of actions has failed.

    '''

    def __init__(self, t1 = [0, 0.0], type1 = None, cmd1 = '', t2 = [0, 0.0], type2 = None, cmd2 = '', t3 = [0, 0.0], type3 = None, cmd3 = '', t4 = [0, 0.0], type4 = None, cmd4 = ''):
        # Decompose parameters and create children flexbe states.
        compound_action_msg = CompoundActionMsg()
        compound_action_msg.action_list = [
                CompoundActionElement( seq_num=t1[0],seq_delay=t1[1], cmd=TextCommand(type=type1,command=cmd1) ),
                CompoundActionElement( seq_num=t2[0],seq_delay=t2[1], cmd=TextCommand(type=type2,command=cmd2) ),
                CompoundActionElement( seq_num=t3[0],seq_delay=t3[1], cmd=TextCommand(type=type3,command=cmd3) ),
                CompoundActionElement( seq_num=t4[0],seq_delay=t4[1], cmd=TextCommand(type=type4,command=cmd4) ) ]

        super(CompoundAction, self).__init__(compound_action_msg = compound_action_msg, outcomes=['success', 'invalid_pose', 'failure'])
