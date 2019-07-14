#!/usr/bin/env python

import rospy

from flexbe_core import EventState as Dummy
from flexbe_core import Logger
from flexbe_core.proxy import ProxyActionClient

from sweetie_bot_text_msgs.msg import TextCommand, CompoundAction as CompoundActionMsg, CompoundActionElement
from sweetie_bot_flexbe_states.execute_joint_trajectory import ExecuteJointTrajectory
from sweetie_bot_flexbe_states.execute_step_sequence import ExecuteStepSequence
from sweetie_bot_flexbe_states.set_joint_state import SetJointState
from sweetie_bot_flexbe_states.generate_step_sequence import GenerateStepSequence
from sweetie_bot_flexbe_states.text_command_state import TextCommandState


class CompoundActionBase(Dummy):
    '''
    Base class for compound action states. Subclass should provide Parameres list which is converted and 
    executed as sequence of actions.

    Compound action consists of several simple actions which are executed at specific time or after each other. 
    Simple action is described by type, command filed (cmd), sequence_number (seq_num) and executon delay (seq_delay).
    See CompoundAction.msg for details.

    Actions are translated into other flexbe states:

    * type 'motion/joint_trajectory' -> ExecuteJointTrajectory(trajectory_param=<cmd>)
    * type 'motion/step_sequence' -> ExecuteStepSequence(trajectory_param=<cmd>)
    * type 'set/joint_state' -> SetJointState(pose_param=<cmd>)
    * type 'generate/step_sequence' -> GenerateStepSequence(trajectory_param=<cmd>)
    * other types -> TextCommand(type=<type>, cmd=<cmd>)

    <= success 				All actions are executed successfully
    <= invalid_pose 			One of the action failed. The cause of failure is inconsistent robot pose, i.e. invalid_pose or partial_movement.
    <= failure 				One of actions has failed. 

    '''
    # Define C-like structure types
    class SimpleAction:
        '''CompoundAction subaction description.'''
        def __init__(self, previous_action, delay, state, flexbe_state, outcome_map, finish_time, description):
            self.previous_action, self.delay, self.state, self.flexbe_state, self.outcome_map, self.finish_time, self.description = previous_action, delay, state, flexbe_state, outcome_map, finish_time, description

    def __init__(self, compound_action_msg = None, outcomes = ['success', 'invalid_pose', 'failure']):
        ''' Init CompoundActionBase with given list of action specs. '''
        # parse supplied compound action
        if compound_action_msg:
            actions = self.parse_compound_action_msg(compound_action_msg)
        else:
            actions = []

        # setup key lists
        #TODO correct key remapping
        #input_keys = sum( map(lambda n: [ ik + str(n) for ik in actions[n].flexbe_state.get_registered_input_keys() ], range(len(actions))), []) 
        #output_keys = sum( map(lambda n: [ ok + str(n) for ok in actions[n].flexbe_state._output_keys ], range(len(actions))), []) 
        #smach.Remapper(userdata,
               #action.flexbe_state.get_registered_input_keys(),
               #action.flexbe_state.get_registered_output_keys(),
               #{key: key + str(n) for key in action.flexbe_state.get_registered_input_keys() + action.flexbe_state.get_registered_output_keys()}) 

        # instance superclass
        super(CompoundActionBase, self).__init__(outcomes=outcomes,
                input_keys = list( set.union(*[ actions[n].flexbe_state._input_keys for n in range(len(actions)) ]) ),
                output_keys = list( set.union(*[ actions[n].flexbe_state._output_keys for n in range(len(actions)) ]) ) )

        # set class fields
        self._start_time = None
        self._pause_time = None
        self._enter_userdata = None
        self._actions = actions

    def load_compound_action_msg(self, action_ns, action_param):
        # derive parameter full name
        if action_ns:
            action_param = action_ns + '/' + action_param

        # Load FollowStepSequenceGoal from Parameter Server
        msg_raw = rospy.get_param(action_param)

        if not isinstance(msg_raw, xmlrpclib.Binary):
            raise TypeError, "CompoundAction ROS parameter '" + action_param + "' is not a binary data."
        # deserialize
        msg = CompoundActionMsg()
        msg.deserialize(goal_raw.data)
        return msg


    def parse_compound_action_msg(self, compound_action_msg):
        ''' Converts sweetie_bot_text_msgs.msg.CompoundActionElement to the list of SimpleActions.'''
        # convert parameteres to actions
        actions = list()
        for i, p in enumerate(compound_action_msg.action_list):
            if not isinstance(p.seq_num, int) or not isinstance(p.seq_delay, float):
                raise TypeError, 'CompoundAction: Incorrect action (seq_num,seq_delay) pair in position %d. (int,float) pair was expected..' % i
            if p.seq_num >= i+1 or p.seq_delay < 0:
                raise TypeError, 'CompoundAction parse error: incorrect action (seq_num,seq_delay) pair in position %d. seq_num must reference only previously defined actions.' % i
            if p.cmd.type == None:
                actions.append(None)
            else:
                # get parameters
                previous_action = None if p.seq_num == 0 else actions[p.seq_num-1]
                delay = p.seq_delay
                description = p.cmd.type + ' ' + p.cmd.command
                Logger.loginfo(description)
                if not isinstance(p.cmd.type, str):
                    raise TypeError, 'CompoundAction: Incorrect action type %d. Type must be string or None.' % i
                # Select action
                if p.cmd.type == 'motion/joint_trajectory':
                    flexbe_state = ExecuteJointTrajectory(trajectory_param = p.cmd.command)
                    outcome_map = {'success': 'success', 'invalid_pose':'invalid_pose', 'partial_movement':'invalid_pose'}
                elif p.cmd.type == 'motion/step_sequence':
                    flexbe_state = ExecuteStepSequence(trajectory_param = p.cmd.command)
                    outcome_map = {'success': 'success', 'invalid_pose':'invalid_pose', 'partial_movement':'invalid_pose'}
                elif p.cmd.type == 'set/joint_state':
                    flexbe_state = SetJointState(pose_param = p.cmd.command)
                    outcome_map = {'success': 'success', 'timeout':'invalid_pose'}
                elif p.cmd.type == 'generate/step_sequence':
                    flexbe_state = GenerateStepSequence(trajectory_param = p.cmd.command)
                    outcome_map = {'success': 'success', 'invalid_pose':'invalid_pose', 'partial_movement':'invalid_pose'}
                else:
                    flexbe_state = TextCommandState(p.cmd.type, p.cmd.command, topic = 'control')
                    outcome_map = {'done': 'success'}
                # add action to list
                actions.append( CompoundActionBase.SimpleAction(previous_action, delay, 'WAITING', flexbe_state, outcome_map, 0.0, description) )
        # remove None actions
        return [ a for a in actions if a != None ]


    def on_start(self):
        # Pass event to substates.
        for action in self._actions:
            action.flexbe_state.on_start()

    def on_stop(self):
        # Pass event to substates.
        for action in self._actions:
            action.flexbe_state.on_stop()

    def on_pause(self):
        # Store pause time
        self._pause_time = rospy.get_rostime()
        # Pause only running actions
        for action in self._actions:
            if actions.state == 'RUNNING':
                action.flexbe_state.on_pause()

    def on_resume(self, userdata):
        # Time correction
        self._start_time += rospy.get_rostime() - self._pause_time
        # Resume only running actions.
        for action in self._actions:
            if actions.state == 'RUNNING':
                action.flexbe_state.on_resume(userdata)

    def on_enter(self, userdata):
        # Get start timestamp
        self._start_time = rospy.get_rostime()
        self._enter_userdata = userdata
        for action in self._actions:
            action.state = 'WAITING'

    def on_exit(self, userdata):
        # If any behavior is running exit it
            for action in self._actions:
                if action.state == 'RUNNING':
                    action.flexbe_state.on_exit(userdata)

    def execute(self, userdata):
        # calculate elasped time
        t = (rospy.get_rostime() - self._start_time).to_sec()

        is_finished = True
        is_failed = False
        outcome = None
        # loop over actons
        for action in self._actions:
            if action.state == 'WAITING':
                is_finished = False
                # check start condidtions
                should_start = False
                if not action.previous_action:
                    should_start = t > action.delay
                else:
                    should_start = action.previous_action.state == 'FINISHED' and (t > action.previous_action.finish_time + action.delay)
                # start action
                if should_start:
                    action.flexbe_state.on_enter(self._enter_userdata)
                    action.state = 'RUNNING'
                    Logger.loginfo('CompoundAction: t = %f: start action: "%s"' % (t, action.description))

            if action.state == 'RUNNING':
                is_finished = False
                # execute state

                # In constructor of EventState user-defined method `execute()` is renamed to `__execute()`.
                # To skip all EventState facilities and operate FlexBe state object directly we should access private method of EventState.
                outcome = action.flexbe_state._EventState__execute(userdata)

                # check outcome
                if outcome and outcome != 'loopback':
                    action.flexbe_state.on_exit(userdata)
                    action.state = 'FINISHED'
                    action.finish_time = t

                    Logger.loginfo('CompoundAction: t = %f: finish action: "%s" outcome: %s' % (t, action.description, outcome))
                    # set flag if action has failed
                    outcome = action.outcome_map.get(outcome) or 'failure'
                    if outcome != 'success':
                        is_failed = True
        # check results
        if is_finished:
            return 'success'
        if is_failed:
            return outcome

