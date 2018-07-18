#!/usr/bin/env python

from collections import namedtuple

import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from sweetie_bot_text_msgs.msg import TextCommand

from execute_stored_trajectory_state import ExecuteStoredJointTrajectoryState
from text_command_state import TextCommandState


'''
Created on 13.10.2017

@author: disRecord
'''

class SweetieBotCompoundAction(EventState):
	'''
	Compound action consists of several simple actions which are executed at specific time or after each other. 
	Simple action is described by type, command filed (cmd) and start execution time label t. 

	Actions are translated into other flexbe states. No ExecuteStoredJointTrajectoryState and TextCommandState are supported.


	-- t1			(int,float)			Time label of the first action. Time label is a tuple which contains two values: previous simple action reference and time delay (seconds). So `(2, 0.5)` means that action is started with 0.5 seconds delay after second action is finished. Zero coresponds to CompoundAction state start time.
	-- type1		string				Type of the first action. None means no action. 'motion/joint_trajectory' maps to ExecuteStoredJointTrajectoryState( trajectory_param='joint_trajectory/'+cmd, action_topic='motion/controller/joint_trajectory') Other values are converted to TextCommandState( type=type, cmd=cmd, topic='control').
	-- cmd1			string				  First action command.
	-- t2			(int,float)		Time label of the second action.
	-- type2		string				  Type of the second action.
	-- cmd2			string				  Second action command.
	-- t3			(int,float)		Time label of the thrid action.
	-- type3		string				  Type of the thrid action.
	-- cmd3			string				  Thrid action command.
	-- type4		string				  Type of the fourth action.
	-- cmd4			string				  Fourth action command.

															
	<= success 				All actions are executed successfully
	<= failure 				One of actions has failed.

	'''

	def __init__(self, t1 = [0, 0.0], type1 = None, cmd1 = '', t2 = [0, 0.0], type2 = None, cmd2 = '', t3 = [0, 0.0], type3 = None, cmd3 = '', t4 = [0, 0.0], type4 = None, cmd4 = ''):
		'''
		Constructor
		'''
		#super(SweetieBotCompoundAction, self).__init__(outcomes=['success', 'failure'])

		# Define C-like structure types
		class Parameters: 
			def __init__(self, t, tp, cmd):
				self.t, self.type, self.cmd = t, tp, cmd

		class SimpleAction:
			def __init__(self, previous_action, delay, state, flexbe_state, success_outcome, finish_time, description):
				self.previous_action, self.delay, self.state, self.flexbe_state, self.success_outcome, self.finish_time, self.description = previous_action, delay, state, flexbe_state, success_outcome, finish_time, description

		# Decompose parameters and create children flexbe states.
		params = [ Parameters(t1, type1, cmd1),
				   Parameters(t2, type2, cmd2),
				   Parameters(t3, type3, cmd3),
				   Parameters(t4, type4, cmd4) ]

		# convert parameteres to actions
		actions = list()
		for i, p in enumerate(params):
			if not isinstance(p.t, (tuple, list)) or not isinstance(p.t[0], int) or not isinstance(p.t[1], float):
				raise TypeError, 'Incorrect action time label %d. Time label must be tuple (int, float).' % i
			if p.t[0] >= i+1 or p.t[0] < 0:
				raise TypeError, 'Incorrect action time label %d. Time label must reference only previously defined actions.' % i

			if p.type == None:
				actions.append(None)
			else:
				# get parameters
				previous_action = None if p.t[0] == 0 else actions[p.t[0]-1]
				delay = p.t[1]
				description = p.type + ' ' + p.cmd
				if not isinstance(p.type, str):
					raise TypeError, 'Incorrect action type %d. Type must be string or None.' % i
				# Select action
				if p.type == 'motion/joint_trajectory':
					flexbe_state = ExecuteStoredJointTrajectoryState(action_topic = 'motion/controller/joint_trajectory', trajectory_param = 'joint_trajectory/' +  p.cmd)
					success_outcome = 'success'
				else:
					flexbe_state = TextCommandState(p.type, p.cmd, topic = 'control')
					success_outcome = 'done'
				# add action to list
				actions.append( SimpleAction(previous_action, delay, 'WAITING', flexbe_state, success_outcome, 0.0, description) )
		# remove None actions
		actions = [ a for a in actions if a != None ]

        # setup key lists
		#TODO correct key remapping
		#input_keys = sum( map(lambda n: [ ik + str(n) for ik in actions[n].flexbe_state.get_registered_input_keys() ], range(len(actions))), []) 
		#output_keys = sum( map(lambda n: [ ok + str(n) for ok in actions[n].flexbe_state._output_keys ], range(len(actions))), []) 
		#smach.Remapper(userdata,
                       #action.flexbe_state.get_registered_input_keys(),
                       #action.flexbe_state.get_registered_output_keys(),
                       #{key: key + str(n) for key in action.flexbe_state.get_registered_input_keys() + action.flexbe_state.get_registered_output_keys()}) 
		# instance superclass
		super(SweetieBotCompoundAction, self).__init__(outcomes=['success', 'failure'],
							input_keys = list( set.union(*[ actions[n].flexbe_state._input_keys for n in range(len(actions)) ]) ),
							output_keys = list( set.union(*[ actions[n].flexbe_state._output_keys for n in range(len(actions)) ]) ) )

		# set class fields
		self._start_time = None
		self._pause_time = None
		self._enter_userdata = None
		self._actions = actions
		
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
				action.flexbe_state.on_resume()

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
					Logger.loginfo('CompoundAction t = %f: start action: "%s"' % (t, action.description))

			if action.state == 'RUNNING':
				is_finished = False
				# execute state
				outcome = action.flexbe_state._EventState__execute(userdata)
				# check outcome
				if outcome and outcome != 'loopback':
					action.flexbe_state.on_exit(userdata)
					action.state = 'FINISHED'
					action.finish_time = t
					# set flag if action has failed
					if outcome != action.success_outcome:
						is_failed = True
					Logger.loginfo('CompoundAction t = %f: finish action: "%s" outcome: %s' % (t, action.description, outcome))
		# check results
		if is_finished:
			return 'success'
		if is_failed:
			return 'failure'

