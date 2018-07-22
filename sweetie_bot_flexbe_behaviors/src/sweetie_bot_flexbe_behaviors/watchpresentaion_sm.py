#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.wait_for_message_state import WaitForMessageState
from sweetie_bot_flexbe_states.rand_head_movements import SweetieBotRandHeadMovements
from flexbe_manipulation_states.moveit_to_joints_state import MoveitToJointsState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Oct 21 2017
@author: disRecord
'''
class WatchPresentaionSM(Behavior):
	'''
	SweetieBot watch Derpfest2017 presentation.
	'''


	def __init__(self):
		super(WatchPresentaionSM, self).__init__()
		self.name = 'WatchPresentaion'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joy_topic = '/hmi/joystick'
		moveit_action = 'move_group'
		# x:57 y:300, x:603 y:211
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['head_pose_joints', 'rand_head_config'])
		_state_machine.userdata.head_pose_joints = [ 0.0, 0.6, 0.0, 0.0, 0.0, 0.0 ]
		_state_machine.userdata.rand_head_config = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:353, x:130 y:353, x:230 y:353, x:330 y:353, x:430 y:353, x:530 y:353
		_sm_randheadwaitkey_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['rand_head_config'], conditions=[
										('failed', [('WaitKey', 'unavailable')]),
										('finished', [('WaitKey', 'received')]),
										('failed', [('RandHeadMovements', 'failed')]),
										('finished', [('RandHeadMovements', 'done')])
										])

		with _sm_randheadwaitkey_0:
			# x:136 y:116
			OperatableStateMachine.add('WaitKey',
										WaitForMessageState(topic=joy_topic, condition=lambda x: any(x.buttons), buffered=False, clear=False),
										transitions={'received': 'finished', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:403 y:118
			OperatableStateMachine.add('RandHeadMovements',
										SweetieBotRandHeadMovements(controller='joint_state_head', duration=600, interval=[1,4], max2356=[0.7,0.3,1,1], min2356=[0.5,0.1,-1,-1]),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'config': 'rand_head_config'})



		with _state_machine:
			# x:200 y:96
			OperatableStateMachine.add('RandHeadWaitKey',
										_sm_randheadwaitkey_0,
										transitions={'finished': 'PlaceHead1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'rand_head_config': 'rand_head_config'})

			# x:193 y:217
			OperatableStateMachine.add('PlaceHead1',
										MoveitToJointsState(move_group='head', joint_names=['joint51','joint52','joint53', 'joint54'], action_topic=moveit_action),
										transitions={'reached': 'finished', 'planning_failed': 'PlaceHead2', 'control_failed': 'PlaceHead2'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'head_pose_joints'})

			# x:189 y:370
			OperatableStateMachine.add('PlaceHead2',
										MoveitToJointsState(move_group='head', joint_names=['joint51','joint52','joint53', 'joint54'], action_topic=moveit_action),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'head_pose_joints'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
