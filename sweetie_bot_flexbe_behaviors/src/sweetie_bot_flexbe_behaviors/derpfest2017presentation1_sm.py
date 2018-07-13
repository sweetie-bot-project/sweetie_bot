#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.wait_for_message_state import WaitForMessageState
from sweetie_bot_flexbe_states.rand_head_movements import SweetieBotRandHeadMovements
from sweetie_bot_flexbe_states.sweetie_bot_compound_action_state import SweetieBotCompoundAction
from flexbe_manipulation_states.moveit_to_joints_state import MoveitToJointsState
from flexbe_manipulation_states.srdf_state_to_moveit import SrdfStateToMoveit
from sweetie_bot_flexbe_behaviors.watchpresentaion_sm import WatchPresentaionSM
from sweetie_bot_flexbe_states.execute_stored_trajectory_state import ExecuteStoredJointTrajectoryState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Oct 21 2017
@author: disRecord
'''
class Derpfest2017Presentation1SM(Behavior):
	'''
	First part of SweetieBot presentation on Derpfest2017.
	'''


	def __init__(self):
		super(Derpfest2017Presentation1SM, self).__init__()
		self.name = 'Derpfest2017Presentation1'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(WatchPresentaionSM, 'WatchPresentaion')
		self.add_behavior(WatchPresentaionSM, 'WatchPresentaion_2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		control_topic = 'control'
		voice_topic = 'control'
		joint_trajectory_action = 'motion/controller/joint_trajectory'
		joy_topic = '/hmi/joystick'
		moveit_action = 'move_group'
		storage = '/sweetie_bot/joint_trajectory'
		# x:157 y:271, x:501 y:342
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.joint_head_turned_right = [ 0.0, 0.6, 0.0, 0.0 ]
		_state_machine.userdata.head_forward_joints = [ 0.15, 0.0, -0.15, 0.0 ]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:803 y:85, x:480 y:341
		_sm_turnright_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['joint_head_turned_right'])

		with _sm_turnright_0:
			# x:282 y:82
			OperatableStateMachine.add('TurnHead',
										MoveitToJointsState(move_group='head', joint_names=['joint51','joint52','joint53','joint54'], action_topic=moveit_action),
										transitions={'reached': 'TurnBody', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'joint_head_turned_right'})

			# x:466 y:69
			OperatableStateMachine.add('TurnBody',
										SweetieBotCompoundAction(t1=[0,0.0], type1='motion/joint_trajectory', cmd1='turn_right', t2=[1,0.0], type2='motion/joint_trajectory', cmd2='turn_right', t3=[2,0.0], type3='motion/joint_trajectory', cmd3='turn_right', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'finished', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})


		# x:30 y:353, x:148 y:355, x:529 y:348, x:443 y:347, x:840 y:329, x:752 y:347
		_sm_waitbegining_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('WaitKey', 'received')]),
										('failed', [('WaitKey', 'unavailable')]),
										('failed', [('RandHeadMovemets', 'failed')]),
										('finished', [('RandHeadMovemets', 'done')])
										])

		with _sm_waitbegining_1:
			# x:167 y:134
			OperatableStateMachine.add('WaitKey',
										WaitForMessageState(topic=joy_topic, condition=lambda x: any(x.buttons), buffered=False, clear=False),
										transitions={'received': 'finished', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:442 y:126
			OperatableStateMachine.add('RandHeadMovemets',
										SweetieBotRandHeadMovements(controller='joint_state_head', duration=600, interval=[3,5], max2356=[0.3,0.3,1.5,1.5], min2356=[-0.3,-0.3,-1.5,-1.5]),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})



		with _state_machine:
			# x:25 y:176
			OperatableStateMachine.add('WaitBegining',
										_sm_waitbegining_1,
										transitions={'finished': 'HeadPose', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:343 y:49
			OperatableStateMachine.add('WaitKey',
										WaitForMessageState(topic=joy_topic, condition=lambda x: any(x.buttons), buffered=False, clear=False),
										transitions={'received': 'TurnRight', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:785 y:172
			OperatableStateMachine.add('AskAboutProto1',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='what_proto1', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_suprised_slow', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey2', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:799 y:264
			OperatableStateMachine.add('WaitKey2',
										WaitForMessageState(topic=joy_topic, condition=lambda x: any(x.buttons), buffered=False, clear=False),
										transitions={'received': 'AskAboutProto1Again', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:797 y:379
			OperatableStateMachine.add('AskAboutProto1Again',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='tell_me_more_about_proto1', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='head_lean_forward_begin', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey4', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:801 y:518
			OperatableStateMachine.add('WaitKey4',
										WaitForMessageState(topic=joy_topic, condition=lambda x: any(x.buttons), buffered=False, clear=False),
										transitions={'received': 'AskWhatHappensToProto1', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:603 y:600
			OperatableStateMachine.add('AskWhatHappensToProto1',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='what_happened_with_proto1', t2=[0,1.2], type2='motion/joint_trajectory', cmd2='hoof_wave', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey3', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:332 y:601
			OperatableStateMachine.add('WaitKey3',
										WaitForMessageState(topic=joy_topic, condition=lambda x: any(x.buttons), buffered=False, clear=False),
										transitions={'received': 'SayItIsOk', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:554 y:51
			OperatableStateMachine.add('TurnRight',
										_sm_turnright_0,
										transitions={'finished': 'WatchPresentaion', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'joint_head_turned_right': 'joint_head_turned_right'})

			# x:81 y:597
			OperatableStateMachine.add('SayItIsOk',
										SweetieBotCompoundAction(t1=[0,0.0], type1='motion/joint_trajectory', cmd1='head_lean_forward_end', t2=[0,0.0], type2='voice/play_wav', cmd2='yes_exactly_tell_me_more', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WatchPresentaion_2', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:137 y:50
			OperatableStateMachine.add('Hello',
										SweetieBotCompoundAction(t1=[0,0.0], type1='motion/joint_trajectory', cmd1='introduce_herself', t2=[0,0.0], type2='voice/play_wav', cmd2='hello_im_sweetie_bot_presentation', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'WaitKey', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:166 y:159
			OperatableStateMachine.add('HeadPose',
										SrdfStateToMoveit(config_name='head_basic', move_group='head', action_topic='move_group', robot_name=''),
										transitions={'reached': 'Hello', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:776 y:30
			OperatableStateMachine.add('WatchPresentaion',
										self.use_behavior(WatchPresentaionSM, 'WatchPresentaion'),
										transitions={'finished': 'AskAboutProto1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'head_pose_joints': 'joint_head_turned_right'})

			# x:92 y:465
			OperatableStateMachine.add('WatchPresentaion_2',
										self.use_behavior(WatchPresentaionSM, 'WatchPresentaion_2'),
										transitions={'finished': 'Greeting', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'head_pose_joints': 'head_forward_joints'})

			# x:73 y:332
			OperatableStateMachine.add('Greeting',
										ExecuteStoredJointTrajectoryState(action_topic=joint_trajectory_action, trajectory_param='joint_trajectory/greeting'),
										transitions={'success': 'finished', 'partial_movement': 'failed', 'invalid_pose': 'failed', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'partial_movement': Autonomy.Off, 'invalid_pose': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'result': 'result'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
