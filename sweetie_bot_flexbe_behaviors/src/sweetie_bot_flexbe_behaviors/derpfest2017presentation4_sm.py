#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_derpfest2017presentation4')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.wait_for_message_state import WaitForMessageState
from sweetie_bot_flexbe_states.rand_head_movements import SweetieBotRandHeadMovements
from flexbe_manipulation_states.srdf_state_to_moveit import SrdfStateToMoveit
from sweetie_bot_flexbe_states.sweetie_bot_compound_action_state import SweetieBotCompoundAction
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Oct 28 2017
@author: disRecord
'''
class Derpfest2017Presentation4SM(Behavior):
	'''
	Forth part of Derpfest2017 presentation.
	'''


	def __init__(self):
		super(Derpfest2017Presentation4SM, self).__init__()
		self.name = 'Derpfest2017Presentation4'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		control_topic = 'control'
		joint_trajectory_topic = 'motion/controller/joint_trajectory'
		joy_topic = '/hmi/joystick'
		moveit_action = 'move_group'
		# x:68 y:631, x:130 y:353
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:414 y:360, x:130 y:353, x:506 y:357, x:233 y:351, x:720 y:363, x:715 y:411
		_sm_randmovements_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('failed', [('WaitKey', 'unavailable')]),
										('finished', [('RandMovements', 'done')]),
										('finished', [('WaitKey', 'received')]),
										('failed', [('RandMovements', 'failed')])
										])

		with _sm_randmovements_0:
			# x:148 y:122
			OperatableStateMachine.add('WaitKey',
										WaitForMessageState(topic=joy_topic, condition=lambda x: any(x.buttons), buffered=False, clear=False),
										transitions={'received': 'finished', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:366 y:118
			OperatableStateMachine.add('RandMovements',
										SweetieBotRandHeadMovements(controller='joint_state_head', duration=120, interval=[3,5], max2356=[0.3,0.3,1.5,1.5], min2356=[-0.3,-0.3,-1.5,-1.5]),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})



		with _state_machine:
			# x:108 y:53
			OperatableStateMachine.add('RandMovements',
										_sm_randmovements_0,
										transitions={'finished': 'HeadBasic', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:351 y:150
			OperatableStateMachine.add('HeadBasic',
										SrdfStateToMoveit(config_name='head_upright', move_group='head', action_topic=moveit_action, robot_name=''),
										transitions={'reached': 'GoodBye', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:329 y:271
			OperatableStateMachine.add('GoodBye',
										SweetieBotCompoundAction(t1=[0,0.0], type1='voice/play_wav', cmd1='thank_you_for_attention_nopony_was_harmed', t2=[0,0.0], type2='motion/joint_trajectory', cmd2='introduce_herself', t3=[2,2.0], type3='voice/play_wav', cmd3='am_i_really_a_good_actress', t4=[2,2.0], type4='motion/joint_trajectory', cmd4='head_suprised'),
										transitions={'success': 'Bow', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})

			# x:288 y:547
			OperatableStateMachine.add('RandMovements2',
										SweetieBotRandHeadMovements(controller='joint_state_head', duration=120, interval=[3,5], max2356=[0.3,0.3,1.5,1.5], min2356=[-0.3,-0.3,-1.5,-1.5]),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:319 y:387
			OperatableStateMachine.add('Bow',
										SweetieBotCompoundAction(t1=[0,2.0], type1='motion/joint_trajectory', cmd1='bow_begin', t2=[1,2.0], type2='motion/joint_trajectory', cmd2='bow_end', t3=[0,0.0], type3=None, cmd3='', t4=[0,0.0], type4=None, cmd4=''),
										transitions={'success': 'RandMovements2', 'failure': 'failed'},
										autonomy={'success': Autonomy.Off, 'failure': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
