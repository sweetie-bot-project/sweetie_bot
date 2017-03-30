#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_sweetieonponirebrik')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from behavior_dotricks.dotricks_sm import DoTricksSM
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from sweetie_bot_flexbe_states.rand_head_movements_state import SweetieRandHeadMovementsState
from flexbe_states.subscriber_state import SubscriberState
from flexbe_states.wait_state import WaitState
from sweetie_bot_flexbe_states.publisher_state import PublisherState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from sensor_msgs.msg import JointState
# [/MANUAL_IMPORT]


'''
Created on Tue Mar 28 2017
@author: disRecord
'''
class SweetieOnPoniRebrikSM(Behavior):
	'''
	Behavior for presentation on PoniRebrik.
	'''


	def __init__(self):
		super(SweetieOnPoniRebrikSM, self).__init__()
		self.name = 'SweetieOnPoniRebrik'

		# parameters of this behavior
		self.add_parameter('be_evil', False)

		# references to used behaviors
		self.add_behavior(DoTricksSM, 'DoTricks')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		voice_topic = 'voice/voice'
		torque_off_service = 'motion/controller/torque_off/set_operational'
		joint_state_control_topic = 'motion/controller/joint_state/out_joints_src_reset'
		joy_topic = '/hmi/joystick'
		# x:68 y:535, x:786 y:525
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.be_evil = self.be_evil

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:538 y:386, x:347 y:384, x:134 y:383, x:831 y:385, x:906 y:386
		_sm_idlebehavior_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('RandomHeadMovements', 'done')]),
										('finished', [('WaitForJoyEvent', 'received')]),
										('failed', [('WaitForJoyEvent', 'unavailable')])
										])

		with _sm_idlebehavior_0:
			# x:474 y:110
			OperatableStateMachine.add('RandomHeadMovements',
										SweetieRandHeadMovementsState(topic=joint_state_control_topic, duration=100, interval=[3, 5], max2356=[ 0.3, 0.3, 1.5, 1.5 ], min2356=[ -0.3, -0.3, -1.5, -1.5 ]),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:148 y:118
			OperatableStateMachine.add('WaitForJoyEvent',
										SubscriberState(topic=joy_topic, blocking=True, clear=True),
										transitions={'received': 'finished', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})



		with _state_machine:
			# x:368 y:279
			OperatableStateMachine.add('IdleBehavior',
										_sm_idlebehavior_0,
										transitions={'finished': 'StartingStance', 'failed': 'AskForAssistance'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:550 y:467
			OperatableStateMachine.add('AskForAssistance',
										TextCommandState(topic=voice_topic, type='voice/play_wav', command='assistance'),
										transitions={'done': 'failed', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:213 y:108
			OperatableStateMachine.add('WaitForMovementFinish',
										WaitState(wait_time=1),
										transitions={'done': 'DoTricks'},
										autonomy={'done': Autonomy.Off})

			# x:174 y:365
			OperatableStateMachine.add('StartingStance',
										PublisherState(topic=joint_state_control_topic, msg_type=JointState, value={'name': ['joint52','joint53','joint55','joint56'], 'position': [0.0, 0.0, 0.0, 0.0]}),
										transitions={'done': 'WaitForMovementFinish', 'failed': 'AskForAssistance'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:492 y:126
			OperatableStateMachine.add('DoTricks',
										self.use_behavior(DoTricksSM, 'DoTricks'),
										transitions={'finished': 'IdleBehavior', 'failed': 'AskForAssistance'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
