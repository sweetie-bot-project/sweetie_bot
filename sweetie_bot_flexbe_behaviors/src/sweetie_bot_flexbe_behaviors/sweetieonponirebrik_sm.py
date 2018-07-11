#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_sweetieonponirebrik')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.decision_state import DecisionState
from sweetie_bot_flexbe_states.text_command_state import TextCommandState
from flexbe_states.wait_state import WaitState
from sweetie_bot_flexbe_states.publisher_state import PublisherState
from behavior_dotricks.dotricks_sm import DoTricksSM
from sweetie_bot_flexbe_states.rand_head_movements_state import SweetieRandHeadMovementsState
from flexbe_states.subscriber_state import SubscriberState
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
		self.add_parameter('wait_time', 1)

		# references to used behaviors
		self.add_behavior(DoTricksSM, 'DoTricks')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		voice_topic = 'control'
		torque_off_service = 'motion/controller/torque_off/set_operational'
		joint_state_control_topic = 'motion/controller/joint_state/out_joints_src_reset'
		joy_topic = '/hmi/joystick'
		eyes_topic = 'control'
		# x:68 y:535, x:786 y:525
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.be_evil = self.be_evil

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:554 y:394, x:130 y:397
		_sm_waitforjoyevent_0 = OperatableStateMachine(outcomes=['received', 'unavailable'])

		with _sm_waitforjoyevent_0:
			# x:288 y:158
			OperatableStateMachine.add('JoyEvent',
										SubscriberState(topic=joy_topic, blocking=True, clear=False),
										transitions={'received': 'CheckEvent', 'unavailable': 'unavailable'},
										autonomy={'received': Autonomy.Off, 'unavailable': Autonomy.Off},
										remapping={'message': 'message'})

			# x:485 y:159
			OperatableStateMachine.add('CheckEvent',
										DecisionState(outcomes=['noactivity', 'activity'], conditions=lambda msg: 'activity' if any(msg.buttons) or any(msg.axes[0:5]) else 'noactivity'),
										transitions={'noactivity': 'JoyEvent', 'activity': 'received'},
										autonomy={'noactivity': Autonomy.Off, 'activity': Autonomy.Off},
										remapping={'input_value': 'message'})


		# x:538 y:386, x:347 y:384, x:134 y:383, x:831 y:385, x:906 y:386
		_sm_idlebehavior_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('RandomHeadMovements', 'done')]),
										('failed', [('WaitForJoyEvent', 'unavailable')]),
										('finished', [('WaitForJoyEvent', 'received')])
										])

		with _sm_idlebehavior_1:
			# x:474 y:110
			OperatableStateMachine.add('RandomHeadMovements',
										SweetieRandHeadMovementsState(topic=joint_state_control_topic, duration=100, interval=[1, 4], max2356=[ 0.3, 0.3, 1, 1], min2356=[ -0.3, -0.3, -1, -1 ]),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:144 y:133
			OperatableStateMachine.add('WaitForJoyEvent',
										_sm_waitforjoyevent_0,
										transitions={'received': 'finished', 'unavailable': 'failed'},
										autonomy={'received': Autonomy.Inherit, 'unavailable': Autonomy.Inherit})



		with _state_machine:
			# x:79 y:99
			OperatableStateMachine.add('CheckEvil',
										DecisionState(outcomes=['good', 'evil'], conditions=lambda x: 'evil' if x else 'good'),
										transitions={'good': 'NormalEyes', 'evil': 'RedEyes'},
										autonomy={'good': Autonomy.Off, 'evil': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:550 y:467
			OperatableStateMachine.add('AskForAssistance',
										TextCommandState(type='voice/play_wav', command='assistance', topic=voice_topic),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:440 y:126
			OperatableStateMachine.add('WaitForMovementFinish',
										WaitState(wait_time=0.0),
										transitions={'done': 'DoTricks'},
										autonomy={'done': Autonomy.Off})

			# x:251 y:503
			OperatableStateMachine.add('StartingStance',
										PublisherState(topic=joint_state_control_topic, msg_type=JointState, value={'name': ['joint51','joint52','joint53','eyes_pitch','eyes_yaw'], 'position': [0.2, 0.0, 0.2, 0.0, 0.0]}),
										transitions={'done': 'CheckEvil2', 'failed': 'AskForAssistance'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:731 y:153
			OperatableStateMachine.add('DoTricks',
										self.use_behavior(DoTricksSM, 'DoTricks'),
										transitions={'finished': 'NormalLook', 'failed': 'AskForAssistance'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'be_evil': 'be_evil'})

			# x:27 y:219
			OperatableStateMachine.add('RedEyes',
										TextCommandState(type='eyes/emotion', command='red_eyes', topic=eyes_topic),
										transitions={'done': 'IdleBehavior'},
										autonomy={'done': Autonomy.Off})

			# x:222 y:65
			OperatableStateMachine.add('NormalEyes',
										TextCommandState(type='eyes/emotion', command='normal', topic=eyes_topic),
										transitions={'done': 'IdleBehavior'},
										autonomy={'done': Autonomy.Off})

			# x:481 y:305
			OperatableStateMachine.add('IdleBehavior',
										_sm_idlebehavior_1,
										transitions={'finished': 'StartingStance', 'failed': 'AskForAssistance'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:181 y:360
			OperatableStateMachine.add('CheckEvil2',
										DecisionState(outcomes=['evil','good'], conditions=lambda x: 'evil' if x else 'good'),
										transitions={'evil': 'EvilLook', 'good': 'WaitForMovementFinish'},
										autonomy={'evil': Autonomy.Off, 'good': Autonomy.Off},
										remapping={'input_value': 'be_evil'})

			# x:188 y:294
			OperatableStateMachine.add('EvilLook',
										TextCommandState(type='eyes/emotion', command='evil_look', topic=eyes_topic),
										transitions={'done': 'WaitForMovementFinish'},
										autonomy={'done': Autonomy.Off})

			# x:880 y:311
			OperatableStateMachine.add('NormalLook',
										TextCommandState(type='eyes/emotion', command='normal_look', topic=eyes_topic),
										transitions={'done': 'IdleBehavior'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
