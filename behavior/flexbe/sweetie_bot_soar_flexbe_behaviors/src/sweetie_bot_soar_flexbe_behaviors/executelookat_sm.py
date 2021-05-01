#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from sweetie_bot_flexbe_states.object_detection_monitor import ObjectDetectionMonitor
from sweetie_bot_flexbe_states.set_operational import SetOperational
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 18 2019
@author: disRecord
'''
class ExecuteLookAtSM(Behavior):
	'''
	Starts LookAt controller and follow detected object, defined by (label, type) pair. Interruptable behavior.
	'''


	def __init__(self):
		super(ExecuteLookAtSM, self).__init__()
		self.name = 'ExecuteLookAt'

		# parameters of this behavior
		self.add_parameter('label', '*')
		self.add_parameter('type', '*')
		self.add_parameter('timeout', 30)
		self.add_parameter('transform_delay', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:353, x:346 y:329
		_state_machine = OperatableStateMachine(outcomes=['succeed', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:353, x:130 y:353, x:230 y:353, x:311 y:368, x:651 y:309, x:537 y:357, x:391 y:376, x:793 y:299
		_sm_lookatcontainer_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('LookAtOperational', 'done')]),
										('failed', [('LookAtOperational', 'failure')]),
										('finished', [('ObjectMonitor', 'no_detections')]),
										('failed', [('ObjectMonitor', 'have_detections')]),
										('failed', [('ObjectMonitor', 'detection_matches')]),
										('failed', [('ObjectMonitor', 'failure')])
										])

		with _sm_lookatcontainer_0:
			# x:101 y:80
			OperatableStateMachine.add('LookAtOperational',
										SetOperational(controller='motion/controller/look_at', operational=True, resources=['head'], sync=True),
										transitions={'done': 'finished', 'failure': 'failed'},
										autonomy={'done': Autonomy.Off, 'failure': Autonomy.Off})

			# x:361 y:83
			OperatableStateMachine.add('ObjectMonitor',
										ObjectDetectionMonitor(detection_topic='detections', label=self.label, type=self.type, exit_states=['no_detections'], pose_topic='motion/controller/look_at/in_pose_ref', pose_frame_id='odom_combined', detection_period=float(self.timeout), transform_delay=0.0),
										transitions={'no_detections': 'finished', 'have_detections': 'failed', 'detection_matches': 'failed', 'failure': 'failed'},
										autonomy={'no_detections': Autonomy.Off, 'have_detections': Autonomy.Off, 'detection_matches': Autonomy.Off, 'failure': Autonomy.Off},
										remapping={'pose': 'pose'})



		with _state_machine:
			# x:153 y:101
			OperatableStateMachine.add('LookAtContainer',
										_sm_lookatcontainer_0,
										transitions={'finished': 'succeed', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
