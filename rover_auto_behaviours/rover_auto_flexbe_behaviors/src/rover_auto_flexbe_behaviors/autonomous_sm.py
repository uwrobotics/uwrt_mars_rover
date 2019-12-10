#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from rover_auto_flexbe_states.move_forward import move_forward
from rover_auto_flexbe_states.nav_to_target import nav_to_target
from rover_auto_flexbe_states.flash_LED import flash_LED as rover_auto_flexbe_states__flash_LED
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Dec 10 2019
@author: Hamza Mahdi
'''
class AutonomousSM(Behavior):
	'''
	rover auto prototype
	'''


	def __init__(self):
		super(AutonomousSM, self).__init__()
		self.name = 'Autonomous'

		# parameters of this behavior
		self.add_parameter('gps_coord', '')
		self.add_parameter('time', 0)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:377 y:433, x:66 y:231
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:334 y:301
		_sm_basic_search_0 = OperatableStateMachine(outcomes=['failed', 'continue'])

		with _sm_basic_search_0:
			# x:30 y:40
			OperatableStateMachine.add('Get Coordinates',
										move_forward(target_time=self.time),
										transitions={'continue': 'Nav to Target', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:153 y:198
			OperatableStateMachine.add('Nav to Target',
										nav_to_target(gps_coordinates=self.gps_coord),
										transitions={'continue': 'continue', 'failed': 'Nav to Target'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})



		with _state_machine:
			# x:72 y:57
			OperatableStateMachine.add('basic_search',
										_sm_basic_search_0,
										transitions={'failed': 'failed', 'continue': 'Flash LED'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit})

			# x:304 y:85
			OperatableStateMachine.add('Flash LED',
										rover_auto_flexbe_states__flash_LED(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
