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
from rover_auto_flexbe_states.flash_LED import flash_LED
from rover_auto_flexbe_states.enter_gps_coords import enter_gps_coords
from rover_auto_flexbe_states.drive_distance import drive_distance
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
		# x:701 y:446, x:66 y:231
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:334 y:308, x:130 y:373
		_sm_go_back_to_coords_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_go_back_to_coords_0:
			# x:30 y:40
			OperatableStateMachine.add('go back to leg',
										nav_to_target(gps_coordinates=null),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:354 y:308, x:130 y:373
		_sm_basic_search2_1 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_basic_search2_1:
			# x:30 y:40
			OperatableStateMachine.add('vanilla drive',
										drive_distance(distance=1),
										transitions={'continue': 'drive to gps', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:154 y:109
			OperatableStateMachine.add('drive to gps',
										nav_to_target(gps_coordinates=null),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:30 y:365, x:334 y:301
		_sm_basic_search_2 = OperatableStateMachine(outcomes=['failed', 'continue'])

		with _sm_basic_search_2:
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
										_sm_basic_search_2,
										transitions={'failed': 'failed', 'continue': 'Flash LED'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit})

			# x:304 y:85
			OperatableStateMachine.add('Flash LED',
										flash_LED(),
										transitions={'continue': 'Enter GPS Leg 2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:478 y:147
			OperatableStateMachine.add('Enter GPS Leg 2',
										enter_gps_coords(),
										transitions={'continue': 'basic_search2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:690 y:201
			OperatableStateMachine.add('basic_search2',
										_sm_basic_search2_1,
										transitions={'finished': 'finished', 'failed': 'go_back_to_coords'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:474 y:244
			OperatableStateMachine.add('go_back_to_coords',
										_sm_go_back_to_coords_0,
										transitions={'finished': 'Enter GPS Leg 2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
