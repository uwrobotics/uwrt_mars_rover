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
from rover_auto_flexbe_states.AR_search import drive_to_AR as rover_auto_flexbe_states__drive_to_AR
from rover_auto_flexbe_states.AR_search import AR_search
from rover_auto_flexbe_states.drive_through_gate import drive_through_gate
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
		# x:945 y:626, x:84 y:390
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:373, x:130 y:373
		_sm_basic_search4_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_basic_search4_0:
			# x:30 y:40
			OperatableStateMachine.add('get fix',
										drive_distance(distance=1),
										transitions={'continue': 'drive to gps', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:30 y:132
			OperatableStateMachine.add('drive to gps',
										nav_to_target(gps_coordinates=null),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:280 y:288, x:130 y:373
		_sm_go_back_to_l3_start_1 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_go_back_to_l3_start_1:
			# x:30 y:40
			OperatableStateMachine.add('drive to L3 start',
										nav_to_target(gps_coordinates=null),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:356 y:334, x:130 y:373
		_sm_basic_search3_2 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_basic_search3_2:
			# x:30 y:40
			OperatableStateMachine.add('get gps fix',
										drive_distance(distance=1),
										transitions={'continue': 'drive to target gps', 'failed': 'get gps fix'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:171 y:150
			OperatableStateMachine.add('drive to target gps',
										nav_to_target(gps_coordinates=null),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:334 y:308, x:130 y:373
		_sm_go_back_to_l2_start_3 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_go_back_to_l2_start_3:
			# x:30 y:40
			OperatableStateMachine.add('go back to leg',
										nav_to_target(gps_coordinates=null),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:436 y:267, x:130 y:373
		_sm_basic_search2_4 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_basic_search2_4:
			# x:30 y:40
			OperatableStateMachine.add('vanilla drive',
										drive_distance(distance=1),
										transitions={'continue': 'drive to gps', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:154 y:132
			OperatableStateMachine.add('drive to gps',
										nav_to_target(gps_coordinates=null),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:30 y:365, x:334 y:301
		_sm_basic_search_5 = OperatableStateMachine(outcomes=['failed', 'continue'])

		with _sm_basic_search_5:
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
										_sm_basic_search_5,
										transitions={'failed': 'failed', 'continue': 'Flash LED'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit})

			# x:304 y:85
			OperatableStateMachine.add('Flash LED',
										flash_LED(),
										transitions={'continue': 'Enter GPS Leg 2', 'failed': 'Flash LED'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:496 y:132
			OperatableStateMachine.add('Enter GPS Leg 2',
										enter_gps_coords(),
										transitions={'continue': 'basic_search2', 'failed': 'Enter GPS Leg 2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:749 y:198
			OperatableStateMachine.add('basic_search2',
										_sm_basic_search2_4,
										transitions={'finished': 'Flash LED 2', 'failed': 'go_back_to_L2_start'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:474 y:244
			OperatableStateMachine.add('go_back_to_L2_start',
										_sm_go_back_to_l2_start_3,
										transitions={'finished': 'Enter GPS Leg 2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:751 y:388
			OperatableStateMachine.add('Enter GPS Leg 3',
										enter_gps_coords(),
										transitions={'continue': 'basic_search3', 'failed': 'Enter GPS Leg 3'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:709 y:482
			OperatableStateMachine.add('basic_search3',
										_sm_basic_search3_2,
										transitions={'finished': 'drive forward', 'failed': 'go_back_to_L3_start'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:473 y:338
			OperatableStateMachine.add('go_back_to_L3_start',
										_sm_go_back_to_l3_start_1,
										transitions={'finished': 'Enter GPS Leg 3', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:485 y:492
			OperatableStateMachine.add('drive forward',
										drive_distance(distance=3),
										transitions={'continue': 'drive to AR pole', 'failed': 'Search for AR'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:300 y:619
			OperatableStateMachine.add('drive to AR pole',
										rover_auto_flexbe_states__drive_to_AR(),
										transitions={'continue': 'Flash LED 3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:280 y:476
			OperatableStateMachine.add('Search for AR',
										AR_search(),
										transitions={'continue': 'drive to AR pole', 'failed': 'go_back_to_L3_start'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:695 y:611
			OperatableStateMachine.add('Flash LED 3',
										flash_LED(),
										transitions={'continue': 'Enter GPS Leg 4', 'failed': 'Flash LED 3'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:741 y:278
			OperatableStateMachine.add('Flash LED 2',
										flash_LED(),
										transitions={'continue': 'Enter GPS Leg 3', 'failed': 'Flash LED 2'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1016 y:72
			OperatableStateMachine.add('Enter GPS Leg 4',
										enter_gps_coords(),
										transitions={'continue': 'basic_search4', 'failed': 'Enter GPS Leg 4'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:980 y:238
			OperatableStateMachine.add('basic_search4',
										_sm_basic_search4_0,
										transitions={'finished': 'Search for gate', 'failed': 'go_back_to_L4_start'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1210 y:329
			OperatableStateMachine.add('Search for gate',
										AR_search(),
										transitions={'continue': 'Drive through gate', 'failed': 'go_back_to_L4_start'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1019 y:406
			OperatableStateMachine.add('Drive through gate',
										drive_through_gate(),
										transitions={'continue': 'Flash LED 4', 'failed': 'go_back_to_L4_start'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1193 y:131
			OperatableStateMachine.add('go_back_to_L4_start',
										nav_to_target(gps_coordinates=null),
										transitions={'continue': 'Enter GPS Leg 4', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1000 y:525
			OperatableStateMachine.add('Flash LED 4',
										flash_LED(),
										transitions={'continue': 'finished', 'failed': 'Flash LED 4'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
