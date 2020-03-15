#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.srdf_state_to_moveit import SrdfStateToMoveit
from binpicking_flexbe_states.calculate_object_pose_state import CalculateObjectPoseState
from binpicking_flexbe_states.take_photo_state import TakePhotoState
from binpicking_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Mar 14 2020
@author: Gerard Harkema
'''
class BinpickingSM(Behavior):
	'''
	Binpicking state machine
	'''


	def __init__(self):
		super(BinpickingSM, self).__init__()
		self.name = 'Binpicking'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		pick_group = 'manipulator'
		# x:336 y:593, x:317 y:372
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('GoHomeStart',
										SrdfStateToMoveit(config_name='HomePos', move_group=pick_group, action_topic='/move_group', robot_name=""),
										transitions={'reached': 'GoPhotoPosition', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:428 y:570
			OperatableStateMachine.add('GoHomeEnd',
										SrdfStateToMoveit(config_name='HomePos', move_group=pick_group, action_topic='/move_group', robot_name=""),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:591 y:42
			OperatableStateMachine.add('CalculateObjectPose',
										CalculateObjectPoseState(target_time=5.0),
										transitions={'continue': 'GoPreGraspPosition', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:217 y:38
			OperatableStateMachine.add('GoPhotoPosition',
										SrdfStateToMoveit(config_name='PhotoPos', move_group=pick_group, action_topic='/move_group', robot_name=""),
										transitions={'reached': 'TakePhoto', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:756 y:43
			OperatableStateMachine.add('GoPreGraspPosition',
										SrdfStateToMoveit(config_name='PreGraspPos', move_group=pick_group, action_topic='/move_group', robot_name=""),
										transitions={'reached': 'GoGraspPosition', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:758 y:308
			OperatableStateMachine.add('GoObjectLiftPosition',
										SrdfStateToMoveit(config_name='PreGraspPos', move_group=pick_group, action_topic='/move_group', robot_name=""),
										transitions={'reached': 'GoHomeTransfer', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:425 y:40
			OperatableStateMachine.add('TakePhoto',
										TakePhotoState(target_time=3.0),
										transitions={'continue': 'CalculateObjectPose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:760 y:133
			OperatableStateMachine.add('GoGraspPosition',
										SrdfStateToMoveit(config_name='GraspPos', move_group=pick_group, action_topic='/move_group', robot_name=""),
										transitions={'reached': 'GraspObject', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:755 y:224
			OperatableStateMachine.add('GraspObject',
										VacuumGripperControlState(target_time=4),
										transitions={'continue': 'GoObjectLiftPosition', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:757 y:398
			OperatableStateMachine.add('GoHomeTransfer',
										SrdfStateToMoveit(config_name='HomePos', move_group=pick_group, action_topic='/move_group', robot_name=""),
										transitions={'reached': 'GoDropPosition', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:756 y:494
			OperatableStateMachine.add('GoDropPosition',
										SrdfStateToMoveit(config_name='DropPos', move_group=pick_group, action_topic='/move_group', robot_name=""),
										transitions={'reached': 'DropObject', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:751 y:575
			OperatableStateMachine.add('DropObject',
										VacuumGripperControlState(target_time=2),
										transitions={'continue': 'GoHomeEnd', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
