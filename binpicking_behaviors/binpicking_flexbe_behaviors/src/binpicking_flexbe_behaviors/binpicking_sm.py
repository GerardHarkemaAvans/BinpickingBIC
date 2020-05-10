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
from binpicking_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
from binpicking_flexbe_states.capture_pointcloud_state import CapturePointcloudState
from binpicking_flexbe_states.calculate_object_pose_state import CalculateObjectPoseState
from miscellaneous_flexbe_states.message_state import MessageState
from binpicking_flexbe_states.compute_grasp_state import ComputeGraspState
from binpicking_flexbe_states.moveit_to_joints_dyn_state import MoveitToJointsDynState as binpicking_flexbe_states__MoveitToJointsDynState
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
		names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
		gripper = 'ee_link'
		# x:336 y:593, x:317 y:372
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.captured_pointcloud = []
		_state_machine.userdata.part_pose = []
		_state_machine.userdata.pick_configuration = []
		_state_machine.userdata.suction_cup_offset = 0.06
		_state_machine.userdata.rotation = 0
		_state_machine.userdata.move_group_prefix = ''
		_state_machine.userdata.move_group = "manipulator"
		_state_machine.userdata.ee_link = 'ee_link'

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

			# x:217 y:38
			OperatableStateMachine.add('GoPhotoPosition',
										SrdfStateToMoveit(config_name='PhotoPos', move_group=pick_group, action_topic='/move_group', robot_name=""),
										transitions={'reached': 'CapturePointcloud', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:743 y:39
			OperatableStateMachine.add('GoPreGraspPosition',
										SrdfStateToMoveit(config_name='PreGraspPos', move_group=pick_group, action_topic='/move_group', robot_name=""),
										transitions={'reached': 'ComputePickPoint', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:911 y:297
			OperatableStateMachine.add('GoObjectLiftPosition',
										SrdfStateToMoveit(config_name='PreGraspPos', move_group=pick_group, action_topic='/move_group', robot_name=""),
										transitions={'reached': 'GoHomeTransfer', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:909 y:210
			OperatableStateMachine.add('GraspObject',
										VacuumGripperControlState(target_time=4),
										transitions={'continue': 'GoObjectLiftPosition', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:909 y:393
			OperatableStateMachine.add('GoHomeTransfer',
										SrdfStateToMoveit(config_name='HomePos', move_group=pick_group, action_topic='/move_group', robot_name=""),
										transitions={'reached': 'GoDropPosition', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:908 y:482
			OperatableStateMachine.add('GoDropPosition',
										SrdfStateToMoveit(config_name='DropPos', move_group=pick_group, action_topic='/move_group', robot_name=""),
										transitions={'reached': 'DropObject', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:903 y:575
			OperatableStateMachine.add('DropObject',
										VacuumGripperControlState(target_time=2),
										transitions={'continue': 'GoHomeEnd', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:384 y:38
			OperatableStateMachine.add('CapturePointcloud',
										CapturePointcloudState(),
										transitions={'continue': 'CalculateObjectPose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud': 'captured_pointcloud'})

			# x:569 y:37
			OperatableStateMachine.add('CalculateObjectPose',
										CalculateObjectPoseState(),
										transitions={'continue': 'PoseMessage', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud': 'captured_pointcloud', 'object_pose': 'part_pose'})

			# x:671 y:137
			OperatableStateMachine.add('PoseMessage',
										MessageState(),
										transitions={'continue': 'GoPreGraspPosition'},
										autonomy={'continue': Autonomy.Off},
										remapping={'message': 'part_pose'})

			# x:920 y:41
			OperatableStateMachine.add('ComputePickPoint',
										ComputeGraspState(joint_names=names, time_out=3.0),
										transitions={'continue': 'MoveToPick', 'failed': 'failed', 'time_out': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'time_out': Autonomy.Off},
										remapping={'move_group': 'move_group', 'move_group_prefix': 'move_group_prefix', 'tool_link': 'ee_link', 'pose': 'part_pose', 'offset': 'suction_cup_offset', 'rotation': 'rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:914 y:125
			OperatableStateMachine.add('MoveToPick',
										binpicking_flexbe_states__MoveitToJointsDynState(),
										transitions={'reached': 'GraspObject', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'move_group_prefix': 'move_group_prefix', 'move_group': 'move_group', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
