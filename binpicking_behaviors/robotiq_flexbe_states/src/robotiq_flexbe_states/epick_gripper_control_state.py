#!/usr/bin/env python


import rospy

import roslib; roslib.load_manifest('robotiq_vacuum_grippers_control')
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from robotiq_vacuum_grippers_control.msg import _RobotiqVacuumGrippers_robot_output  as outputMsg


'''

Created on June 21 2020

@author: Gerard Harkema

'''

class EpickVacuumGripperControlState(EventState):
	'''
	State to control Robotiq Epick suction gripper"

	-- enable	 		bool 		'true' to activates the gripper, 'false' to deactivate it
	-- vacuum_power			float		vacuum power 1..255
	-- setteling_time 		float 		Time which needs to have passed since the behavior started.
	<= continue 					if the gripper activation or de-activation has been succesfully achieved
	<= failed 						otherwise

	'''

	def __init__(self, enable, vacuum_power, setteling_time):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(EpickVacuumGripperControlState, self).__init__(outcomes = ['continue', 'failed'])

		# Store state parameter for later use.
		self._target_time = rospy.Duration(setteling_time)

		# The constructor is called when building the state machine, not when actually starting the behavior.
		# Thus, we cannot save the starting time now and will do so later.
		self._start_time = None
		
		#rospy.init_node('RobotiqVacuumGrippersSimpleController')
		self._pub = rospy.Publisher('RobotiqVacuumGrippersRobotOutput', outputMsg.RobotiqVacuumGrippers_robot_output, queue_size = 10)
		
	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

		if rospy.Time.now() - self._start_time > self._target_time:
			return 'continue' # One of the outcomes declared above.


	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

	        #command = outputMsg.RobotiqVacuumGrippers_robot_output();
	        #command.rPR = userdata.vacuum_power
	        #self._pub.publish(command)


		self._start_time = rospy.Time.now()
		time_to_wait = (self._target_time - (rospy.Time.now() - self._start_time)).to_sec()

		if time_to_wait > 0:
			Logger.loginfo('Need to wait for %.1f seconds.' % time_to_wait)
			Logger.loginfo('Grasp/Drop object')
		

	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass # Nothing to do


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.
		pass # Nothing to do

	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do
