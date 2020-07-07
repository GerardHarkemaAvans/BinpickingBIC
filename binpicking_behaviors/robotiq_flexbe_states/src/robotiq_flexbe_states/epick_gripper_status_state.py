#!/usr/bin/env python


import roslib; roslib.load_manifest('robotiq_vacuum_grippers_control')
roslib.load_manifest('robotiq_modbus_rtu')
import rospy

import robotiq_vacuum_grippers_control.baseRobotiqVacuumGrippers
import robotiq_modbus_rtu.comModbusRtu
import os, sys
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from robotiq_vacuum_grippers_control.msg import _RobotiqVacuumGrippers_robot_input  as inputMsg
from robotiq_vacuum_grippers_control.msg import _RobotiqVacuumGrippers_robot_output as outputMsg


'''

Created on June 21 2020

@author: Gerard Harkema

'''

class EpickVacuumGripperStatusState(EventState):
	'''
	State to control Robotiq Epick suction gripper"

	-- device			String		device name eg. /dev/ttyUSB0
	<= continue 					if the status has been succesfully achieved
	<= failed 						otherwise

	'''

	def __init__(self, device):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(EpickVacuumGripperStatusState, self).__init__(outcomes = ['continue', 'failed'])

		# Store state parameter for later use.
		self._target_time = rospy.Duration(setteling_time)

		# The constructor is called when building the state machine, not when actually starting the behavior.
		# Thus, we cannot save the starting time now and will do so later.
		self._start_time = None
		
		#Gripper is a Vacuum with a TCP connection
		self._gripper = robotiq_vacuum_grippers_control.baseRobotiqVacuumGrippers.robotiqbaseRobotiqVacuumGrippers()
		self._gripper.client = robotiq_modbus_rtu.comModbusRtu.communication()

                #We connect to the address received as an argument
                self._connected = self._gripper.client.connectToDevice(device)

		
	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.


		return 'continue' # One of the outcomes declared above.


	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		#Send the most recent command
		if(self._connected):
		      status = gripper.getStatus()
		      rospy.loginfo(self.statusInterpreter(status))



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
		
	def statusInterpreter(status):
	    """Generate a string according to the current value of the status variables."""

	    output = '\n-----\nVacuum gripper status interpreter\n-----\n'

	    #gACT
	    output += 'gACT = ' + str(status.gACT) + ': '
	    if(status.gACT == 0):
	       output += 'Gripper reset\n'
	    if(status.gACT == 1):
	       output += 'Gripper activation\n'

	    #gMOD
	    output += 'gMOD = ' + str(status.gMOD) + ': '
	    if(status.gMOD == 0):
		output += 'Gripper set to automatic mode\n'
	    if(status.gMOD == 1):
		output += 'Gripper set to advanced mode\n'

	    #gGTO
	    output += 'gGTO = ' + str(status.gGTO) + ': '
	    if(status.gGTO == 0):
		output += 'Standby (or performing activation/automatic release)\n'
	    if(status.gGTO == 1):
		output += 'Go to Position Request\n'

	    #gSTA
	    output += 'gSTA = ' + str(status.gSTA) + ': '
	    if(status.gSTA == 0):
		output += 'Gripper is not activated. See Fault Status if Gripper is activated\n'
	    if(status.gSTA == 1):
		output += 'Activation in progress\n'
	    if(status.gSTA == 2):
		output += 'Not used\n'
	    if(status.gSTA == 3):
		output += 'Gripper is operational\n'

	    #gOBJ
	    output += 'gOBJ = ' + str(status.gOBJ) + ': '
	    if(status.gOBJ == 0):
		output += 'Unknown object detection. Regulating towards requested vacuum/pressure\n'
	    if(status.gOBJ == 1):
		output += 'Object detected. Minimum vacuum value reached\n'
	    if(status.gOBJ == 2):
		output += 'Object detected. Maximum vacuum value reached\n'
	    if(status.gOBJ == 3):
		output += 'No object detected. Object loss, dropped or gripping timeout reached\n'
	 
	    #gFLT
	    output += 'gFLT = ' + str(status.gFLT) + ': '
	    if(status.gFLT == 0x00):
		output += 'No Fault\n'
	    if(status.gFLT == 0x03):
		output += 'Very porous material detected\n'
	    if(status.gFLT == 0x05):
		output += 'Priority Fault: Action delayed, initialization must be completed prior to action\n'
	    if(status.gFLT == 0x06):
		output += 'Gripping timeout. rGTO must be re-asserted (rGTO=0 then rGTO=1) or one of the following parameters must be changed (rMOD, rPR, rSP, rFR)\n'
	    if(status.gFLT == 0x07):
		output += 'The Activation bit not set. Activation bit must be set prior to action (rACT=1)\n'
	    if(status.gFLT == 0x08):
		output += 'Maximum operating temperature exceeded, wait for cool-down\n'
	    if(status.gFLT == 0x09):
		output += 'No communication during at least 1 second. This fault will only be returned once if the next valid communication is a "read command" of the FAULT STATUS register\n'

	    if(status.gFLT == 0x0A):
		output += 'Under minimum operating voltage\n'
	    if(status.gFLT == 0x0B):
		output += 'Automatic release in progress (Vacuum/pressure detected)\n'
	    if(status.gFLT == 0x0C):
		output += 'Internal fault; contact support@robotiq.com\n'
	    if(status.gFLT == 0x0F):
		output += 'Automatic release completed (Vacuum/pressure not detected)\n' 

	    #gPR
	    output += 'gPR = ' + str(status.gPR) + ': '
	    output += 'Echo of the requested pressure for the vacuum gripper: ' + str(status.gPR) + '\n'

	    #gPO
	    output += 'gPO = ' + str(status.gPO) + ': '
	    output += 'Actual Vacuum/Pressure measured in suction cup: ' + str(100-status.gPO) + '\n'

	    return output

