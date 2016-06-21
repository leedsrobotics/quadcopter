#!/usr/bin/env python

import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay
from time import sleep
import threading
from PySide import QtCore, QtGui

class KeyMapping(object):
	Takeoff          = QtCore.Qt.Key.Key_Y
	Land             = QtCore.Qt.Key.Key_H
	Emergency        = QtCore.Qt.Key.Key_Space
	Start		 = QtCore.Qt.Key.Key_S
	SwitchCam	 = QtCore.Qt.Key.Key_C
	Recalibrate	 = QtCore.Qt.Key.Key_R


class KeyboardController(DroneVideoDisplay):
	def __init__(self):
		super(KeyboardController,self).__init__()

		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0
		self.z_velocity = 0

	def keyPressEvent(self, event):
		key = event.key()

		if controller is not None and not event.isAutoRepeat():
			if key == KeyMapping.Emergency:
				print "EMERGENCY"
				controller.SendEmergency()
			elif key == KeyMapping.Takeoff:
				print "TAKEOFF"
				controller.SendTakeoff()
			elif key == KeyMapping.Land:
				print "LANDING"
				controller.SendLand()
			elif key == KeyMapping.SwitchCam:
				controller.SendSwitchCam()
			elif key == KeyMapping.Recalibrate:
				controller.SendRecalibration()
			elif key == KeyMapping.Start:
				print "STARTING"
				thread = threading.Thread(target=self.runSimulation, args=())
				thread.daemon = True
				thread.start()


	def runSimulation(self):
		#controller.SendTakeoff()
		#sleep(3)
		#controller.SetCommand(0, 0, 0, -1)
		#sleep(4)
		#controller.SetCommand(0, 0, 0, 1)
		#sleep(3)
		#controller.SendLand()

		controller.SendTakeoff()
		sleep(3)
		controller.SetCommand(0, 1, 0, 0)




if __name__=='__main__':
	import sys
	rospy.init_node('ardrone_keyboard_controller')

	app = QtGui.QApplication(sys.argv)
	controller = BasicDroneController()
	display = KeyboardController()

	display.show()

	status = app.exec_()

	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
