#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from ardrone_autonomy.srv import CamSelect
from std_msgs.msg import Int32MultiArray
from drone_video_display import DroneVideoDisplay

from std_srvs.srv import Empty as Empty_1
# An enumeration of Drone Statuses
from drone_status import DroneStatus

from time import sleep


# Some Constants
COMMAND_PERIOD = 100 #ms


class BasicDroneController(object):
	def __init__(self):
		# Holds the current drone status
		self.status = -1
		self.altd = 0
		self.ax = 0
		self.ay = 0
		self.az = 0
		self.vx = 0
		self.vy = 0
		self.vz = 0
		self.rotX = 0
		self.rotY = 0
		self.rotZ = 0
		self.magX = 0
		self.magY = 0
		self.magZ = 0
		self.pressure = 0
		self.motor1 = 0
		self.motor2 = 0
		self.motor3 = 0
		self.motor4 = 0
		self.tags_count = 0
		self.tags_xc = []
		self.tags_yc  =  []
		self.tags_distance = []

		self.correctionRoll = 0

		self.camChannel = 0

		self.foundQR = False

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)

		self.listenQr = rospy.Subscriber('/ardrone/qr', Int32MultiArray, self.ReceiveQr)

		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)

		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist)

		self.servToggle = rospy.ServiceProxy('/ardrone/setcamchannel', CamSelect)

		self.pubRecalibrate = rospy.ServiceProxy('/ardrone/imu_recalib', Empty_1)

		# Setup regular publishing of control packets
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

		self.video = DroneVideoDisplay()
		self.returnHome = False

		# Land the drone if we are shutting down
		rospy.on_shutdown(self.SendLand)

	def ReceiveNavdata(self,navdata):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment
		self.status = navdata.state
		self.altd = navdata.altd
		self.ax = navdata.ax
		self.ay = navdata.ay
		self.az = navdata.az
		self.vx = navdata.vx
		self.vy = navdata.vy
		self.vz = navdata.vz
		self.rotX = navdata.rotX
		self.rotY = navdata.rotY
		self.rotZ = navdata.rotZ
		self.magX = navdata.magX
		self.magY = navdata.magY
		self.magZ = navdata.magZ
		self.pressure = navdata.pressure
		self.motor1 = navdata.motor1
		self.motor2 = navdata.motor2
		self.motor3 = navdata.motor3
		self.motor4 = navdata.motor4
		self.tags_count = navdata.tags_count
		self.tags_xc = navdata.tags_xc
		self.tags_yc = navdata.tags_yc
		self.tags_distance = navdata.tags_distance

		sp = 0.1

		if self.rotX < -1:
			self.SetCommand(-sp, 0, 0, 0)
		elif self.rotX > 1:
			self.SetCommand(sp, 0, 0, 0)

		if self.rotY < -1:
			self.SetCommand(0, sp, 0, 0)
		elif self.rotY > 1:
			self.SetCommand(0, -sp, 0, 0)



		#self.AutoCorrection()
		#if self.foundQR == False:
			#self.SetCommand(0.08, 0.11, 0, 0)


	def ReceiveQr(self, qr_loc):
		#self.SendLand()
		qr_loc = qr_loc.data

		if len(qr_loc) == 8:
			if self.camChannel == 1:
				self.SendLand()

			if self.foundQR == False:
				self.SetCommand(0.08,-1,0,0)
				sleep(0.5)
				self.SetCommand(0.08, 0.01, 0, 0)
			self.foundQR = True
			##self.SetCommand(0.08, 0.01, 0, 0)



			middleX = ( (qr_loc[6] - qr_loc[0]) / 2) + qr_loc[0]
			middleY = ( (qr_loc[3] - qr_loc[1]) / 2) + qr_loc[1]


			if middleX and middleY:
				sp = 0.1
				roll = 0
				z_velocity = 0
				pitch = 0.1
				lower_bound = 475
				upper_bound = 525

				isCenteredX = False
				isCenteredY = False


				if middleX < lower_bound:
					roll = sp # Go left
				elif middleX > upper_bound:
					roll = -sp # Go right
				else:
					isCenteredX = True

				if middleY < lower_bound:
					pass
				elif middleY > upperbound:
					pitch = -sp # Go down
				else:
					isCenteredY = True

				if isCenteredX and isCenteredY:
					self.SendLand()
				else:
					self.SetCommand(roll, pitch + 0.1, 0, z_velocity)


	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.status == DroneStatus.Landed):
			self.pubTakeoff.publish(Empty())
			self.SetCommand(0,0,0,0)

	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())

	def SendRecalibration(self):
		# Send reset calibration message to ardrone driver
		self.pubRecalibrate()

	def SendSwitchCam(self):
		# Switch between the two camera channels
		if self.camChannel == 1:
			self.camChannel = 0
		elif self.camChannel == 0:
			self.camChannel = 1
		self.servToggle(self.camChannel)

	def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		# Called by the main program to set the current command
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		self.command.linear.z  = z_velocity
		self.command.angular.z = yaw_velocity

	def SendCommand(self,event):
		# The previously set command is then sent out periodically if the drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)


	def AutoCorrection(self):
		if self.video.feature0 is None:
			self.video.feature0, self.video.kp0, self.video.des0 = self.video.FeaturePoint()
			self.video.image0 = self.video.Ros2CV()
			#pass

		else:

			correctionX = 0
			correctionY = 0
			correctionZ = 0

			if self.returnHome:
				if self.video.graphX > self.homeX * 1.1:
					correctionX = abs(self.video.graphX - 200) / 180
				elif self.video.graphX < self.homeX * 0.9:
					correctionX = -abs(self.video.graphX - 200) / 180

				# Proportional control for Y position
				if self.video.graphY > self.homeY * 1.1:
					correctionY = abs(self.video.graphY - 200) / 180
				elif self.video.graphY < self.homeY * 0.9:
					correctionY = -abs(self.video.graphY - 200) / 180
				self.SetCommand(correctionX, correctionY, 0, correctionZ)
