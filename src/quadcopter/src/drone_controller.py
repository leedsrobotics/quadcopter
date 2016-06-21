#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Allow floating point division /, and integer division //
from __future__ import division

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('quadcopter')
import rospy

# Import the messages we're interested in sending and receiving
# for sending commands to the drone
from geometry_msgs.msg import Twist  
# for land/takeoff/emergency/flattrim/recalibrate servies	 
from std_msgs.msg import Empty  
from std_srvs.srv import Empty as Empty_1
# for receiving navdata feedback     	 
from ardrone_autonomy.msg import Navdata 
# enable cameras
from ardrone_autonomy.srv import CamSelect 

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# Gain access to the drone's camera
from drone_video_display import DroneVideoDisplay

# Libraries for opencv and opencv's image format numpy
import cv2
import numpy as np

# Include the time modules
import time

# Include the math modules
import math

# Some Constants
COMMAND_PERIOD = 10 #ms


class BasicDroneController(object):
	def __init__(self): 
		# Holds the current drone status and initialise variables to store sensor data
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

		# Value to store current camera, 0 is front camera, 1 is bottom camera
		self.camChannel = 0
		self.swapCameraAtLaunch = True

		# Used so that the order of routines can be controlled
		self.auto = False		
		self.autoYaw = True						
		self.autoDirection = False

		# Initialise variables to use with set command function
		self.Pitch = 0
		self.Roll = 0
		self.Yaw = 0
		self.zVelocity = 0

		# Initialise variables to convert yaw degree to 0-360 (180 to - 180 by default)
		# and initialise it to 0
		self.modifiedRotZ = 0
		self.originalRotZ = 0
		self.currentRotZ = 0
		# To make sure the original yaw degree is stored only once
		self.assign = True

		# Boolean value to enable a higher priority yaw control when true
		self.YawRight = False

		# To decide if the current facing direction is good enough or not
		# need to have at least 5 matches, before processing 
		self.min_match = 5

		# Sample size value, collect enough samples then smooth but taking average
		self.sample_size = 50

		# Use to store the X, Y, Z coords
		self.x_list = []
		self.y_list = []
		self.z_list = []

		# Location value used to return to a specific location
		self.homeX = 200
		self.homeY = 200
		self.homeZ = 200

		# Initialise to false, enabled through key press or onces area exceeded
		self.returnHome = False

		# Predefined area for fly space
		self.areaX = 350
		self.areaY = 350
		self.areaZ = 350

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata) 
		
		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)
		
		self.pubFlatTrim = rospy.Publisher('/ardrone/flatTrim', Empty)
		self.pubRecalibrate = rospy.ServiceProxy('/ardrone/imu_recalib', Empty_1)

		# Allow switching between cameras
		self.servToggle = rospy.ServiceProxy('/ardrone/setcamchannel', CamSelect)
		
		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist)

		# Setup regular publishing of control packets
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

		# Land the drone if we are shutting down
		rospy.on_shutdown(self.SendLand)

		# Gain access to the drone video display class
		self.video = DroneVideoDisplay()


	def ReceiveNavdata(self,navdata):
		# Set the bottom camera as default launch camera
		#if (self.swapCameraAtLaunch):
			#self.SendSwitchCam()
			#self.swapCameraAtLaunch = False

		# Debug routine
		self.DisplayNavdata(navdata)

		# Execute only if the quadcopter is in a suitable state, e.g. in the air
		#if  self.status == DroneStatus.Flying or self.status == DroneStatus.Hovering:

		# To convert from -180 - 180 to 0 - 360 and initialise yaw degree to 0
		self.ConvertAngle()

		# If it's in auto mode
		# ---------------------Single Direction Solution----------------
		if self.auto:
			# Initialise and start tracking the quadcopter
			self.AutoCorrection()

		# Yaw to all 4 directions and store feature images, keypoints, descriptors etc
		# ---------------- 4 Direction Solution ------------------
		#if (self.autoYaw and self.auto):
			# Yaw 360 degrees while keeping position
			#self.AutoYaw()		

		# Handles what the quadcopter will do depending on which direction it's facing
		# if auto mode is enabled, if auto mode is enabled and initialising step is 
		# ---------------- 4 Direction Solution ------------------
		#if self.auto and self.autoDirection:
			#self.Analysis()

		#if self.YawRight:
			#self.SetCommand(yaw_velocity = -1)
			#time.sleep(1)
			#self.YawRight = False

		# Using two images taken by the keys U and I to compute a depth map
		# and approximate distance using the depth map
		#self.video.DepthMap()


	# Function that will handle the initialising step, which will store an image, its keypoints and descriptor
	# if nothing is stored. Then it will start finding matches and tracking the quadcopter's position 
	# ---------------------Single Direction Solution---------------- 
	def AutoCorrection(self):
		if self.video.feature0 is None:	
			# Store a image, a salint image, its keypoints and descriptors
			# Display image in a certain location
			self.video.feature0, self.video.kp0, self.video.des0 = self.video.FeaturePoint()
			self.video.image0 = self.video.Ros2CV()

		else:
			
			correctionX = 0
			correctionY = 0
			correctionZ = 0

			# If return home mode enabled
			if self.returnHome:
				# Proportional control for x position
				if self.video.graphX > self.homeX * 1.1:
					correctionX = abs(self.video.graphX - 200) / 180
					#print "Move Left"
				elif self.video.graphX < self.homeX * 0.9:
					correctionX = -abs(self.video.graphX - 200) / 180
					#print "Move Right"
				#else:
					#print "Roll OK"

				# Proportional control for Y position
				if self.video.graphY > self.homeY * 1.1:
					correctionY = abs(self.video.graphY - 200) / 180
					#print "Move Forward"
				elif self.video.graphY < self.homeY * 0.9:
					correctionY = -abs(self.video.graphY - 200) / 180
					#print "Move Backward"
				#else:
					#print "Pitch OK"

				# Proportional control for Z position
				#if self.video.graphZ >= self.homeZ * 1.05:
					#self.zVelocity = abs(currentZ - 200) / 200
					#print "Move Up"
				#elif self.video.graphZ <= self.homeZ * 0.95:
					#self.zVelocity = - abs(currentZ - 200) / 200
					#print "Move Down"
				#else:
					#print "Altitude OK"
				#pass
				self.SetCommand(correctionX, correctionY, 0, correctionZ)
				#self.SetCommand(roll = self.Roll, pitch = self.Pitch, z_velocity = self.zVelocity)
				#self.Roll = 0
				#self.Pitch = 0
				#self.zVelocity = 0

			# Finds good matches between the query and the train image
			self.video.FeatureMatching(self.video.des0, self.video.trainDES)

			# Check if there are enough matches before proceeding
			if len(self.video.filtered_matches) > self.min_match:
				# work out current position base on where the train 
				# image's keypoints are mapped to the query image
				self.CurrentLocation(self.video.kp0, self.video.trainKP, self.video.filtered_matches)

				# Update matches
				self.video.updateMatches = True

			# Enable return home mode, if the predefined area is past
			#if self.video.graphX > self.areaX:
				#self.returnHome = True
			#elif self.video.graphX < 400 - self.areaX:
				#self.returnHome = True

			#if self.video.graphY > self.areaY:
				#self.returnHome = True
			#elif self.video.graphY < 400 - self.areaY:
				#self.returnHome = True

			#if self.video.graphZ > self.areaZ:
				#self.returnHome = True
			#elif self.video.graphZ < 400 - self.areaZ:	
				#self.returnHome = True


	# Function that will adjust the quadcopter's facing direction until it's 
	# perpendicular to one of the 4 direction
	# ---------------- 4 Direction Solution ------------------
	def AutoDirection(self):
		if self.currentRotZ >= 358 or self.currentRotZ <= 2 or 88 <= self.currentRotZ <= 92 \
		or 178 <= self.currentRotZ <= 182 or 268 <= self.currentRotZ <= 272:
			self.Yaw = 0
		# Choose the closest direction to yaw to
		elif 0 < self.currentRotZ <= 90:
			if abs(self.currentRotZ - 0) >= abs(self.currentRotZ - 90):
				# Proportional control, the further away from target, faster it yaws
				self.Yaw = -(abs(self.currentRotZ - 90)/44)
				#print "yaw right"
			else:
				self.Yaw = (abs(self.currentRotZ - 0)/44)
				#print "yaw left"

		elif 90 < self.currentRotZ <= 180:
			if abs(self.currentRotZ - 90) >= abs(self.currentRotZ - 180):
				self.Yaw = -(abs(self.currentRotZ - 180)/44)
				#print "yaw right"
			else:
				self.Yaw = (abs(self.currentRotZ - 90)/44)
				#print "yaw left"

		elif 180 < self.currentRotZ <= 270:
			if abs(self.currentRotZ - 180) >= abs(self.currentRotZ - 270):
				self.Yaw = -(abs(self.currentRotZ - 270)/44)
				#print "yaw right"
			else:
				self.Yaw = (abs(self.currentRotZ - 180)/44)
				#print "yaw left"			

		else:
			if abs(self.currentRotZ - 270) >= abs(self.currentRotZ - 359.99):
				self.Yaw = --(abs(self.currentRotZ - 359.99)/44)
				#print "yaw right"
			else:
				self.Yaw = -(abs(self.currentRotZ - 270)/44)
				#print "yaw left"

		self.SetCommand(yaw_velocity = self.Yaw)
	

	# Function that will handle when it's perpendicular to one of the 4 directions
	# and when it's not
	# ---------------- 4 Direction Solution ------------------
	def Analysis(self):
		# Choose a direction to yaw to if the drone is not already 
		# facing one of the 4 directions perfectly ( with an off set )
		if self.currentRotZ >= 358 or self.currentRotZ <= 2:
			# Finds good matches between the query and the train image
			self.video.FeatureMatching(self.video.des0, self.video.trainDES)

			# Check if there are enough matches before proceeding
			if len(self.video.filtered_matches) > self.self.min_match:
				# work out current position base on where the train 
				# image's keypoints are mapped to the query image
				self.CurrentLocation(self.video.kp0, self.video.trainKP, self.video.filtered_matches)

				# Update matches
				self.video.updateMatches = True	

		elif 88 <= self.currentRotZ <= 92:
			# Finds good matches between the query and the train image
			self.video.FeatureMatching(self.video.des90, self.video.trainDES)

			# Check if there are enough matches before proceeding
			if len(self.video.filtered_matches) > self.self.min_match:
				# work out current position base on where the train 
				# image's keypoints are mapped to the query image
				self.CurrentLocation(self.video.kp90, self.video.trainKP, self.video.filtered_matches)

				# Update matches
				self.video.updateMatches = True			

		elif 178 <= self.currentRotZ <= 182:
			# Finds good matches between the query and the train image
			self.video.FeatureMatching(self.video.des180, self.video.trainDES)

			# Check if there are enough matches before proceeding
			if len(self.video.filtered_matches) > self.self.min_match:
				# work out current position base on where the train 
				# image's keypoints are mapped to the query image
				self.CurrentLocation(self.video.kp180, self.video.trainKP, self.video.filtered_matches)

				# Update matches
				self.video.updateMatches = True		

		elif 268 <= self.currentRotZ <= 272:
			# Finds good matches between the query and the train image
			self.video.FeatureMatching(self.video.des270, self.video.trainDES)

			# Check if there are enough matches before proceeding
			if len(self.video.filtered_matches) > self.self.min_match:
				# work out current position base on where the train 
				# image's keypoints are mapped to the query image
				self.CurrentLocation(self.video.kp270, self.video.trainKP, self.video.filtered_matches)

				# Update matches
				self.video.updateMatches = True	

		else:
			# Choose which direction to yaw to
			self.AutoDirection()

			# No point updating using the same information
			self.video.updateGraph = False
			self.video.updateMatches = False


	# Function that will initialise the quadcopter by yawing 360 degrees while
	# taking 4 images from 4 different directions as query images, store feature
	# point image and its keypoints and descriptors
	# ---------------- 4 Direction Solution ------------------
	def AutoYaw(self):
		# If it's currently between a certain degree and image is not taken yet
		if (self.currentRotZ >= 358 or self.currentRotZ <= 2) and self.video.feature0 is None:
			# Store a image, a salint image, its keypoints and descriptors
			# Display image in a certain location
			self.video.feature0, self.video.kp0, self.video.des0 = self.video.FeaturePoint()
			self.video.image0 = self.video.Ros2CV()	

		# We want the set command to apply early, before it takes time to slow down
		elif (88 <= self.currentRotZ <= 92) and self.video.feature90 is None:
			# Stop the quadcopter
			self.SetCommand()

			# Store a image, a salint image, its keypoints and descriptors
			# Display image in a certain location
			self.video.feature90, self.video.kp90, self.video.des90 = self.video.FeaturePoint()
			self.video.image90 = self.video.Ros2CV()

		# We want the set command to apply early, before it takes time to slow down
		elif (178 <= self.currentRotZ <= 182) and self.video.feature180 is None:
			# Stop the quadcopter
			self.SetCommand()

			# Store a image, a salint image, its keypoints and descriptors
			# Display image in a certain location
			self.video.feature180, self.video.kp180, self.video.des180 = self.video.FeaturePoint()
			self.video.image180 = self.video.Ros2CV()

		# We want the set command to apply early, before it takes time to slow down
		elif (268 <= self.currentRotZ <= 272) and self.video.feature270 is None:
			# Stop the quadcopter
			self.SetCommand()

			# Store a image, a salint image, its keypoints and descriptors
			# Display image in a certain location
			self.video.feature270, self.video.kp270, self.video.des270 = self.video.FeaturePoint()
			self.video.image270 = self.video.Ros2CV()

			# Done initialising and proceed to next step
			self.autoYaw = False
			self.autoDirection = True

		else:
			# Yaw faster the further away from the target
			self.Yaw = -abs(90 - (self.currentRotZ % 90))

		# Set command using yaw
		self.SetCommand(yaw_velocity = self.Yaw)

		# Reset yaw speed to 0 after it has been applied
		self.Yaw = 0


	# Function that takes one of the query image's keypoints and the matches
	# of it and the train image as the input to work out the current X, Y location
	def CurrentLocation(self, queryKP, trainKP, filtered_matches):
		# Initialise value, this value will store the x, z coordinates
		# of where the train image is mapped onto the query image
		query_xCoords = 0
		train_xCoords = 0
		query_zCoords = 0
		train_zCoords = 0

		# Initialise variables to store distance between a keypoint and each
		# other matched keypoints in the query and train image
		query_coords_difference = 0
		train_coords_difference = 0

		# Store the first index for each query and train, so that the index
		# keypoint can be used to be subtracted by every other keypoint
		original_query_index = filtered_matches[0].queryIdx 
		original_train_index = filtered_matches[0].trainIdx 

		count = 1

		try:
			# Loop through each match
			for matches in filtered_matches:
				# Store the keypoint index for the query and train image
				query_index = matches.queryIdx
				train_index = matches.trainIdx

				# Stores next index
				next_query_index = filtered_matches[count].queryIdx
				next_train_index = filtered_matches[count].trainIdx	

				# To get index for next key point
				if len(filtered_matches) > count + 1: 
					count += 1		
			
				# Take the distance difference, x1 - x2, y1 - y2
				query_difference_A = abs(queryKP[query_index].pt[0] - queryKP[next_query_index].pt[0])
				query_difference_B = abs(queryKP[query_index].pt[1] - queryKP[next_query_index].pt[1])			

				train_difference_A = abs(trainKP[train_index].pt[0] - trainKP[next_train_index].pt[0])
				train_difference_B = abs(trainKP[train_index].pt[1] - trainKP[next_train_index].pt[1])

				# Using the distance of two side of the triangle, compute the length of the longest side,   C^2 = A^2 + B^2
				query_difference_C = math.sqrt((query_difference_A * query_difference_A) + (query_difference_B * query_difference_B))
				train_difference_C = math.sqrt((train_difference_A * train_difference_A) + (train_difference_B * train_difference_B))

				# Add to total difference 
				query_coords_difference += query_difference_C
				train_coords_difference += train_difference_C

				# Add all the x coordinate of the keypoints
				query_xCoords += queryKP[query_index].pt[0]
				train_xCoords += trainKP[train_index].pt[0]

				# Add all the z coordinate of the keypoints
				query_zCoords += queryKP[query_index].pt[1]
				train_zCoords += trainKP[train_index].pt[1]

			# Work out the average xCoords
			query_X_average = query_xCoords / len(filtered_matches)
			train_X_average = train_xCoords / len(filtered_matches)

			# Work out the average xCoords
			query_Z_average = query_zCoords / len(filtered_matches)
			train_Z_average = train_zCoords / len(filtered_matches)
		
			# Work out the average difference of distance betweeen the 2 key point sets
			query_coords_difference /= len(filtered_matches)
			train_coords_difference /= len(filtered_matches)

			# Wide angle lens on the camera have a scaling effect, towards the left hand
			# side and the left hand side of the image, things will scale down more and more
			# at the left or right edge of the image, it's found that things were scaled down
			# to about 75% of original size, to fix that the distance is being scale back up
			# according to the x distance, the more to the left or right, scaling down increases
			# they are scaled back up the more to the left or right, the key points are 
			query_coords_difference *= (1 + (0.25 / 320 * abs(320 - query_X_average)))
			train_coords_difference *= (1 + (0.25 / 320 * abs(320 - train_X_average)))

			# Calculate the coordinates of where the quadcopter is and scale it down
			currentX = 200 - ((train_X_average - query_X_average) / 3)
			currentY = 200 - (200 - ((query_coords_difference / train_coords_difference) * 200))
			currentZ = 200 - ((train_Z_average - query_Z_average) / 1.5)

			# Add to end of list
			self.x_list.append(currentX)
			self.y_list.append(currentY)
			self.z_list.append(currentZ)
		except IndexError:
			pass

		# Start drawing graph when there are enough samples, smooth the values
		# All list should have the same size, only need to check one
		if len(self.x_list) >= self.sample_size:
			# Taking the average to smooth out noises
			self.video.graphX = int(sum(self.x_list) / len(self.x_list)) 
			self.video.graphY = int(sum(self.y_list) / len(self.y_list)) 
			self.video.graphZ = int(sum(self.z_list) / len(self.z_list)) 

			# Update graph
			self.video.updateGraph = True

			# Delete first value from list
			self.x_list.pop(0)
			self.y_list.pop(0)
			self.z_list.pop(0)
			

	# Function to print data from different sensors for debug purposes
	def DisplayNavdata(self, navdata):
		# Store navdata in corresponding variables	
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

		# To convert from -180 - 180 to 0 - 360 and initialise yaw degree to 0
		#self.ConvertAngle()

		# Print nav data received, uncomment if needed
		#print "motor1 = ", self.motor1, ", motor2 = ", self.motor2, ", motor3 = ", self.motor3, ", motor4 = ", self.motor4
		#print "magX = ", self.magX, ", magY = ", self.magY, ", magZ = ", self.magZ
		#print "rotX = ", self.rotX, ", rotY = ", self.rotY, ", rotZ = ", self.rotZ
		#print "ax = ", self.ax, ", ay = ", self.ay, ", az = ", self.az
		#print "vx = ", self.vx, ", vy = ", self.vy, ", vz = ", self.vz
		#print "pressure = ", self.pressure
		#print "current = ", self.currentRotZ#, "   origin = ", self.rotZ
		#print "origin = ", self.rotZ
		pass


	# Function To convert from -180 - 180 to 0 - 360 and initialise yaw degree to 0
	def ConvertAngle(self):
		# multiply by -1 so that it ranges from 0 to 360 clockwise
		rotZ = self.rotZ * -1
	
		# Convert from -180 to 180 range to 0 to 360
		if (rotZ  < 0):
			self.modifiedRotZ = rotZ + 360
		elif (rotZ >= 0):
			self.modifiedRotZ = rotZ

		# Store the original angle, so that we initialise our current angle to 0 on start up
		# by subtracting the stored angle 
		if self.assign:
			self.originalRotZ = self.modifiedRotZ
			# Assign only once
			self.assign = False

		# Initialise rotation to 0
		self.currentRotZ = (self.modifiedRotZ - self.originalRotZ) % 360


	def SendRecalibration(self):
		# Send reset calibration message to ardrone driver
		self.pubRecalibrate()


	def SendFlatTrim(self):
		# Send flat trim message to ardrone driver
		# Note we only send a flat trim message if the drone is landed and on flat surface
		if(self.status == DroneStatus.Landed):
			self.pubFlatTrim.publish(Empty())


	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.status == DroneStatus.Landed):
			# Call flat trim and recalibrate services before take off
			self.SendFlatTrim()
			self.SendRecalibration()

			# Wait 1 seconds for the services to apply
			time.sleep(1)
	
			# Takeoff after the services are applied
			self.pubTakeoff.publish(Empty())


	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())


	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())


	def SendSwitchCam(self):
		# Switch between the two camera channels
		if self.camChannel == 1:
			self.camChannel = 0
		elif self.camChannel == 0:
			self.camChannel = 1
		self.servToggle(self.camChannel)


	def SwitchMode(self):
		# Switch between auto and manual mode
		if (self.auto):
			self.auto = False
			print "Switched to manual"
		elif (not self.auto):
			self.auto = True
			print "Switched to auto"

	def ToggleHome(self):
		# Toggle return home mode on and off
		if (self.returnHome):
			self.returnHome = False
			print "Return home disabled"
		elif (not self.returnHome):
			self.returnHome = True
			print "Return home enabled"


	def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		# Called by the main program to set the current command
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		self.command.linear.z  = z_velocity
		self.command.angular.z = yaw_velocity


	def SendCommand(self,event):
		# The previously set command is then sent out periodically if the drone is flying
		# Only send command if it's in the air and at a stable altitude
		if self.status == DroneStatus.Flying or self.status == DroneStatus.Hovering or self.status == DroneStatus.GotoHover:
			self.pubCommand.publish(self.command)

