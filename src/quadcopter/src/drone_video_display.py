#!/usr/bin/env python

# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This display window listens to the drone's video feeds and updates the display at regular intervals
# It also tracks the drone's status and any connection problems, displaying them in the window's status bar
# By default it includes no control functionality. The class can be extended to implement key or mouse listeners if required

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('quadcopter')
import rospy

# Import the two types of messages we're interested in
# for receiving the video feed
from sensor_msgs.msg import Image
# for receiving navdata feedback
from ardrone_autonomy.msg import Navdata

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# The GUI libraries
from PySide import QtCore, QtGui

# Libraries for opencv and opencv's image format numpy
import cv2
import numpy as np

# Provides conversion between ros image and numpy
import ros_numpy

from matplotlib import pyplot as plt

from time import sleep

# Some Constants
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 10 #ms


class DroneVideoDisplay(QtGui.QMainWindow):
	StatusMessages = {
		DroneStatus.Emergency : 'Emergency',
		DroneStatus.Inited    : 'Initialized',
		DroneStatus.Landed    : 'Landed',
		DroneStatus.Flying    : 'Flying',
		DroneStatus.Hovering  : 'Hovering',
		DroneStatus.Test      : 'Test (?)',
		DroneStatus.TakingOff : 'Taking Off',
		DroneStatus.GotoHover : 'Going to Hover Mode',
		DroneStatus.Landing   : 'Landing',
		DroneStatus.Looping   : 'Looping (?)'
		}
	DisconnectedMessage = 'Disconnected'
	UnknownMessage = 'Unknown Status'


	def __init__(self):
		# Construct the parent class
		super(DroneVideoDisplay, self).__init__()

		# Setup our very basic GUI - a label which fills the whole window and holds our image
		self.setWindowTitle('AR.Drone Video Feed')
		# Set location and size
		self.setGeometry(740, 480, 250, 100)#self.width(), self.height()
		self.imageBox = QtGui.QLabel(self)
		self.setCentralWidget(self.imageBox)

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata)

		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo   = rospy.Subscriber('/ardrone/image_raw', Image, self.ReceiveImage)

		# Holds the image frame received from the drone and later processed by the GUI
		self.image = None
		self.imageLock = Lock()

		# Holds the left and right image for creating depth map
		self.imageL = None
		self.imageR = None

		# Holds the original images in all 4 directions (left, right, front, back)
		self.image0 = None
		self.image90 = None
		self.image180 = None
		self.image270 = None

		# Holds the query feature point images in all 4 directions (left, right, front, back)
		self.feature0 = None
		self.feature90 = None
		self.feature180 = None
		self.feature270 = None

		# Holds the query image's keypoints and descriptors found in images from 4 different directions
		self.kp0 = None
		self.kp90 = None
		self.kp180 = None
		self.kp270 = None
		self.des0 = None
		self.des90 = None
		self.des180 = None
		self.des270 = None

		# Holds the train image's keypoints and descriptors
		self.trainKP = None
		self.trainDES = None

		# Holds the current camera frame
		self.current_image = None

		# Stores the current coords of the quadcopter's position
		self.graphX = 200
		self.graphY = 200
		self.graphZ = 200

		# Creates a 400 by 400 image with the colour black as the graph, and the trail of the quadcopter
		self.pitch_roll_graph = np.zeros((400, 400), dtype = 'uint8')
		self.pitch_roll_trail = np.zeros((400, 400), dtype = 'uint8')
		self.altitude_roll_graph = np.zeros((400, 400), dtype = 'uint8')
		self.altitude_roll_trail = np.zeros((400, 400), dtype = 'uint8')

		# Control when to update graph and matches
		self.updateGraph = False
		self.updateMatches = False

		# Stores the filtered matches
		self.filtered_matches = []

		# Holds the status message to be displayed on the next GUI update
		self.statusMessage = ''

		# Tracks whether we have received data since the last connection check
		# This works because data comes in at 50Hz but we're checking for a connection at 4Hz
		self.communicationSinceTimer = False
		self.connected = False

		# A timer to check whether we're still connected
		self.connectionTimer = QtCore.QTimer(self)
		self.connectionTimer.timeout.connect(self.ConnectionCallback)
		self.connectionTimer.start(CONNECTION_CHECK_PERIOD)

		# A timer to redraw the GUI
		self.redrawTimer = QtCore.QTimer(self)
		self.redrawTimer.timeout.connect(self.RedrawCallback)
		self.redrawTimer.start(GUI_UPDATE_PERIOD)


	# Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
	def ConnectionCallback(self):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False


	def RedrawCallback(self):
		if self.image is not None:
			# We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
			# Lock the resources while displaying multiple windows, to avoid fatal error 11, (resource temporarily unavailable)
			self.imageLock.acquire()

			try:
				# Convert the ROS image into a QImage which we can display
				#image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))

				# Display video feed
				self.DisplayMainFrame()

				# Update graph if new X, Y coords are computed
				if self.updateGraph:
					self.Draw2DGraph()

				# Update matches it's in the correct state, e.g facing one of the directions
				if self.updateMatches:
					self.DrawMatches(self.image0, self.kp0, \
					self.current_image, self.trainKP, self.filtered_matches)

			finally:
				self.imageLock.release()

			# We could  do more processing (eg OpenCV) here if we wanted to, but for now lets just display the window.
			#self.resize(image.width(),image.height())
			#self.imageBox.setPixmap(image)

			#self.DepthMap()

		# Update the status bar to show the current drone status & battery level
		self.statusBar().showMessage(self.statusMessage if self.connected else self.DisconnectedMessage)


	def ReceiveImage(self,data):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
		self.imageLock.acquire()
		try:
			# Save the ros image for processing by the display thread
			self.image = data
		finally:
			self.imageLock.release()


	def ReceiveNavdata(self,navdata):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# Update the message to be displayed
		msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
		self.statusMessage = '{} (Battery: {}%)'.format(msg,int(navdata.batteryPercent))


	# Function that will take the current frame from camera, convert it to a format
	# that opencv can use and return it
	def Ros2CV(self):
		# Convert the ros image to numpy for use with opencv
		image = ros_numpy.numpify(self.image)

		# Separate the colour channels, so that the colours can be flipped to rgb
		# Slice will do the same thing as split but less costly
		#
		#b, g, r = cv2.split(cv_image)
		#cv_image = cv2.merge((r, g, b))
		#
		# Copy the Content of the slice
		r = np.zeros((image.shape[0], image.shape[1]), dtype = image.dtype)
		g = np.zeros((image.shape[0], image.shape[1]), dtype = image.dtype)
		b = np.zeros((image.shape[0], image.shape[1]), dtype = image.dtype)
		r[:, :] = image[:, :, 2]
		g[:, :] = image[:, :, 1]
		b[:, :] = image[:, :, 0]

		# Merge the colours in rgb format
		cv_image = cv2.merge((r, g, b))

		# Return the converted image
		return cv_image


	# Function that will take an cv image and a value that will filter the keypoints
	# by their score (higher means less keypoints, but they will be more useful)
	# and draw them on a copy of the original image and return it, as well as the
	# keypoints and descriptors found
	def FeaturePoint(self, cv_image = None, kp_filter = 2000):
		# If no input given, use current frame
		if cv_image is None:
			cv_image = self.Ros2CV()

		# Get a grayscale image of cv_image
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

		harris = cv2.FeatureDetector_create('HARRIS')
		harris_kp = harris.detect(gray, None)

		sift = cv2.DescriptorExtractor_create('SIFT')
		sleep(15)
		kp, des = sift.compute(gray, harris_kp)
		sleep(15)
		# Initiate SIFT detector with specified parameters
		#orb = cv2.ORB(nfeatures = kp_filter, edgeThreshold = 31, scaleFactor = 1.2, nlevels = 8, WTA_K = 2)

		# Find all the keypoints and descriptors
		#kp, des = orb.detectAndCompute(gray, None)

		# Draw keypoints on the image , #, color = (255, 0, 0), flags = 0)
		feature_image = cv2.drawKeypoints(cv_image, kp)

		# Return the image with feature points, as well as keypoints and descriptors found
		return feature_image, kp, des


	# Function that will take in the descriptor of the current camera frame and one of the 4 train images
	def FeatureMatching(self, query_des, train_des):
		# Create a BFMatcher object
		bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck = False)

		# Match query and train descriptors
		matches_found = bf.match(query_des, train_des)

		# Lowe's ratio test
		#good = []
		#for m, n in matches_found:
			#if m.distance < 0.6 * n.distance:
				#good.append(m)

		# Sort the mapped feature points by their distance
		#matches_found = sorted(matches_found, key = lambda x:x.distance)

		# We only want the good matches, the first 30 for example
		if len(matches_found) > 30:
			self.filtered_matches = matches_found[:30]


	# Function that will display the video feed of the drone in a specified location
	def DisplayMainFrame(self):
		# Get the current frame in in numpy format
		self.current_image = self.Ros2CV()

		# Start a new window to display the video stream with feature points
		# The second parameter here is the filter for keypoints, with a higher filter
		# less keypoints will be picked up, but they will be more useful
		feature_image, self.trainKP, self.trainDES = self.FeaturePoint(self.current_image, 100)

		# Show the image with feature points
		cv2.imshow('Video Feed', feature_image)

		# Move window to a specific location
		cv2.cv.MoveWindow('Video Feed', 10, 30)


	# Function to visualise the matching between query and train image keypoints
	def DrawMatches(self, query_image, queryKP, train_image, trainKP, filtered_matches):
		try:
			# Store the dimensions of the images
			query_rows = query_image.shape[0]
			query_cols = query_image.shape[1]
			train_rows = train_image.shape[0]
			train_cols = train_image.shape[1]

			# Create a zero matrix using the height of the query image and
			# width of both combined
			visualise = np.zeros((query_rows, query_cols + train_cols + 50, 3), dtype = 'uint8')

			# Copy the the query image to the left of the zero matrix
			visualise[:query_rows, :query_cols] = query_image

			# Copy the the train image to the right of the zero matrix
			visualise[:train_rows, query_cols + 50:] = train_image

			# Loop through each match
			for matches in filtered_matches:
				# Get the keypoint index for the query and train image
				query_index = matches.queryIdx
				train_index = matches.trainIdx

				# Get the x, y coordinates of the keypoints using the index found
				(queryX, queryY) = queryKP[query_index].pt
				(trainX, trainY) = trainKP[train_index].pt

				# Draw a circle at the keypoints(round to int) with the parameters
				# target image, coordinates, radius of circle, colour and thickness or circle
				# notice, to draw circles on the train image, we add the width of query
				# image to its x coords, as the image is on the right of the zero matrix
				cv2.circle(visualise, (int(queryX), int(queryY)), 4, (255, 0, 0), 1)
				cv2.circle(visualise, (int(trainX) + query_cols + 50, int(trainY)), 4, (255, 0, 0), 1)

				# Draw a line to connect the 2 points with the parameters
				# target image, queryKP coords, trainKP coords, colour, thickness of line
				# Again add query image width to the x coords for the train image coords
				cv2.line(visualise, (int(queryX), int(queryY)), (int(trainX + query_cols + 50), int(trainY)), (255, 0, 0), 1)

		except IndexError:
				pass

		# Show the image with feature point matchings on it
		cv2.imshow('Visualise', visualise)

		# Move window to a specific location
		cv2.cv.MoveWindow('Visualise', 10, 640)


	# Function used to display the current location of the quadcopter
	def Draw2DGraph(self):
		# Draw a white circle on the 2d graph to represent where the quadcopter is
		# target image, coordinates, radius of circle, colour and thickness or circle
		cv2.circle(self.pitch_roll_graph, (self.graphX, self.graphY), 3, (255, 255, 255), 9)

		# Draw a white circle on the 2d graph to represent where the quadcopter is
		# target image, coordinates, radius of circle, colour and thickness or circle
		cv2.circle(self.altitude_roll_graph, (self.graphX, self.graphZ), 3, (255, 255, 255), 9)

		# Display the 2D graph in a specific location
		cv2.imshow('Vertical Pitch, Horizontal Roll', self.pitch_roll_graph)
		cv2.cv.MoveWindow('Vertical Pitch, Horizontal Roll', 690, 30)

		# Display the 2D graph in a specific location
		cv2.imshow('Vertical altitude, Horizontal Roll', self.altitude_roll_graph)
		cv2.cv.MoveWindow('Vertical altitude, Horizontal Roll', 1100, 30)

		# Earse the previous circle by drawing a black circle using the previous coords
		# using a bigger radius and thickness
		cv2.circle(self.pitch_roll_graph, (self.graphX, self.graphY), 120, (0, 0, 0), 255)

		# Earse the previous circle by drawing a black circle using the previous coords
		# using a bigger radius and thickness
		cv2.circle(self.altitude_roll_graph, (self.graphX, self.graphZ), 120, (0, 0, 0), 255)

		# Draw a white circle on the 2d graph to represent where the quadcopter is
		# target image, coordinates, radius of circle, colour and thickness or circle
		cv2.circle(self.pitch_roll_trail, (self.graphX, self.graphY), 2, (255, 255, 255), 1)

		# Display the 2D graph in a specific location
		cv2.imshow('Pitch Roll Trail', self.pitch_roll_trail)
		cv2.cv.MoveWindow('Pitch Roll Trail', 1510, 30)

		# Draw a white circle on the 2d graph to represent where the quadcopter is
		# target image, coordinates, radius of circle, colour and thickness or circle
		cv2.circle(self.altitude_roll_trail, (self.graphX, self.graphZ), 2, (255, 255, 255), 1)

		# Display the 2D graph in a specific location
		cv2.imshow('Altitude Roll Trail', self.altitude_roll_trail)
		cv2.cv.MoveWindow('Altitude Roll Trail', 1510, 460)


	# Function that will take two images and show the disparity, can also be used to approximate depth
	def DepthMap(self):
		# Make sure there are two images
		if self.imageL is not None and self.imageR is not None:

			# Convert to grey scale images
			self.imageL = cv2.cvtColor(self.imageL, cv2.COLOR_BGR2GRAY)
			self.imageR = cv2.cvtColor(self.imageR, cv2.COLOR_BGR2GRAY)

			stereo = cv2.StereoBM(cv2.STEREO_BM_BASIC_PRESET, ndisparities = 32, SADWindowSize = 15)
			disparity = stereo.compute(self.imageL, self.imageR)

			#cv2.imshow("Window2", disparity)
			plt.imshow(disparity,'gray')
			plt.show()
