import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image    	 # for receiving the video feed
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# The GUI libraries
from PySide import QtCore, QtGui

from PyQt4.QtCore import *
from PyQt4.QtGui import *

from time import sleep

import cv2
import ros_numpy
import numpy as np
import zbar
import Image as Img
from std_msgs.msg import Int32MultiArray

# Some Constants
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms
DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected


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
		self.imageBox = QtGui.QLabel(self)
		self.setCentralWidget(self.imageBox)

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)

		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)

		self.publishQr = rospy.Publisher('/ardrone/qr', Int32MultiArray)

		# Holds the image frame received from the drone and later processed by the GUI
		self.image = None
		self.imageLock = Lock()

		self.tags = []
		self.tagLock = Lock()

		self.feature0 = None
		self.kp0 = None
		self.des0 = None
		self.image0 = None

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

	# Function that will take the current frame from camera, convert it to a format
	# that opencv can use and return it
	def Ros2CV(self):
		# Convert the ros image to numpy for use with opencv
		image = ros_numpy.numpify(self.image)
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

	def convertQPixmapToZbar(self):
		cv_image = self.Ros2CV()

		cv_image = Img.fromarray(cv_image, 'RGB')

		cv_image = cv_image.convert('L')
		width, height = cv_image.size

		#print cv_image.toString()
		zbar_img = zbar.Image(width, height, 'Y800', cv_image.tostring())

		return zbar_img

	# Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
	def ConnectionCallback(self):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

	def RedrawCallback(self):
		if self.image is not None:
			# We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
			self.imageLock.acquire()
			try:
					# Convert the ROS image into a QImage which we can display
					image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))
					if len(self.tags) > 0:
						self.tagLock.acquire()
						try:
							painter = QtGui.QPainter()
							painter.begin(image)
							painter.setPen(QtGui.QColor(0,255,0))
							painter.setBrush(QtGui.QColor(0,255,0))
							for (x,y,d) in self.tags:
								r = QtCore.QRectF((x*image.width())/1000-DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,DETECT_RADIUS*2,DETECT_RADIUS*2)
								painter.drawEllipse(r)
								painter.drawText((x*image.width())/1000+DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,str(d/100)[0:4]+'m')
							painter.end()
						finally:
							self.tagLock.release()

					zbarimg = self.convertQPixmapToZbar()
					scanner = zbar.ImageScanner()
					nCodes = scanner.scan(zbarimg)
					if nCodes > 0:
						sym = iter(zbarimg).next()
						qr_loc = sym.location

						middleX = ( (qr_loc[3][0] - qr_loc[0][0]) / 2) + qr_loc[0][0]
						middleY = ( (qr_loc[1][1] - qr_loc[0][1]) / 2) + qr_loc[0][1]

						painter = QtGui.QPainter()
						painter.begin(image)
						painter.setPen(QtGui.QColor(0,255,0))
						painter.setBrush(QtGui.QColor(0,255,0))
						r = QtCore.QRectF(middleX-DETECT_RADIUS,middleY-DETECT_RADIUS,DETECT_RADIUS*2,DETECT_RADIUS*2)
						painter.drawEllipse(r)

						for x in range(0,4):
							r = QtCore.QRectF(qr_loc[x][0]-DETECT_RADIUS,qr_loc[x][1]-DETECT_RADIUS,DETECT_RADIUS*2,DETECT_RADIUS*2)
							painter.drawEllipse(r)
							painter.drawText(qr_loc[x][0]+DETECT_RADIUS,qr_loc[x][1]-DETECT_RADIUS,str(qr_loc[x][0]) + ", " + str(qr_loc[x][1]))


						painter.end()

						qrpoints = Int32MultiArray()

						qrpoints.data = list(qr_loc[0] + qr_loc[1] + qr_loc[2] + qr_loc[3])
						self.publishQr.publish(qrpoints)

			finally:
				self.imageLock.release()

			# We could  do more processing (eg OpenCV) here if we wanted to, but for now lets just display the window.
			cv_image = self.Ros2CV()

			self.resize(image.width(),image.height())
			self.imageBox.setPixmap(image)

		# Update the status bar to show the current drone status & battery level
		self.statusBar().showMessage(self.statusMessage if self.connected else self.DisconnectedMessage)

	def ReceiveImage(self,data):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
		self.imageLock.acquire()
		try:
			self.image = data # Save the ros image for processing by the display thread
		finally:
			self.imageLock.release()

	def ReceiveNavdata(self,navdata):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# Update the message to be displayed
		msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
		self.statusMessage = '{} (Battery: {}%)'.format(msg,int(navdata.batteryPercent))

		self.tagLock.acquire()
		try:
			if navdata.tags_count > 0:
				self.tags = [(navdata.tags_xc[i],navdata.tags_yc[i],navdata.tags_distance[i]) for i in range(0,navdata.tags_count)]
			else:
				self.tags = []
		finally:
			self.tagLock.release()


	def FeaturePoint(self, cv_image = None, kp_filter = 2000):
		# If no input given, use current frame
		if cv_image is None:
			cv_image = self.Ros2CV()

		# Get a grayscale image of cv_image
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

		#sleep(10)
		harris = cv2.FeatureDetector_create('HARRIS')
		harris_kp = harris.detect(gray, None)
		#sleep(10)
		sift = cv2.DescriptorExtractor_create('SIFT')
		#sleep(10)
		kp, des = sift.compute(gray, harris_kp)
		#sleep(10)
		feature_image = cv2.drawKeypoints(cv_image, kp)

		return feature_image, kp, des

if __name__=='__main__':
	import sys
	rospy.init_node('ardrone_video_display')
	app = QtGui.QApplication(sys.argv)
	display = DroneVideoDisplay()
	display.show()
	status = app.exec_()
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
