from Mapper.QuadMapNode import QuadMapNode
from PyQt5.QtCore import QThread, QTimer, pyqtSignal
import PyKDL
import math

class Mapper(QThread):
	"""docstring for Mapper"""

	mapGenerated = pyqtSignal(list, int)

	def __init__(self, parent, rosBridge, cellSize = 0.04, mapSize = 2.56):
		super().__init__(parent)

		self.map = QuadMapNode(mapSize, cellSize)

		self.timer = QTimer(self)
		self.timer.timeout.connect(self.timerCallback)
		self.timer.start(200)

		self.robotPosition = PyKDL.Frame()
		self.path = [(0, PyKDL.Frame(self.robotPosition))]

		# All sensors 'aim' down their their Y axis
		# [Assuming laying down mode]
		# top sensor is offset by ~10cm on Y axis from base frame
		self.topRangeSensorFrame = PyKDL.Frame(PyKDL.Vector(0, 0.1, 0))
		# right sensor is offset by half frame width in X and slightly above wheel radius in Y, also rotated
		self.rightRangeSensorFrame = PyKDL.Frame(PyKDL.Rotation.RotZ(-math.pi / 2), PyKDL.Vector(0.065, 0.06, 0))
		# left sensor symmetrical to right sensor
		self.leftRangeSensorFrame = PyKDL.Frame(PyKDL.Rotation.RotZ(math.pi / 2), PyKDL.Vector(-0.065, 0.06, 0))

		rosBridge.odometryChanged.connect(self.odometryHandler)
		rosBridge.rangesChanged.connect(self.rangeReadingsHandler)

	def odometryHandler(self, message):
		# Construct a Frame out of the message
		poseTime = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1000000000
		position = message.pose.pose.position
		orientation = message.pose.pose.orientation
		poseFrame = PyKDL.Frame(PyKDL.Rotation(orientation.x, orientation.y, orientation.z, orientation.w), PyKDL.Vector(position.x, position.y, position.z))
		# Apply odometry transform onto robotPosition frame
		self.robotPosition = self.robotPosition * poseFrame
		# Save the frame and time to the path
		self.path.append((poseTime, PyKDL.Frame(self.robotPosition)))

	def rangeReadingsHandler(self, message):
		# Apply sensors' transforms to the current odometry position
		# Calculate linear functions' parameters
		# Pass rangings to the quadmap
		pass

	def timerCallback(self):
		# Build new map and fire the signal
		pass

	def run(self):
		# Empty?
		pass
