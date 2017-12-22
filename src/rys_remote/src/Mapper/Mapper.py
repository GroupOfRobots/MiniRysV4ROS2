from Mapper.QuadMapNode import QuadMapNode
from PyQt5.QtCore import QThread, QTimer, pyqtSignal
import PyKDL
import math

class Mapper(QThread):
	'''docstring for Mapper'''

	'''list of positions: (x, y), angle of current position: w, list of detected obstacles: (x, y)'''
	mapGenerated = pyqtSignal(list, float, list)

	startPosition = PyKDL.Frame(PyKDL.Rotation.RotZ(math.pi / 2))

	def __init__(self, parent, rosBridge, cellSize = 0.04, mapSize = 2.56):
		super().__init__(parent)

		self.updated = True

		self.cellSize = cellSize
		self.mapSize = mapSize

		self.map = QuadMapNode(mapSize, cellSize)

		self.timer = QTimer(self)
		self.timer.timeout.connect(self.timerCallback)
		self.timer.start(200)

		# Robot is Y-forward-oriented, so it starts with angle=PI/2
		self.robotPosition = PyKDL.Frame(Mapper.startPosition)
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
		# Construct a PyKDL.Frame out of the message
		position = message.pose.pose.position
		orientation = message.pose.pose.orientation
		positionVector = PyKDL.Vector(position.x, position.y, position.z)
		orientationQuaternion = PyKDL.Rotation.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
		poseFrame = PyKDL.Frame(orientationQuaternion, positionVector)

		# TODO: odometry filtering (e.g. from IMU) goes here

		# Apply odometry transform onto robotPosition frame
		self.robotPosition = self.robotPosition * poseFrame

		print('time: %f heading: %f' % (message.header.stamp.sec + message.header.stamp.nanosec * 0.000000001, self.robotPosition.M.GetRPY()[2]))

		# Save the frame and time to the path
		poseTime = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1000000000
		self.path.append((poseTime, PyKDL.Frame(self.robotPosition)))

		# Mark path as updated so we'll emit it to GUI
		self.updated = True

	def rangeReadingsHandler(self, message):
		# Left
		# First, apply sensor's transform to the current odometry position
		leftSensorPosition = self.robotPosition * self.leftRangeSensorFrame
		# Then, calculate linear function parameters
		leftSensorX0 = leftSensorPosition.p.x()
		leftSensorX1 = leftSensorX0 + math.cos(leftSensorPosition.M.GetRPY()[2]) * message.left
		leftSensorY0 = leftSensorPosition.p.y()
		leftSensorA = leftSensorPosition.M.GetRPY()[2]
		leftSensorB = leftSensorY0 - leftSensorA * leftSensorX0
		# Then, pass calculated parameters to quadmap
		self.map.addScan(leftSensorA, leftSensorB, leftSensorX0, leftSensorX1)

		# Right, same thing
		rightSensorPosition = self.robotPosition * self.rightRangeSensorFrame
		rightSensorX0 = rightSensorPosition.p.x()
		rightSensorX1 = rightSensorX0 + math.cos(rightSensorPosition.M.GetRPY()[2]) * message.right
		rightSensorY0 = rightSensorPosition.p.y()
		rightSensorA = rightSensorPosition.M.GetRPY()[2]
		rightSensorB = rightSensorY0 - rightSensorA * rightSensorX0
		self.map.addScan(rightSensorA, rightSensorB, rightSensorX0, rightSensorX1)

		# Top
		topSensorPosition = self.robotPosition * self.topRangeSensorFrame
		topSensorX0 = topSensorPosition.p.x()
		topSensorX1 = topSensorX0 + math.cos(topSensorPosition.M.GetRPY()[2]) * message.top
		topSensorY0 = topSensorPosition.p.y()
		topSensorA = topSensorPosition.M.GetRPY()[2]
		topSensorB = topSensorY0 - topSensorA * topSensorX0
		self.map.addScan(topSensorA, topSensorB, topSensorX0, topSensorX1)

		self.updated = True

	def timerCallback(self):
		# Check whether there was an update
		if not self.updated:
			return
		self.updated = False

		# First, odometry position list
		positions = list()
		for frameTuple in self.path:
			x = frameTuple[1].p.x()
			y = frameTuple[1].p.y()
			positions.append((x, y))

		# Second, current heading
		# Get yaw from last element of the path
		# positionAngle = self.path[len(self.path) - 1][1].M.GetRPY()[2]
		positionAngle = self.robotPosition.M.GetRPY()[2]

		# Third, obstacle list
		obstacles = self.map.getOccupancyMap()

		# Lastly, emit the signal
		self.mapGenerated.emit(positions, positionAngle, obstacles)

	def clearMap(self):
		self.robotPosition = PyKDL.Frame(Mapper.startPosition)
		self.path = [(0, PyKDL.Frame(self.robotPosition))]
		self.updated = True

	def run(self):
		# Empty?
		pass
