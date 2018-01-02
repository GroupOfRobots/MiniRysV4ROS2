from Mapper.QuadMapNode import QuadMapNode
from PyQt5.QtCore import QThread, QTimer, pyqtSignal
import PyKDL
import math

class Mapper(QThread):
	'''docstring for Mapper'''

	'''list of positions: (x, y), angle of current position: w, list of detected obstacles: (x, y)'''
	mapGenerated = pyqtSignal(list, float, list)

	# startPosition = PyKDL.Frame(PyKDL.Rotation.RotZ(math.pi / 2))
	startPosition = PyKDL.Frame()
	headingAdjustmentFrame = PyKDL.Frame(PyKDL.Rotation.RotZ(math.pi / 2))

	def __init__(self, parent, rosBridge, cellSize = 0.04, mapSize = 2.56, sensorMinThreshold = 0.04, sensorMaxThreshold = 1.2):
		super().__init__(parent)

		self.updated = True

		self.cellSize = cellSize
		self.mapSize = mapSize
		self.sensorMinThreshold = sensorMinThreshold
		self.sensorMaxThreshold = sensorMaxThreshold

		self.map = QuadMapNode(self.mapSize, self.cellSize)

		self.timer = QTimer(self)
		self.timer.timeout.connect(self.timerCallback)
		self.timer.start(200)

		# Robot is Y-forward-oriented, so it starts with angle=PI/2
		self.robotPosition = PyKDL.Frame(Mapper.startPosition)
		self.path = [(0, PyKDL.Frame(self.robotPosition))]
		self.odoSeq = 0

		# All sensors 'aim' down their their Y axis
		# [Assuming laying down mode]
		# top sensor is offset by ~10cm on Y axis from base frame
		self.topRangeSensorFrame = PyKDL.Frame(PyKDL.Vector(0, 0.086, 0))
		# right sensor is offset by half frame width in X and slightly above wheel radius in Y, also rotated
		self.rightRangeSensorFrame = PyKDL.Frame(PyKDL.Rotation.RotZ(-math.pi / 2), PyKDL.Vector(0.059, 0.067, 0))
		# left sensor symmetrical to right sensor
		self.leftRangeSensorFrame = PyKDL.Frame(PyKDL.Rotation.RotZ(math.pi / 2), PyKDL.Vector(-0.059, 0.067, 0))

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

		# Debug print, enable if odometry frames are suspected to be lost
		# print('s: %d h: %f' % (self.odoSeq, self.robotPosition.M.GetRPY()[2]))
		# print('s: %d' % self.odoSeq)
		self.odoSeq = self.odoSeq + 1

		# Save the frame and time to the path
		poseTime = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1000000000
		self.path.append((poseTime, PyKDL.Frame(self.robotPosition)))

		# Mark path as updated so we'll emit it to GUI
		self.updated = True

	def rangeReadingsHandler(self, message):
		# Left
		# First, decide whether to interpret the reading at all
		leftSensorRange = message.left * 0.001
		if leftSensorRange > self.sensorThreshold and leftSensorRange < self.sensorMaxThreshold:
			# Then, apply current odometry transform to the frame of left sensor
			leftSensorPosition = self.leftRangeSensorFrame * self.robotPosition * Mapper.headingAdjustmentFrame
			# Then, calculate linear function parameters
			leftSensorX0 = leftSensorPosition.p.x()
			leftSensorY0 = leftSensorPosition.p.y()
			leftSensorAngle = leftSensorPosition.M.GetRPY()[2]
			# Ranges are in millimeters, our calculation - in meters
			leftSensorX1 = leftSensorX0 + math.cos(leftSensorAngle) * leftSensorRange
			leftSensorA = math.tan(leftSensorAngle)
			leftSensorB = leftSensorY0 - leftSensorA * leftSensorX0
			# Then, pass calculated parameters to quadmap
			self.map.addScan(leftSensorA, leftSensorB, leftSensorX0, leftSensorX1)

		# Right, same thing
		rightSensorRange = message.right * 0.001
		if rightSensorRange > self.sensorThreshold and rightSensorRange < self.sensorMaxThreshold:
			rightSensorPosition = self.rightRangeSensorFrame * self.robotPosition * Mapper.headingAdjustmentFrame
			rightSensorX0 = rightSensorPosition.p.x()
			rightSensorY0 = rightSensorPosition.p.y()
			rightSensorAngle = rightSensorPosition.M.GetRPY()[2]
			rightSensorX1 = rightSensorX0 + math.cos(rightSensorAngle) * rightSensorRange
			rightSensorA = math.tan(rightSensorAngle)
			rightSensorB = rightSensorY0 - rightSensorA * rightSensorX0
			self.map.addScan(rightSensorA, rightSensorB, rightSensorX0, rightSensorX1)

		# Top
		topSensorRange = message.top * 0.001
		if topSensorRange > self.sensorMinThreshold and topSensorRange < self.sensorMaxThreshold:
			topSensorPosition = self.topRangeSensorFrame * self.robotPosition * Mapper.headingAdjustmentFrame
			topSensorX0 = topSensorPosition.p.x()
			topSensorY0 = topSensorPosition.p.y()
			topSensorAngle = topSensorPosition.M.GetRPY()[2]
			topSensorX1 = topSensorX0 + math.cos(topSensorAngle) * topSensorRange
			topSensorA = math.tan(topSensorAngle)
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
			# frameTuple[0] is time, [1] is a Frame
			x = frameTuple[1].p.x()
			y = frameTuple[1].p.y()
			positions.append((x, y))

		# Second, current heading - yaw from current position plus 90deg (aim towards Y)
		positionAngle = self.robotPosition.M.GetRPY()[2] + math.pi / 2
		if positionAngle > math.pi:
			positionAngle -= 2 * math.pi

		# Third, obstacle list
		obstacles = self.map.getOccupancyMap()

		# Lastly, emit the signal
		self.mapGenerated.emit(positions, positionAngle, obstacles)

	def clearMap(self):
		self.robotPosition = PyKDL.Frame(Mapper.startPosition)
		self.path = [(0, PyKDL.Frame(self.robotPosition))]
		self.map = QuadMapNode(self.mapSize, self.cellSize)
		self.updated = True

	def run(self):
		# Empty?
		pass
