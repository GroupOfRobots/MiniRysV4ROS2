import rclpy
import sys
import math
from time import sleep
from PyQt5.QtCore import QThread, pyqtSignal
from ROS.RysRemoteNode import RysRemoteNode

class QTRosBridge(QThread):
	"""docstring for RosBridge"""

	batteryChanged = pyqtSignal(int, int, int)
	imuChanged = pyqtSignal(float, float)
	temperatureChanged = pyqtSignal(int)
	rangesChanged = pyqtSignal(object)
	odometryChanged = pyqtSignal(object)
	regulatorSettingsSetDone = pyqtSignal(bool, str)
	regulatorSettingsGetDone = pyqtSignal(object)
	regulationChanged = pyqtSignal(float, float, float, float)

	def __init__(self, parent, robotName, nodeName, enableMotorsMessageRate = 0.5, steeringMessageRate = 10):
		super().__init__(parent)
		self.exitFlag = False
		self.previousRoll = 0.0

		rclpy.init(args = sys.argv)

		callbacks = {
			'battery': self.batteryCallback,
			'imu': self.imuSubscriptionCallback,
			'ranges': self.rangeSensorSubscriptionCallback,
			'temperature': self.temperatureSensorCallback,
			'odometry': self.odometryCallback,
			'regulation': self.regulationCallback,
		}
		messageRates = {
			'enableMotors': enableMotorsMessageRate,
			'steering': steeringMessageRate,
		}
		self.node = RysRemoteNode(robotName, nodeName, callbacks, messageRates)

	""" Public methods """

	def setEnabled(self, enabled):
		self.node.setEnabled(enabled)

	def setBalancingEnabled(self, balancingEnabled):
		self.node.setBalancingEnabled(balancingEnabled)

	def setSteering(self, throttle, rotation, precision):
		self.node.setSteering(throttle, rotation, precision)

	def stopExecution(self):
		self.exitFlag = True

	""" Subscription callbacks """

	def batteryCallback(self, message):
		self.batteryChanged.emit(message.voltage_cell1 * 1000, message.voltage_cell2 * 1000, message.voltage_cell3 * 1000)

	def imuSubscriptionCallback(self, message):
		if math.isnan(message.orientation.x):
			print('Got IMU message with invalid value')
			return

		qx = message.orientation.x
		qy = message.orientation.y
		qz = message.orientation.z
		qw = message.orientation.w

		roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
		# To degrees - it's easier to display
		roll = roll * 180 / math.pi

		if roll is self.previousRoll:
			return
		self.previousRoll = roll

		self.imuChanged.emit(roll, 0)

	def temperatureSensorCallback(self, message):
		self.temperatureChanged.emit(int(message.temperature))

	def rangeSensorSubscriptionCallback(self, message):
		self.rangesChanged.emit(message)

	def odometryCallback(self, message):
		self.odometryChanged.emit(message)

	def regulationCallback(self, message):
		self.regulationChanged.emit(message.roll, message.set_roll, message.speed, message.set_speed)

	""" QT signal handlers """

	def regulatorSettingsSetRequested(self, parameters):
		self.node.requestSetRegulatorSettings(parameters)

	def regulatorSettingsGetRequested(self):
		self.node.requestGetRegulatorSettings()

	""" Thread 'run' method """

	def run(self):
		# Start main thread loop
		while rclpy.ok() and not self.exitFlag:
			rclpy.spin_once(self.node)

			(setResponse, getResponse) = self.node.getRequestResponses()
			if setResponse is not None:
				self.regulatorSettingsSetDone.emit(setResponse.success, setResponse.error_text)

			if getResponse is not None:
				self.regulatorSettingsGetDone.emit(getResponse)

			sleep(0.01)

		# Cleanup: disable motors
		self.setEnabled(False)
		# Cleanup: destroy timers (threads!)
		self.node.destroyTimers()

		self.node.destroy_node()
		rclpy.shutdown()
