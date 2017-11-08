import rclpy
import sys
from math import pi
from time import sleep
from PyQt5.QtCore import QThread, pyqtSignal
from ROS.RysRemoteNode import RysRemoteNode

class QTRosBridge(QThread):
	"""docstring for RosBridge"""

	batteryChanged = pyqtSignal(int, int, int)
	imuChanged = pyqtSignal(float, float)
	temperatureChanged = pyqtSignal(int)
	rangesChanged = pyqtSignal(int, int, int, int, int)
	regulatorSettingsSetDone = pyqtSignal(bool, str)
	regulatorSettingsGetDone = pyqtSignal(object)

	def __init__(self, nodeName, parent = None, steeringMessageRate = 10, enableMessageRate = 0.5):
		super().__init__(parent)
		self.enableMessageRate = enableMessageRate
		self.steeringMessageRate = steeringMessageRate
		self.enabled = False
		self.balancingEnabled = False
		self.exitFlag = False

		self.previousRoll = 0.0
		self.previousRotationX = 0.0

		rclpy.init(args = sys.argv)

		enableTimerTime = 1.0 / self.enableMessageRate
		steeringTimerTime = 1.0 / self.steeringMessageRate
		self.node = RysRemoteNode(nodeName, self.batteryCallback, self.imuSubscriptionCallback, self.temperatureSensorCallback, self.rangeSensorSubscriptionCallback, enableTimerTime, steeringTimerTime)

	""" Public methods """

	def setEnabled(self, enabled):
		self.node.setEnabled(enabled)

	def setBalancingEnabled(self, balancingEnabled):
		self.node.setBalancingEnabled(balancingEnabled)

	def setSteering(self, throttle, rotation, precision):
		self.node.setSteering(throttle, rotation, precision)

	def calibrateImu(self):
		self.node.calibrateImu()

	def stopExecution(self):
		self.exitFlag = True

	""" Subscription callbacks """

	def batteryCallback(self, message):
		self.batteryChanged.emit(message.voltage_cell1 * 1000, message.voltage_cell2 * 1000, message.voltage_cell3 * 1000)

	def imuSubscriptionCallback(self, message):
		roll = message.roll * 180 / pi
		rotationX = message.rotation_x
		if roll is self.previousRoll and rotationX is self.previousRotationX:
			return
		self.previousRoll = roll
		self.previousRotationX = rotationX

		self.imuChanged.emit(roll, rotationX)

	def temperatureSensorCallback(self, message):
		print('Received: %f' % (message.data))
		self.temperatureChanged.emit(int(message.data))

	def rangeSensorSubscriptionCallback(self, message):
		self.rangesChanged.emit(message.front, message.back, message.top, message.left, message.right)

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
