from PyQt5 import QtWidgets, QtCore, QtGui
from Gamepad import GamepadBridge
from UI.Layouts import Ui_RysRemoteMainWindow
from ROS import RosBridge

class RysRemoteMainWindow(QtWidgets.QMainWindow):
	"""
	Main window class for RysRemote.
	Inherits from QMainWindow.
	"""

	regulatorSettingsSetRequested = QtCore.pyqtSignal(object)
	regulatorSettingsGetRequested = QtCore.pyqtSignal()

	def __init__(self, node, parent = None):
		super(RysRemoteMainWindow, self).__init__(parent)

		self.node = node
		self.qtParent = parent

		self.balancingEnabled = True
		self.throttle = 0
		self.rotation = 0
		self.gamepadID = -1
		self.throttleAxis = -1
		self.rotationAxis = -1

		self.ui = Ui_RysRemoteMainWindow()
		self.ui.setupUi(self)

		self.gamepadScene = QtWidgets.QGraphicsScene(self)
		self.ui.steeringGraphicsView.setScene(self.gamepadScene)
		self.ui.steeringGraphicsView.fitInView(self.gamepadScene.sceneRect(), QtCore.Qt.KeepAspectRatio)

		self.ui.imuCalibrateButton.clicked.connect(self.imuCalibrateClickedHandler)
		self.ui.setRegulatorParametersButton.clicked.connect(self.setRegulatorParametersClickedHandler)
		self.ui.getRegulatorParametersButton.clicked.connect(self.getRegulatorParametersClickedHandler)
		self.ui.enableButton.clicked.connect(self.enableClickedHandler)
		self.ui.balancingEnabledCheckBox.toggled.connect(self.balancingEnabledChangedHandler)
		self.ui.gamepadComboBox.currentIndexChanged.connect(self.gamepadChangedHandler)
		self.ui.throttleComboBox.currentTextChanged.connect(self.throttleAxisChangedHandler)
		self.ui.rotationComboBox.currentTextChanged.connect(self.rotationAxisChangedHandler)

		self.gamepadBridge = GamepadBridge(self)
		self.gamepadBridge.gamepadAxisChanged.connect(self.gamepadAxisChangedHandler)
		self.gamepadBridge.gamepadButtonChanged.connect(self.gamepadButtonChangedHandler)
		self.gamepadBridge.gamepadListUpdated.connect(self.gamepadListUpdatedHandler)

		self.rosBridge = RosBridge(node, self)
		self.rosBridge.imuChanged.connect(self.imuChangedHandler)
		self.rosBridge.sonarChanged.connect(self.sonarChangedHandler)
		self.rosBridge.regulatorSettingsSetDone.connect(self.regulatorSettingsSetDoneHandler)
		self.rosBridge.regulatorSettingsGetDone.connect(self.regulatorSettingsGetDoneHandler)
		self.regulatorSettingsSetRequested.connect(self.rosBridge.regulatorSettingsSetRequested)
		self.regulatorSettingsGetRequested.connect(self.rosBridge.regulatorSettingsGetRequested)

		self.adjustSize()
		self.repaintSteering()

		self.gamepadBridge.start()
		self.rosBridge.start()

		# Initialization actions
		self.rosBridge.setEnabled(False)

	""" UI event handlers """

	def imuCalibrateClickedHandler(self):
		self.rosBridge.calibrateImu()

	def setRegulatorParametersClickedHandler(self):
		parameters = {
			'speedFilterFactor': self.ui.speedFilteringSpinBox.value(),
			'rollFilterFactor': self.ui.rollFilteringSpinBox.value(),
			'lqrEnabled': self.ui.lqrEnabledCheckBox.isChecked(),
			'pidSpeedRegulatorEnabled': self.ui.pidSpeedStageEnabledCheckBox.isChecked(),
			'pidSpeedKp': self.ui.pidSpeedKpSpinBox.value(),
			'pidSpeedKi': self.ui.pidSpeedKiSpinBox.value(),
			'pidSpeedKd': self.ui.pidSpeedKdSpinBox.value(),
			'pidAngleKp': self.ui.pidAngleKpSpinBox.value(),
			'pidAngleKi': self.ui.pidAngleKiSpinBox.value(),
			'pidAngleKd': self.ui.pidAngleKdSpinBox.value(),
			'pidAngularVelocityFactor': self.ui.pidAngularVelocitySpinBox.value(),
			'lqrAngularVelocityK': self.ui.lqrAngularVelocityKSpinBox.value(),
			'lqrAngleK': self.ui.lqrAngleKSpinBox.value(),
		}

		self.regulatorSettingsSetRequested.emit(parameters)

	def getRegulatorParametersClickedHandler(self):
		self.regulatorSettingsGetRequested.emit()

	def enableClickedHandler(self):
		self.enabled = not self.enabled
		self.rosBridge.setEnabled(self.enabled)
		text = "Disable" if self.enabled else "Enable"

		self.ui.enableButton.setText(text)
		color = "red" if self.enabled else "green"
		self.ui.enableButton.setStyleSheet("background-color: %s;" % color)

	def balancingEnabledChangedHandler(self, value):
		# balancingEnabled = self.ui.balancingEnabledCheckBox.isChecked()
		# self.rosBridge.setBalancingEnabled(balancingEnabled)
		self.rosBridge.setBalancingEnabled(value)

	def gamepadChangedHandler(self, index):
		try:
			self.gamepadID = int(index)
		except ValueError:
			self.gamepadID = -1

	def throttleAxisChangedHandler(self, event):
		try:
			self.throttleAxis = int(event)
		except ValueError:
			self.throttleAxis = -1
		self.throttle = 0

	def rotationAxisChangedHandler(self, event):
		try:
			self.rotationAxis = int(event)
		except ValueError:
			self.rotationAxis = -1
		self.rotation = 0

	""" Gamepad bridge event handlers """

	def gamepadAxisChangedHandler(self, gamepadAxisEvent):
		gamepadID = gamepadAxisEvent.gamepadID
		axis = gamepadAxisEvent.axis
		value = gamepadAxisEvent.value
		print("axis event: joy %d, axis %d, value %f" % (gamepadID, axis, value))

		update = False
		if gamepadID is self.gamepadID:
			if axis is self.throttleAxis:
				self.throttle = value
				update = True
			elif axis is self.rotationAxis:
				self.rotation = value
				update = True

		if update:
			self.rosBridge.setSteering(self.throttle, self.rotation)
			self.repaintSteering()

	def gamepadButtonChangedHandler(self, gamepadButtonEvent):
		gamepadID = gamepadButtonEvent.gamepadID
		button = gamepadButtonEvent.button
		value = gamepadButtonEvent.value
		print("butt event: joy %d, butt %d, value %f" % (gamepadID, button, value))

	def gamepadListUpdatedHandler(self, gamepadList):
		self.gamepadID = -1

		for i in range(self.ui.gamepadComboBox.count()):
			self.ui.gamepadComboBox.removeItem(0)

		if len(gamepadList) > 0:
			for i in range(len(gamepadList)):
				self.ui.gamepadComboBox.addItem(gamepadList[i], i)
			self.ui.gamepadComboBox.setEnabled(True)
			self.ui.throttleComboBox.setEnabled(True)
			self.ui.rotationComboBox.setEnabled(True)

			self.gamepadID = 0
			self.throttleAxis = -1
			self.rotationAxis = -1
		else:
			self.ui.gamepadComboBox.setEnabled(False)
			self.ui.throttleComboBox.setEnabled(False)
			self.ui.rotationComboBox.setEnabled(False)
			self.gamepadID = -1
			self.throttleAxis = -1
			self.rotationAxis = -1

	""" ROS event handlers """

	def imuChangedHandler(self, roll, rotationX):
		self.ui.rollDial.setValue(float(roll))
		self.ui.rollValueLabel.setText("    Roll: %f" % roll)
		self.ui.rotationXValueLabel.setText("Rotation: %f" % rotationX)

	def sonarChangedHandler(self, front, back, top):
		self.ui.sonarFrontBar.setValue(front)
		self.ui.sonarBackBar.setValue(back)
		self.ui.sonarTopBar.setValue(top)

	def regulatorSettingsSetDoneHandler(self, success, errorText):
		if not success:
			QtWidgets.QMessageBox.Error(self.qtParent, "Error setting regulator parameters: %s" % errorText)

	def regulatorSettingsGetDoneHandler(self, parameters):
		pass

	""" Miscelanneous event handlers """

	def quitEventHandler(self):
		# "Join" all QThreads
		# Gamepad bridge
		if self.gamepadBridge.isRunning():
			loop = QtCore.QEventLoop()
			self.gamepadBridge.finished.connect(loop.quit)
			self.gamepadBridge.stopExecution()
			loop.exec_()

		# Ros bridge
		if self.rosBridge.isRunning():
			loop = QtCore.QEventLoop()
			self.rosBridge.finished.connect(loop.quit)
			self.rosBridge.stopExecution()
			loop.exec_()

	""" Other methods """

	def repaintSteering(self):
		self.gamepadScene.clear()
		height = self.ui.steeringGraphicsView.size().height()
		width = self.ui.steeringGraphicsView.size().width()
		self.gamepadScene.setSceneRect(0.0, 0.0, width, height)

		brush = QtGui.QBrush(QtGui.QColor(200, 0, 0), QtCore.Qt.SolidPattern)
		pen = QtGui.QPen(brush, 5.0)

		y = (self.throttle * 0.5 + 0.5) * height
		x = (self.rotation * 0.5 + 0.5) * width
		dotSize = 10
		self.gamepadScene.addEllipse(x - dotSize / 2, y - dotSize / 2, dotSize, dotSize, pen, brush)
