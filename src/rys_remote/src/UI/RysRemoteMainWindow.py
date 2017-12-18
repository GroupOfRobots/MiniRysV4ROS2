from PyQt5 import QtWidgets, QtCore, QtGui
from UI.Layouts import Ui_RysRemoteMainWindow
from UI.RysRemoteRegulatorSettingsDialog import RysRemoteRegulatorSettingsDialog

class RysRemoteMainWindow(QtWidgets.QMainWindow):
	'''
	Main window class for RysRemote.
	Inherits from QMainWindow.
	'''

	def __init__(self, parent, gamepadBridge, rosBridge, mapper):
		super(RysRemoteMainWindow, self).__init__(parent)

		self.enabled = False
		self.throttle = 0
		self.rotation = 0
		self.gamepadID = -1
		self.throttleAxis = -1
		self.rotationAxis = -1

		self.pathEnabled = True
		self.positionEnabled = True
		self.obstaclesEnabled = True

		self.mapPositions = list()
		self.mapPositionAngle = 0
		self.mapObstacles = list()

		self.ui = Ui_RysRemoteMainWindow()
		self.ui.setupUi(self)

		self.gamepadScene = QtWidgets.QGraphicsScene(self)
		self.ui.steeringGraphicsView.setScene(self.gamepadScene)
		self.ui.steeringGraphicsView.fitInView(self.gamepadScene.sceneRect(), QtCore.Qt.KeepAspectRatio)

		self.mapScene = QtWidgets.QGraphicsScene(self)
		self.ui.mapGraphicsView.setScene(self.mapScene)
		self.ui.mapGraphicsView.fitInView(self.mapScene.sceneRect(), QtCore.Qt.KeepAspectRatio)

		self.ui.enableButton.clicked.connect(self.enableClickedHandler)
		self.ui.balancingEnabledCheckBox.toggled.connect(self.balancingEnabledChangedHandler)
		self.ui.regulatorSettingsButton.clicked.connect(self.regulatorSettingsButtonHandler)
		self.ui.gamepadComboBox.currentIndexChanged.connect(self.gamepadChangedHandler)
		self.ui.throttleComboBox.currentTextChanged.connect(self.throttleAxisChangedHandler)
		self.ui.rotationComboBox.currentTextChanged.connect(self.rotationAxisChangedHandler)
		self.ui.pathCheckBox.toggled.connect(self.pathToggledHandler)
		self.ui.positionCheckBox.toggled.connect(self.positionToggledHandler)
		self.ui.obstaclesCheckBox.toggled.connect(self.obstaclesToggledHandler)

		gamepadBridge.gamepadAxisChanged.connect(self.gamepadAxisChangedHandler)
		gamepadBridge.gamepadButtonChanged.connect(self.gamepadButtonChangedHandler)
		gamepadBridge.gamepadListUpdated.connect(self.gamepadListUpdatedHandler)

		rosBridge.batteryChanged.connect(self.batteryChangedHandler)
		rosBridge.imuChanged.connect(self.imuChangedHandler)
		rosBridge.temperatureChanged.connect(self.temperatureChangedHandler)
		rosBridge.rangesChanged.connect(self.rangesChangedHandler)
		self.rosBridge = rosBridge

		mapper.mapGenerated.connect(self.mapGeneratedHandler)
		self.ui.clearMapButton.clicked.connect(mapper.clearMap)
		self.mapper = mapper

		self.adjustSize()
		self.repaintSteering()

	''' UI event handlers '''

	def enableClickedHandler(self):
		self.enabled = not self.enabled
		self.rosBridge.setEnabled(self.enabled)
		text = 'Disable' if self.enabled else 'Enable'

		self.ui.enableButton.setText(text)
		color = 'red' if self.enabled else 'green'
		self.ui.enableButton.setStyleSheet('background-color: %s;' % color)

	def balancingEnabledChangedHandler(self, value):
		balancingEnabled = self.ui.balancingEnabledCheckBox.isChecked()
		self.rosBridge.setBalancingEnabled(balancingEnabled)
		# self.rosBridge.setBalancingEnabled(value)

	def regulatorSettingsButtonHandler(self):
		# Open regulator setting dialog
		regulatorSettingsDialog = RysRemoteRegulatorSettingsDialog(self, self.rosBridge)
		regulatorSettingsDialog.show()
		regulatorSettingsDialog.exec_()

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

	def pathToggledHandler(self):
		self.pathEnabled = self.ui.pathCheckBox.isChecked()

	def positionToggledHandler(self):
		self.positionEnabled = self.ui.positionCheckBox.isChecked()

	def obstaclesToggledHandler(self):
		self.obstaclesEnabled = self.ui.obstaclesCheckBox.isChecked()

	''' Gamepad bridge event handlers '''

	def gamepadAxisChangedHandler(self, gamepadAxisEvent):
		gamepadID = gamepadAxisEvent.gamepadID
		axis = gamepadAxisEvent.axis
		value = gamepadAxisEvent.value

		multiplier = self.ui.multiplierDoubleSpinBox.value()

		update = False
		if gamepadID is self.gamepadID:
			if axis is self.throttleAxis:
				self.throttle = value * multiplier
				self.ui.gamepadXLabel.setText('X: %1.4f' % value)
				update = True
			elif axis is self.rotationAxis:
				self.rotation = value * multiplier
				self.ui.gamepadYLabel.setText('Y: %1.4f' % value)
				update = True

		if update:
			precision = self.ui.precisionSpinBox.value()
			self.rosBridge.setSteering(self.throttle, self.rotation, precision)
			self.repaintSteering()

	def gamepadButtonChangedHandler(self, gamepadButtonEvent):
		pass
		# gamepadID = gamepadButtonEvent.gamepadID
		# button = gamepadButtonEvent.button
		# value = gamepadButtonEvent.value

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

	''' ROS event handlers '''

	def batteryChangedHandler(self, cell1, cell2, cell3):
		maxValue = 5000
		if (cell1 > maxValue or cell2 > maxValue or cell3 > maxValue):
			self.ui.cell1Bar.setMaximum(15000)
			self.ui.cell1Bar.setValue(cell1 + cell2 + cell3)
			self.ui.cell2Bar.setValue(0)
			self.ui.cell3Bar.setValue(0)

			self.ui.cell2Bar.setDisabled(True)
			self.ui.cell3Bar.setDisabled(True)
		else:
			self.ui.cell1Bar.setMaximum(maxValue)
			self.ui.cell1Bar.setValue(cell1)
			self.ui.cell2Bar.setValue(cell2)
			self.ui.cell3Bar.setValue(cell3)

			self.ui.cell2Bar.setDisabled(False)
			self.ui.cell3Bar.setDisabled(False)

	def imuChangedHandler(self, roll, rotationX):
		self.ui.rollDial.setValue(int(roll))
		self.ui.rollValueLabel.setText('    Roll: %f' % roll)
		self.ui.rotationXValueLabel.setText('Rotation: %f' % rotationX)

	def rangesChangedHandler(self, message):
		front, back, top, left, right = message
		if (front >= 0):
			self.ui.rangeFrontBar.setEnabled(True)
			maxValue = self.ui.rangeFrontBar.maximum()
			self.ui.rangeFrontBar.setValue(front if front < maxValue else maxValue)
		else:
			self.ui.rangeFrontBar.setEnabled(False)

		if (back >= 0):
			self.ui.rangeBackBar.setEnabled(True)
			maxValue = self.ui.rangeBackBar.maximum()
			self.ui.rangeBackBar.setValue(back if back < maxValue else maxValue)
		else:
			self.ui.rangeBackBar.setEnabled(False)

		if (top >= 0):
			self.ui.rangeTopBar.setEnabled(True)
			maxValue = self.ui.rangeTopBar.maximum()
			self.ui.rangeTopBar.setValue(top if top < maxValue else maxValue)
		else:
			self.ui.rangeTopBar.setEnabled(False)

		if (left >= 0):
			self.ui.rangeLeftBar.setEnabled(True)
			maxValue = self.ui.rangeLeftBar.maximum()
			self.ui.rangeLeftBar.setValue(left if left < maxValue else maxValue)
		else:
			self.ui.rangeLeftBar.setEnabled(False)

		if (right >= 0):
			self.ui.rangeRightBar.setEnabled(True)
			maxValue = self.ui.rangeRightBar.maximum()
			self.ui.rangeRightBar.setValue(right if right < maxValue else maxValue)
		else:
			self.ui.rangeRightBar.setEnabled(False)

	def temperatureChangedHandler(self, temperature):
		self.ui.temperatureBar.setValue(temperature)

	def regulatorSettingsSetDoneHandler(self, success, errorText):
		if success:
			self.ui.statusBar.showMessage('Regulator parameters set!', 3000)
		else:
			self.ui.statusBar.showMessage('Error setting regulator parameters: %s' % errorText, 5000)

	def regulatorSettingsGetDoneHandler(self, parameters):
		self.ui.statusBar.showMessage('Regulator parameters retrieved!', 3000)

		self.ui.speedFilteringSpinBox.setValue(parameters.speed_filter_factor)
		self.ui.rollFilteringSpinBox.setValue(parameters.angle_filter_factor)
		self.ui.lqrEnabledCheckBox.setChecked(parameters.lqr_enabled)
		self.ui.pidSpeedStageEnabledCheckBox.setChecked(parameters.pid_speed_regulator_enabled)
		self.ui.pidSpeedKpSpinBox.setValue(parameters.pid_speed_kp)
		self.ui.pidSpeedKiSpinBox.setValue(parameters.pid_speed_ki)
		self.ui.pidSpeedKdSpinBox.setValue(parameters.pid_speed_kd)
		self.ui.pidAngleKpSpinBox.setValue(parameters.pid_angle_kp)
		self.ui.pidAngleKiSpinBox.setValue(parameters.pid_angle_ki)
		self.ui.pidAngleKdSpinBox.setValue(parameters.pid_angle_kd)
		self.ui.lqrLinearVelocityKSpinBox.setValue(parameters.lqr_linear_velocity_k)
		self.ui.lqrAngularVelocityKSpinBox.setValue(parameters.lqr_angular_velocity_k)
		self.ui.lqrAngleKSpinBox.setValue(parameters.lqr_angle_k)

	''' Mapper events '''

	def mapGeneratedHandler(self, positions, positionAngle, obstacles):
		self.mapPositions = positions
		self.mapPositionAngle = positionAngle
		self.mapObstacles = obstacles
		self.repaintMap()

	''' Other methods '''

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

	def repaintMap(self):
		width = self.ui.mapGraphicsView.size().width()
		height = self.ui.mapGraphicsView.size().height()
		brush = QtGui.QBrush(QtGui.QColor(0, 0, 200), QtCore.Qt.SolidPattern)
		pen = QtGui.QPen(brush, 1.0)

		halfMapSize = self.mapper.mapSize / 2

		self.mapScene.clear()
		self.mapScene.setSceneRect(0.0, 0.0, width, height)

		if self.pathEnabled:
			# Draw path
			dotSize = 1
			for position in self.mapPositions:
				# x and y are in <-mapSize/2; mapSize/2> range
				# First, scale them to <0; 1>, then multiply by map scene size
				x = (position[1] / halfMapSize + 0.5) * width
				y = (position[0] / halfMapSize + 0.5) * height
				self.mapScene.addEllipse(x - dotSize / 2, y - dotSize / 2, dotSize, dotSize, pen, brush)
		if self.positionEnabled:
			# Draw robot's position
			pass
		if self.obstaclesEnabled:
			# Draw obstacles
			pass
