import math
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
		self.ui.steeringGraphicsView.scale(1, -1)
		self.ui.steeringGraphicsView.setScene(self.gamepadScene)
		self.ui.steeringGraphicsView.fitInView(self.gamepadScene.sceneRect(), QtCore.Qt.KeepAspectRatio)

		self.mapScene = QtWidgets.QGraphicsScene(self)
		self.ui.mapGraphicsView.scale(1, -1)
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

		# self.heightChanged.connect(self.updateScenes)
		# self.widthChanged.connect(self.updateScenes)
		self.adjustSize()
		# self.repaintSteering()

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
		self.repaintMap()

	def positionToggledHandler(self):
		self.positionEnabled = self.ui.positionCheckBox.isChecked()
		self.repaintMap()

	def obstaclesToggledHandler(self):
		self.obstaclesEnabled = self.ui.obstaclesCheckBox.isChecked()
		self.repaintMap()

	''' Gamepad bridge event handlers '''

	def gamepadAxisChangedHandler(self, gamepadAxisEvent):
		gamepadID = gamepadAxisEvent.gamepadID
		axis = gamepadAxisEvent.axis
		value = gamepadAxisEvent.value

		multiplier = self.ui.multiplierDoubleSpinBox.value()
		throttleMultiplier = -1 if self.ui.reverseThrottleCheckBox.isChecked() else 1

		update = False
		if gamepadID is self.gamepadID:
			if axis is self.throttleAxis:
				self.throttle = value * multiplier * throttleMultiplier
				self.ui.gamepadXLabel.setText('X: %1.4f' % self.throttle)
				update = True
			elif axis is self.rotationAxis:
				self.rotation = value * multiplier
				self.ui.gamepadYLabel.setText('Y: %1.4f' % self.rotation)
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
		front = message.front
		back = message.back
		top = message.top
		left = message.left
		right = message.right
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
		# print('obstacles', obstacles)
		self.repaintMap()

	''' Other methods '''

	def resizeEvent(self, event):
		self.repaintSteering()
		self.repaintMap()

	def repaintSteering(self):
		height = self.ui.steeringGraphicsView.size().height()
		width = self.ui.steeringGraphicsView.size().width()

		brush = QtGui.QBrush(QtGui.QColor(200, 0, 0), QtCore.Qt.SolidPattern)
		pen = QtGui.QPen(brush, 5.0)
		dotSize = 10

		self.gamepadScene.clear()
		self.gamepadScene.setSceneRect(0.0, 0.0, width, height)
		y = (self.throttle * 0.5 + 0.5) * height
		x = (self.rotation * 0.5 + 0.5) * width
		self.gamepadScene.addEllipse(x - dotSize / 2, y - dotSize / 2, dotSize, dotSize, pen, brush)

	def repaintMap(self):
		width = self.ui.mapGraphicsView.size().width()
		height = self.ui.mapGraphicsView.size().height()
		brush = QtGui.QBrush(QtGui.QColor(0, 0, 0), QtCore.Qt.SolidPattern)
		pen = QtGui.QPen(brush, 1.0)

		mapSize = self.mapper.mapSize

		sceneSize = min(width, height)
		xOffset = (width - sceneSize) * 0.5
		yOffset = (height - sceneSize) * 0.5

		self.mapScene.clear()
		self.mapScene.setSceneRect(0.0, 0.0, width, height)

		# Draw map borders (visual hint)
		self.mapScene.addLine(xOffset - 0.5, yOffset - 0.5, xOffset + sceneSize - 0.5, yOffset - 0.5, pen)
		self.mapScene.addLine(xOffset - 0.5, yOffset - 0.5, xOffset - 0.5, yOffset + sceneSize - 0.5, pen)
		self.mapScene.addLine(xOffset + sceneSize - 0.5, yOffset - 0.5, xOffset + sceneSize - 0.5, yOffset + sceneSize - 0.5, pen)
		self.mapScene.addLine(xOffset - 0.5, yOffset + sceneSize - 0.5, xOffset + sceneSize - 0.5, yOffset + sceneSize - 0.5, pen)
		# Center lines (axis)
		self.mapScene.addLine(xOffset - 0.5, yOffset + sceneSize / 2 - 0.5, xOffset + sceneSize - 0.5, yOffset + sceneSize / 2 - 0.5, pen)
		self.mapScene.addLine(xOffset + sceneSize / 2 - 0.5, yOffset - 0.5, xOffset + sceneSize / 2 - 0.5, yOffset + sceneSize - 0.5, pen)

		# Draw path
		if self.pathEnabled and len(self.mapPositions):
			brush = QtGui.QBrush(QtGui.QColor(0, 0, 200), QtCore.Qt.SolidPattern)
			dotSize = 1.0
			pen = QtGui.QPen(brush, dotSize)
			for position in self.mapPositions:
				# x and y are in <-mapSize/2; mapSize/2> range
				# First, scale them to <0; 1>, then multiply by map scene size
				x = (0.5 + position[0] / mapSize) * sceneSize + xOffset
				y = (0.5 + position[1] / mapSize) * sceneSize + yOffset
				self.mapScene.addRect(x - dotSize / 2, y - dotSize / 2, dotSize, dotSize, pen, brush)

		# Draw robot's position
		if self.positionEnabled and len(self.mapPositions):
			brush = QtGui.QBrush(QtGui.QColor(0, 200, 0), QtCore.Qt.SolidPattern)
			dotSize = 2.0
			pen = QtGui.QPen(brush, dotSize)
			length = 20.0

			position = self.mapPositions[len(self.mapPositions) - 1]
			x = (0.5 + position[0] / mapSize) * sceneSize + xOffset
			y = (0.5 + position[1] / mapSize) * sceneSize + yOffset
			x2 = x + length * math.cos(self.mapPositionAngle)
			y2 = y + length * math.sin(self.mapPositionAngle)
			self.mapScene.addLine(x, y, x2, y2, pen)
			dotSize = 4.0
			self.mapScene.addEllipse(x - dotSize / 2, y - dotSize / 2, dotSize, dotSize, pen, brush)

		# Draw obstacles
		if self.obstaclesEnabled and len(self.mapObstacles):
			dotSize = 1.0
			obstacleSize = sceneSize / (self.mapper.mapSize / self.mapper.cellSize)

			for obstacle in self.mapObstacles:
				brush = QtGui.QBrush(QtGui.QColor(255, (1 - obstacle[2]) * 255, (1 - obstacle[2]) * 255), QtCore.Qt.SolidPattern)
				pen = QtGui.QPen(brush, dotSize)
				x = (0.5 + obstacle[0] / mapSize) * sceneSize + xOffset - obstacleSize
				y = (0.5 + obstacle[1] / mapSize) * sceneSize + yOffset - obstacleSize
				self.mapScene.addRect(x, y, obstacleSize, obstacleSize, pen, brush)
