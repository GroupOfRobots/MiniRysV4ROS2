from PyQt5 import QtWidgets, QtCore
from UI.Layouts import Ui_RegulatorSettingsDialog

class RysRemoteRegulatorSettingsDialog(QtWidgets.QDialog):
	'''
	Dialog for managing regulator settings for RysRemote.
	Inherits from QDialog.
	'''

	regulatorSettingsSetRequested = QtCore.pyqtSignal(object)
	regulatorSettingsGetRequested = QtCore.pyqtSignal()

	def __init__(self, parent, rosBridge):
		super(RysRemoteRegulatorSettingsDialog, self).__init__(parent)

		self.enabled = False
		self.throttle = 0
		self.rotation = 0
		self.gamepadID = -1
		self.throttleAxis = -1
		self.rotationAxis = -1

		self.ui = Ui_RegulatorSettingsDialog()
		self.ui.setupUi(self)

		self.ui.setRegulatorParametersButton.clicked.connect(self.setRegulatorParametersClickedHandler)
		self.ui.getRegulatorParametersButton.clicked.connect(self.getRegulatorParametersClickedHandler)

		rosBridge.regulatorSettingsSetDone.connect(self.regulatorSettingsSetDoneHandler)
		rosBridge.regulatorSettingsGetDone.connect(self.regulatorSettingsGetDoneHandler)
		self.regulatorSettingsSetRequested.connect(rosBridge.regulatorSettingsSetRequested)
		self.regulatorSettingsGetRequested.connect(rosBridge.regulatorSettingsGetRequested)
		self.rosBridge = rosBridge

		self.adjustSize()

	''' UI event handlers '''

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
			'lqrLinearVelocityK': self.ui.lqrLinearVelocityKSpinBox.value(),
			'lqrAngularVelocityK': self.ui.lqrAngularVelocityKSpinBox.value(),
			'lqrAngleK': self.ui.lqrAngleKSpinBox.value(),
		}

		self.regulatorSettingsSetRequested.emit(parameters)

	def getRegulatorParametersClickedHandler(self):
		self.regulatorSettingsGetRequested.emit()

	''' ROS event handlers '''

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
