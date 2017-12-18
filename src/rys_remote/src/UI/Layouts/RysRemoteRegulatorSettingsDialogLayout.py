# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'RysRemoteRegulatorSettingsDialogLayout.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_RegulatorSettingsDialog(object):
    def setupUi(self, RegulatorSettingsDialog):
        RegulatorSettingsDialog.setObjectName("RegulatorSettingsDialog")
        RegulatorSettingsDialog.resize(712, 320)
        self.verticalLayout = QtWidgets.QVBoxLayout(RegulatorSettingsDialog)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.pidGroupBox = QtWidgets.QGroupBox(RegulatorSettingsDialog)
        self.pidGroupBox.setObjectName("pidGroupBox")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.pidGroupBox)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.pidStage1HorizontalLayout = QtWidgets.QHBoxLayout()
        self.pidStage1HorizontalLayout.setObjectName("pidStage1HorizontalLayout")
        self.pidSpeedStageLabel = QtWidgets.QLabel(self.pidGroupBox)
        self.pidSpeedStageLabel.setMinimumSize(QtCore.QSize(120, 40))
        self.pidSpeedStageLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.pidSpeedStageLabel.setObjectName("pidSpeedStageLabel")
        self.pidStage1HorizontalLayout.addWidget(self.pidSpeedStageLabel)
        self.pidSpeedVerticalLayout = QtWidgets.QVBoxLayout()
        self.pidSpeedVerticalLayout.setObjectName("pidSpeedVerticalLayout")
        self.pidSpeedStageEnabledCheckBox = QtWidgets.QCheckBox(self.pidGroupBox)
        self.pidSpeedStageEnabledCheckBox.setChecked(True)
        self.pidSpeedStageEnabledCheckBox.setObjectName("pidSpeedStageEnabledCheckBox")
        self.pidSpeedVerticalLayout.addWidget(self.pidSpeedStageEnabledCheckBox)
        self.pidSpeedKpSpinBox = QtWidgets.QDoubleSpinBox(self.pidGroupBox)
        self.pidSpeedKpSpinBox.setMinimumSize(QtCore.QSize(80, 0))
        self.pidSpeedKpSpinBox.setDecimals(4)
        self.pidSpeedKpSpinBox.setMaximum(100.0)
        self.pidSpeedKpSpinBox.setSingleStep(0.0001)
        self.pidSpeedKpSpinBox.setProperty("value", 1.0)
        self.pidSpeedKpSpinBox.setObjectName("pidSpeedKpSpinBox")
        self.pidSpeedVerticalLayout.addWidget(self.pidSpeedKpSpinBox)
        self.pidSpeedKiSpinBox = QtWidgets.QDoubleSpinBox(self.pidGroupBox)
        self.pidSpeedKiSpinBox.setMinimumSize(QtCore.QSize(80, 0))
        self.pidSpeedKiSpinBox.setDecimals(4)
        self.pidSpeedKiSpinBox.setMaximum(100.0)
        self.pidSpeedKiSpinBox.setSingleStep(0.0001)
        self.pidSpeedKiSpinBox.setProperty("value", 0.0)
        self.pidSpeedKiSpinBox.setObjectName("pidSpeedKiSpinBox")
        self.pidSpeedVerticalLayout.addWidget(self.pidSpeedKiSpinBox)
        self.pidSpeedKdSpinBox = QtWidgets.QDoubleSpinBox(self.pidGroupBox)
        self.pidSpeedKdSpinBox.setMinimumSize(QtCore.QSize(80, 0))
        self.pidSpeedKdSpinBox.setDecimals(4)
        self.pidSpeedKdSpinBox.setMaximum(1000.0)
        self.pidSpeedKdSpinBox.setSingleStep(0.0)
        self.pidSpeedKdSpinBox.setProperty("value", 0.008)
        self.pidSpeedKdSpinBox.setObjectName("pidSpeedKdSpinBox")
        self.pidSpeedVerticalLayout.addWidget(self.pidSpeedKdSpinBox)
        self.pidStage1HorizontalLayout.addLayout(self.pidSpeedVerticalLayout)
        self.verticalLayout_5.addLayout(self.pidStage1HorizontalLayout)
        self.line = QtWidgets.QFrame(self.pidGroupBox)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout_5.addWidget(self.line)
        self.pidStage2HorizontalLayout = QtWidgets.QHBoxLayout()
        self.pidStage2HorizontalLayout.setObjectName("pidStage2HorizontalLayout")
        self.pidAngleStageLabel = QtWidgets.QLabel(self.pidGroupBox)
        self.pidAngleStageLabel.setMinimumSize(QtCore.QSize(120, 40))
        self.pidAngleStageLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.pidAngleStageLabel.setObjectName("pidAngleStageLabel")
        self.pidStage2HorizontalLayout.addWidget(self.pidAngleStageLabel)
        self.pidAngleVerticalLayout = QtWidgets.QVBoxLayout()
        self.pidAngleVerticalLayout.setObjectName("pidAngleVerticalLayout")
        self.pidAngleKpSpinBox = QtWidgets.QDoubleSpinBox(self.pidGroupBox)
        self.pidAngleKpSpinBox.setMinimumSize(QtCore.QSize(80, 0))
        self.pidAngleKpSpinBox.setDecimals(4)
        self.pidAngleKpSpinBox.setMaximum(1000.0)
        self.pidAngleKpSpinBox.setProperty("value", 0.009)
        self.pidAngleKpSpinBox.setObjectName("pidAngleKpSpinBox")
        self.pidAngleVerticalLayout.addWidget(self.pidAngleKpSpinBox)
        self.pidAngleKiSpinBox = QtWidgets.QDoubleSpinBox(self.pidGroupBox)
        self.pidAngleKiSpinBox.setMinimumSize(QtCore.QSize(80, 0))
        self.pidAngleKiSpinBox.setDecimals(4)
        self.pidAngleKiSpinBox.setMaximum(1000.0)
        self.pidAngleKiSpinBox.setProperty("value", 0.004)
        self.pidAngleKiSpinBox.setObjectName("pidAngleKiSpinBox")
        self.pidAngleVerticalLayout.addWidget(self.pidAngleKiSpinBox)
        self.pidAngleKdSpinBox = QtWidgets.QDoubleSpinBox(self.pidGroupBox)
        self.pidAngleKdSpinBox.setMinimumSize(QtCore.QSize(80, 0))
        self.pidAngleKdSpinBox.setDecimals(4)
        self.pidAngleKdSpinBox.setMaximum(1000.0)
        self.pidAngleKdSpinBox.setProperty("value", 0.001)
        self.pidAngleKdSpinBox.setObjectName("pidAngleKdSpinBox")
        self.pidAngleVerticalLayout.addWidget(self.pidAngleKdSpinBox)
        self.pidStage2HorizontalLayout.addLayout(self.pidAngleVerticalLayout)
        self.verticalLayout_5.addLayout(self.pidStage2HorizontalLayout)
        self.horizontalLayout_2.addWidget(self.pidGroupBox)
        self.lqrGroupBox = QtWidgets.QGroupBox(RegulatorSettingsDialog)
        self.lqrGroupBox.setObjectName("lqrGroupBox")
        self.verticalLayout_8 = QtWidgets.QVBoxLayout(self.lqrGroupBox)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.lqrEnabledCheckBox = QtWidgets.QCheckBox(self.lqrGroupBox)
        self.lqrEnabledCheckBox.setChecked(True)
        self.lqrEnabledCheckBox.setObjectName("lqrEnabledCheckBox")
        self.verticalLayout_8.addWidget(self.lqrEnabledCheckBox)
        self.lqrLinearVelocityKHorizontalLayout = QtWidgets.QHBoxLayout()
        self.lqrLinearVelocityKHorizontalLayout.setObjectName("lqrLinearVelocityKHorizontalLayout")
        self.lqrLinearVelocityKLabel = QtWidgets.QLabel(self.lqrGroupBox)
        self.lqrLinearVelocityKLabel.setMinimumSize(QtCore.QSize(100, 20))
        self.lqrLinearVelocityKLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.lqrLinearVelocityKLabel.setObjectName("lqrLinearVelocityKLabel")
        self.lqrLinearVelocityKHorizontalLayout.addWidget(self.lqrLinearVelocityKLabel)
        self.lqrLinearVelocityKSpinBox = QtWidgets.QDoubleSpinBox(self.lqrGroupBox)
        self.lqrLinearVelocityKSpinBox.setMinimumSize(QtCore.QSize(90, 0))
        self.lqrLinearVelocityKSpinBox.setDecimals(4)
        self.lqrLinearVelocityKSpinBox.setMinimum(-9999.0)
        self.lqrLinearVelocityKSpinBox.setMaximum(9999.0)
        self.lqrLinearVelocityKSpinBox.setSingleStep(0.001)
        self.lqrLinearVelocityKSpinBox.setProperty("value", -0.0316)
        self.lqrLinearVelocityKSpinBox.setObjectName("lqrLinearVelocityKSpinBox")
        self.lqrLinearVelocityKHorizontalLayout.addWidget(self.lqrLinearVelocityKSpinBox)
        self.verticalLayout_8.addLayout(self.lqrLinearVelocityKHorizontalLayout)
        self.line_9 = QtWidgets.QFrame(self.lqrGroupBox)
        self.line_9.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_9.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_9.setObjectName("line_9")
        self.verticalLayout_8.addWidget(self.line_9)
        self.lqrAngularVelocityKHorizontalLayout = QtWidgets.QHBoxLayout()
        self.lqrAngularVelocityKHorizontalLayout.setObjectName("lqrAngularVelocityKHorizontalLayout")
        self.lqrAngularVelocityKLabel = QtWidgets.QLabel(self.lqrGroupBox)
        self.lqrAngularVelocityKLabel.setMinimumSize(QtCore.QSize(100, 20))
        self.lqrAngularVelocityKLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.lqrAngularVelocityKLabel.setObjectName("lqrAngularVelocityKLabel")
        self.lqrAngularVelocityKHorizontalLayout.addWidget(self.lqrAngularVelocityKLabel)
        self.lqrAngularVelocityKSpinBox = QtWidgets.QDoubleSpinBox(self.lqrGroupBox)
        self.lqrAngularVelocityKSpinBox.setMinimumSize(QtCore.QSize(90, 0))
        self.lqrAngularVelocityKSpinBox.setDecimals(4)
        self.lqrAngularVelocityKSpinBox.setMinimum(-9999.0)
        self.lqrAngularVelocityKSpinBox.setMaximum(9999.0)
        self.lqrAngularVelocityKSpinBox.setSingleStep(0.001)
        self.lqrAngularVelocityKSpinBox.setProperty("value", -42.3121)
        self.lqrAngularVelocityKSpinBox.setObjectName("lqrAngularVelocityKSpinBox")
        self.lqrAngularVelocityKHorizontalLayout.addWidget(self.lqrAngularVelocityKSpinBox)
        self.verticalLayout_8.addLayout(self.lqrAngularVelocityKHorizontalLayout)
        self.line_3 = QtWidgets.QFrame(self.lqrGroupBox)
        self.line_3.setFrameShape(QtWidgets.QFrame.HLine)
        self.line_3.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line_3.setObjectName("line_3")
        self.verticalLayout_8.addWidget(self.line_3)
        self.lqrAngleKHorizontalLayout = QtWidgets.QHBoxLayout()
        self.lqrAngleKHorizontalLayout.setObjectName("lqrAngleKHorizontalLayout")
        self.lqrAngleKLabel = QtWidgets.QLabel(self.lqrGroupBox)
        self.lqrAngleKLabel.setMinimumSize(QtCore.QSize(100, 20))
        self.lqrAngleKLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.lqrAngleKLabel.setObjectName("lqrAngleKLabel")
        self.lqrAngleKHorizontalLayout.addWidget(self.lqrAngleKLabel)
        self.lqrAngleKSpinBox = QtWidgets.QDoubleSpinBox(self.lqrGroupBox)
        self.lqrAngleKSpinBox.setMinimumSize(QtCore.QSize(90, 0))
        self.lqrAngleKSpinBox.setDecimals(4)
        self.lqrAngleKSpinBox.setMinimum(-9999.0)
        self.lqrAngleKSpinBox.setMaximum(9999.0)
        self.lqrAngleKSpinBox.setSingleStep(0.001)
        self.lqrAngleKSpinBox.setProperty("value", -392.3354)
        self.lqrAngleKSpinBox.setObjectName("lqrAngleKSpinBox")
        self.lqrAngleKHorizontalLayout.addWidget(self.lqrAngleKSpinBox)
        self.verticalLayout_8.addLayout(self.lqrAngleKHorizontalLayout)
        self.horizontalLayout_2.addWidget(self.lqrGroupBox)
        self.regulatorControlGroupBox = QtWidgets.QGroupBox(RegulatorSettingsDialog)
        self.regulatorControlGroupBox.setObjectName("regulatorControlGroupBox")
        self.verticalLayout_9 = QtWidgets.QVBoxLayout(self.regulatorControlGroupBox)
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.speedFilteringHorizontalLayout = QtWidgets.QHBoxLayout()
        self.speedFilteringHorizontalLayout.setObjectName("speedFilteringHorizontalLayout")
        self.speedFilteringLabel = QtWidgets.QLabel(self.regulatorControlGroupBox)
        self.speedFilteringLabel.setMinimumSize(QtCore.QSize(100, 20))
        self.speedFilteringLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.speedFilteringLabel.setObjectName("speedFilteringLabel")
        self.speedFilteringHorizontalLayout.addWidget(self.speedFilteringLabel)
        self.speedFilteringSpinBox = QtWidgets.QDoubleSpinBox(self.regulatorControlGroupBox)
        self.speedFilteringSpinBox.setMinimumSize(QtCore.QSize(70, 0))
        self.speedFilteringSpinBox.setDecimals(2)
        self.speedFilteringSpinBox.setMaximum(1.0)
        self.speedFilteringSpinBox.setSingleStep(0.01)
        self.speedFilteringSpinBox.setProperty("value", 1.0)
        self.speedFilteringSpinBox.setObjectName("speedFilteringSpinBox")
        self.speedFilteringHorizontalLayout.addWidget(self.speedFilteringSpinBox)
        self.verticalLayout_9.addLayout(self.speedFilteringHorizontalLayout)
        self.rollFilteringHorizontalLayout = QtWidgets.QHBoxLayout()
        self.rollFilteringHorizontalLayout.setObjectName("rollFilteringHorizontalLayout")
        self.rollFilteringLabel = QtWidgets.QLabel(self.regulatorControlGroupBox)
        self.rollFilteringLabel.setMinimumSize(QtCore.QSize(100, 20))
        self.rollFilteringLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.rollFilteringLabel.setObjectName("rollFilteringLabel")
        self.rollFilteringHorizontalLayout.addWidget(self.rollFilteringLabel)
        self.rollFilteringSpinBox = QtWidgets.QDoubleSpinBox(self.regulatorControlGroupBox)
        self.rollFilteringSpinBox.setMinimumSize(QtCore.QSize(70, 0))
        self.rollFilteringSpinBox.setDecimals(2)
        self.rollFilteringSpinBox.setMaximum(1.0)
        self.rollFilteringSpinBox.setSingleStep(0.01)
        self.rollFilteringSpinBox.setProperty("value", 1.0)
        self.rollFilteringSpinBox.setObjectName("rollFilteringSpinBox")
        self.rollFilteringHorizontalLayout.addWidget(self.rollFilteringSpinBox)
        self.verticalLayout_9.addLayout(self.rollFilteringHorizontalLayout)
        self.horizontalLayout_2.addWidget(self.regulatorControlGroupBox)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.getRegulatorParametersButton = QtWidgets.QPushButton(RegulatorSettingsDialog)
        self.getRegulatorParametersButton.setObjectName("getRegulatorParametersButton")
        self.horizontalLayout.addWidget(self.getRegulatorParametersButton)
        self.setRegulatorParametersButton = QtWidgets.QPushButton(RegulatorSettingsDialog)
        self.setRegulatorParametersButton.setObjectName("setRegulatorParametersButton")
        self.horizontalLayout.addWidget(self.setRegulatorParametersButton)
        self.closeButton = QtWidgets.QPushButton(RegulatorSettingsDialog)
        self.closeButton.setObjectName("closeButton")
        self.horizontalLayout.addWidget(self.closeButton)
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.retranslateUi(RegulatorSettingsDialog)
        self.closeButton.released.connect(RegulatorSettingsDialog.accept)
        QtCore.QMetaObject.connectSlotsByName(RegulatorSettingsDialog)

    def retranslateUi(self, RegulatorSettingsDialog):
        _translate = QtCore.QCoreApplication.translate
        RegulatorSettingsDialog.setWindowTitle(_translate("RegulatorSettingsDialog", "RysRemote - Regulator Settings"))
        self.pidGroupBox.setTitle(_translate("RegulatorSettingsDialog", "PID"))
        self.pidSpeedStageLabel.setText(_translate("RegulatorSettingsDialog", "1st stage\n"
"(speed -> angle)"))
        self.pidSpeedStageEnabledCheckBox.setText(_translate("RegulatorSettingsDialog", "Enabled"))
        self.pidAngleStageLabel.setText(_translate("RegulatorSettingsDialog", "2nd stage\n"
"(angle -> output)"))
        self.lqrGroupBox.setTitle(_translate("RegulatorSettingsDialog", "LQR"))
        self.lqrEnabledCheckBox.setText(_translate("RegulatorSettingsDialog", "Enabled"))
        self.lqrLinearVelocityKLabel.setText(_translate("RegulatorSettingsDialog", "Linear\n"
"velocity K"))
        self.lqrAngularVelocityKLabel.setText(_translate("RegulatorSettingsDialog", "Angular\n"
"velocity K"))
        self.lqrAngleKLabel.setText(_translate("RegulatorSettingsDialog", "Angle K"))
        self.regulatorControlGroupBox.setTitle(_translate("RegulatorSettingsDialog", "Filtering"))
        self.speedFilteringLabel.setText(_translate("RegulatorSettingsDialog", "Speed filtering"))
        self.rollFilteringLabel.setText(_translate("RegulatorSettingsDialog", "Roll filtering"))
        self.getRegulatorParametersButton.setText(_translate("RegulatorSettingsDialog", "Get regulator parameters"))
        self.setRegulatorParametersButton.setText(_translate("RegulatorSettingsDialog", "Set regulator parameters"))
        self.closeButton.setText(_translate("RegulatorSettingsDialog", "Close"))

