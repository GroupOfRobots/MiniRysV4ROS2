# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'RysRemoteMainWindowLayout.ui'
#
# Created by: PyQt5 UI code generator 5.8.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_RysRemoteMainWindow(object):
    def setupUi(self, RysRemoteMainWindow):
        RysRemoteMainWindow.setObjectName("RysRemoteMainWindow")
        RysRemoteMainWindow.resize(690, 499)
        self.centralwidget = QtWidgets.QWidget(RysRemoteMainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.receivingGroupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.receivingGroupBox.setObjectName("receivingGroupBox")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.receivingGroupBox)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.sonarVerticalLayout = QtWidgets.QVBoxLayout()
        self.sonarVerticalLayout.setObjectName("sonarVerticalLayout")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.sonarFrontLabel = QtWidgets.QLabel(self.receivingGroupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sonarFrontLabel.sizePolicy().hasHeightForWidth())
        self.sonarFrontLabel.setSizePolicy(sizePolicy)
        self.sonarFrontLabel.setMinimumSize(QtCore.QSize(50, 20))
        self.sonarFrontLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.sonarFrontLabel.setObjectName("sonarFrontLabel")
        self.horizontalLayout_8.addWidget(self.sonarFrontLabel)
        self.sonarFrontBar = QtWidgets.QProgressBar(self.receivingGroupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sonarFrontBar.sizePolicy().hasHeightForWidth())
        self.sonarFrontBar.setSizePolicy(sizePolicy)
        self.sonarFrontBar.setMinimumSize(QtCore.QSize(100, 0))
        self.sonarFrontBar.setMaximum(800)
        self.sonarFrontBar.setProperty("value", 24)
        self.sonarFrontBar.setInvertedAppearance(False)
        self.sonarFrontBar.setObjectName("sonarFrontBar")
        self.horizontalLayout_8.addWidget(self.sonarFrontBar)
        self.sonarVerticalLayout.addLayout(self.horizontalLayout_8)
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.sonarBackLabel = QtWidgets.QLabel(self.receivingGroupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sonarBackLabel.sizePolicy().hasHeightForWidth())
        self.sonarBackLabel.setSizePolicy(sizePolicy)
        self.sonarBackLabel.setMinimumSize(QtCore.QSize(50, 20))
        self.sonarBackLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.sonarBackLabel.setObjectName("sonarBackLabel")
        self.horizontalLayout_11.addWidget(self.sonarBackLabel)
        self.sonarBackBar = QtWidgets.QProgressBar(self.receivingGroupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sonarBackBar.sizePolicy().hasHeightForWidth())
        self.sonarBackBar.setSizePolicy(sizePolicy)
        self.sonarBackBar.setMinimumSize(QtCore.QSize(100, 0))
        self.sonarBackBar.setMaximum(800)
        self.sonarBackBar.setProperty("value", 24)
        self.sonarBackBar.setInvertedAppearance(False)
        self.sonarBackBar.setObjectName("sonarBackBar")
        self.horizontalLayout_11.addWidget(self.sonarBackBar)
        self.sonarVerticalLayout.addLayout(self.horizontalLayout_11)
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.sonarTopLabel = QtWidgets.QLabel(self.receivingGroupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sonarTopLabel.sizePolicy().hasHeightForWidth())
        self.sonarTopLabel.setSizePolicy(sizePolicy)
        self.sonarTopLabel.setMinimumSize(QtCore.QSize(50, 20))
        self.sonarTopLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.sonarTopLabel.setObjectName("sonarTopLabel")
        self.horizontalLayout_9.addWidget(self.sonarTopLabel)
        self.sonarTopBar = QtWidgets.QProgressBar(self.receivingGroupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sonarTopBar.sizePolicy().hasHeightForWidth())
        self.sonarTopBar.setSizePolicy(sizePolicy)
        self.sonarTopBar.setMinimumSize(QtCore.QSize(100, 0))
        self.sonarTopBar.setMaximum(800)
        self.sonarTopBar.setProperty("value", 24)
        self.sonarTopBar.setInvertedAppearance(False)
        self.sonarTopBar.setObjectName("sonarTopBar")
        self.horizontalLayout_9.addWidget(self.sonarTopBar)
        self.sonarVerticalLayout.addLayout(self.horizontalLayout_9)
        self.horizontalLayout.addLayout(self.sonarVerticalLayout)
        self.imuVerticalLayout = QtWidgets.QVBoxLayout()
        self.imuVerticalLayout.setObjectName("imuVerticalLayout")
        self.rollDial = QtWidgets.QDial(self.receivingGroupBox)
        self.rollDial.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.rollDial.sizePolicy().hasHeightForWidth())
        self.rollDial.setSizePolicy(sizePolicy)
        self.rollDial.setMinimumSize(QtCore.QSize(100, 100))
        self.rollDial.setMinimum(-150)
        self.rollDial.setMaximum(150)
        self.rollDial.setProperty("value", -90)
        self.rollDial.setSliderPosition(-90)
        self.rollDial.setTracking(True)
        self.rollDial.setOrientation(QtCore.Qt.Horizontal)
        self.rollDial.setInvertedAppearance(False)
        self.rollDial.setInvertedControls(False)
        self.rollDial.setWrapping(False)
        self.rollDial.setNotchTarget(6.0)
        self.rollDial.setNotchesVisible(True)
        self.rollDial.setObjectName("rollDial")
        self.imuVerticalLayout.addWidget(self.rollDial)
        self.rollValueLabel = QtWidgets.QLabel(self.receivingGroupBox)
        self.rollValueLabel.setMinimumSize(QtCore.QSize(50, 0))
        self.rollValueLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.rollValueLabel.setObjectName("rollValueLabel")
        self.imuVerticalLayout.addWidget(self.rollValueLabel)
        self.horizontalLayout.addLayout(self.imuVerticalLayout)
        self.verticalLayout_2.addWidget(self.receivingGroupBox)
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setObjectName("groupBox")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.groupBox)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.speedPIDLabel = QtWidgets.QLabel(self.groupBox)
        self.speedPIDLabel.setMinimumSize(QtCore.QSize(120, 40))
        self.speedPIDLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.speedPIDLabel.setObjectName("speedPIDLabel")
        self.verticalLayout_3.addWidget(self.speedPIDLabel)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.speedPSpinBox = QtWidgets.QDoubleSpinBox(self.groupBox)
        self.speedPSpinBox.setMinimumSize(QtCore.QSize(70, 0))
        self.speedPSpinBox.setDecimals(4)
        self.speedPSpinBox.setMaximum(10.0)
        self.speedPSpinBox.setSingleStep(0.001)
        self.speedPSpinBox.setProperty("value", 0.03)
        self.speedPSpinBox.setObjectName("speedPSpinBox")
        self.horizontalLayout_5.addWidget(self.speedPSpinBox)
        self.speedISpinBox = QtWidgets.QDoubleSpinBox(self.groupBox)
        self.speedISpinBox.setMinimumSize(QtCore.QSize(70, 0))
        self.speedISpinBox.setDecimals(4)
        self.speedISpinBox.setMaximum(100.0)
        self.speedISpinBox.setSingleStep(0.0001)
        self.speedISpinBox.setProperty("value", 0.0001)
        self.speedISpinBox.setObjectName("speedISpinBox")
        self.horizontalLayout_5.addWidget(self.speedISpinBox)
        self.speedDSpinBox = QtWidgets.QDoubleSpinBox(self.groupBox)
        self.speedDSpinBox.setMinimumSize(QtCore.QSize(70, 0))
        self.speedDSpinBox.setDecimals(4)
        self.speedDSpinBox.setMaximum(100.0)
        self.speedDSpinBox.setSingleStep(0.001)
        self.speedDSpinBox.setProperty("value", 0.008)
        self.speedDSpinBox.setObjectName("speedDSpinBox")
        self.horizontalLayout_5.addWidget(self.speedDSpinBox)
        self.verticalLayout_3.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_6.addLayout(self.verticalLayout_3)
        self.verticalLayout_5 = QtWidgets.QVBoxLayout()
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.anglePIDLabel = QtWidgets.QLabel(self.groupBox)
        self.anglePIDLabel.setMinimumSize(QtCore.QSize(120, 40))
        self.anglePIDLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.anglePIDLabel.setObjectName("anglePIDLabel")
        self.verticalLayout_5.addWidget(self.anglePIDLabel)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.anglePSpinBox = QtWidgets.QDoubleSpinBox(self.groupBox)
        self.anglePSpinBox.setMinimumSize(QtCore.QSize(70, 0))
        self.anglePSpinBox.setDecimals(1)
        self.anglePSpinBox.setMaximum(1000.0)
        self.anglePSpinBox.setProperty("value", 50.0)
        self.anglePSpinBox.setObjectName("anglePSpinBox")
        self.horizontalLayout_2.addWidget(self.anglePSpinBox)
        self.angleISpinBox = QtWidgets.QDoubleSpinBox(self.groupBox)
        self.angleISpinBox.setMinimumSize(QtCore.QSize(70, 0))
        self.angleISpinBox.setDecimals(2)
        self.angleISpinBox.setProperty("value", 0.05)
        self.angleISpinBox.setObjectName("angleISpinBox")
        self.horizontalLayout_2.addWidget(self.angleISpinBox)
        self.angleDSpinBox = QtWidgets.QDoubleSpinBox(self.groupBox)
        self.angleDSpinBox.setMinimumSize(QtCore.QSize(70, 0))
        self.angleDSpinBox.setDecimals(1)
        self.angleDSpinBox.setMaximum(100.0)
        self.angleDSpinBox.setProperty("value", 20.0)
        self.angleDSpinBox.setObjectName("angleDSpinBox")
        self.horizontalLayout_2.addWidget(self.angleDSpinBox)
        self.verticalLayout_5.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_6.addLayout(self.verticalLayout_5)
        self.verticalLayout_4.addLayout(self.horizontalLayout_6)
        self.setPIDsButton = QtWidgets.QPushButton(self.groupBox)
        self.setPIDsButton.setObjectName("setPIDsButton")
        self.verticalLayout_4.addWidget(self.setPIDsButton)
        self.horizontalLayout_3.addLayout(self.verticalLayout_4)
        self.filteringVerticalLayout = QtWidgets.QVBoxLayout()
        self.filteringVerticalLayout.setObjectName("filteringVerticalLayout")
        self.imuCalibrateButton = QtWidgets.QPushButton(self.groupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.imuCalibrateButton.sizePolicy().hasHeightForWidth())
        self.imuCalibrateButton.setSizePolicy(sizePolicy)
        self.imuCalibrateButton.setBaseSize(QtCore.QSize(0, 0))
        self.imuCalibrateButton.setObjectName("imuCalibrateButton")
        self.filteringVerticalLayout.addWidget(self.imuCalibrateButton)
        self.speedFilteringHorizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.speedFilteringHorizontalLayout_2.setObjectName("speedFilteringHorizontalLayout_2")
        self.angularVelocityLabel = QtWidgets.QLabel(self.groupBox)
        self.angularVelocityLabel.setMinimumSize(QtCore.QSize(100, 20))
        self.angularVelocityLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.angularVelocityLabel.setObjectName("angularVelocityLabel")
        self.speedFilteringHorizontalLayout_2.addWidget(self.angularVelocityLabel)
        self.angularVelocitySpinBox = QtWidgets.QDoubleSpinBox(self.groupBox)
        self.angularVelocitySpinBox.setMinimumSize(QtCore.QSize(70, 0))
        self.angularVelocitySpinBox.setDecimals(3)
        self.angularVelocitySpinBox.setMaximum(1000.0)
        self.angularVelocitySpinBox.setSingleStep(0.01)
        self.angularVelocitySpinBox.setProperty("value", 0.009)
        self.angularVelocitySpinBox.setObjectName("angularVelocitySpinBox")
        self.speedFilteringHorizontalLayout_2.addWidget(self.angularVelocitySpinBox)
        self.filteringVerticalLayout.addLayout(self.speedFilteringHorizontalLayout_2)
        self.speedFilteringHorizontalLayout = QtWidgets.QHBoxLayout()
        self.speedFilteringHorizontalLayout.setObjectName("speedFilteringHorizontalLayout")
        self.speedFilteringLabel = QtWidgets.QLabel(self.groupBox)
        self.speedFilteringLabel.setMinimumSize(QtCore.QSize(100, 20))
        self.speedFilteringLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.speedFilteringLabel.setObjectName("speedFilteringLabel")
        self.speedFilteringHorizontalLayout.addWidget(self.speedFilteringLabel)
        self.speedFilteringSpinBox = QtWidgets.QDoubleSpinBox(self.groupBox)
        self.speedFilteringSpinBox.setMinimumSize(QtCore.QSize(70, 0))
        self.speedFilteringSpinBox.setDecimals(2)
        self.speedFilteringSpinBox.setMaximum(1.0)
        self.speedFilteringSpinBox.setSingleStep(0.01)
        self.speedFilteringSpinBox.setProperty("value", 0.98)
        self.speedFilteringSpinBox.setObjectName("speedFilteringSpinBox")
        self.speedFilteringHorizontalLayout.addWidget(self.speedFilteringSpinBox)
        self.filteringVerticalLayout.addLayout(self.speedFilteringHorizontalLayout)
        self.rollFilteringHorizontalLayout = QtWidgets.QHBoxLayout()
        self.rollFilteringHorizontalLayout.setObjectName("rollFilteringHorizontalLayout")
        self.rollFilteringLabel = QtWidgets.QLabel(self.groupBox)
        self.rollFilteringLabel.setMinimumSize(QtCore.QSize(100, 20))
        self.rollFilteringLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.rollFilteringLabel.setObjectName("rollFilteringLabel")
        self.rollFilteringHorizontalLayout.addWidget(self.rollFilteringLabel)
        self.rollFilteringSpinBox = QtWidgets.QDoubleSpinBox(self.groupBox)
        self.rollFilteringSpinBox.setMinimumSize(QtCore.QSize(70, 0))
        self.rollFilteringSpinBox.setDecimals(2)
        self.rollFilteringSpinBox.setMaximum(1.0)
        self.rollFilteringSpinBox.setSingleStep(0.01)
        self.rollFilteringSpinBox.setProperty("value", 0.98)
        self.rollFilteringSpinBox.setObjectName("rollFilteringSpinBox")
        self.rollFilteringHorizontalLayout.addWidget(self.rollFilteringSpinBox)
        self.filteringVerticalLayout.addLayout(self.rollFilteringHorizontalLayout)
        self.setFilteringParamsButton = QtWidgets.QPushButton(self.groupBox)
        self.setFilteringParamsButton.setObjectName("setFilteringParamsButton")
        self.filteringVerticalLayout.addWidget(self.setFilteringParamsButton)
        self.horizontalLayout_3.addLayout(self.filteringVerticalLayout)
        self.verticalLayout_2.addWidget(self.groupBox)
        self.steeringGroupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.steeringGroupBox.setObjectName("steeringGroupBox")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.steeringGroupBox)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.enableButton = QtWidgets.QPushButton(self.steeringGroupBox)
        self.enableButton.setMinimumSize(QtCore.QSize(100, 100))
        self.enableButton.setAutoFillBackground(False)
        self.enableButton.setStyleSheet("background-color: green;")
        self.enableButton.setFlat(False)
        self.enableButton.setObjectName("enableButton")
        self.horizontalLayout_4.addWidget(self.enableButton)
        self.steeringVerticalLayout = QtWidgets.QVBoxLayout()
        self.steeringVerticalLayout.setObjectName("steeringVerticalLayout")
        self.steeringGraphicsView = QtWidgets.QGraphicsView(self.steeringGroupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.steeringGraphicsView.sizePolicy().hasHeightForWidth())
        self.steeringGraphicsView.setSizePolicy(sizePolicy)
        self.steeringGraphicsView.setMinimumSize(QtCore.QSize(100, 100))
        self.steeringGraphicsView.setSizeIncrement(QtCore.QSize(20, 20))
        self.steeringGraphicsView.setBaseSize(QtCore.QSize(200, 200))
        self.steeringGraphicsView.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.steeringGraphicsView.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.steeringGraphicsView.setOptimizationFlags(QtWidgets.QGraphicsView.DontAdjustForAntialiasing|QtWidgets.QGraphicsView.DontSavePainterState)
        self.steeringGraphicsView.setObjectName("steeringGraphicsView")
        self.steeringVerticalLayout.addWidget(self.steeringGraphicsView)
        self.horizontalLayout_4.addLayout(self.steeringVerticalLayout)
        self.axisVerticalLayout = QtWidgets.QVBoxLayout()
        self.axisVerticalLayout.setObjectName("axisVerticalLayout")
        self.gamepadHorizontalLayout = QtWidgets.QHBoxLayout()
        self.gamepadHorizontalLayout.setObjectName("gamepadHorizontalLayout")
        self.gamepadComboBoxLabel = QtWidgets.QLabel(self.steeringGroupBox)
        self.gamepadComboBoxLabel.setMinimumSize(QtCore.QSize(60, 30))
        self.gamepadComboBoxLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.gamepadComboBoxLabel.setObjectName("gamepadComboBoxLabel")
        self.gamepadHorizontalLayout.addWidget(self.gamepadComboBoxLabel)
        self.gamepadComboBox = QtWidgets.QComboBox(self.steeringGroupBox)
        self.gamepadComboBox.setEnabled(False)
        self.gamepadComboBox.setMinimumSize(QtCore.QSize(70, 30))
        self.gamepadComboBox.setObjectName("gamepadComboBox")
        self.gamepadComboBox.addItem("")
        self.gamepadHorizontalLayout.addWidget(self.gamepadComboBox)
        self.axisVerticalLayout.addLayout(self.gamepadHorizontalLayout)
        self.throttleHorizontalLayout = QtWidgets.QHBoxLayout()
        self.throttleHorizontalLayout.setObjectName("throttleHorizontalLayout")
        self.throttleComboBoxLabel = QtWidgets.QLabel(self.steeringGroupBox)
        self.throttleComboBoxLabel.setMinimumSize(QtCore.QSize(60, 30))
        self.throttleComboBoxLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.throttleComboBoxLabel.setObjectName("throttleComboBoxLabel")
        self.throttleHorizontalLayout.addWidget(self.throttleComboBoxLabel)
        self.throttleComboBox = QtWidgets.QComboBox(self.steeringGroupBox)
        self.throttleComboBox.setEnabled(False)
        self.throttleComboBox.setMinimumSize(QtCore.QSize(70, 30))
        self.throttleComboBox.setObjectName("throttleComboBox")
        self.throttleComboBox.addItem("")
        self.throttleComboBox.setItemText(0, "[none]")
        self.throttleComboBox.addItem("")
        self.throttleComboBox.addItem("")
        self.throttleComboBox.addItem("")
        self.throttleComboBox.addItem("")
        self.throttleComboBox.addItem("")
        self.throttleHorizontalLayout.addWidget(self.throttleComboBox)
        self.axisVerticalLayout.addLayout(self.throttleHorizontalLayout)
        self.rotationHorizontalLayout = QtWidgets.QHBoxLayout()
        self.rotationHorizontalLayout.setObjectName("rotationHorizontalLayout")
        self.rotationComboBoxLabel = QtWidgets.QLabel(self.steeringGroupBox)
        self.rotationComboBoxLabel.setMinimumSize(QtCore.QSize(60, 30))
        self.rotationComboBoxLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.rotationComboBoxLabel.setObjectName("rotationComboBoxLabel")
        self.rotationHorizontalLayout.addWidget(self.rotationComboBoxLabel)
        self.rotationComboBox = QtWidgets.QComboBox(self.steeringGroupBox)
        self.rotationComboBox.setEnabled(False)
        self.rotationComboBox.setMinimumSize(QtCore.QSize(70, 30))
        self.rotationComboBox.setObjectName("rotationComboBox")
        self.rotationComboBox.addItem("")
        self.rotationComboBox.setItemText(0, "[none]")
        self.rotationComboBox.addItem("")
        self.rotationComboBox.addItem("")
        self.rotationComboBox.addItem("")
        self.rotationComboBox.addItem("")
        self.rotationComboBox.addItem("")
        self.rotationHorizontalLayout.addWidget(self.rotationComboBox)
        self.axisVerticalLayout.addLayout(self.rotationHorizontalLayout)
        self.horizontalLayout_4.addLayout(self.axisVerticalLayout)
        self.verticalLayout_2.addWidget(self.steeringGroupBox)
        RysRemoteMainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(RysRemoteMainWindow)
        QtCore.QMetaObject.connectSlotsByName(RysRemoteMainWindow)

    def retranslateUi(self, RysRemoteMainWindow):
        _translate = QtCore.QCoreApplication.translate
        RysRemoteMainWindow.setWindowTitle(_translate("RysRemoteMainWindow", "RysRemote"))
        self.receivingGroupBox.setTitle(_translate("RysRemoteMainWindow", "Receiving"))
        self.sonarFrontLabel.setText(_translate("RysRemoteMainWindow", "Front"))
        self.sonarFrontBar.setFormat(_translate("RysRemoteMainWindow", "%v"))
        self.sonarBackLabel.setText(_translate("RysRemoteMainWindow", "Back"))
        self.sonarBackBar.setFormat(_translate("RysRemoteMainWindow", "%v"))
        self.sonarTopLabel.setText(_translate("RysRemoteMainWindow", "Top"))
        self.sonarTopBar.setFormat(_translate("RysRemoteMainWindow", "%v"))
        self.rollValueLabel.setText(_translate("RysRemoteMainWindow", "-90.0"))
        self.groupBox.setTitle(_translate("RysRemoteMainWindow", "Control"))
        self.speedPIDLabel.setText(_translate("RysRemoteMainWindow", "Speed PID\n"
"(speed -> angle)"))
        self.anglePIDLabel.setText(_translate("RysRemoteMainWindow", "Angle PID\n"
"(angle -> output)"))
        self.setPIDsButton.setText(_translate("RysRemoteMainWindow", "Set PID parameters"))
        self.imuCalibrateButton.setText(_translate("RysRemoteMainWindow", "Calibrate IMU"))
        self.angularVelocityLabel.setText(_translate("RysRemoteMainWindow", "Angular vel fact"))
        self.speedFilteringLabel.setText(_translate("RysRemoteMainWindow", "Speed filtering"))
        self.rollFilteringLabel.setText(_translate("RysRemoteMainWindow", "Roll filtering"))
        self.setFilteringParamsButton.setText(_translate("RysRemoteMainWindow", "Set filtering parameters"))
        self.steeringGroupBox.setTitle(_translate("RysRemoteMainWindow", "Steering"))
        self.enableButton.setText(_translate("RysRemoteMainWindow", "Enable"))
        self.gamepadComboBoxLabel.setText(_translate("RysRemoteMainWindow", "Gamepad"))
        self.gamepadComboBox.setItemText(0, _translate("RysRemoteMainWindow", "[none]"))
        self.throttleComboBoxLabel.setText(_translate("RysRemoteMainWindow", "Throttle"))
        self.throttleComboBox.setItemText(1, _translate("RysRemoteMainWindow", "0"))
        self.throttleComboBox.setItemText(2, _translate("RysRemoteMainWindow", "1"))
        self.throttleComboBox.setItemText(3, _translate("RysRemoteMainWindow", "2"))
        self.throttleComboBox.setItemText(4, _translate("RysRemoteMainWindow", "3"))
        self.throttleComboBox.setItemText(5, _translate("RysRemoteMainWindow", "4"))
        self.rotationComboBoxLabel.setText(_translate("RysRemoteMainWindow", "Rotation"))
        self.rotationComboBox.setItemText(1, _translate("RysRemoteMainWindow", "0"))
        self.rotationComboBox.setItemText(2, _translate("RysRemoteMainWindow", "1"))
        self.rotationComboBox.setItemText(3, _translate("RysRemoteMainWindow", "2"))
        self.rotationComboBox.setItemText(4, _translate("RysRemoteMainWindow", "3"))
        self.rotationComboBox.setItemText(5, _translate("RysRemoteMainWindow", "4"))

