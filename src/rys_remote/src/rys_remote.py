#!/usr/bin/env python3

import sys
from PyQt5 import QtWidgets, QtCore
from Gamepad import GamepadBridge
from Mapper import Mapper
from ROS import QTRosBridge
from UI.RysRemoteMainWindow import RysRemoteMainWindow

gamepadBridge = None
rosBridge = None
mapper = None

def main(args = None):
	if args is None:
		args = sys.argv

	robotName = 'rys'
	nodeName = 'remote'

	app = QtWidgets.QApplication(args)

	global gamepadBridge
	global rosBridge
	global mapper
	gamepadBridge = GamepadBridge(app)
	rosBridge = QTRosBridge(app, robotName, nodeName)
	mapper = Mapper(app, rosBridge, cellSize = 0.04, mapSize = 2.56)

	mainWindow = RysRemoteMainWindow(app, gamepadBridge, rosBridge, mapper)
	app.aboutToQuit.connect(mainWindow.quitEventHandler)

	gamepadBridge.start()
	rosBridge.start()
	mapper.start()

	mainWindow.show()
	sys.exit(app.exec_())

def quitEventHandler():
	# "Join" all QThreads
	# Gamepad bridge
	if gamepadBridge.isRunning():
		loop = QtCore.QEventLoop()
		gamepadBridge.finished.connect(loop.quit)
		gamepadBridge.stopExecution()
		loop.exec_()

	# Ros bridge
	if rosBridge.isRunning():
		loop = QtCore.QEventLoop()
		rosBridge.finished.connect(loop.quit)
		rosBridge.stopExecution()
		loop.exec_()

	# Mapper
	if mapper.isRunning():
		loop = QtCore.QEventLoop()
		mapper.finished.connect(loop.quit)
		mapper.stopExecution()
		loop.exec_()

if __name__ == '__main__':
	main()
