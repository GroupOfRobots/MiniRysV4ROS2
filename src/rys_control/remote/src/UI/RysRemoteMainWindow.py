from PyQt5 import QtWidgets, QtCore
from src.Gamepad import GamepadBridge
from src.UI.Layouts import Ui_RysRemoteMainWindow

class RysRemoteMainWindow(QtWidgets.QMainWindow):
	"""docstring for RysRemoteMainWindow"""

	def __init__(self, parent = None):
		super(RysRemoteMainWindow, self).__init__(parent)

		self.ui = Ui_RysRemoteMainWindow()
		self.ui.setupUi(self)

		self.gamepadBridge = GamepadBridge(self)
		self.gamepadBridge.gamepadAxisChanged.connect(self.gamepadAxisChangedHandler)
		self.gamepadBridge.gamepadButtonChanged.connect(self.gamepadButtonChangedHandler)
		self.gamepadBridge.start()

		self.adjustSize()

	def gamepadAxisChangedHandler(self, gamepadAxisEvent):
		gamepadID = gamepadAxisEvent.gamepadID
		axis = gamepadAxisEvent.axis
		value = gamepadAxisEvent.value
		print("axis event: joy %d, axis %d, value %f" % (gamepadID, axis, value))

	def gamepadButtonChangedHandler(self, gamepadButtonEvent):
		gamepadID = gamepadButtonEvent.gamepadID
		button = gamepadButtonEvent.button
		value = gamepadButtonEvent.value
		print("butt event: joy %d, butt %d, value %f" % (gamepadID, button, value))

	def quitEventHandler(self):
		loop = QtCore.QEventLoop()
		self.gamepadBridge.finished.connect(loop.quit)
		self.gamepadBridge.stopExecution()
		loop.exec_()
