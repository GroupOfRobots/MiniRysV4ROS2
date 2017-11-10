from PyQt5.QtWidgets import QApplication
from UI.RysRemoteMainWindow import RysRemoteMainWindow

class RysRemoteUI(object):
	"""docstring for RysRemoteUI"""

	def __init__(self, robotName, nodeName):
		super(RysRemoteUI, self).__init__()
		self.robotName = robotName
		self.nodeName = nodeName

	def exec_(self, args = None):
		app = QApplication(args)
		window = RysRemoteMainWindow(None, self.robotName, self.nodeName)
		app.aboutToQuit.connect(window.quitEventHandler)
		window.show()
		return app.exec_()
