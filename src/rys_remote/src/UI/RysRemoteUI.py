from PyQt5.QtWidgets import QApplication
from UI.RysRemoteMainWindow import RysRemoteMainWindow

class RysRemoteUI(object):
	"""docstring for RysRemoteUI"""

	def __init__(self, nodeName):
		super(RysRemoteUI, self).__init__()
		self.nodeName = nodeName

	def exec_(self, args = None):
		app = QApplication(args)
		window = RysRemoteMainWindow(self.nodeName)
		app.aboutToQuit.connect(window.quitEventHandler)
		window.show()
		return app.exec_()
