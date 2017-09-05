from PyQt5.QtWidgets import QApplication
from UI.RysRemoteMainWindow import RysRemoteMainWindow

class RysRemoteUI(object):
	"""docstring for RysRemoteUI"""

	def __init__(self, node):
		super(RysRemoteUI, self).__init__()
		self.node = node

	def exec_(self, args = None):
		app = QApplication(args)
		window = RysRemoteMainWindow(self.node)
		app.aboutToQuit.connect(window.quitEventHandler)
		window.show()
		return app.exec_()
