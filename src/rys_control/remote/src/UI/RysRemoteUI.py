from PyQt5 import QtWidgets
from src.UI.RysRemoteMainWindow import RysRemoteMainWindow

class RysRemoteUI(object):
	"""docstring for RysRemoteUI"""

	def __init__(self):
		super(RysRemoteUI, self).__init__()

	def exec_(self, args = None):
		app = QtWidgets.QApplication(args)
		window = RysRemoteMainWindow()
		app.aboutToQuit.connect(window.quitEventHandler)
		window.show()
		return app.exec_()
