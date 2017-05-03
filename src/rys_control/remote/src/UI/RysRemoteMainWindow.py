from PyQt5 import QtWidgets
from UI import PyGameSurfaceImageWidget

class RysRemoteMainWindow(QtWidgets.QMainWindow):
	"""docstring for RysRemoteMainWindow"""
	def __init__(self, pyGameSurface, parent = None):
		super(RysRemoteMainWindow, self).__init__(parent)

		self.pyGameSurface = pyGameSurface

		# self.ui.layout.removeWidget(self.ui.imageWidget)
		# self.ui.imageWidget.close()
		# self.ui.imageWidget = PyGameSurfaceImageWidget(self.pyGameSurface)
		# self.ui.layout.addWidget(self.ui.imageWidget)
		# self.ui.layout.update()

		self.setCentralWidget(PyGameSurfaceImageWidget(self.pyGameSurface))
		self.adjustSize()
