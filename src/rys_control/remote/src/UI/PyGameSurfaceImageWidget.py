from PyQt5 import QtGui, QtWidgets

class PyGameSurfaceImageWidget(QtWidgets.QWidget):
	"""docstring for PyGameSurfaceImageWidget"""
	def __init__(self, pyGameSurface, parent = None):
		super(PyGameSurfaceImageWidget, self).__init__(parent)
		self.pyGameSurface = pyGameSurface

		width = pyGameSurface.get_width()
		height = pyGameSurface.get_height()
		self.data = pyGameSurface.get_buffer().raw
		self.image = QtGui.QImage(self.data, width, height, QtGui.QImage.Format_RGB32)

		self.setMinimumSize(width, height)

	def paintEvent(self, event):
		qPainter = QtGui.QPainter()
		qPainter.begin(self)
		qPainter.drawImage(0, 0, self.image)
		qPainter.end()
