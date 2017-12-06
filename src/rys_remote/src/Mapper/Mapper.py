import QuadMapNode
from PyQt5.QtCore import QThread, QTimer, pyqtSignal
# from PyKDL import Frame

class Mapper(QThread):
	"""docstring for Mapper"""

	mapGenerated = pyqtSignal(list, int)

	def __init__(self, parent = None, cellSize = 0.04, mapSize = 2.56):
		super().__init__(parent)

		self.map = QuadMapNode(mapSize, cellSize)

		self.timer = QTimer(self)
		self.timer.timeout.connect(self.timerCallback)
		self.timer.start(200)

		# self.robotPosition = Frame()

	def odometryHandler(self, message):
		# Apply odometry transform onto robotPosition frame
		pass

	def rangeReadingsHandler(self, front, back, top, left, right):
		# Apply sensors' transforms to the current odometry position
		# Calculate linear functions' parameters
		# Pass rangings to the quadmap
		pass

	def timerCallback(self):
		# Build new map and fire the signal
		pass

	def run(self):
		# Empty?
		pass
