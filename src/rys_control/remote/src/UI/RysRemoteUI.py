import pygame
from PyQt5 import QtWidgets
from UI import RysRemoteMainWindow

class RysRemoteUI(object):
	"""docstring for RysRemoteUI"""
	def __init__(self):
		super(RysRemoteUI, self).__init__()

	def exec_(self, args = None):
		pygame.init()
		surface = pygame.Surface((640, 480))
		surface.fill((64, 128, 192, 224))
		pygame.draw.circle(surface, (255, 255, 255, 255), (100, 100), 50)

		app = QtWidgets.QApplication(args)
		window = RysRemoteMainWindow(surface)
		window.show()
		return app.exec_()
