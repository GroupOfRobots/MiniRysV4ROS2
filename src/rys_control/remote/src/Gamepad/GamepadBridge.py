import pygame
from PyQt5.QtCore import QThread, pyqtSignal
from Gamepad.GamepadAxisEvent import GamepadAxisEvent
from Gamepad.GamepadButtonEvent import GamepadButtonEvent

class GamepadBridge(QThread):
	"""
	Gamepad (Joystick) bridge between pygame and PyQt5.
	Passes gamepad list, axis and button events to PyQt as signals.
	Inherits from QThread and shall be used as one.
	"""

	gamepadAxisChanged = pyqtSignal(GamepadAxisEvent)
	gamepadButtonChanged = pyqtSignal(GamepadButtonEvent)
	gamepadListUpdated = pyqtSignal(list)

	def __init__(self, parent = None):
		super(GamepadBridge, self).__init__(parent)
		self.exitFlag = False

		self.axisValues = dict()
		self.buttonValues = dict()

	def run(self):
		pygame.init()
		pygame.joystick.init()

		joysticks = pygame.joystick.get_count()
		print("%s joystick(s) detected!" % str(joysticks))

		joystickList = list()
		for i in range(joysticks):
			joystick = pygame.joystick.Joystick(i)
			joystick.init()
			joystickName = joystick.get_name()
			joystickList.append(joystickName)
			print("Joystick %s name: %s" % (str(i), joystickName))
		self.gamepadListUpdated.emit(joystickList)

		while not self.exitFlag:
			pygame.time.Clock().tick(100)

			# terminate if any QUIT events are present
			for event in pygame.event.get(pygame.QUIT):
				self.exitFlag = True
				break
			# terminate if the KEYUP event was for the Esc key
			for event in pygame.event.get(pygame.KEYUP):
				if event.key == pygame.K_ESCAPE:
					self.exitFlag = True
					break

			if self.exitFlag:
				break

			# pygame.JOYBALLMOTION
			# pygame.JOYHATMOTION
			for event in pygame.event.get(pygame.JOYAXISMOTION):
				axisID = "%d_%d" % (event.joy, event.axis)
				if axisID in self.axisValues and self.axisValues[axisID] == event.value:
					continue
				self.axisValues[axisID] = event.value

				gamepadEvent = GamepadAxisEvent(event.joy, event.axis, event.value)
				self.gamepadAxisChanged.emit(gamepadEvent)

			for event in pygame.event.get((pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP)):
				buttonDown = True if event.type == pygame.JOYBUTTONDOWN else False

				buttonID = "%d_%d" % (event.joy, event.button)
				if buttonID in self.buttonValues and self.buttonValues[buttonID] == buttonDown:
					return
				self.buttonValues[buttonID] = buttonDown

				gamepadEvent = GamepadButtonEvent(event.joy, event.button, buttonDown)
				self.gamepadButtonChanged.emit(gamepadEvent)

		pygame.quit()

	def stopExecution(self):
		pygame.event.post(pygame.event.Event(pygame.QUIT))
		self.exitFlag = True
