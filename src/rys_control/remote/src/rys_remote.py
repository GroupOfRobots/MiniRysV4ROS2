#!/usr/bin/env python3

import sys
from src.UI import RysRemoteUI

def main(args = None):
	if args is None:
		args = sys.argv

	ui = RysRemoteUI()
	sys.exit(ui.exec_(args))

if __name__ == '__main__':
	main()


# http://nullege.com/codes/show/src%40g%40a%40GameDevelopment-HEAD%40examples%40pygame%40basics%40pygame_test.py/27/pygame.joystick.init/python

# http://www.pygame.org/docs/ref/joystick.html

# Micromouse/BtSerial/tests/gamepadTest.py
# class CustomThread(threading.Thread); def run(self)

# TSPGen
