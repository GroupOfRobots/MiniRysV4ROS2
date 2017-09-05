class GamepadButtonEvent(object):
    """docstring for GamepadButtonEvent"""

    def __init__(self, gamepadID, button, value):
        super(GamepadButtonEvent, self).__init__()
        self.gamepadID = gamepadID
        self.button = button
        self.value = value
