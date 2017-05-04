class GamepadAxisEvent(object):
    """docstring for GamepadAxisEvent"""

    def __init__(self, gamepadID, axis, value):
        super(GamepadAxisEvent, self).__init__()
        self.gamepadID = gamepadID
        self.axis = axis
        self.value = value
