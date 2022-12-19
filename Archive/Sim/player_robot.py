from graphics import *

from robot import Robot
import keyboard


SPEED = 300.0
class PlayerRobot(Robot):
    def __init__(self, window: GraphWin, center: Point, width: int, height: int):
        super().__init__(window, center, width, height)

    def update(self):
        self.set_velocity(0, 0)
        if keyboard.is_pressed('w'):
            self.set_y_velocity(SPEED)
        if keyboard.is_pressed('s'):
            self.set_y_velocity(-SPEED)
        if keyboard.is_pressed('a'):
            self.set_x_velocity(-SPEED)
        if keyboard.is_pressed('d'):
            self.set_x_velocity(SPEED)
