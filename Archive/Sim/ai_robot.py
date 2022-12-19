from graphics import *

from player_robot import PlayerRobot
from robot import Robot

SPEED = 300.0
class AiRobot(Robot):
    def __init__(self, window: GraphWin, center: Point, width: int, height: int):
        super().__init__(window, center, width, height)

    def update(self, player_robot: PlayerRobot):
        self.set_velocity(-player_robot.velocity_x, -player_robot.velocity_y)

