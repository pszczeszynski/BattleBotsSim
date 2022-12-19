from graphics import *

from ai_robot import AiRobot
from player_robot import PlayerRobot
from robot import Robot


WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 1000

ROBOT_WIDTH = 100
ROBOT_HEIGHT = 100

FIELD_WIDTH = 1000
FIELD_HEIGHT = 1000
class Game:
    instance = None

    def __init__(self):
        Game.instance = self
        self.window = GraphWin('Sim', WINDOW_WIDTH, WINDOW_HEIGHT)

    @staticmethod
    def get_instance():
        return Game.instance

    # starts the game
    def start(self):
        player_robot = PlayerRobot(self.window, Point(WINDOW_WIDTH/2, WINDOW_HEIGHT/2), ROBOT_WIDTH, ROBOT_HEIGHT)
        player_robot.init_draw()

        ai_robot = AiRobot(self.window, Point(WINDOW_WIDTH/2, WINDOW_HEIGHT/2), ROBOT_WIDTH, ROBOT_HEIGHT)
        ai_robot.init_draw()

        while True:
            player_robot.update()
            player_robot.draw()
            ai_robot.update(player_robot)
            ai_robot.draw()

    @staticmethod
    def field_to_screen_coords(x: float, y: float) -> tuple[float, float]:
        return (x / FIELD_WIDTH) * WINDOW_WIDTH, (1 - (y / FIELD_HEIGHT)) * WINDOW_HEIGHT
