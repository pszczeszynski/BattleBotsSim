from graphics import *

# from game import Game


class GameObject:
    def __init__(self, window: GraphWin, graphicsObject: GraphicsObject, x: int, y: int):
        self._graphicsObject = graphicsObject

        self.window = window
        self._x = x
        self._y = y

        self._prev_x = self._x
        self._prev_y = self._y

        self.velocity_x = 0
        self.velocity_y = 0

        self.curr_time = time.time()
        self.last_time = time.time()

    # called once before future draws
    def init_draw(self):
        self._graphicsObject.draw(self.window)

    # called every frame
    def draw(self):
        self.curr_time = time.time()
        delta_time = self.curr_time - self.last_time
        self.translate(self.velocity_x * delta_time, self.velocity_y * delta_time)

        delta_x = self._x - self._prev_x
        delta_y = self._y - self._prev_y

        self._graphicsObject.move(delta_x, delta_y)

        self._prev_x = self._x
        self._prev_y = self._y

        self.last_time = self.curr_time

    def set_velocity(self, x: float, y: float):
        self.velocity_x = x
        self.velocity_y = y

    def set_x_velocity(self, x: float):
        self.velocity_x = x

    def set_y_velocity(self, y: float):
        self.velocity_y = y

    def translate_x(self, x: float):
        self._x += x

    def translate_y(self, y: float):
        self._y += y

    def translate(self, x: float, y: float):
        self._x += x
        self._y += y

