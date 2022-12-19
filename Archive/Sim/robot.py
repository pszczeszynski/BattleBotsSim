from graphics import *

from game_object import GameObject


class Robot(GameObject):

    # initialize with window
    def __init__(self, window: GraphWin, center: Point, width: int, height: int):
        self.width = width
        self.height = height
        self.rect_drawable = Rectangle(Point(center.x - width / 2, center.y - width/2),
                                       Point(center.x + width / 2, center.y + width/2))

        self.rect_drawable.setFill(color_rgb(255, 100, 100))
        super().__init__(window, self.rect_drawable, center.x, center.y)

