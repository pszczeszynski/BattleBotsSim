import pygame
import math
import keras
import numpy as np
import time

# Define constants
WIDTH, HEIGHT = 1920, 1080
FPS = 3000000 # set to 60 for real time


MANUAL_MODE = False

# Initialize Pygame
pygame.init()
font = pygame.font.Font(None, 36)  # Font for displaying score
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

def angle_wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


# Define PygameObject class
class PygameObject:
    def __init__(self, x, y, angle_rad):
        self.x = x
        self.y = y
        self.angle_rad = angle_rad
        self.components = []
        self.angle_rad_unwrapped = angle_rad
        self.turn_amount = 0
        self.drive_amount = 0

    def add_component(self, component):
        self.components.append(component)

    def update(self, drive_amount, turn_amount):
        self.drive_amount = drive_amount
        self.turn_amount = turn_amount
        self.x += math.cos(self.angle_rad) * drive_amount * self.speed
        self.y -= math.sin(self.angle_rad) * drive_amount * self.speed
        self.angle_rad += turn_amount * math.pi / 180 * 4
        self.angle_rad_unwrapped += turn_amount * math.pi / 180 * 4
        self.angle_rad = angle_wrap(self.angle_rad)

    def draw(self):
        for component in self.components:
            component.draw()

class Component:
    def __init__(self, object: PygameObject, rect, color):
        self.rect: pygame.Rect = rect
        self.color = color
        self.object: PygameObject = object

    def draw(self):
        # Create a larger surface for the rectangle to rotate within
        max_dim = max(self.rect.width, self.rect.height)
        rotated_surface = pygame.Surface((max_dim*2, max_dim*2), pygame.SRCALPHA)
        # Adjust the rectangle to the center of the larger surface
        adjusted_rect = self.rect.copy()
        adjusted_rect.center = (adjusted_rect.center[0] + rotated_surface.get_rect().center[0], adjusted_rect.center[1] + rotated_surface.get_rect().center[1])

        pygame.draw.rect(rotated_surface, self.color, adjusted_rect)

        rotated_surface = pygame.transform.rotate(rotated_surface, math.degrees(self.object.angle_rad))
        rect_position = rotated_surface.get_rect(center=(self.object.x, self.object.y))

        screen.blit(rotated_surface, rect_position)       

# Define Robot class
class Robot(PygameObject):
    def __init__(self, x, y, color):
        super().__init__(x, y, 0)
        self.radius = 10
        self.speed = 5
        self.color = color
        self.front_component = Component(self, pygame.Rect(0, -self.radius, self.radius, self.radius * 2), (0, 255, 255, 100))
        self.back_component = Component(self, pygame.Rect(-self.radius, -self.radius, self.radius, self.radius * 2), (255, 255, 0, 100))
        self.add_component(Component(self, pygame.Rect(-self.radius, -self.radius, self.radius * 2, self.radius * 2), self.color))
        self.add_component(self.back_component)  # Back component
        self.add_component(self.front_component)  # Front component

        self.score = 0

    def update(self, drive_amount, turn_amount):
        super().update(drive_amount, turn_amount)

    def draw(self):
        super().draw()

    def is_colliding(self, other_robot):
        our_angle = -self.angle_rad

        # check if we are colliding with the other robot at all
        if math.sqrt((self.x - other_robot.x)**2 + (self.y - other_robot.y)**2) > self.radius * 2:
            return False

        return True
        # draw a line from us to them for debugging
        pygame.draw.line(screen, (255, 0, 0), (self.x, self.y), (other_robot.x, other_robot.y))

        # draw a line out of us at our angle
        pygame.draw.line(screen, (0, 255, 0), (self.x, self.y), (self.x + math.cos(our_angle) * 100, self.y + math.sin(our_angle) * 100))

        # draw
        pygame.display.flip()


        # calculate angle to them
        angle_to_other_robot = math.atan2(other_robot.y - self.y, other_robot.x - self.x)


        THRESH_ANGLE = math.pi / 2
        # check if they are infront of us relative to us
        if abs(angle_wrap(angle_to_other_robot - our_angle)) < THRESH_ANGLE:
            return True

        return False


def rotate_point(x, y, angle_rad):
    # rotate point by angle
    rotated_x = x * math.cos(angle_rad) - y * math.sin(angle_rad)
    rotated_y = x * math.sin(angle_rad) + y * math.cos(angle_rad)

    return rotated_x, rotated_y

def convert_to_relative_coords(point: tuple, robot: Robot):
    """
    Converts a point to relative coordinates to the robot
    """
    # convert to relative coordinates
    relative_x = point[0] - robot.x
    relative_y = point[1] - robot.y

    # rotate by robot angle
    relative_x, relative_y = rotate_point(relative_x, relative_y, robot.angle_rad)

    return relative_x, relative_y
    
def simulate(ref_model: keras.Model, new_model: keras.Model):
    # Create robots
    red_bot = Robot(WIDTH/2 - 100, HEIGHT/2, (255, 0, 0))
    blue_bot = Robot(WIDTH/2 + 100, HEIGHT/2, (0, 0, 255))

    WIN_SCORE = 10

    MAX_TIME = 10
    MAX_FRAMES = MAX_TIME * 60
    avg_distance_between_robots = 0

    frame = 0


    start_time = time.time()

    def draw_screen():
        screen.fill((0, 0, 0))
        red_bot.draw()
        blue_bot.draw()

        # Draw score
        score_text = font.render(str(red_bot.score) + " - " + str(blue_bot.score), True, (255, 255, 255))
        screen.blit(score_text, (WIDTH/2 - score_text.get_width()/2, 10))

        pygame.display.flip()

    # Game loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False


        # convert to relative coordinates
        red_pos_relative_to_blue = convert_to_relative_coords((red_bot.x, red_bot.y), blue_bot)
        red_angle_relative_to_blue = angle_wrap(red_bot.angle_rad - blue_bot.angle_rad)

        blue_pos_relative_to_red = convert_to_relative_coords((blue_bot.x, blue_bot.y), red_bot)
        blue_angle_relative_to_red = angle_wrap(blue_bot.angle_rad - red_bot.angle_rad)

        red_pos_relative_to_blue_normalized = np.array([red_pos_relative_to_blue[0] / WIDTH, red_pos_relative_to_blue[1] / HEIGHT])
        red_angle_relative_to_blue_normalized = red_angle_relative_to_blue / (math.pi)

        blue_pos_relative_to_red_normalized = np.array([blue_pos_relative_to_red[0] / WIDTH, blue_pos_relative_to_red[1] / HEIGHT])
        blue_angle_relative_to_red_normalized = blue_angle_relative_to_red / (math.pi)


        # make prediction based on current state
        input_data_red = np.array([blue_pos_relative_to_red_normalized[1]])# np.array([[blue_pos_relative_to_red_normalized[0], blue_pos_relative_to_red_normalized[1], blue_angle_relative_to_red_normalized]])
        input_data_blue = np.array([red_pos_relative_to_blue_normalized[1]])# np.array([[red_pos_relative_to_blue_normalized[0], red_pos_relative_to_blue_normalized[1], red_angle_relative_to_blue_normalized]])

        # print("input_data_red" + str(input_data_red))

        red_prediction = new_model.predict(input_data_red)[0]
        blue_prediction = ref_model.predict(input_data_blue)[0]

        drive_red = red_prediction[0] * 2 - 1
        turn_red = red_prediction[1] * 2 - 1
        
        drive_blue = blue_prediction[0] * 2 - 1
        turn_blue = blue_prediction[1] * 2 - 1

        if drive_red > 1:
            drive_red = 1
        if drive_red < -1:
            drive_red = -1
        if turn_red > 1:
            turn_red = 1
        if turn_red < -1:
            turn_red = -1
        if drive_blue > 1:
            drive_blue = 1
        if drive_blue < -1:
            drive_blue = -1
        if turn_blue > 1:
            turn_blue = 1
        if turn_blue < -1:
            turn_blue = -1


        # print("FPS:" + str(frame / (time.time() - start_time)))
        if MANUAL_MODE:
            turn_blue = 0
            drive_blue = 0
            turn_red = 0
            drive_red = 0
            # get keyboard inputs
            keys = pygame.key.get_pressed()
            # update drive and turn based on keyboard inputs
            if keys[pygame.K_w]:
                drive_red = 1
            if keys[pygame.K_s]:
                drive_red = -1
            if keys[pygame.K_a]:
                turn_red = -1
            if keys[pygame.K_d]:
                turn_red = 1
            
            if keys[pygame.K_UP]:
                drive_blue = 1
            if keys[pygame.K_DOWN]:
                drive_blue = -1
            if keys[pygame.K_LEFT]:
                turn_blue = -1
            if keys[pygame.K_RIGHT]:
                turn_blue = 1
            
            drive_red = 0.1
            turn_red = -blue_pos_relative_to_red_normalized[1]

            print("blue_pos_relative_to_red_normalized", blue_pos_relative_to_red_normalized[1])

        # Update robot positions
        red_bot.update(drive_red, turn_red)
        blue_bot.update(drive_blue, turn_blue)

        # Check collisions and update scores
        r1_c = red_bot.is_colliding(blue_bot)
        r2_c = blue_bot.is_colliding(red_bot)

        # if only one of robots is colliding, give them a point
        if r1_c:
            red_bot.score += 1
        elif r2_c:
            blue_bot.score += 1

        if frame % 10 == 0:
            # print("prediction red: " + str(red_prediction[1]))
            # Draw screen
            draw_screen()


        clock.tick(FPS)

        avg_distance_between_robots += math.sqrt((red_bot.x - blue_bot.x)**2 + (red_bot.y - blue_bot.y)**2)

        frame += 1
        # break if one of the robots has won or if time has run out
        if not MANUAL_MODE:
            if red_bot.score >= WIN_SCORE or blue_bot.score >= WIN_SCORE or frame > MAX_FRAMES:
                break


    avg_distance_between_robots /= frame
    avg_distance_between_robots /= math.sqrt(WIDTH**2)
    # print("avg_distance_between_robots: " + str(avg_distance_between_robots))
    # print("abs(red_bot.angle_rad_unwrapped) / (2 * math.pi): " + str(abs(red_bot.angle_rad_unwrapped) / (2 * math.pi)))

    score = red_bot.score -avg_distance_between_robots

    # take sigmoid to force between 0 and 1
    score = 1 / (1 + math.exp(-score))

    return score# red_bot.score - blue_bot.score

    # Quit the game
    pygame.quit()

