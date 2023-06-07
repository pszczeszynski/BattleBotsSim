import pygame
import math

# Define constants
WIDTH, HEIGHT = 800, 600
FPS = 60

# Initialize Pygame
pygame.init()
font = pygame.font.Font(None, 36)  # Font for displaying score
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()


# Define PygameObject class
class PygameObject:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle
        self.components = []

    def add_component(self, component):
        self.components.append(component)

    def update(self, drive_amount, turn_amount):
        self.x += math.cos(self.angle) * drive_amount * self.speed
        self.y -= math.sin(self.angle) * drive_amount * self.speed
        self.angle += turn_amount * math.pi / 180 * 2

    def draw(self):
        for component in self.components:
            component.draw(self.x, self.y, self.angle)

class Component:
    def __init__(self, rect, color):
        self.rect: pygame.Rect = rect
        self.color = color

    def draw(self, x, y, angle):
        # Create a larger surface for the rectangle to rotate within
        max_dim = max(self.rect.width, self.rect.height)
        rotated_surface = pygame.Surface((max_dim*2, max_dim*2), pygame.SRCALPHA)
        # Adjust the rectangle to the center of the larger surface
        adjusted_rect = self.rect.copy()
        adjusted_rect.center = (adjusted_rect.center[0] + rotated_surface.get_rect().center[0], adjusted_rect.center[1] + rotated_surface.get_rect().center[1])

        pygame.draw.rect(rotated_surface, self.color, adjusted_rect)

        rotated_surface = pygame.transform.rotate(rotated_surface, math.degrees(angle))
        rect_position = rotated_surface.get_rect(center=(x, y))

        screen.blit(rotated_surface, rect_position)
    
    def check_collision(self, other_component, angle1, angle2):
        return self.rectangles_overlap(self.rect, angle1, other_component.rect, angle2)

    def rectangles_overlap(self, rect1: pygame.Rect, angle1: float, rect2: pygame.Rect, angle2: float) -> bool:
        rect1_surface = pygame.Surface((rect1.width, rect1.height), pygame.SRCALPHA)
        pygame.draw.rect(rect1_surface, (0, 0, 0, 0), pygame.Rect(0, 0, rect1.width, rect1.height))

        rect2_surface = pygame.Surface((rect2.width, rect2.height), pygame.SRCALPHA)
        pygame.draw.rect(rect2_surface, (0, 0, 0, 0), pygame.Rect(0, 0, rect2.width, rect2.height))

        rotated_rect1 = pygame.transform.rotate(rect1_surface, math.degrees(angle1))
        rotated_rect2 = pygame.transform.rotate(rect2_surface, math.degrees(angle2))

        rect1_position = rotated_rect1.get_rect(center=rect1.center)
        rect2_position = rotated_rect2.get_rect(center=rect2.center)

        if rect1_position.colliderect(rect2_position):
            return True

        return False

# Define Robot class
class Robot(PygameObject):
    def __init__(self, x, y, color):
        super().__init__(x, y, 0)
        self.radius = 20
        self.speed = 5
        self.color = color
        self.add_component(Component(pygame.Rect(-self.radius, -self.radius, self.radius * 2, self.radius * 2), self.color))
        self.add_component(Component(pygame.Rect(-self.radius, -self.radius, self.radius, self.radius * 2), (255, 255, 0)))  # Back component
        self.add_component(Component(pygame.Rect(0, -self.radius, self.radius, self.radius * 2), (0, 255, 255)))  # Front component

        self.score = 0

    def update(self, drive_amount, turn_amount):
        super().update(drive_amount, turn_amount)

    def draw(self):
        super().draw()

    def check_collision(self, other_robot):
        for our_component in self.components:
            for their_component in other_robot.components:
                if our_component.check_collision(their_component, self.angle, other_robot.angle):
                    self.score += 1
                    return True

        return False

    def rectangles_overlap(self, rect1: pygame.Rect, angle1: float, rect2: pygame.Rect, angle2: float) -> bool:
        rect1_surface = pygame.Surface((rect1.width, rect1.height), pygame.SRCALPHA)
        pygame.draw.rect(rect1_surface, (0, 0, 0, 0), pygame.Rect(0, 0, rect1.width, ect1.height))

        rect2_surface = pygame.Surface((rect2.width, rect2.height), pygame.SRCALPHA)
        pygame.draw.rect(rect2_surface, (0, 0, 0, 0), pygame.Rect(0, 0, rect2.width, rect2.height))

        rotated_rect1 = pygame.transform.rotate(rect1_surface, math.degrees(angle1))
        rotated_rect2 = pygame.transform.rotate(rect2_surface, math.degrees(angle2))

        rect1_position = rotated_rect1.get_rect(center=rect1.center)
        rect2_position = rotated_rect2.get_rect(center=rect2.center)

        if rect1_position.colliderect(rect2_position):
            return True

        return False


# Create robots
robot1 = Robot(WIDTH/2 - 100, HEIGHT/2, (255, 0, 0))
robot2 = Robot(WIDTH/2 + 100, HEIGHT/2, (0, 0, 255))

# Game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Get user input for robot movements
    keys = pygame.key.get_pressed()
    drive_amount1 = (keys[pygame.K_w] - keys[pygame.K_s])  # -1 to 1 for forward and backward
    turn_amount1 = (keys[pygame.K_a] - keys[pygame.K_d])  # -1 to 1 for left and right
    drive_amount2 = (keys[pygame.K_UP] - keys[pygame.K_DOWN])
    turn_amount2 = (keys[pygame.K_LEFT] - keys[pygame.K_RIGHT])

    # Update robot positions
    robot1.update(drive_amount1, turn_amount1)
    robot2.update(drive_amount2, turn_amount2)

    # Check collisions and update scores
    robot1.check_collision(robot2)
    robot2.check_collision(robot1)

    # Clear the screen
    screen.fill((0, 0, 0))

    # Draw robots
    robot1.draw()
    robot2.draw()

    # Display scores
    score_text = font.render(f"Red: {robot1.score} Blue: {robot2.score}", True, (255, 255, 255))
    screen.blit(score_text, (10, 10))

    # Update the display
    pygame.display.flip()
    clock.tick(FPS)

# Quit the game
pygame.quit()