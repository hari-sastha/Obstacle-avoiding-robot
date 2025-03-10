import pygame
import math
import numpy as np

def distance(point1, point2):
    """Calculate Euclidean distance"""
    return np.linalg.norm(np.array(point1) - np.array(point2))

class Robot:
    def __init__(self, startpos, width):
        self.m2p = 3779.52  # Meters to Pixels Conversion
        self.w = width
        self.x, self.y = startpos
        self.heading = 0
        self.vl = self.vr = 0.01 * self.m2p
        self.maxspeed = 0.02 * self.m2p
        self.minspeed = 0.01 * self.m2p
        self.min_obs_dist = 100
        self.count_down = 5  # Seconds
        self.is_moving = True  # Initially the robot is moving

    def move_backward(self):
        self.vr = -self.minspeed
        self.vl = -self.minspeed / 2

    def move_forward(self):
        self.vr = self.minspeed
        self.vl = self.minspeed

    def avoid_obstacles(self, point_cloud, dt):
        closest_obs = None
        min_dist = np.inf

        for point in point_cloud:
            dist = distance([self.x, self.y], point)
            if dist < min_dist:
                min_dist = dist
                closest_obs = (point, dist)

        if closest_obs and closest_obs[1] < self.min_obs_dist and self.count_down > 0:
            self.count_down -= dt
            self.move_backward()
        else:
            self.count_down = 5
            self.move_forward()

    def kinematics(self, dt):
        self.x += ((self.vl + self.vr) / 2) * math.cos(self.heading) * dt
        self.y -= ((self.vl + self.vr) / 2) * math.sin(self.heading) * dt
        self.heading += (self.vr - self.vl) / self.w * dt

        self.heading %= 2 * math.pi
        self.vr = max(min(self.maxspeed, self.vr), self.minspeed)
        self.vl = max(min(self.maxspeed, self.vl), self.minspeed)

    def stop(self):
        """Stop the robot"""
        self.is_moving = False
        self.vr = 0
        self.vl = 0


class Graphics:
    def __init__(self, dimensions, robot_img_path, map_img_path):
        pygame.init()
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)

        self.height, self.width = dimensions
        self.map = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Obstacle Avoidance")

        # Load Images (Use the absolute file paths)
        self.robot = pygame.image.load(r"C:\Users\Harisastha\robo\robo.png")
        self.map_img = pygame.image.load(r"C:\Users\Harisastha\robo\field.png")

        self.map.blit(self.map_img, (0, 0))

    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center=(x, y))
        self.map.blit(rotated, rect)

    def draw_sensor_data(self, point_cloud):
        for point in point_cloud:
            pygame.draw.circle(self.map, self.red, point, 3, 0)

    def draw_end_point(self, end_pos):
        """Draw the end point as a red dot."""
        if end_pos:
            pygame.draw.circle(self.map, self.red, end_pos, 5)  # Red dot for end point


class Ultrasonic:
    def __init__(self, sensor_range, game_map):
        self.sensor_range = sensor_range
        self.map = game_map
        self.map_width, self.map_height = pygame.display.get_surface().get_size()

    def sense_obstacles(self, x, y, heading):
        obstacles = []
        start_angle = heading - self.sensor_range[1]
        finish_angle = heading + self.sensor_range[1]

        for angle in np.linspace(start_angle, finish_angle, 10, False):
            x2 = x + self.sensor_range[0] * math.cos(angle)
            y2 = y - self.sensor_range[0] * math.sin(angle)

            for i in range(100):
                u = i / 100
                xi = int(x2 * u + x * (1 - u))
                yi = int(y2 * u + y * (1 - u))

                if 0 < xi < self.map_width and 0 < yi < self.map_height:
                    color = self.map.get_at((xi, yi))
                    self.map.set_at((xi, yi), (0, 208, 255))

                    if color[:3] == (0, 0, 0):  # Black pixels indicate obstacles
                        obstacles.append([xi, yi])
                        break
        return obstacles

# ======= Main Program =======
MAP_DIMENSIONS = (600, 1200)

# Initialize Graphics
gfx = Graphics(MAP_DIMENSIONS, 'robo.png', 'field.png')

# Initialize Robot
start = (200, 200)
robot = Robot(start, 0.01 * 3779.52)

# Initialize Sensor
sensor_range = (250, math.radians(40))
ultrasonic = Ultrasonic(sensor_range, gfx.map)

# Simulation Variables
dt = 0
last_time = pygame.time.get_ticks()

# Variable for the end point
end_pos = None

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            # Set end position on mouse click
            end_pos = pygame.mouse.get_pos()
            print(f"End Position Set: {end_pos}")

    # Calculate delta time
    dt = (pygame.time.get_ticks() - last_time) / 1000
    last_time = pygame.time.get_ticks()

    # Clear the screen and redraw the background
    gfx.map.blit(gfx.map_img, (0, 0))

    # Move robot and avoid obstacles
    robot.kinematics(dt)
    point_cloud = ultrasonic.sense_obstacles(robot.x, robot.y, robot.heading)
    robot.avoid_obstacles(point_cloud, dt)

    # Draw robot and obstacles
    gfx.draw_robot(robot.x, robot.y, robot.heading)
    gfx.draw_sensor_data(point_cloud)

    # Check distance from robot to end point
    if end_pos and distance((robot.x, robot.y), end_pos) < 10:  # Stop if within 10 pixels of the endpoint
        print("Robot has reached the endpoint!")
        robot.stop()  # Stop the robot
        gfx.draw_end_point(end_pos)  # Draw the endpoint
        break  # Exit the loop once the robot reaches the endpoint

    # Draw the endpoint
    if end_pos:
        gfx.draw_end_point(end_pos)

    pygame.display.update()

pygame.quit()
