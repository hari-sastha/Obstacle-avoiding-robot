import pygame
import math
import numpy as np
import random  # Importing random to control random turning

def distance(point1, point2):
    """Calculate Euclidean distance"""
    return np.linalg.norm(np.array(point1) - np.array(point2))

class Robot:
    def __init__(self, startpos, width):
        self.m2p = 3779.52  # Meters to Pixels Conversion
        self.w = width
        self.x, self.y = startpos
        self.heading = 0
        self.vl = self.vr = 0.01 * self.m2p  # Slow down speed
        self.maxspeed = 0.02 * self.m2p
        self.minspeed = 0.01 * self.m2p
        self.min_obs_dist = 100
        self.count_down = 5  # Seconds

    def move_backward(self):
        self.vr = -self.minspeed
        self.vl = -self.minspeed / 2

    def move_forward(self):
        self.vr = self.minspeed
        self.vl = self.minspeed

    def move_towards(self, target, dt):
        """Move the robot towards the target"""
        angle_to_target = math.atan2(target[1] - self.y, target[0] - self.x)
        self.heading = angle_to_target  # Update heading
        self.move_forward()

    def avoid_obstacles(self, point_cloud, yellow_points, dt, end_pos):
        closest_obs = None
        min_dist = np.inf

        for point in point_cloud:
            dist = distance([self.x, self.y], point)
            if dist < min_dist:
                min_dist = dist
                closest_obs = (point, dist)

        # Move towards the detected yellow point
        if yellow_points:
            target = yellow_points[0]  # Choose the first detected yellow point
            self.move_towards(target, dt)
        elif end_pos and distance((self.x, self.y), end_pos) < 10:
            self.move_towards(end_pos, dt)
        elif closest_obs and closest_obs[1] < self.min_obs_dist and self.count_down > 0:
            self.count_down -= dt
            self.move_backward()
            if random.choice([True, False]):  
                self.turn_left()
            else:
                self.turn_right()
        else:
            self.count_down = 5
            self.move_forward()

    def kinematics(self, dt):
        """Update robot's position based on its speed and heading"""
        self.x += ((self.vl + self.vr) / 2) * math.cos(self.heading) * dt
        self.y -= ((self.vl + self.vr) / 2) * math.sin(self.heading) * dt
        self.heading += (self.vr - self.vl) / self.w * dt

        self.heading %= 2 * math.pi
        self.vr = max(min(self.maxspeed, self.vr), self.minspeed)
        self.vl = max(min(self.maxspeed, self.vl), self.minspeed)

    def stop(self):
        """Stop the robot"""
        self.vr = 0
        self.vl = 0

    def turn_left(self):
        """Turn the robot to the left"""
        self.heading += math.radians(30)  

    def turn_right(self):
        """Turn the robot to the right"""
        self.heading -= math.radians(30)  


class Graphics:
    def __init__(self, dimensions, robot_img_path, map_img_path):
        pygame.init()
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.red = (255, 0, 0)
        self.yellow = (255, 255, 0)  # Yellow color for goal

        self.height, self.width = dimensions
        self.map = pygame.display.set_mode((self.width, self.height), pygame.FULLSCREEN)
        pygame.display.set_caption("Obstacle Avoidance")

        # Load Images (Use the absolute file paths)
        self.robot = pygame.image.load(robot_img_path)
        self.map_img = pygame.image.load(map_img_path)

        self.map.blit(self.map_img, (0, 0))

    def draw_robot(self, x, y, heading):
        """Draw the robot at the specified position"""
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center=(x, y))
        self.map.blit(rotated, rect)

    def draw_sensor_data(self, point_cloud):
        """Draw sensor data points (obstacles)"""
        for point in point_cloud:
            pygame.draw.circle(self.map, self.red, point, 3, 0)

    def draw_end_point(self, end_pos):
        """Draw the end point as a yellow dot"""
        if end_pos:
            pygame.draw.circle(self.map, self.yellow, end_pos, 10)  # Yellow color


class Ultrasonic:
    def __init__(self, sensor_range, game_map):
        self.sensor_range = sensor_range
        self.map = game_map
        self.map_width, self.map_height = pygame.display.get_surface().get_size()

    def sense_obstacles(self, x, y, heading):
        """Sense obstacles (black areas) and detect yellow (goal) in front of the robot"""
        obstacles = []
        yellow_points = []
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

                    if color[:3] == (0, 0, 0):  # Black (Obstacle)
                        obstacles.append([xi, yi])
                        break
                    elif color[:3] == (255, 255, 0):  # Yellow (Goal)
                        yellow_points.append([xi, yi])

        return obstacles, yellow_points


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

# Variables for user-defined endpoints
end_pos = None

running = True
fullscreen = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
            elif event.key == pygame.K_F11:
                fullscreen = not fullscreen
                if fullscreen:
                    gfx.map = pygame.display.set_mode((gfx.width, gfx.height), pygame.FULLSCREEN)
                else:
                    gfx.map = pygame.display.set_mode((gfx.width, gfx.height))
        elif event.type == pygame.MOUSEBUTTONDOWN:
            # Set end position on mouse click
            end_pos = pygame.mouse.get_pos()
            print(f"End Position Set: {end_pos}")

    # Calculate delta time
    dt = (pygame.time.get_ticks() - last_time) / 1000
    last_time = pygame.time.get_ticks()

    # Clear the screen and redraw background
    gfx.map.blit(gfx.map_img, (0, 0))

    # Move the robot based on its kinematics and avoid obstacles
    robot.kinematics(dt)
    gfx.draw_robot(robot.x, robot.y, robot.heading)
    point_cloud, yellow_points = ultrasonic.sense_obstacles(robot.x, robot.y, robot.heading)
    robot.avoid_obstacles(point_cloud, yellow_points, dt, end_pos)
    gfx.draw_sensor_data(point_cloud)

    # Draw the end point if it's set
    gfx.draw_end_point(end_pos)

    pygame.display.update()

pygame.quit()