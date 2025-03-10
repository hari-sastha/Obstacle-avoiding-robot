import math
import pygame
from ROBOT import Graphics, Robot, Ultrasonic

MAP_DIMENSIONS = (1200, 600)
robot_image_path = 'robo.png'
map_image_path = 'field.png'

gfx = Graphics(MAP_DIMENSIONS, robot_image_path, map_image_path)
robot = Robot(0.01 * 3779.52)
sensor_range = 150, math.radians(40)
ultrasonic = Ultrasonic(sensor_range, gfx.map)

dt = 0
last_time = pygame.time.get_ticks()
running = True
setting_start = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            if setting_start:
                robot.set_start(pos)
                setting_start = False
                print(f"Start Position Set: {pos}")
            else:
                robot.set_target(pos)
                print(f"Target Position Set: {pos}")

    dt = (pygame.time.get_ticks() - last_time) / 1000
    last_time = pygame.time.get_ticks()
    gfx.map.blit(gfx.map_img, (0, 0))

    if robot.x is not None and robot.y is not None:
        gfx.draw_start(robot.x, robot.y)
    if robot.target_x is not None and robot.target_y is not None:
        gfx.draw_target(robot.target_x, robot.target_y)
        if not robot.reached_goal:
            robot.kinematics(dt, ultrasonic)
        gfx.draw_robot(robot.x, robot.y, robot.heading)
    pygame.display.update()

pygame.quit()
