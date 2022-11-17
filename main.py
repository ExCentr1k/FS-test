from ast import Try
from importlib.resources import path
from logging import root
from traceback import print_tb
from turtle import heading
import pygame
import math
import os
import time
from sklearn.cluster import KMeans
import numpy as np

import environment, sensors
from tkinter import *

"""root = Tk()
embed = Frame(root, width=1600, height=1000)


os.environ['SDL_WINDOWID'] = str(embed.winfo_id())
os.environ['SDL_VIDEODRIVER'] = 'windib'

canvas = Canvas(root, bg = 'grey', width=1800, height=800)
canvas.focus_set()
canvas.pack(anchor='nw', fill = 'both', expand = 1)"""

env = environment.buildEnvironment()
#env.map.fill((128, 128, 128))
#env.originalMap = env.map.copy()

laser = sensors.LaserSensor(200, uncertainty=(0.4, 0.01))

env.infomap = env.map.copy()

start = (975, 730)
robot = environment.Robot(start, 0.01 * 3779.52)

sensor_range = 300, math.radians(35)
path_sensor = 300, math.radians(90)

ultra_sonic = environment.Ultrasonic(sensor_range, env.map)
path_lidar = environment.Ultrasonic(path_sensor, env.map)

dt = 0
last_time = pygame.time.get_ticks()
rect = pygame.Rect(860, 735, 0, 0).inflate(15, 130)
color = (255, 255, 255)
pygame.draw.rect(env.infomap, color, rect)

running = True
sensorOn = True
collide_start = False
sensor_data = []

"""def draw_sensor_data_tk(point_cloud):
    for point in point_cloud:
        canvas.create_oval(point[0], point[1], point[0] + 1, point[1] + 1, fill = "black", width = 0)"""

right_bound = [[975, 730]]
left_bound = [[975, 730]]

while running:
    #robot.minspeed = 0
    if sensorOn:
        position = (robot.x, robot.y)
        laser.position = position
        sensor_data = laser.sense_obtacles(robot.heading)    # track boundaries
        #test = env.triangulation()
        #env.triangulation(robot.x, robot.y, sensor_data)
        #env.pointCloud = sensor_data        #test!!!

        #robot.lap(robot.x, robot.y)
        if robot.lapcount >= 2:
            sensorOn = False

    dt = (pygame.time.get_ticks() - last_time) / 1000
    last_time = pygame.time.get_ticks()

    env.map.blit(env.infomap, (0, 0))

    if sensorOn:
        env.show_sensorData(sensor_data)

    #env.pointCloud = laser.sense_obtacles(robot.heading)
    env.draw_boundaries(sensor_data)
    robot.kinematics(dt, color)
    env.draw_robot(robot.x, robot.y, robot.heading)
    env.trail((robot.x, robot.y), sensorOn)
    pygame.draw.circle(env.map, (80, 50, 90), (robot.x, robot.y), 275, 2)
    pygame.draw.circle(env.map, (80, 50, 90), (robot.x, robot.y), 225, 2)
    pygame.draw.circle(env.map, (80, 50, 90), (robot.x, robot.y), 100, 2)
    #point_cloud = laser.sense_obtacles(robot.heading)
    point_cloud = ultra_sonic.sense_obstacles(robot.x, robot.y, robot.heading, sensorOn)
    path_points = path_lidar.sense_obstacles2(robot.x, robot.y, robot.heading, sensorOn)
    #env.draw_boundaries(sensor_data)
    if sensorOn:
            """right = KMeans(n_clusters=2)
            #left = KMeans(n_clusters=2)
            right.fit(path_points)
            #right_bound.append(right.cluster_centers_)
            pygame.draw.lines(env.map, (255, 0, 0), False, right.cluster_centers_, 2)
            env.draw_boundaries_cluster(right.cluster_centers_, 1)"""
   
            right = KMeans(n_clusters=2)
            left = KMeans(n_clusters=2)
            right.fit(path_points[0])
            #right_bound.append(right.cluster_centers_)
            pygame.draw.lines(env.map, (255, 0, 0), False, right.cluster_centers_, 2)
            #env.draw_boundaries_cluster(robot.x, robot.y, right.cluster_centers_, 1)
            #left.fit(point_cloud[1])
            #left_bound.append(left.cluster_centers_)
            #pygame.draw.lines(env.map, (255, 0, 0), False, left.cluster_centers_, 2)
            #env.draw_boundaries_cluster(robot.x, robot.y, left.cluster_centers_, 2)
            #env.triangulation2(robot.x, robot.y, left.cluster_centers_, right.cluster_centers_, robot.heading)

    robot.avoid_obstacles(point_cloud)
    env.draw_sensor_data(point_cloud[0], "left")
    env.draw_sensor_data(point_cloud[1], "right")
    """canvas.create_oval(robot.x, robot.y, robot.x + 3, robot.y + 3)
    draw_sensor_data_tk(path_points[0], "left")
    draw_sensor_data_tk(path_points[1], "right")"""

    #env.Delau(robot.x, robot.y, path_points, robot.heading)
    #start_index_new = env.triangulation2(robot.x, robot.y, path_points)

    collide = rect.collidepoint(robot.x, robot.y)
    if collide:
        if collide_start == False:
            collide_start = True
            robot.lap(robot.x, robot.y)
            color = (255, 0, 0)
            pygame.draw.rect(env.map, color, rect)
            pygame.display.flip()
    else:
        color = (255, 255, 255)
        collide_start = False

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                time.sleep(3)

    pygame.display.update()
    #root.update()