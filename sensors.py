from doctest import OutputChecker
from tkinter.messagebox import RETRY
from turtle import distance
import numpy as np
import pygame
import math

def uncertainty_add(distance, angle, sigma):
    mean = np.array([distance, angle])
    covariance = np.diag(sigma ** 2)
    distance, angle = np.random.multivariate_normal(mean, covariance)
    distance = max(distance, 0)
    angle = max(angle, 0)
    return [distance, angle]

class LaserSensor:
    def __init__(self, Range, uncertainty):
        self.Range = Range
        self.speed = 4
        self.sigma = np.array([uncertainty[0], uncertainty[1]])
        self.position = (0,0)
        self.exmap = pygame.image.load('test_track.png')
        self.W, self.H = pygame.display.get_surface().get_size()
        self.map = pygame.display.set_mode((self.W, self.H))
        self.map.blit(self.exmap, (0,0))
        self.data = []
        self.pointCloud_left = []
        self.pointCloud_right = []
        self.radius = 30
        self.Pink = (245, 90, 230)

    def distance(self, obstaclePosition):
        px = (obstaclePosition[0] - self.position[0]) ** 2
        py = (obstaclePosition[1] - self.position[1]) ** 2
        return math.sqrt(px + py)

    def Ad2pos(self, distance, angle, carPosition):
        x = distance * math.cos(angle) + carPosition[0]
        y = -distance * math.sin(angle) + carPosition[1]
        return(int(x), int(y))

    def sense_obtacles(self, heading):
        sensor_range = math.radians(90)
        x1, y1 = self.position[0], self.position[1]
        start_angle = heading - sensor_range
        finish_angle = heading + sensor_range
        for angle in np.linspace(0, 2 * math.pi, 30, False):
        #for angle in np.linspace(start_angle, finish_angle, 60, False):    
            x2, y2 = (x1 + self.Range * math.cos(angle), y1 - self.Range * math.sin(angle))
            for i in range(0, 100):
                u = i / 100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.W and 0 < y < self.H:
                    color = self.map.get_at((x, y))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        distance = self.distance((x, y))
                        output = uncertainty_add(distance, angle, self.sigma)
                        output.append(self.position)
                        point = self.Ad2pos(output[0], output[1], output[2])
                        if point not in self.pointCloud_right:
                            self.pointCloud_right.append(point)
                        #self.data.append(output)
                        break
                    if (color[0], color[1], color[2]) == (0, 0, 200):
                        distance = self.distance((x, y))
                        output = uncertainty_add(distance, angle, self.sigma)
                        output.append(self.position)
                        point = self.Ad2pos(output[0], output[1], output[2])
                        if point not in self.pointCloud_left:
                            self.pointCloud_left.append(point)
                        #self.data.append(output)
                        break          

        return [self.pointCloud_left, self.pointCloud_right]

    def sense_obtacles_new(self, heading):
        sensor_range = math.radians(80)
        x1, y1 = self.position[0], self.position[1]
        start_angle = heading - sensor_range
        finish_angle = heading + sensor_range
        #for angle in np.linspace(0, 2 * math.pi, 30, False):
        for angle in np.linspace(0, 2* math.pi, 30, False):    
            x2, y2 = (x1 + self.Range * math.cos(angle), y1 - self.Range * math.sin(angle))
            for i in range(0, 50):
                u = i / 50
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.W and 0 < y < self.H:
                    color = self.map.get_at((x, y))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        distance = self.distance((x, y))
                        output = uncertainty_add(distance, angle, self.sigma)
                        output.append(self.position)
                        point = self.Ad2pos(output[0], output[1], output[2])
                        if point not in self.pointCloud_right:
                            self.pointCloud_right.append(point)
                        #self.data.append(output)
                        break
                    if (color[0], color[1], color[2]) == (0, 0, 200):
                        distance = self.distance((x, y))
                        output = uncertainty_add(distance, angle, self.sigma)
                        output.append(self.position)
                        point = self.Ad2pos(output[0], output[1], output[2])
                        if point not in self.pointCloud_left:
                            self.pointCloud_left.append(point)
                        #self.data.append(output)
                        break

            """ for center in self.pointCloud_left:
                for i in range(1, len(self.pointCloud_left)):
                    if i < len(self.pointCloud_left):
                        #center = pointCloud_left[i]
                        offset = self.pointCloud_left[i]
                        if abs(center[0] - offset[0]) < self.radius and abs(center[1] - offset[1]) < self.radius:
                            self.pointCloud_left.pop(i)   """                 

        return [self.pointCloud_left, self.pointCloud_right]

