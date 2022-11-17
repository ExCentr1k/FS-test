from dis import dis
import math
from operator import index
from pyclbr import readmodule
from re import I
from turtle import distance, heading, left
from unicodedata import unidata_version
import numpy as np
import pygame
import random
from scipy import rand, interpolate


class buildEnvironment:
    def __init__(self):
        pygame.init()
        self.pointCloud = []
        self.externalMap = pygame.image.load('test_track.png')
        #self.originalMap = pygame.image.load('bahrain_3.png')
        self.maph = self.externalMap.get_height()
        self.mapw = self.externalMap.get_width()
        self.robot = pygame.image.load('f1_car.png')
        #self.car = pygame.rect(self.robot.get_height(), self.robot.get_width())
        self.MapWindowName = 'Navigation System'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.mapw, self.maph))
        self.map.blit(self.externalMap, (0,0))
        self.trail_set = []
        self.threshold_max = 127
        self.threshold_min = 63    # sensetivity for boundaries

        self.black = (0, 0, 0)
        self.gray = (70, 70, 70)
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.white = (255, 255, 255)
        self.Pink = (245, 90, 230)

    

    def show_sensorData(self, pointCloud_lidar):
        #self.infomap = self.map.copy()
        for point in pointCloud_lidar[0]:
            self.map.set_at((int(point[0]), int(point[1])), (255, 128, 0))
        for point in pointCloud_lidar[1]:
            self.map.set_at((int(point[0]), int(point[1])), (0, 255, 255))

    def line_check(self, startpoint, endpoint):
        px = (startpoint[0] - endpoint[0]) ** 2
        py = (startpoint[1] - endpoint[1]) ** 2
        return math.sqrt(px + py)

    def draw_boundaries(self, pointCloud_lidar):
        #self.infomap = self.map.copy()
        for i in range(0, len(pointCloud_lidar[1]) - 1, 10):
            if self.threshold_min < self.line_check(pointCloud_lidar[1][i],
                    pointCloud_lidar[1][i+1]) < self.threshold_max:
                pygame.draw.line(self.infomap, (0, 255, 255), pointCloud_lidar[1][i],
                    pointCloud_lidar[1][i+1], 1)
        for i in range(0, len(pointCloud_lidar[0]) - 1, 10):
            if self.threshold_min < self.line_check(pointCloud_lidar[0][i],
                    pointCloud_lidar[0][i+1]) < self.threshold_max:
                pygame.draw.line(self.infomap, (255, 128, 0), pointCloud_lidar[0][i],
                    pointCloud_lidar[0][i+1], 1) 

    def draw_boundaries_cluster(self, x, y, right, code):
        centers = []
        cone1 = right[0][0], right[0][1]
        cone2 = right[1][0], right[1][1]
        #cone3 = right[2][0], right[2][1]
        if self.distance_trian(cone1, cone2) <= 150:
            centers.append(cone1)
            centers.append(cone2)
        """if self.distance_trian(cone2, cone3) <= 135:
            centers.append(cone2)
            #centers.append(cone3)
        if self.distance_trian(cone1, cone3) <= 135:
            centers.append(cone1)
            #centers.append(cone3)             
        list(set(centers))"""
        if code == 2:       
            pygame.draw.lines(self.infomap, (255, 128, 0), False, right, 2)
        else:
            pygame.draw.lines(self.infomap, (0, 255, 255), False, right, 2)

    def distance_trian(self, cone1, cone2):
        px = (cone1[0] - cone2[0]) ** 2
        py = (cone1[1] - cone2[1]) ** 2
        return math.sqrt(px + py)

    def B_spline(self, waypoints):
        x = []
        y = []

        for point in waypoints:
            x.append(point[0])
            y.append(point[1])

        tck, *rest = interpolate.splprep([x, y], k = 2)
        u = np.linspace(0, 1, num = 100)
        smooth = interpolate.splev(u, tck)

        return smooth

    def triangulation12(self, x, y, pcl):
        cone1 = []
        cone2 = []
        if len(pcl[0]) > 1:
            pcl_left = pcl[0]
            i = random.randrange(0, len(pcl_left) - 1)
            j = random.randrange(0, len(pcl_left) - 1)
            cone1 = (pcl_left[i])
            cone3 = (pcl_left[j])
            if self.distance_trian(cone1, cone3) < 30:
                j = random.randrange(0, len(pcl_left) - 1)
                cone3 = (pcl_left[j])
            #cone1.append(pcl_left[j])
            #distance = self.distance_trian(x, y, pcl_left[i])

            #pygame.draw.circle(self.map, (255, 210, 50), (predicted_x1, predicted_y1), 9, 0)
            #self.infomap.set_at((int(predicted_x), int(predicted_y)), (0, 0, 255))
            #pygame.draw.line(self.map, (255, 0, 0), (x, y), pcl_left[i], 1)

        if len(pcl[1]) > 1:
            pcl_right = pcl[1]
            i = random.randrange(0, len(pcl_right) - 1)
            j = random.randrange(0, len(pcl_right) - 1)
            cone2 = (pcl_right[i])
            cone4 = (pcl_right[j])
            #cone2.append(pcl_right[j])

        if len(pcl[0]) > 1 and len(pcl[1]) > 1:
            #pygame.draw.line(self.map, (0, 255, 0), cone1, cone2, 1)
            #pygame.draw.circle(self.map, (255, 255, 255), (abs(predicted_x1+predicted_x2) / 2, abs(predicted_y1+predicted_y2) / 2), 5, 0)
            
            pygame.draw.circle(self.map, (80, 50, 90), ((cone1[0] + cone2[0])/2, (cone1[1] + cone2[1])/2), 7, 0)
            pygame.draw.circle(self.map, (255, 0, 0), ((cone3[0] + cone4[0])/2, (cone3[1] + cone4[1])/2), 7, 0)
            #pygame.draw.circle(self.map, (80, 50, 90), ((cone1[0] + cone2[0])/2, (cone1[1] + cone2[1])/2), 7, 0)
            pygame.draw.lines(self.map, (0, 255, 0), True, (cone1, cone2, (x, y)), 2)
            #print(int((cone1[0] + cone2[0])/2), int((cone1[1] + cone2[1])/2))
            return (int((cone1[0] + cone2[0])/2), int((cone1[1] + cone2[1])/2))

    def triangulation2(self, x, y, left, right, heading):
        mark1 = (left[0][0] + right[0][0])/2, (left[0][1] + right[0][1])/2
        mark2 = (left[1][0] + right[1][0] + 1)/2, (left[1][1] + right[1][1] + 1)/2
        pygame.draw.lines(self.map, (255, 0, 162), False, ((x, y), mark1, mark2), 2)
        x2 = x + 10 * math.cos(heading)
        y2 = y - 10 * math.sin(heading)
        smooth = self.B_spline(((x, y), (x2, y2), mark2))
        x_smooth, y_smooth = smooth
        #pygame.draw.lines(self.map, (0, 255, 0), True, (), 2)
        for xn, yn in zip(x_smooth, y_smooth):
            pygame.draw.circle(self.map, (255, 215, 0), (xn, yn), 2, 0)

    def Delau(self, x, y, pcl, heading):
        pcl_left = pcl[0]
        pcl_right = pcl[1]
        car_pos = x, y
        if len(pcl_left) > 2 and len(pcl_right) > 2:
            if len(pcl_left) < len(pcl_right):
                for i in range(len(pcl_left)):
                    cone1 = [pcl_left[i]]
                    cone2 = [pcl_right[i]]
                    cone3 = [pcl_left[1]]
                    cone4 = [pcl_right[1]]
                    cone5 = [pcl_left[len(pcl_left) -1]]
                    cone6 = [pcl_right[len(pcl_right) - 1]]
                    for j in range(1, len(pcl_left)):
                        if rcheck(cone1, cone3) == True and cone1 != cone3:
                            cone3 = [pcl_left[j]]
                        else:
                            break
                    for j in range(1, len(pcl_right)):
                        if rcheck(cone2, cone4) == True and cone2 != cone4:
                            cone4 = [pcl_right[j]]
                        else:
                            break
            else:
                for i in range(len(pcl_right)):
                    cone1 = [pcl_left[i]]
                    cone2 = [pcl_right[i]]
                    cone3 = [pcl_left[1]]
                    cone4 = [pcl_right[1]]
                    cone5 = [pcl_left[len(pcl_left) -1]]
                    cone6 = [pcl_right[len(pcl_right) - 1]]
                    for j in range(1, len(pcl_left)):
                        cone3 = [pcl_left[j]]
                        if rcheck(cone1, cone3) == True:
                            cone3 = [pcl_left[j]]
                        else:
                            break
                    for j in range(1, len(pcl_right)):
                        cone4 = [pcl_right[j]]
                        if rcheck(cone2, cone4) == True:
                            cone4 = [pcl_right[j]]
                        else:
                            break   
            if cone1 != None and cone2 != None and cone3 != None and cone4 != None and cone5 != None and cone6 != None:                
                pygame.draw.lines(self.map, (0, 255, 0), True, ((cone1[0][0],cone1[0][1]), (cone2[0][0],cone2[0][1]),
                (cone3[0][0],cone3[0][1]), (cone4[0][0],cone4[0][1])), 2)

                x2 = x + 10 * math.cos(heading)
                y2 = y - 10 * math.sin(heading)
                pygame.draw.circle(self.map, (150, 0, 0), (x2, y2), 5, 0)
                mark1 = (cone1[0][0] + cone2[0][0])/2, (cone1[0][1] + cone2[0][1])/2
                mark2 = (cone3[0][0] + cone2[0][0] + 1)/2, (cone3[0][1] + cone2[0][1] + 1)/2
                mark3 = (cone3[0][0] + cone4[0][0])/2, (cone3[0][1] + cone4[0][1])/2
             
                smooth = self.B_spline(((x, y), (x2, y2), mark3))
                x_smooth, y_smooth = smooth
                counter = 0
                next_x = 0 
                next_y = 0
                flag = True
                for xn, yn in zip(x_smooth, y_smooth):
                    counter += 1
                    if counter > 10 and flag == True:
                        next_x = xn
                        next_y = yn
                        flag = False
                    pygame.draw.circle(self.map, (255, 215, 0), (xn, yn), 2, 0)
                #self.draw_robot(next_x, next_y, heading)    
                #pygame.draw.lines(self.map, (0, 0, 150), False, ((x, y), (x2, y2), mark1, mark2, mark3), 2)            


    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center = (x, y))
        self.map.blit(rotated, rect)

    def trail(self, pos, color_flag):
        for i in range(0, len(self.trail_set) - 1):
            if color_flag == True:
                pygame.draw.line(self.map, self.white, (self.trail_set[i][0], self.trail_set[i][1]),
                    (self.trail_set[i+1][0], self.trail_set[i+1][1]), 2)
            else:
                pygame.draw.line(self.map, self.white, (self.trail_set[i][0], self.trail_set[i][1]),
                    (self.trail_set[i+1][0], self.trail_set[i+1][1]), 2)
        if self.trail_set.__sizeof__() > 13000:
              self.trail_set.pop(0)
        self.trail_set.append(pos)
    

    def draw_sensor_data(self, point_cloud, code):
        if code == "left":
            for point in point_cloud:
                pygame.draw.circle(self.map, self.Red, point, 3, 0)
        else:
            for point in point_cloud:
                pygame.draw.circle(self.map, self.Green, point, 3, 0)

#################################### robot
def rcheck(point1, point2):
    if ((point2[0][0] - point1[0][0])**2 + (point2[0][1] - point1[0][1])**2) < 40**2:
        return True
    else:
        if distance_1(point1, point2) < 120:
            return False
        else:
            return True

def distance_1(point1, point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1 - point2)


class Robot:
    def __init__(self, startpos, width):

        self.m2p = 3779.52  #meter to pixels

        self.braking = False
        self.w = width
        self.x = startpos[0]
        self.y = startpos[1]
        self.heading = math.pi

        self.vl = 0.01 * self.m2p
        self.vr = 0.01 * self.m2p

        self.maxspeed = 0.05 * self.m2p
        self.minspeed = 0.05 * self.m2p

        self.lapcount = 0

        self.min_obs_dist = 51
        self.count_down = 5

    def avoid_obstacles(self, point_cloud):     #right and left boundaries
        closest_obs_r = (point_cloud[0], 500)
        dist = np.inf
        # for rightside point_cloud[0] - black
        if len(point_cloud[0]) >= 1:
            for point in point_cloud[0]:
                if dist > distance_1([self.x, self.y], point):
                        dist = distance_1([self.x, self.y], point)
                        closest_obs_r = (point, dist)

        # for leftside  point_cloud[1] - blue
        closest_obs_l = (point_cloud[1], 500)
        dist = np.inf        
        if len(point_cloud[1]) >= 1:
            for point in point_cloud[1]:
                if dist > distance_1([self.x, self.y], point):
                        dist = distance_1([self.x, self.y], point)
                        closest_obs_l = (point, dist)
       
        self.move_forward()
        if closest_obs_l[1] < self.min_obs_dist + (self.w / 2) + 7:# or len(point_cloud[1]) > len(point_cloud[0]):# and self.count_down > 0:
            self.turn_right()
        elif closest_obs_r[1] < self.min_obs_dist + (self.w / 2) + 7:# or len(point_cloud[0]) > len(point_cloud[1]):
            #self.count_down -= dt
            #print('right ' + str(closest_obs_r[1]))
            self.turn_left()

    def turn_right(self):
        self.vr = self.minspeed / 4
        self.vl = self.minspeed
        if self.lapcount > 1:
            self.vr = self.maxspeed / 4
            self.vl = self.maxspeed 

    def turn_left(self):
        self.vr = self.minspeed
        self.vl = self.minspeed  / 4
        if self.lapcount > 1:
            self.vr = self.maxspeed 
            self.vl = self.maxspeed / 4

    def move_forward(self):
        self.vr = self.minspeed
        self.vl = self.minspeed

        if self.lapcount > 1:
            self.vr = self.maxspeed
            self.vl = self.maxspeed

    def kinematics(self, dt, color):
        self.x += ((self.vl + self.vr)/2) * math.cos(self.heading) * dt
        self.y -= ((self.vl + self.vr)/2) * math.sin(self.heading) * dt
        self.heading += (self.vr - self.vl) / self.w * dt

        if self.heading > 2 * math.pi or self.heading < -2 * math.pi:
            self.heading = 0

    def lap(self, x, y):	
        self.lapcount += 1

class Ultrasonic:
    def __init__(self, sensor_range, map):
        self.sensor_range = sensor_range
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        self.map = map

    def sense_obstacles(self, x, y, heading, sensorOn):
        obstacles = [], []
        x1, y1 = x, y
        start_angle = heading - self.sensor_range[1]
        finish_angle = heading + self.sensor_range[1]
        for angle in np.linspace(start_angle, finish_angle, 30, False):
            x2 = x1 + self.sensor_range[0] * math.cos(angle)
            y2 = y1 - self.sensor_range[0] * math.sin(angle)
            for i in range(0, 100):
                u = i / 100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.map_width and 0 < y < self.map_height:
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 208, 255))
                    if sensorOn == True:
                        if (color[0], color[1], color[2]) == (0, 0, 0):
                            obstacles[0].append([x, y])
                            break
                        elif (color[0], color[1], color[2]) == (0, 0, 200):
                            obstacles[1].append([x, y])
                            break
                    else:
                        if (color[0], color[1], color[2]) == (0, 255, 255):
                            obstacles[0].append([x, y])
                            break
                        elif (color[0], color[1], color[2]) == (255, 128, 0):
                            obstacles[1].append([x, y])
                            break    
        return obstacles

    def sense_obstacles2(self, x, y, heading, sensorOn):
        obstacles = [], []
        x1, y1 = x, y
        start_angle = heading - self.sensor_range[1]
        finish_angle = heading + self.sensor_range[1]
        for angle in np.linspace(start_angle, finish_angle, 100, False):
            x2 = x1 + self.sensor_range[0] * math.cos(angle)
            y2 = y1 - self.sensor_range[0] * math.sin(angle)
            for i in range(0, 100):
                u = i / 100
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.map_width and 0 < y < self.map_height:
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 208, 255))
                    if sensorOn == True:
                        if (color[0], color[1], color[2]) == (0, 0, 0):
                            obstacles[0].append([x, y])
                            break
                        elif (color[0], color[1], color[2]) == (0, 0, 200):
                            obstacles[1].append([x, y])
                            break
                    else:
                        if (color[0], color[1], color[2]) == (0, 255, 255):
                            obstacles[0].append([x, y])
                            break
                        elif (color[0], color[1], color[2]) == (255, 128, 0):
                            obstacles[1].append([x, y])
                            break    
        return obstacles
