### Copyright Michael@bots4all
#%% Load modules
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import time
import lib.elegoolib as elib

#%% Clear working space
plt.close('all')

#%% Capture image from camera
cv.namedWindow('Camera')
cv.moveWindow('Camera', 10, 10)


robot = elib.robotcar()
robot.robot_type = 'Car'
robot.mpu_type = 'MPU6050'
robot.motordriver_type = 'TB6612'
robot.ip = "192.168.4.1"
robot.port = 100
robot.connect()
robotvision = elib.robotVision()


def plt_update(self,mot):
        self.ag = np.vstack((self.ag, mot))
        plt.clf()
        for i in range(6):
            plt.plot(self.ag[:,i], label = self.ag_name[i])
        plt.legend(loc = 'upper left')
        plt.pause(0.01)
        plt.show()

#%% Plot MPU data
ag = np.empty([0, 6])
ag_name = ['ax', 'ay', 'az', 'gx', 'gy', 'gz']

#%% Main Variables
speed = 150
ang = [90, 10, 170]
dist = [0, 0, 0]
dist_min = 30

print('Do cmd')
robot.rotate_ultra(10)
robot.rotate_ultra(90)


while 1:
    # Capture camera image
    img = robot.captureimg()
    cv.imshow('Camera', img)
    cv.waitKey(1) & 0xFF == ord('0')

    img3 = robotvision.detectYoloObj(img)  
    cv.imshow('Object View', img3)

    # img2,ball, dist, ang_rad, ang_deg,dx = robot.find_col_ball("yellow")
    # cv.imshow('Camera View', img2)
    # print("Distance to ball ",dist)
    # print("Angle to ball (Radians) ", ang_rad)
    # print("Angle to ball (Deg) ", ang_deg)

    # Check if car was lifted off the ground to interrupt the while loop
    # if robot.checkoffgnd():
    #     print('Car off ground')
    #     break
    # # Get MPU data and plot it

    
    # # Check distance to obstacle
    # dist[0] = robot.measure_dist()
    # print("dist0 is",dist[0])
    
    # if dist[0] <= dist_min:
    #     # Detected an obstacle, stop
    #     robot.motionstop()
    #     # Rotate ultrasonic unit right and left and measure distance
    #     found_dir = 0
    #     for i in range(1,3):
    #         robot.rotatecamera(ang[i])
    #         dist[i] = robot.measure_dist()
    #         # Check measured distance
    #         if dist[i] > dist_min:
    #             found_dir = 1
    #     # Rotate ultrasonic unit straight
    #     robot.rotatecamera(90)
    #     # Choose new direction
    #     if ~found_dir:
    #         robot.move_back(speed)
    #         time.sleep(0.3)
    #     if dist[1] > dist[2]:
    #         robot.move_right(speed)
    #     else:
    #         robot.move_left(speed)
    #     time.sleep(0.3)
    # robot.move_fwd(speed)
    

#%% Close socket
robot.close()