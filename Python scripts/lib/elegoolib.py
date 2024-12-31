import numpy as np
import cv2
from urllib.request import urlopen
import socket
import sys
import json
import re
import imutils
from collections import deque
import time
from ultralytics import YOLO

class ElegooPlatform:
    def __init__(self):
        """
    Filters a list of products by a given attribute and minimum value, and then sorts the filtered products by the attribute.

    Args:
        ip The IP address of the robot (Usually 192.168.1.4).
        port The port number to connect to on the robot (Usually 100).
        robot_type The type of robot (self._car or Tank).
        mpu_type The type of MPU (MPU6050 or QMI8658C).
        motordriver_type The type of motor driver (TB6612 or DRV8835).
    """
        self.ip = ""
        self.port = 100
        self._car = socket.socket()
        self._cmd_no = 0
        self._connected = False
        self.img = None
        self.speed = 100 # Set speed of car
        self._urltoopen = 'http://' + self.ip + '/capture'
        self._off = [0.007,  0.022,  0.091,  0.012, -0.011, -0.05]
        self.robot_type = None  # Car or Tank
        self.mpu_type = None  # MPU6050 or QMI8658C
        self.motordriver_type = None  # TB6612 or DRV8835

    def connect(self):
        print('Connect to {0}:{1}'.format(self.ip, self.port))
        try:
            self._car.connect((self.ip, self.port))
        except:
            print('Error: ', sys.exc_info()[0])
            sys.exit()
        print('Connected!')
        self._connected = True
        
        #%% Read first data from socket
        # print('Receive from {0}:{1}'.format(self.ip, self.port))
        # try:
        #     data = self._car.recv(1024).decode()
        # except:
        #     print('Error: ', sys.exc_info()[0])
        #     sys.exit()
        # print('Received: ', data)

        return self._connected
    
    def capturestream(self):
        self._cmd_no += 1
        self._urltoopen = 'http://' + self.ip + ':81/stream'

    def captureimg(self):
        self._cmd_no += 1
        self._urltoopen = 'http://' + self.ip + '/capture'
        print(str(self._cmd_no) + ': capture image')
        url_response = urlopen(self._urltoopen)
        img_array = np.array(bytearray(url_response.read()), dtype=np.uint8)
        img = cv2.imdecode(img_array, -1)
        try:
            # cv2.imshow('Camera', img)
            # cv2.waitKey(1) & 0xFF == ord('0')
            self.img = img
        except:
            print('Error: ', sys.exc_info()[0])
            return "error"
        return self.img

    def move_fwd(self, speed):
        return self._cmd('move', 'forward', '', speed)
    
    def move_back(self, speed):
        return self._cmd('move', 'back', '', speed)
    
    def move_left(self, speed):
        return self._cmd('move', 'left', '', speed)

    def move_right(self, speed):
        return self._cmd('move', 'right', '', speed)
    
    def motionstop(self):
        return self._cmd('stop')
    
    def rotate_ultra(self, angle):
        return self._cmd('rotate', 'ultrasonic', '', angle)
    
    def rotate_camera(self, direction):
        # if direction == 'up':
        #     direct = 1
        # elif direction == 'down':
        #     direct = 2
        # elif direction == 'left':
        #     direct = 3
        # elif direction == 'right':
        #     direct = 4
 
        return self._cmd('rotate', 'camera','',direction)
    
    def measure_dist(self):
        return self._cmd('measure', 'distance')
    
    def checkoffgnd(self):
        return self._cmd('check')
    
    def measure_gyromotion(self):
        if self.mpu_type == 'MPU6050':
            return self._cmd('measure', 'motion')
        else:
            return "Non Supported Function"
            
    def select_autofollowmode(self):
        return self._cmd('select', 'follow')
    
    def select_obstaclemode(self):
        return self._cmd('select', 'obstacle')
    
    def select_linefollowmode(self):
        return self._cmd('select', 'linetracking')
    
    def check_ir(self,irmodule):
        return self._cmd('readir',irmodule)
    
    def set_param_tracking_sensitivity(self,sensitivity):
        if sensitivity >= 50 or sensitivity <= 1000:
            return self._cmd('setparam','trackingsensor','',sensitivity)
        else:
            return "Sensitivity value out of range - between 50 and 1000"
        
    def set_motor_speed(self,left_motor,right_motor):
        if left_motor >= 0 or left_motor <= 255:
            return "Left motor speed out of range - between 0 and 255"
        elif right_motor >= 0 or right_motor <= 255:
            return "Right motor speed out of range - between 0 and 255"
        else:
            return self._cmd('movespeed','','','',left_motor,right_motor)
        

    def _cmd(self, do, what = '', where = '', at = '',e1 = '',e2 = '',e3 = '',e4 = '',e5 = '',e6 = '',e7 = '',e8 = ''):
        """
    Sends a command to the robot and receives a response.

    Args:
        do (str): The action to perform.
        what (str): The object to act on.
        where (str): The direction to move.
        at (int): The speed or angle to move at.
        e1 (int): Extra parameter 1.
        e2 (int): Extra parameter 2.
        e3 (int): Extra parameter 3.
        e4 (int): Extra parameter 4.
        e5 (int): Extra parameter 5.
        e6 (int): Extra parameter 6.
        e7 (int): Extra parameter 7.
        e8 (int): Extra parameter 8.


    Returns:
        int: The response from the robot.

    Raises:
        ValueError: If the action is invalid.

    Examples:
        >>> _cmd('move', 'car', 'forward', 100)
        1
        >>> _cmd('move', 'car', 'back', 100)
        1
        >>> _cmd('move', 'car', 'left', 100)
        1
        >>> _cmd('move', 'car', 'right', 100)
        1
    """
        # Speed is between 0 and 255
        self._cmd_no += 1
        msg = {"H":str(self._cmd_no)} # dictionary
        if do == 'move':
            msg["N"] = 3
            what = ' car '
            if where == 'forward':
                msg["D1"] = 3
            elif where == 'back':
                msg["D1"] = 4
            elif where == 'left':
                msg["D1"] = 1
            elif where == 'right':
                msg["D1"] = 2
            msg["D2"] = at # at is speed here
            where = where + ' '
        elif do == 'movetime': # at is time here
            msg["N"] = 2
            what = ' car '
            if where == 'forward':
                msg["D1"] = 3
            elif where == 'back':
                msg["D1"] = 4
            elif where == 'left':
                msg["D1"] = 1
            elif where == 'right':
                msg["D1"] = 2
            msg["D2"] = at
        elif do == 'stop':
            msg.update({"N":1,"D1":0,"D2":0,"D3":1})
            what = ' car'
        elif do == 'rotate':
            if what == 'camera':
                msg.update({"N": 106,"D1":at})
                what = ' camera'
            elif what == 'ultrasonic':
                msg.update({"N":5,"D1":1,"D2":at}) # at is an angle here
                what = ' ultrasonic unit'
            elif what == 'servoupdown':
                msg.update({"N":5,"D1":2,"D2":at}) # at is an angle here
                what = ' servo up down'
            where = ' '
        elif do == 'measure':
            if what == 'distance':
                msg.update({"N":21,"D1":2})
            elif what == 'objdetected':
                msg.update({"N":21,"D1":1})
            elif what == 'motion':
                msg["N"] = 6
                what = 'mpu data'
            what = ' ' + what
        elif do == 'check':
            msg["N"] = 23
            what = ' off the ground'
        elif do == 'select':
            msg["N"] = 101
            if what == 'linetracking':
                msg["D1"] = 1
            elif what == 'obstacle':
                msg["D1"] = 2
            elif what == 'follow':
                msg["D1"] = 3
        elif do == 'readir':
            msg["N"] = 22
            if what == 'L':
                msg["D1"] = 0
            elif what == 'M':
                msg["D1"] = 1
            elif what == 'R':
                msg["D1"] = 2
        elif do == 'sound':
            msg["N"] = 6
            msg["D1"] = what
            msg["T"] = at
        elif do == 'light':
            if what == 'matrix':
                msg["N"] = 9
                msg["D1"] = at
                msg["D2"] = e1
                msg["D3"] = e2
                msg["D4"] = e3
                msg["D5"] = e4
                msg["D6"] = e5
                msg["D7"] = e6
                msg["D8"] = e7
            elif what == 'matrixnumber':
                msg["N"] = 10
                msg["D1"] = at
            else:
                msg["N"] = 7
                if what == 'all':
                    msg["D1"] = 0
                elif what == 'left':
                    msg["D1"] = 1
                elif what == 'right':
                    msg["D1"] = 3
                elif what == 'upper':
                    msg["D1"] = 2
                elif what == 'lower':
                    msg["D1"] = 4
                elif what == 'middle':
                    msg["D1"] = 5
                msg["D2"] = where
                msg["D3"] = at
                msg["D4"] = e1
                msg["T"] = e2
        elif do == 'setparam':
            if what == 'trackingsensor':
                msg["N"] = 104
                msg["D1"] = at
            elif what == 'light':
                msg["N"] = 105
                msg["D1"] = at
        elif do == 'motor':
            msg["N"] = 1
            if what == 'left':
                msg["D1"] = 1
            elif what == 'right':
                msg["D1"] = 2       
            elif what == 'all':
                msg["D1"] = 0 
            msg["D2"] = at

            if e1 == 'CW':
                msg["D3"] = 1
            elif e1 == 'CCW':
                msg["D3"] = 2
            else:
                msg["D3"] = 0
        elif do == 'movespeed':
            msg["N"] = 4
            msg["D1"] = e1
            msg["D2"] = e2
       
        msg_json = json.dumps(msg)
        print(str(self._cmd_no) + ': ' + do + what + where + str(at), end = ': ')
        try:
            self._car.send(msg_json.encode())
        except socket.error:
            print("Socket error", sys.exc_info()[0])
            self._connected = False
            return 0
        except:
            print('Error: ', sys.exc_info()[0])
            sys.exit()

        while 1:
            try:
                res = self._car.recv(1024).decode()
                if not res:
                    print('No data received')
                    self._connected = False
                    break
                if '_' in res:
                    break
            except:
                print('Probably disconnected ', sys.exc_info()[0])
                self._connected = False
                break

        if len(res) == 0:
            res = 0
        else:
            res = re.search('_(.*)}', res).group(1)
        
        if res == 'ok' or res == 'true':
            res = 1
        elif res == 'false':
            res = 0
        elif msg.get("N") == 6:
            print("mpu result")
            print(res)
            # res = res.split(",")
            # res = [int(x)/16384 for x in res] # convert to units of g
            # res[2] = res[2] - 1 # subtract 1G from az
            # res = [round(res[i] - self._off[i], 4) for i in range(6)]
        elif not isinstance(res, int):
            res = 0
        else:
            res = int(res)
        return res

    def close(self):
        self._connected = False
        self._car.close()

    def __del__(self):
        self._connected = False
        self._car.close()
        self.close()



class robotcar(ElegooPlatform):
    def __init__(self):
        super().__init__()

    def move_motor(self, motor, dir,speed):
        return self._cmd('motor', motor,'', speed, dir)

    def move_fwd_time(self, time):      # Car only
        raise NotImplementedError("Non Supported Function")

    def move_back_time(self, time):    # Car only
        raise NotImplementedError("Non Supported Function") 
    
    def move_left_time(self, time): # Car only
        raise NotImplementedError("Non Supported Function")
    
    def move_right_time(self, time): # Car only
        raise NotImplementedError("Non Supported Function")


class robottank(ElegooPlatform):
    def __init__(self):
        super().__init__()

    def move_motor(self, motor, speed):
        return self._cmd('motor', motor,'', speed)

    def move_fwd_time(self, time):      # Tank only
        if self.robot_type == 'tank':
            return self._cmd('movetime', 'forward', '', time) 
        else :
            return "Non Supported Function"
    
    def move_back_time(self, time):    # Tank only
        if self.robot_type == 'tank':
            return self._cmd('movetime', 'back', '', time) 
        else :
            return "Non Supported Function"
    
    def move_left_time(self, time): # Tank only
        if self.robot_type == 'tank':
            return self._cmd('movetime', 'left', '', time) 
        else :
            return "Non Supported Function"
    
    def move_right_time(self, time): # Tank only
        if self.robot_type == 'tank':
            return self._cmd('movetime', 'right', '', time) 
        else :
            return "Non Supported Function"
    
    def playsound(self,freq,duration):
        return self._cmd('sound',freq,duration)
    
    def set_light(self,light,rval,gval,bval,duration):
        return self._cmd('light',light,rval,gval,bval,duration)
    
    def linetracking_coordinates(self):
        return self._cmd('retrieve', 'line_cordinates')
    
    def set_light_bright(self,bright):
        return self._cmd('setparam','light','',bright)
    
    def set_matrix_led(self,led_state,row1,row2,row3,row4,row5,row6,row7):
        return self._cmd('light','matrix','',led_state,row1,row2,row3,row4,row5,row6,row7)
    
    def set_matrix_number(self,number):
        return self._cmd('light','matrixnumber','',number)

class robotVision():
    def __init__(self):
        self._origheight = 0
        self._origwidth = 0

    # Function to get class colors
    def getColours(self,cls_num):
        base_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
        color_index = cls_num % len(base_colors)
        increments = [(1, -2, 1), (-2, 1, -1), (1, -1, 2)]
        color = [base_colors[color_index][i] + increments[color_index][i] * 
        (cls_num // len(base_colors)) % 256 for i in range(3)]
        return tuple(color)
        
    def find_ball(self,colobj):
        found = 0
        ang_tol = 10        # tolerance for rotation angle
        ang = [90, ang_tol, 180 - ang_tol] # head rotation angles
        dist = [0, 0, 0]    # measured distances to obstacles at rotation angles
        dist_min = 30       # min distance to obstacle (cm)
        d180 = 90           # eq rotation distance for 180 deg turn
        dturn = 60          # eq rotation distance for smaller than 180 deg turns
        for n in range(2):
            if n == 1:
                if dist[1] > dist[2]:
                    self._cmd(self._car, do = 'move', where = 'right', at = self.speed)
                else:
                    self._cmd(self._car, do = 'move', where = 'left', at = self.speed)
                #time.sleep(d180/self.speed)
                self._cmd(self._car, do = 'stop')
            for i in range(3):
                self._cmd(self._car, do = 'rotate', at = ang[i])
                dist[i] = self._cmd(self._car, do = 'measure', what = 'distance')
                img2,ball, bd, ba_rad, ba_deg = self.find_col_ball(colobj)
                if ball:
                    if ((i == 1 and ba_deg < -ang_tol) or
                        (i == 2 and ba_deg > +ang_tol)):
                        # Rotate head more precisely to ball angle to measure distances
                        um_ang = ang[i] - ba_deg
                        self._cmd(self._car, do = 'rotate', at = um_ang)
                        d = self._cmd(self._car, do = 'measure', what = 'distance')
                        img2,ball, bd, ba_rad, ba_deg = self.find_col_ball(colobj)
                    else:
                        um_ang = ang[i]
                        d = dist[i]
                    if not ball: continue
                    if d > dist_min:
                        found = 1
                        print('found ball: bdist =', round(bd,1), 'dist =', d)
                        self._cmd(self._car, do = 'rotate', at = 90)
                        steer_ang = 90 - um_ang + ba_deg
                        if steer_ang > ang_tol:
                            self._cmd(self._car, do = 'move', where = 'right', at = self.speed)
                        elif steer_ang < -ang_tol:
                            self._cmd(self._car, do = 'move', where = 'left', at = self.speed)
                        print('steering angle =', steer_ang)
                        #time.sleep(dturn/speed*abs(steer_ang)/180)
                        self._cmd(self._car, do = 'stop')
                        #time.sleep(0.5)
                        img2,_, bd, ba_rad, ba_deg = self.find_col_ball(colobj)
                    break
            if found:
                break
        if not found:
            self._cmd(self._car, do = 'rotate', at = 90)

    def find_col_ball(self,colordetect):
        # Filter image by color
        img = self.captureimg()
        mask = cv2.medianBlur(img, 5)
        img_hsv = cv2.cvtColor(mask, cv2.COLOR_BGR2HSV)
        if colordetect == 'red':
            lower = np.array([0, 70, 50], dtype="uint8")
            upper = np.array([10, 255, 255], dtype="uint8")
        elif colordetect == 'green':
            lower = np.array([29, 86, 6], dtype="uint8")
            upper = np.array([64, 255, 255], dtype="uint8")
        elif colordetect == 'blue':
            lower = np.array([110, 50, 50], dtype="uint8")
            upper = np.array([130, 255, 255], dtype="uint8")
        elif colordetect == 'yellow':
            lower = np.array([20, 90, 90], dtype="uint8")
            upper = np.array([30, 255, 255], dtype="uint8")
        elif colordetect == 'orange':
            lower = np.array([5, 50, 50], dtype="uint8")
            upper = np.array([15, 255, 255], dtype="uint8")
        elif colordetect == 'purple':
            lower = np.array([140, 50, 50], dtype="uint8")
            upper = np.array([160, 255, 255], dtype="uint8")
        elif colordetect == 'white':
            lower = np.array([0, 0, 200], dtype="uint8")
            upper = np.array([180, 25, 255], dtype="uint8")
        elif colordetect == 'black':
            lower = np.array([0, 0, 0], dtype="uint8")
            upper = np.array([180, 255, 30], dtype="uint8")
        else:
            lower = np.array([0, 70, 50], dtype="uint8")
            upper = np.array([10, 255, 255], dtype="uint8")
            print('Color not recognized')  
        # lower = np.array([50, 70, 60], dtype="uint8") # 50, 70, 60
        # upper = np.array([90, 255, 255], dtype="uint8") # 90, 200, 230
        mask = cv2.inRange(img_hsv, lower, upper)
        # Detect contours
        mask = cv2.erode(mask, None, iterations = 2)
        mask = cv2.dilate(mask, None, iterations = 2)
        cont = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cont = imutils.grab_contours(cont)
        # Evaluate all contours
        yh = 491        # y-coordinate of line of horizon, contours above it are ignored
        ball = 0        # flag indicating a presence of a ball of the given color
        dist = None     # distance to the ball
        ang_rad = 0     # angle to the ball in rad
        ang_deg = 0     # angle to the ball in deg
        area = 0        # area of contour
        area_max = 20   # contours with area smaller than this will be ignored 
        ncont = len(cont)
        if ncont > 0:
            for n in range(ncont):
                # Find center and area of contour
                M = cv2.moments(cont[n])
                _xc = int(M['m10']/M['m00'])
                _yc = 600 - int(M['m01']/M['m00'])  # make y = 0 at image bottom
                area = M['m00']
                # Find ball with largest area below line of horizon
                if _yc < yh and area > area_max:
                    area_max = area
                    ball = 1
                    nc = n
                    xc = _xc - 400    # make x axis go through image center
                    yc = _yc
                    center = (_xc, 600 - _yc)   # need only for plotting
        # Calculate distance and angle to the ball
        if ball:
            cv2.drawContours(img, cont, nc, (0,0,255), 1)    # draw selected contour
            cv2.circle(img, center, 1, (0,0,255), 2)         # draw center
            cv2.putText(img, '(' + str(xc) + ', ' + str(yc) + ')', center, 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
            dy = 4.31*(745.2 + yc)/(yh - yc)    # distance to ball along y
            if xc < 0: dy = dy*(1 - xc/1848)    # correction factor for negative xc
            dx = 0.00252*xc*dy                  # distance to ball along x
            dist = np.sqrt(dx**2 + dy**2)       # distance to ball
            ang_rad = np.arctan(dx/dy)          # angle to ball in rad
            ang_deg = round(ang_rad*180/np.pi)  # angle to ball in deg
            #print('bd =', round(dist), 'ba =', ang_deg)
        else:
            print('no ball')
        cv2.line(img, (400,0), (400,600), (0,0,255), 1)           # center line
        cv2.line(img, (0,600 - yh), (800,600 - yh), (0,0,255), 1) # line of horizon
        # cv2.imshow('Camera', img)
        # cv2.waitKey(1)
        return img,ball, dist, ang_rad, ang_deg,dx
    

    #%% Track the ball
    def track_ball(self,coltrack):
        img,ball, bd, ba_rad, ba_deg =self.find_col_ball(coltrack)

        if ball:
            # Calculate left and right wheel speeds to reach the ball
            r = bd/(2*np.sin(ba_rad))    # required turning radius
            if r > 0 and r <= 707:  # turn right
                s0 = 1.111
                ra = -17.7
                rb = 98.4
            else:    # turn left or go straight
                s0 = 0.9557  # vl/vr speed ratio to go straight
                ra = 5.86
                rb = -55.9
            speed_ratio = s0*(r - ra)/(r + rb)
            speed_ratio = max(0, speed_ratio)
            if r > 0 and r <= 707:  # turn right
                lspeed = self.speed
                rspeed = round(self.speed*speed_ratio)
            else:                   # turn left or go straight
                lspeed = round(self.speed*speed_ratio)
                rspeed = self.speed
            self._cmd(self._car, do = 'set', at = [rspeed, lspeed])
    
    def detectYolo8Obj(self, frame):
        # Load the model
        yolo = YOLO('yolov8s.pt')
        frame = imutils.resize(frame, width=600)
        results = yolo.track(frame, stream=False)
        for result in results:
        # get the classes names
            classes_names = result.names

            # iterate over each box
            for box in result.boxes:
                # check if confidence is greater than 40 percent
                if box.conf[0] > 0.2:
                    # get coordinates
                    [x1, y1, x2, y2] = box.xyxy[0]
                    # convert to int
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    # get the class
                    cls = int(box.cls[0])

                    # get the class name
                    class_name = classes_names[cls]
                    print("Found object ",class_name, " with confidence ", box.conf[0])
                    
                    # get the respective colour
                    colour = self.getColours(cls)

                    # draw the rectangle
                    cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)

                    # put the class name and confidence on the image
                    cv2.putText(frame, f'{classes_names[int(box.cls[0])]} {box.conf[0]:.2f}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)
        return frame

    def detectYoloObj(self, frame,objtodetect):
        # Load the model
        #cv2.imwrite('robotcapture2.png', frame)
        yolo = YOLO('yolov8m-worldv2.pt')
        self._origheight, self._origwidth, channels = frame.shape
        frame = imutils.resize(frame, width=640)
        listfoundobj = []
        # results = yolo.track(frame, stream=False)
        results = yolo(frame)

        for result in results:
        # get the classes names
            classes_names = result.names

            # iterate over each box
            for box in result.boxes:
                # check if confidence is greater than 40 percent
                if box.conf[0] > 0.05:
                    # get coordinates
                    [x1, y1, x2, y2] = box.xyxy[0]
                    # convert to int
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    xh, yh = (x1 + x2) // 2, (y1 + y2) // 2
                    print("Center of ball at ", xh, yh)
                    # get the class
                    cls = int(box.cls[0])

                    # get the class name
                    class_name = classes_names[cls]
                    print("Found object ",class_name, " with confidence ", box.conf[0])
                    
                    # get the respective colour
                    colour = self.getColours(cls)

                    # draw the rectangle
                    cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)

                    if classes_names[int(box.cls[0])] == objtodetect:
                        print("Found object ",objtodetect, " with confidence ", box.conf[0])
                        listfoundobj.append(box)
                        cv2.circle(frame, (xh, yh), 5, (0, 0, 255), -1)
                    # get the respective colour
                    colour = self.getColours(cls)

                        # draw the rectangle
                    cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
                    # put the class name and confidence on the image
                    cv2.putText(frame, f'{classes_names[int(box.cls[0])]} {box.conf[0]:.2f}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, colour, 2)
        
        return frame,listfoundobj
    

    def findobject(self,imgresult):
        if not imgresult:
            print("No object detected")
        else:
            for boxres in imgresult:
                [x1, y1, x2, y2] = boxres.xyxy[0]
                # convert to int
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                xh, yh = (x1 + x2) // 2, (y1 + y2) // 2
                # Now send the robot that way
                print("Center of ball at ", xh, yh)
                # Distance from center of imgage
                xoff = xh - self._origwidth/2
                yoff = yh - self._origheight/2
                print("Distance from center of image ", xoff, yoff)
                # Now need to calculate the distance to move the robot left or right so the object is in the center of the image
                
                







        
