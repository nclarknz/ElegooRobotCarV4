import cv2
import time
import lib.elegoolib as elib


# robot = elib.robotcar()
# robot.robot_type = 'Car'
# robot.mpu_type = 'MPU6050'
# robot.motordriver_type = 'TB6612'
# robot.ip = "192.168.4.1"
# robot.port = 100
# robot.connect()
robotvision = elib.robotVision()

while 1:

    # img4 = robot.captureimg()
    # cv2.imwrite("robotcapture2.png",img4)
    # Read the local image
    img4 = cv2.imread("robotcapture2.png")
    img3,imgresult = robotvision.detectYoloObj(img4,"sports ball")  
    cv2.imshow('Local Image', img4)
    cv2.imshow('Object View', img3)
    cv2.waitKey(1) & 0xFF == ord('0')
    print("Sports Ball detected", imgresult)
    # if imgresult.size == 0:
    #     print("No object detected")
    # else:
    for boxres in imgresult:
        [x1, y1, x2, y2] = boxres.xyxy[0]
        # convert to int
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        # Now send the robot that way
        xh, yh = (x1 + x2) // 2, (y1 + y2) // 2
        # robot.move_to(xh, yh)
        robotvision.findobject(imgresult)






    time.sleep(1)
# cv2.imshow('Object View', img3)   
# cv2.waitKey(0)
cv2.destroyAllWindows()