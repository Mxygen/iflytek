#usr/bin/python3.7 
import cv2
import numpy as np
import os # 添加 os 模块导入
import rospy
from sensor_msgs.msg import LaserScan
import threading
import math
def test1():
    os.system("roslaunch ucar_controller base_driver.launch")

def test2():
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        print(frame.shape)
        # cv2.imshow("frame", frame)
        key = cv2.waitKey(1)
        if key == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()
def test3():
    rospy.init_node("test")
    rospy.Subscriber("/scan",LaserScan,callback)
    rospy.spin()

number = 0
def callback(data):
    global number
    print(f"\rdata number:{number} data:{data.ranges[number]}",end="",flush=True)

def key_input():
    global number
    # cap = cv2.VideoCapture(0)
    while True:
        # ret,frame = cap.read()
        # cv2.imshow("image",frame)
        cv2.namedWindow("image",cv2.WINDOW_NORMAL)
        key = cv2.waitKey(0)
        if key == ord("q"):
            break
        elif key == ord("a"):
            number += 1
        elif key == ord("d"):
            number -= 1


def test4():
    thread = threading.Thread(target=key_input)
    thread.start()
    test3()


if __name__ == "__main__":
    
    # test4()
    print(math.atan(1))