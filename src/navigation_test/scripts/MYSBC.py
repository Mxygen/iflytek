#!/usr/bin/python3.7
# coding=UTF-8
import math
import time
import os
import datetime
import json
from math import fabs
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys, select, termios, tty
import threading
import asyncio
import gc
 
from navigation_test import cap_flag


class SBController(object):
    # 内部类
    class Btn(object):
        def __init__(self, name):
            # if name == '' or name not in SBController.KEYMAP['btns']:
            #     raise ValueError("Check keyname! keyname must in 'KEYMAP' and not ''.")
            self.name = name
            self.keymap = SBController.KEYMAP['btns'].index(name)
            self.sta = False
            self.bak = self.sta

    class Axe(object):
        def __init__(self, name):
            # if name == '' or name not in SBController.KEYMAP['axes']:
            #     raise ValueError("Check keyname! keyname must in 'KEYMAP' and not ''.")
            self.name = name
            self.keymap = SBController.KEYMAP['axes'].index(name)
            self.sta = 0

    ####################################################################################################################
    # KEYMAP = {'axes': ['L_x', 'L_y', 'LT', 'R_x', 'R_y', 'RT', 'Cross_x', 'Cross_y'],
    #           'btns': ['A', 'B', 'X', 'Y', 'LB', 'RB', 'SELECT', 'START', 'Home','M1','M2']}#BYL
    Now_Day = str(datetime.date.today())
    #bese_dirver自带joy模式
    #十字上下：调整线速度   十字左右:调整角速度    
    #A  :CMD模式     B    :JOY模式            
    #左摇杆  ：平移   LT旋转                    
    #解决：base_driver中取消订阅joy
    KEYMAP = {'axes': ['L_x', 'L_y', 'LT', 'R_x', 'R_y', 'RT', 'Cross_x', 'Cross_y', '', ''],  #
              'btns': ['A', 'B', 'X', 'Y', 'LB', 'RB', 'BACK', 'START', '', '', '', '']}  # HWS_X9
    AXES_NUM = KEYMAP['axes'].__len__()
    BTNS_NUM = KEYMAP['btns'].__len__()

    CLASS_NAME = [
        "sweet_1", "sweet_2", "sweet_3",
        "fruit_1", "fruit_2", "fruit_3", "vegetable_1", "vegetable_2",
        "vegetable_3","green", "red",
        # , "baton_4", "baton_5",
        # "body_armor_1", "body_armor_2", "body_armor_3", "body_armor_4", "body_armor_5",
        # "teargas_1", "teargas_2", "teargas_3", "teargas_4", "teargas_5",
        "Mix",
    ]
    CLASS_CNT = {
        "sweet_1": 0, "sweet_2": 0, "sweet_3": 0,
        "fruit_1": 0, "fruit_2": 0, "fruit_3": 0, "vegetable_1": 0,
        "vegetable_2": 0,"vegetable_3": 0,"green":0,"red":0,
        #"baton_1": 0, "baton_2": 0, "baton_3": 0, "baton_4": 0, "baton_5": 0,
        # "body_armor_1": 0, "body_armor_2": 0, "body_armor_3": 0, "body_armor_4": 0, "body_armor_5": 0,
        #"teargas_1": 0, "teargas_2": 0, "teargas_3": 0, "teargas_4": 0, "teargas_5": 0,
        "Mix":0,
    }
    CLASS_NUM = CLASS_NAME.__len__()
    LRxy = {'L_x': 'L', 'L_y': 'L', 'R_x': 'R', 'R_y': 'R'}
    keyBinding = {
        'L': [0, 1, 0],  # 左摇杆移动 (-2,2)       #x**2+y**2<=1
        'R': [0, 2, 0],  # 右摇杆转向 (-3,3)
        'A': [False, 0, 0],  # 拍照
        'B': [0, 0, 0],  # 开启显示
        'X': [0, 0, 0],  # 视频录制
        'Y': [0, 0, 0],  # 记录json
        'LB': [0, 0, 0],  # 减速
        'RB': [0, 0, 0],  # 加速
        'LT': [0, 0, 0],  # 切换调速对象
        'RT': [0, 0, 0],  # 按住拍照
        'Cross_y': [0, 0, 0],  # 切换类别
        'Cross_x': [5, 1, 0],  # 自动拍照频率加减|单次拍照数加减#'SELECT'切换
        'BACK': [0, 0, 0],  # 模式选择(-1:仅控制模式, 0:默认模式(按A开启拍照,再按关闭))
        'START':[0,0,0]
    }

    P_x = {'L': [0, 1, 2, 3], 'R': [0, 1, 2, 3], 'Z': [-1.57, -1, 0, 1 ,1.57 , 2, 3.14]}
    P_y = {'L': [0, 0.5, 1, 1.5], 'R': [0, 1, 2, 3], 'Z': [-1.7 ,-1.2 , 0 ,1.2 ,1.7 ,2 , 3]}

    
    # Polyfit = {'L': np.polyfit(P_x['L'], P_y['L'], 2), 'R': np.polyfit(P_x['L'], P_y['L'], 2),
    #        'Z': np.polyfit(P_x['Z'], P_y['Z'], 2)}
    Polyfit = {'L': np.polyfit(P_x['L'], P_y['L'], 2), 'R': np.polyfit(P_x['R'], P_y['R'], 2),
               'Z': np.polyfit(P_x['Z'], P_y['Z'], 2)}
    #Changed

    def P_2(self, a, b, c, d):
        return a * d ** 2 + b * d + c  # 三次拟合

    dead_area_p = {'L_x': (0, 0.1), 'L_y': (0, 0.1), 'R_x': (0, 0), 'R_y': (0, 0),
                   'LT': (0, 0), 'RT': (0, 0), 'Cross_x': (0, 0), 'Cross_y': (0, 0)}  # 实验室手柄死区不小
    axe_p = {'L': 0.4 ** (1 / 2), 'R': 0.25 ** (1 / 2)}        #左右摇杆速度档位
    lim = {'L': [-3, 3], 'R': [-3, 3]}

    TwistMsg = Twist()

    OVER=0#关闭
    img_path = '/home/ucar/Desktop/ucar_img'
    
    ####################################################################################################################
    def __init__(self):
        self.today_date = datetime.datetime.now().strftime("%Y_%m_%d")
        self.img_path = os.path.join(self.img_path, self.today_date)
        for i in self.CLASS_NAME:
            path=os.path.join(self.img_path, i)
            if not os.path.exists(path):
                os.makedirs(path)
        self.json_path = os.path.join(self.img_path, self.today_date + '.json')
        try:
            print("reading json...")
            with open(self.json_path, 'r') as file:
                self.CLASS_CNT.update(json.load(file))
        except FileNotFoundError:
            print(f"Warning: {self.json_path} not found. Starting with empty counts.")
        except json.JSONDecodeError:
            print(f"Error: {self.json_path} is not a valid JSON file. Check the file content.")


        self.node = rospy.init_node('SB_control')
        # rospy.on_shutdown(self.shutdown)
        self.axes = [self.Axe('L_x'),  # 横向移动
                     self.Axe('L_y'),  # 纵向移动
                     self.Axe('R_x'),  #
                     self.Axe('R_y'),  # 转向
                     self.Axe('Cross_y'),
                     self.Axe('Cross_x'),
                     self.Axe('LT'),  # 切换调速
                     self.Axe('RT'),]
        self.btns = [self.Btn('RB'),  # 减挡
                     self.Btn('LB'),  # 加挡
                     self.Btn('A'),  #
                     self.Btn('B'),  #
                     self.Btn('X'),
                     self.Btn('Y'),
                     self.Btn('START'), 
                     self.Btn('BACK'),
                     ]
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.mov_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.image_pub = rospy.Publisher('/camera/image', Image, queue_size=1)
        # rospy.init_node("camera_publisher")
        ##############################################
        if cap_flag:
            print("open cap...")
            self.cap = cv2.VideoCapture(0) 
            self.cap.set(10, 0.6)
            self.cap.set(11, 0.51)
            self.cap.set(12, 0.5)
            self.cap.set(13, 0.5)
            if not self.cap.isOpened():
                print("CAP isnt opened")
                self.keyBinding['BACK'][0] = -1
            ##############################################
            print("OK")

    def GSPhoto(self):
        while 1:
            ref, frame = self.cap.read()
            cv2.imshow("Camera", frame)
            cv2.waitKey(100) 
 
            if not ref:
                time.sleep(0.1)
                continue

            flipped_frame = cv2.flip(frame, 1)
                # 发布图像数据
            # try:
            #     image_message = self.bridge.imgmsg_to_cv2(frame, encoding="bgr8")
            #     self.image_pub.publish(flipped_frame)
            # except Exception as e:
            #     rospy.logerr(f"图像转换错误: {str(e)}")
            #拍摄保存        
            if self.keyBinding['A'][0] or self.keyBinding['B'][0]:
                class_name = self.CLASS_NAME[self.keyBinding['Cross_y'][0]]
                i = self.CLASS_CNT[class_name]
                # global text
                # if  self.keyBinding['X'][0]:
                #     end = cv2.getTickCount()
                #     out.write(flipped_frame)
                #     T = (end - start) / cv2.getTickFrequency()
                #     text = " {:.2f}s".format(T)
                #     print("录制时间：{:.2f}s\r".format(T), end='')
                # else:
                text = str(i)
                success = cv2.imwrite(self.img_path + '/' + class_name + '/' + class_name + '_' + text + ".jpg", flipped_frame)
                if not success:
                    print("Failed to save image")
                else:
                    print("拍摄：" + str(i) + "\r", end='')
                    self.CLASS_CNT[class_name] +=1
                del frame,flipped_frame
                gc.collect() 
                time.sleep(0.5)
                continue
            pass
            print('',end='')
            if self.OVER:
                self.cap.release()
                break
            time.sleep(0.2)
            if 'frame' in locals():del frame
            if 'flipped_frame' in locals():del flipped_frame
            gc.collect() 
    def save_json(self):
        if os.path.exists(self.json_path):
            with open(self.json_path, 'r+') as file:
                data = json.load(file)
                data.update(self.CLASS_CNT)
                file.seek(0)
                json.dump(data, file, indent=4)
        else:
            with open(self.json_path, 'w') as file:
                json.dump(self.CLASS_CNT, file, indent=4)
        print('Json is saved')

    def on_btn_pressed(self, btn):
        if btn.name == 'LB':  # 减速
            if self.keyBinding['LT'][0] == 0: 
                self.keyBinding['L'][1] -= 1
                if self.keyBinding['L'][1] < 0:
                    self.keyBinding['L'][1] = 0
                self.axe_p['L'] = (0.4 * (1 + self.keyBinding['L'][1])) ** (1 / 2)  #L/R[1]速度档位
            else:
                self.keyBinding['R'][1] -= 1
                if self.keyBinding['R'][1] < 0:
                    self.keyBinding['R'][1] = 0
                self.axe_p['R'] = (0.5 * (1 + self.keyBinding['R'][1])) ** (1 / 2)
        elif btn.name == 'RB':  # 加速
            if self.keyBinding['LT'][0] == 0:
                self.keyBinding['L'][1] += 1
                if self.keyBinding['L'][1] > 3:
                    self.keyBinding['L'][1] = 3
                self.axe_p['L'] = (0.4 * (1 + self.keyBinding['L'][1])) ** (1 / 2)
            else:
                self.keyBinding['R'][1] += 1
                if self.keyBinding['R'][1] > 3:
                    self.keyBinding['R'][1] = 3
                self.axe_p['R'] = (0.5 * (1 + self.keyBinding['R'][1])) ** (1 / 2)
        elif btn.name == 'A':
            self.keyBinding['A'][0] = not self.keyBinding['A'][0]

        elif btn.name =='Y':
            SJ = threading.Thread(target=self.save_json, args=())
            SJ.start()
            del SJ
            gc.collect() 
        elif btn.name == 'BACK' and not self.keyBinding['BACK'][0] == -1:
            self.keyBinding['BACK'][0] = not self.keyBinding['BACK'][0]

        elif btn.name == 'START' :
            print("\nOVER")
            self.OVER=1
        elif btn.name == 'B':
            self.keyBinding['B'][0] = 1
        pass

    def on_btn_released(self, btn):
        if btn.name == 'RT':
            self.keyBinding['RT'][0] = 0
        elif btn.name == 'B':
            self.keyBinding['B'][0] = 0
        pass

    def axes_update(self, axe):
        if axe.name == 'LT':  # 切换调速对象
            self.keyBinding['LT'][0] = not self.keyBinding['LT'][0]
        # elif axe.name == 'RT':  # 扳机，爽
        #     self.keyBinding['RT'][0] = 1
        elif 'L' in axe.name:
            axe.sta = self.axe_p['L'] * self.P_2(self.Polyfit['L'][0], self.Polyfit['L'][1], self.Polyfit['L'][2],
                                                 axe.sta)  # 曲线映射调整
            if axe.sta > self.lim['L'][1]:
                axe.sta = self.lim['L'][1]
            elif axe.sta < self.lim['L'][0]:
                axe.sta = self.lim['L'][0]
        elif 'R' in axe.name:
            axe.sta = self.axe_p['R'] * self.P_2(self.Polyfit['R'][0], self.Polyfit['R'][1], self.Polyfit['R'][2],
                                                 axe.sta)


        elif axe.name == 'Cross_y'and not axe.sta==0:  # 更改类别
            a = 0
            if axe.sta == 1:
                a = self.keyBinding['Cross_y'][0] + 1
            elif axe.sta == -1:
                a = (self.keyBinding['Cross_y'][0] - 1) if (self.keyBinding['Cross_y'][0] - 1) > 0 else 0
            if not a % self.CLASS_NUM==self.keyBinding['Cross_y'][0]:
                print("类别:"+self.CLASS_NAME[ a % self.CLASS_NUM])
            self.keyBinding['Cross_y'][0] = a % self.CLASS_NUM


        elif axe.name == 'Cross_x' and not axe.sta==0:  #
            b=0
            if self.keyBinding['BACK'][0]:  # 拍照次数
                if axe.sta == -1:
                    b = (self.keyBinding['Cross_x'][1] + 1)
                elif axe.sta == 1:
                    b = (self.keyBinding['Cross_x'][1] - 1) if (self.keyBinding['Cross_x'][0] - 1) > 1 else 1
                self.keyBinding['Cross_x'][1] = b
            else:  # 拍照频率
                if axe.sta == -1:
                    b = (self.keyBinding['Cross_x'][0] + 1)
                elif axe.sta == 1:
                    b = (self.keyBinding['Cross_x'][0] - 1) if (self.keyBinding['Cross_x'][0] - 1) > 0 else 0
                self.keyBinding['Cross_x'][0] = b 

        if fabs(axe.sta) <= self.dead_area_p[axe.name][1] - self.dead_area_p[axe.name][0]:
            axe.sta = 0  # 死区处理


    ####################################################################################################################
    def calculate_Z(self):
        Z = math.atan2(self.axes[3].sta, self.axes[2].sta)
        if Z >= math.pi / 2:
            Z = -(Z - math.pi / 2)
        elif Z < -math.pi / 2:
            Z = -3 / 2 * math.pi - Z
        elif math.pi / 2 > Z >= 0:
            Z = math.pi / 2 - Z
        elif -math.pi / 2 <= Z < 0:
            Z = -(Z - math.pi / 2)
        # if Z>=math.pi/2 :
        #     Z=Z-math.pi/2
        # elif  Z<-math.pi/2:
        #     Z=3/2*math.pi+Z
        # elif math.pi/2>Z>=0:
        #     Z=-math.pi/2+Z
        # elif -math.pi/2<=Z<0:
        #     Z=Z-math.pi/2
        if fabs(Z) < 0.001:
            Z = 0
        Z = (self.axes[2].sta ** 2 + self.axes[3].sta ** 2) ** (1 / 2) * self.P_2(
            self.Polyfit['Z'][0], self.Polyfit['Z'][1], self.Polyfit['Z'][2], Z)
        if fabs(Z) < 0.01:
            Z = 0
        if Z > self.lim['R'][1]:
            Z = self.lim['R'][1]
        elif Z < self.lim['R'][0]:
            Z = self.lim['R'][0]
        return Z

    ####################################################################################################################
    def joy_callback(self, joy_msg):
        for btn in self.btns:
            btn.sta = joy_msg.buttons[btn.keymap]
            if btn.sta != btn.bak:
                if btn.sta:
                    self.on_btn_pressed(btn)
                else:
                    self.on_btn_released(btn)
            btn.bak = btn.sta
        for axe in self.axes:
            axe.sta = joy_msg.axes[axe.keymap]
            self.axes_update(axe)
        # self.keyBinding['L'] = self.axes[0].sta ** 2 + self.axes[1].sta ** 2
        # self.keyBinding['R'] = self.axes[2].sta ** 2 + self.axes[3].sta ** 2

    def KeyAction(self):
        while 1:
            self.TwistMsg.linear.x = self.axes[1].sta
            self.TwistMsg.linear.y = self.axes[0].sta
            self.TwistMsg.linear.z = 0
            self.TwistMsg.angular.x = 0
            self.TwistMsg.angular.y = 0
            self.TwistMsg.angular.z = self.calculate_Z()
            #print(self.TwistMsg)
            self.mov_pub.publish(self.TwistMsg)#cmd_Vel
            time.sleep(0.01)
            if self.OVER:
                self.mov_pub.publish(Twist())
                rospy.signal_shutdown("shutdown")  
                break



if __name__ == "__main__":
    SB = SBController()
    print(f"Value of cap_flag: {cap_flag}") # 添加这行来检查 cap_flag 的值
    t1 = threading.Thread(target=SB.KeyAction, args=())
    if cap_flag:
        t2 = threading.Thread(target=SB.GSPhoto, args=())
        t2.start()
    t1.start()



'''
Author: AyefLev 592162794@qq.com
Date: 2024-06-5 14:08:15
LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
LastEditTime: 2024-06-17 10:22:57
Description:kao,我居然开始产这种级别的屎山哇，自己也看不懂，md得开始学学好好注释了
'''
