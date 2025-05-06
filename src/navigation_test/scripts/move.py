#!/usr/bin/python3
# coding=UTF-8

import rospy
import os
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tools import PID_Position
import math


class Rorate:
    def __init__(self,Kp = 3,Ki = 0,Kd = 0.3):
        self.spin_PID = PID_Position(Kp=Kp,Ki=Ki,Kd=Kd,Debug=True)
        self.current_angular_z = 0.0
        self.current_delta_theta = 0.0
        self.control_rate = 50
        self.last_angular_z = 0.0
        self.alpha = 0.6
        rospy.Subscriber("/imu", Imu, self.Imu_callback)    
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def Imu_callback(self,data):
        self.current_angular_z = data.angular_velocity.z
        temp = self.alpha * self.current_angular_z + (1 - self.alpha) * self.last_angular_z
        self.current_delta_theta += temp / self.control_rate
        self.last_angular_z = self.current_angular_z

    def normalize_angle(self, angle):
        """
        将角度归一化到[-pi, pi]范围内
        """ 
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def tmp_clear(self):
        self.spin_PID.clear()
        self.current_delta_theta = 0.0  # 重置累积角度
        self.target_theta = 0.0
        self.current_angular_z = 0.0
        self.current_delta_theta = 0.0
        self.last_angular_z = 0.0

    def rorate(self,theta):
        self.tmp_clear()
        # rospy.init_node('rorate', anonymous=True)


        rospy.wait_for_message("/imu", Imu)

        self.target_theta = theta * math.pi / 180  # 使用更精确的π值
        vel_msg = Twist()
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            print(f"当前转过角度: {self.current_delta_theta*180/math.pi:.2f}°, 目标角度: {theta}°")
            # if abs(self.current_delta_theta - self.target_theta) < 0.03:  # 约3度的误差
            #     vel_msg.angular.z = 0
            #     vel_pub.publish(vel_msg)
            #     break
            vel_msg.angular.z = self.spin_PID.update(self.target_theta, self.current_delta_theta)
            self.vel_pub.publish(vel_msg)
            if abs(self.current_angular_z) < 0.01 and abs(self.current_delta_theta - self.target_theta) < 0.05:
                vel_msg.angular.z = 0
                self.vel_pub.publish(vel_msg)
                break
            rate.sleep()


RT = Rorate()

if __name__ == '__main__':
    rospy.init_node('rorate', anonymous=True)
    # thread = threading.Thread(target=os.system,args=("roslaunch ucar_controller base_driver.launch",))
    # thread.start()
    RT.rorate(-90)
    RT.rorate(90)
    

    ...