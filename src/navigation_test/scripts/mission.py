#usr/bin/python3.7
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped,Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8,String
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import math
from move import RT
from actionlib import SimpleActionClient
from loguru import logger
import time


class Mission:

    lidar_Mapping = {
        "start":343,#360,
        "end":565#543
        #567
    }
    screen_angle = 87.92 #92.34
    safe_distance = 0.25


    def __init__(self):

        self.detect_pub = rospy.Publisher("/detect",Int8,queue_size=10)
        self.Lidar_mid = 454#446
        self.Pose = PoseWithCovarianceStamped()
        self.global_lidar = []
        self.resolution = 360 / 909 
        self.Lidar = []
        self.Detect = []
        self.Menu = {
            "Fruit":["apple","nana","melon"],
            "Dessert":["coke","milk","pie"],
            "Vegetable":["tom","pot","pep"]
        }
        rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.amcl_callback)
        rospy.Subscriber("/scan",LaserScan,self.lidar_callback)
        logger.remove()
        logger.add("/home/ucar/ucar_ws/src/navigation_test/scripts/log/debug.log",
                level="INFO",
                format="{time} {level} {message}",
                filter=lambda record: "debug" in record["message"].lower())
        logger.error("-----------------------------------------------------------------")


    def least_squares(self,Points:list):
        n = len(Points)
        x_ = sum(Points[i][0] for i in range(n)) / n
        y_ = sum(Points[i][1] for i in range(n)) / n
        xy_sum = sum(Points[i][0]*Points[i][1] for i in range(n))
        x_2 = sum(Points[i][0]**2 for i in range(n))
        k = (xy_sum - n*x_*y_) / (x_2 - n*x_**2)
        return k
    

    def amcl_callback(self,data):
        self.Pose = data
        self.orientation = [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ]

    def lidar_callback(self,data):
        self.Lidar = data.ranges[self.lidar_Mapping["start"]:self.lidar_Mapping["end"]]
        self.global_lidar = data.ranges

    def lidar2points(self,list,base_angle):
        length = len(list)
        new_list = []
        for i in range(length):
            if list[i] == float("inf") or list[i] == 0:
                continue
            temp_angle = math.radians((i-length/2)*self.resolution + base_angle)
            new_list.append((list[i]*math.cos(temp_angle),
                             list[i]*math.sin(temp_angle)))
        return new_list

    def visual_handle(self,ttl=1,result=0):

        rospy.sleep(1)
        self.detect_pub.publish(1)
        try:
            temp = rospy.wait_for_message("/rknn_result",String,timeout=ttl).data.split("|")
            logger.info(f"debug: received data: {temp}")
            if temp[0] in self.Menu[self.shop]:
                rospy.sleep(0.5)
                self.menu = temp[0]
                ratio = (320 - float(temp[1])) /640
                Lidar_index = int(((640-float(temp[1]))/640)*(self.lidar_Mapping["end"]-self.lidar_Mapping["start"]))
                logger.info(f"debug: Lidar_index: {Lidar_index}")
                # while self.Lidar[Lidar_index] == float('inf'):
                #     Lidar_index += 1
                tempList = [dis for dis in self.Lidar[Lidar_index-1:Lidar_index+1] if (dis != float('inf') and dis != 0)]
                Dist = sum(tempList)/len(tempList)
                logger.info(f"debug: Dist: {Dist}")
                screen_angle = ratio*self.screen_angle
                logger.info(f"debug: screen_angle: {screen_angle}")
                amcl_angle = euler_from_quaternion(self.orientation)[2]
                logger.info(f"debug: amcl_angle: {math.degrees(amcl_angle)}")
                k = self.least_squares(self.lidar2points(self.global_lidar[Lidar_index + self.lidar_Mapping["start"]-5:Lidar_index+self.lidar_Mapping["start"]+5],screen_angle))
                
                logger.info(f"debug: k: {k}")
                # Dist = self.safe_distance
                (x,y)= (Dist*math.cos(math.radians(screen_angle)) + 0.09,Dist*math.sin(math.radians(screen_angle)))
                logger.info(f"debug: original Position x: {x}, y: {y}")
                theta = math.atan(k)
                if abs(k) < 0.18:
                    return False
                if k == 0:
                    if y > 0:
                        self.r_theta = math.pi/2
                    else:
                        self.r_theta = -math.pi/2
                else:
                    self.r_theta = math.atan(-1/k)



                logger.info(f"debug: r_theta: {math.degrees(self.r_theta)}")
                logger.info(f"debug: theta: {math.degrees(theta)}")
                # if Lidar_index + self.lidar_Mapping["start"] >= self.Lidar_mid:#quadrant 1
                #     tmp_y = y + self.safe_distance * math.sin(r_theta)
                #     tmp_x = x - self.safe_distance * math.cos(r_theta)
                # else:
                #     tmp_y = y - self.safe_distance * math.sin(r_theta)
                #     tmp_x = x - self.safe_distance * math.cos(r_theta)
            
                if Dist > 1.3:
                    self.tmp_y = y - 0.6 * math.sin(self.r_theta)
                    self.tmp_x = x - 0.6 * math.cos(self.r_theta)
                else:
                    self.tmp_y = y - self.safe_distance * math.sin(self.r_theta)
                    self.tmp_x = x - self.safe_distance * math.cos(self.r_theta)



                logger.info(f"debug: tmp_x: {self.tmp_x}, tmp_y: {self.tmp_y}")

                screen_angle = math.atan(self.tmp_y/self.tmp_x)
                Dist = math.sqrt(self.tmp_y**2 + self.tmp_x**2)
                logger.info(f"debug: new Dist: {Dist}")
                logger.info(f"debug: new screen_angle: {screen_angle}")
                logger.info(f"debug: new local position: x : {(Dist * math.cos(screen_angle)):.2f} , y : {(Dist * math.sin(screen_angle)):.2f}")
                position = Point()
                position.x = self.Pose.pose.pose.position.x + math.cos(amcl_angle+screen_angle) * Dist
                position.y = self.Pose.pose.pose.position.y + math.sin(amcl_angle+screen_angle) * Dist

                if position.x < 0.25:
                    position.x = 0.25
                elif position.x > 2.4:
                    position.x = 2.4
                if position.y < 2.05:
                    position.y = 2.05
                if position.y > 4.4:
                    position.y = 4.4

                orientation = quaternion_from_euler(0,0,amcl_angle+self.r_theta)
                logger.info(f"debug: amcl_pose: {self.Pose.pose.pose.position}")
                logger.info(f"debug: local_pose: x:{(math.cos(amcl_angle+screen_angle) * Dist):.2f} , y:{(math.sin(amcl_angle+screen_angle) * Dist):.2f}")
                self.Detect.append((temp[0],position,orientation,Dist))
                logger.info(f"debug: Detect: {self.Detect}")
                self.detect_pub.publish(0)
                return True
        except rospy.ROSException:
            self.detect_pub.publish(0)
            logger.info("debug: timeout")
            return False
    
    def pose_cal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.Detect[0][1].x
        goal.target_pose.pose.position.y = self.Detect[0][1].y
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation.x = self.Detect[0][2][0]
        goal.target_pose.pose.orientation.y = self.Detect[0][2][1]
        goal.target_pose.pose.orientation.z = self.Detect[0][2][2]
        goal.target_pose.pose.orientation.w = self.Detect[0][2][3]
        self.target = self.Detect[0][0]
        self.Dist = self.Detect[0][3]
        self.Detect.pop(0)
        self.temp_goal = goal
        return self.target,goal,self.Dist


    def QR_Decode(self):
        self.detect_pub.publish(4)
        temp = rospy.wait_for_message("/rknn_result",String)
        self.detect_pub.publish(0)
        return temp.data

    def mission_start(self,client:SimpleActionClient,shop:str,goals:list,result):

        
        self.shop = shop

        if __name__ != "__main__":
            center = goals[0]
        if result != 0:
            return self.visual_handle(result=result)
        # msi.visual_handle()
        # exit()
        # _,goal = msi.pose_cal()
        # client.send_goal(goal)
        # client.wait_for_result()
        # if client.get_result():
        #     logger.info("debug: goal reached")
        # return 0

        if not self.visual_handle():
            RT.rorate(30)

            if not self.visual_handle():
                RT.rorate(-75)

                self.visual_handle()
        if self.Detect:
            return self.pose_cal()
        # for goal in goals:
        center.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(center)
        client.wait_for_result()
        for loop in range(4):
            if self.visual_handle(ttl = 1):
                return self.pose_cal()
            RT.rorate(90)


        for goal in goals[1:]:
            goal.target_pose.header.stamp = rospy.Time.now()
            client.send_goal(goal)
            client.wait_for_result()
            for loop in range(4):
                if self.visual_handle(ttl = 1):
                    return self.pose_cal()
                RT.rorate(90)
        


        if not self.Detect:
            logger.info("debug: no detect")
            return None,None
        # rospy.spin()    

        # goal = MoveBaseGoal()
        # goal.target_pose.header.frame_id = "map"
        # goal.target_pose.header.stamp = rospy.Time.now()
        # goal.target_pose.pose.position.x = msi.Detect[0][1].x
        # goal.target_pose.pose.position.y = msi.Detect[0][1].y
        # goal.target_pose.pose.position.z = 0
        # goal.target_pose.pose.orientation.x = 0
        # goal.target_pose.pose.orientation.y = 0
        # goal.target_pose.pose.orientation.z = 0
        # goal.target_pose.pose.orientation.w = 1

        # return msi.Detect[0][0],goal
                
        # return msi.menu,goal

        
    def traffic_light(self,ttl=1):

        rospy.sleep(1)
        self.detect_pub.publish(2)
        try:
            temp = rospy.wait_for_message("/rknn_result",String,timeout=ttl).data.split("|")
            logger.info(f"debug: received data: {temp}")
            if temp[0] == "green":
                rospy.sleep(0.5)
                ratio = (320 - float(temp[1])) /640
                Lidar_index = int(((640-float(temp[1]))/640)*(self.lidar_Mapping["end"]-self.lidar_Mapping["start"]))
                logger.info(f"debug: Lidar_index: {Lidar_index}")
                
                tempList = [dis for dis in self.Lidar[Lidar_index-1:Lidar_index+1] if (dis != float('inf') and dis != 0)]
                Dist = sum(tempList)/len(tempList)
                logger.info(f"debug: Dist: {Dist}")
                screen_angle = ratio*self.screen_angle
                logger.info(f"debug: screen_angle: {screen_angle}")
                amcl_angle = euler_from_quaternion(self.orientation)[2]
                logger.info(f"debug: amcl_angle: {math.degrees(amcl_angle)}")
        
                (x,y)= (Dist*math.cos(math.radians(screen_angle)) + 0.09,Dist*math.sin(math.radians(screen_angle)))
                logger.info(f"debug: original Position x: {x}, y: {y}")
            
                
                tmpX = self.Pose.pose.pose.position.x + math.cos(amcl_angle+screen_angle) * Dist
                logger.info(f"debug: tmpX: {tmpX}")
                self.detect_pub.publish(0)
                if tmpX < 4:
                    return True
                else:return False
            else:return False
        except rospy.ROSException:
            self.detect_pub.publish(0)
            logger.info("debug: timeout or Red")
            return False


    



msi = Mission()


if __name__ == "__main__":
    rospy.init_node("test")
    # rospy.sleep(1)
    # RT.rorate(30)
    # logger.remove()
    # logger.add("/home/ucar/ucar_ws/src/navigation_test/scripts/log/debug.log",
    #            level="INFO",
    #            format="{time} {level} {message}",
    #            filter=lambda record: "debug" in record["message"].lower())
    pub = rospy.Publisher("/detect",Int8,queue_size=10)
    # pub2 = rospy.Publisher("/rknn_target",String,queue_size=10)

    pub.publish(1)
    # pub2.publish("Fruit")
    client = SimpleActionClient("move_base",MoveBaseAction)
    msi.mission_start(client,"Vegetable",[])
    # rospy.spin()
    # print(None)
    
    # time.sleep(1)

