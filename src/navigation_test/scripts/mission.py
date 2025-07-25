#usr/bin/python3.7
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped,Point, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8,String
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import math
from move import RT
from actionlib import SimpleActionClient
from loguru import logger
import time
from nav_msgs.srv import GetPlan

class Mission:

    lidar_Mapping = {
        "start":343,#360,
        "end":565#543
        #567
    }
    screen_angle = 87.92 #92.34
    # safe_distance = 0.25
    safe_distance = 0.25

    def __init__(self):
        self.detect_pub:rospy.Publisher = rospy.Publisher("/detect",Int8,queue_size=10)
        self.Lidar_mid:int = 454#446
        self.Pose:PoseWithCovarianceStamped = PoseWithCovarianceStamped()
        self.global_lidar:list = []
        self.res = 2
        self.resolution:float = 360 / 909 
        self.Lidar:list = []
        self.Detect:list = []
        self.client:SimpleActionClient = None 
        self.cancel:bool = False
        self.unReachableCount:int = 0
        self.monitor:bool = False
        self.count = 0
        self.Menu:dict = {
            "Fruit":["apple","nana","melon"],
            "Dessert":["coke","milk","pie"],
            "Vegetable":["tom","pot","pep"]
        }
        # self.lidarImport()
        rospy.loginfo("Waiting for make_plan service...")
        rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.amcl_callback)
        rospy.Subscriber("/scan",LaserScan,self.lidar_callback)
        rospy.Subscriber("/move_base/GlobalPlanner/check_reachable",Int8,self.check_reachable_cb)
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

    def visual_handle(self,ttl=0.5):

        # rospy.sleep(1)

        if self.res == 1:
            self.count += 1 
            self.detect_pub.publish(3)
        else:
            self.count += 1 
            self.detect_pub.publish(1)
        print(f"detect times {self.count}")
        try:
            print("waiting for answer...")
            temp = rospy.wait_for_message("/rknn_result",String,timeout=ttl).data.split("|")
            rospy.wait_for_message("/scan",LaserScan,timeout=ttl)
            logger.info(f"debug: received data: {temp}")
            self.detect_pub.publish(0)
            quantity = int(temp.pop(0))
            self.menu = temp.pop(0)
            for i in range(quantity):
                camPos = temp.pop(0)
                if self.res != 1:
                    camDist = float(temp.pop(0))
                else:
                    camDist = -1
                ratio = (320 - float(camPos)) /640


                Lidar_index = int(((640-float(camPos))/640)*(self.lidar_Mapping["end"]-self.lidar_Mapping["start"]))
                logger.info(f"debug: Lidar_index: {Lidar_index}")
                while self.Lidar[Lidar_index] == float('inf') or self.Lidar[Lidar_index] == 0:
                    print("invalid Lidar data, retrying...")
                    Lidar_index += 1
                tempList = [dis for dis in self.Lidar[Lidar_index-1:Lidar_index+1] if (dis != float('inf') and dis != 0)]
                if tempList:
                    Dist = sum(tempList)/len(tempList)
                else:
                    print("tempList NULL!!!")
                    continue
                logger.info(f"debug: Dist: {Dist}")
                
            


                screen_angle = ratio*self.screen_angle
                logger.info(f"debug: screen_angle: {screen_angle}")

                if self.res != 1 and camDist != -1 and camDist < 1.5 and Dist < 1.5:
                    print(f"camDist: {camDist} LidarOriginal: {Dist}")
                    distCal = math.sqrt(Dist * Dist + 0.065 * 0.065 - 2 * Dist * 0.065 * math.cos(math.radians(screen_angle)))
                    err = abs(distCal - camDist) / camDist
                    print(f"error : {err}")
                    if err > 0.15:
                        print(f"target {i + 1} is fake")
                        logger.info(f"target {i + 1} is fake")
                        continue
                    Dist = camDist
                amcl_angle = euler_from_quaternion(self.orientation)[2]
                logger.info(f"debug: amcl_angle: {math.degrees(amcl_angle)}")
                k = self.least_squares(self.lidar2points(self.global_lidar[Lidar_index + self.lidar_Mapping["start"]-5:Lidar_index+self.lidar_Mapping["start"]+5],screen_angle))
                

                logger.info(f"debug: k: {k}")
                # Dist = self.safe_distance
                if Dist == camDist:
                    (x,y)= (Dist*math.cos(math.radians(screen_angle)) + 0.09 + 0.065,Dist*math.sin(math.radians(screen_angle)))
                else:
                    (x,y)= (Dist*math.cos(math.radians(screen_angle)) + 0.09,Dist*math.sin(math.radians(screen_angle)))
                logger.info(f"debug: original Position x: {x}, y: {y}")
                theta = math.atan(k)
                # if abs(k) < 0.18:
                #     return False
  


                if abs(k) < 0.1:
                    if y > 0:
                        self.r_theta = math.pi/2
                    else:
                        self.r_theta = -math.pi/2
                else:
                    self.r_theta = math.atan(-1/k)


                logger.info(f"debug: r_theta: {math.degrees(self.r_theta)}")
                logger.info(f"debug: theta: {math.degrees(theta)}")
        
                if Dist > 1.5:
                    self.tmp_y = y - 1.2 * math.sin(math.radians(screen_angle))
                    self.tmp_x = x - 1.2 * math.cos(math.radians(screen_angle))
                    self.res = 1
                else:
                    self.tmp_y = y - self.safe_distance * math.sin(self.r_theta)
                    self.tmp_x = x - self.safe_distance * math.cos(self.r_theta)
                    self.res = 0
                # self.tmp_y = y - self.safe_distance * math.sin(self.r_theta)
                # self.tmp_x = x - self.safe_distance * math.cos(self.r_theta)
                # self.res = 0


                logger.info(f"debug: tmp_x: {self.tmp_x}, tmp_y: {self.tmp_y}")

                screen_angle = math.atan(self.tmp_y/self.tmp_x)
                Dist = math.sqrt(self.tmp_y**2 + self.tmp_x**2)
                logger.info(f"debug: new Dist: {Dist}")
                logger.info(f"debug: new screen_angle: {screen_angle}")
                logger.info(f"debug: new local position: x : {(Dist * math.cos(screen_angle)):.2f} , y : {(Dist * math.sin(screen_angle)):.2f}")
                position = Point()
                position.x = self.Pose.pose.pose.position.x + math.cos(amcl_angle+screen_angle) * Dist
                position.y = self.Pose.pose.pose.position.y + math.sin(amcl_angle+screen_angle) * Dist

                if position.x < 0.1:
                    position.x = 0.1
                elif position.x > 2.4:
                    position.x = 2.4
                if position.y < 2.05:
                    position.y = 2.05
                if position.y > 4.4:
                    position.y = 4.4

                if self.res == 1:
                    orientation = quaternion_from_euler(0,0,amcl_angle+screen_angle)
                else:
                    orientation = quaternion_from_euler(0,0,amcl_angle+self.r_theta)
                
                logger.info(f"debug: amcl_pose: {self.Pose.pose.pose.position}")
                logger.info(f"debug: local_pose: x:{(math.cos(amcl_angle+screen_angle) * Dist):.2f} , y:{(math.sin(amcl_angle+screen_angle) * Dist):.2f}")
                self.Detect.append((self.menu,position,orientation,Dist))
                logger.info(f"debug: Detect: {self.Detect}")

                return True
        except rospy.ROSException:
            self.detect_pub.publish(0)
            logger.info("debug: timeout")
            self.res = 2
            return False
        except Exception as e:
            self.detect_pub.publish(0)
            logger.error(f"debug: error occurred: {e}")
            self.res = 2
            return False

    def feedback_cb(self,data):
        if self.cancel:
            self.client.cancel_goal()
            self.cancel = False

    

    def pose_cal(self,last_goal:str):
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
        if last_goal is not None and last_goal != self.target:
            self.target = last_goal
        self.Dist = self.Detect[0][3]
        self.Detect.pop(0)
        self.temp_goal = goal
        return self.target,goal,self.res


    def QR_Decode(self):
        self.detect_pub.publish(4)
        temp = rospy.wait_for_message("/rknn_result",String)
        self.detect_pub.publish(0)
        return temp.data

    def mission_start(self,client:SimpleActionClient,shop:str,goals:list,last_goal:str = None):
        self.shop = shop
        self.client = client
        # if result != 0:
        #     return self.visual_handle(result=result)
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
                if not self.visual_handle():
                    RT.rorate(-30)
                    self.visual_handle()

        if self.Detect:
            return self.pose_cal(last_goal)
        # for goal in goals:

        print("Searching...")
        for idx,goal in enumerate(goals):
            goal[0].target_pose.header.stamp = rospy.Time.now()
            self.client.send_goal(goal[0],feedback_cb=self.feedback_cb)
            self.monitor = True
            self.GoalCount = idx
            if not self.client.wait_for_result():
                continue
            if self.visual_handle():
                return self.pose_cal(last_goal)
            for loop in range(len(goal[1])):
                RT.rorate(goal[1][loop])


                
                if self.visual_handle():
                    return self.pose_cal(last_goal)
               
        


        if not self.Detect:
            logger.info("debug: no detect")
            return None,None


        
    def traffic_light(self,ttl=1):

        rospy.sleep(0.5)
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
            
                
                tmpX = self.Pose.pose.pose.position.x + math.cos(amcl_angle+math.radians(screen_angle)) * Dist
                logger.info(f"debug: amclPose x: {self.Pose.pose.pose.position.x}")
                logger.info(f"debug: tmpX: {tmpX}")
                self.detect_pub.publish(0)
                if tmpX < 3.75:
                    return True
                else:return False
            else:return False
        except rospy.ROSException:
            self.detect_pub.publish(0)
            logger.info("debug: timeout or Red")
            return False

    def check_reachable_cb(self,data):
        if data.data == 1 and self.monitor:
            self.unReachableCount += 1
            if self.unReachableCount >= 3:
                self.cancel = True
                logger.info(f"debug: Goal unreachable! Goal count: {self.GoalCount}")
        else:
            self.unReachableCount = 0
    


    def lidarImport(self):
        return True
        lidar_data = []
        with open('/home/ucar/ucar_ws/src/navigation_test/scripts/lidar.txt', 'r') as file:
            content = file.read()
        
            # 按空格和换行符分割所有数据
            data_strings = content.split()
            
            # 转换为浮点数并添加到列表
            for data_str in data_strings:
                if data_str.strip():  # 确保不是空字符串
                    try:
                        distance = float(data_str)
                        lidar_data.append(distance)
                    except ValueError:
                        lidar_data.append(float('inf'))  # 如果转换失败，添加无穷大
                        continue
        self.lidarImported = lidar_data
        print(f"Lidar data imported, length: {len(self.lidarImported)}")
    @staticmethod
    def CheckFake(data):
        if data == float('inf') or data == 0:
            return True
        return False
    
    def laserLocate(self):
        print("waiting")
        rospy.wait_for_message("/scan",LaserScan,timeout=5)
        # yaw = euler_from_quaternion(self.orientation)[2]
        # if abs(yaw - math.pi/2) / math.pi/2 > 0.1:
        #     RT.rorate(math.degrees(math.pi/2 - yaw))
        
        yDir = 0
        xDir = 0
        best_x = 10
        best_y = 10
        for i in range(454):
            if self.CheckFake(self.global_lidar[i]) or self.CheckFake(self.global_lidar[i+454]):
                continue
            # if self.CheckFake(self.lidarImported[i]) or self.CheckFake(self.global_lidar[i]):
            #     continue
            # if self.CheckFake(self.lidarImported[i+454]) or self.CheckFake(self.global_lidar[i+454]):
            #     continue
            # if abs(self.lidarImported[i] - self.global_lidar[i]) > 0.2:
            #     continue

            temp_x = abs((self.global_lidar[i] + self.global_lidar[i+454]) * math.cos(math.radians((i-454)*360/909)))
            temp_y = abs((self.global_lidar[i] + self.global_lidar[i+454]) * math.sin(math.radians((i-454)*360/909)))             
            if abs(temp_x - 6) / 6 < 0.05:
                # if temp_x < best_x:
                    xDir = i
                    best_x = temp_x
                    print(f"x :{best_x}")
                    
            elif abs(temp_y - 5) / 5 < 0.05:
                # if temp_y < best_y:
                    yDir = i
                    best_y = temp_y
                    print(best_y)

        if xDir > 227:
            xBias = - (self.global_lidar[xDir] - self.global_lidar[xDir + 454])* abs(math.cos(math.radians((xDir-454)*360/909)))
        else:
            xBias = (self.global_lidar[xDir] - self.global_lidar[xDir + 454])* abs(math.cos(math.radians((xDir-454)*360/909)))                
        yBias = (self.global_lidar[yDir] - self.global_lidar[yDir + 454])* abs(math.sin(math.radians((yDir-454)*360/909)))
        xBias /= 2
        yBias /= 2
        print(f"data Index: xDir:{xDir}, yDir:{yDir}")
        print(f"xBias-0.25: {xBias + 0.25}, yBias: {yBias}")







# rospy.init_node("test")
msi = Mission()


if __name__ == "__main__":
    # sub = rospy.Subscriber("/move_base/GlobalPlanner/check_reachable",Int8,msi.check_reachable_cb)
    # client = SimpleActionClient("move_base",MoveBaseAction)
    # client.wait_for_server()
    # goal = MoveBaseGoal()
    # goal.target_pose.header.frame_id = "map"
    # goal.target_pose.header.stamp = rospy.Time.now()
    # goal.target_pose.pose.position.x = 2
    # goal.target_pose.pose.position.y = 0
    # goal.target_pose.pose.position.z = 0
    # goal.target_pose.pose.orientation.x = 0
    # goal.target_pose.pose.orientation.y = 0
    # goal.target_pose.pose.orientation.z = 0
    # goal.target_pose.pose.orientation.w = 1
    # client.send_goal(goal)
    # print("send goal")
    # rospy.spin()
    # client.wait_for_result()
    # # msi.Timer.shutdown()
    # print(msi.unReachableCount)

    msi.laserLocate()
