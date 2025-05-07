#usr/bin/python3.7
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped,Point
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
        "start":342,
        "end":569
        #567
    }
    screen_angle = 92.34
    safe_distance = 0.05


    def __init__(self):
        self.visual_pub = rospy.Publisher("/rknn_target",String,queue_size=10)
        self.detect_pub = rospy.Publisher("/detect",Int8,queue_size=10)
        self.Lidar_mid = 446
        self.Pose = PoseWithCovarianceStamped
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

    def visual_handle(self,ttl=1):
        self.detect_pub.publish(1)
        try:
            temp = rospy.wait_for_message("/rknn_result",String,timeout=ttl).data.split("|")
            logger.info(f"debug: received data: {temp}")
            if temp[0] in self.Menu[self.shop]:
                self.menu = temp[0]
                ratio = (320 - float(temp[1])) /640
                Lidar_index = int(((640-float(temp[1]))/640)*(self.lidar_Mapping["end"]-self.lidar_Mapping["start"]))
                while self.Lidar[Lidar_index] == float('inf'):
                    Lidar_index += 1
                Dist = self.Lidar[Lidar_index]
                logger.info(f"debug: Dist: {Dist}")
                screen_angle = ratio*self.screen_angle
                logger.info(f"debug: screen_angle: {math.radians(screen_angle)}")
                amcl_angle = euler_from_quaternion(self.orientation)[2]
                logger.info(f"debug: amcl_angle: {amcl_angle}")
                k = self.least_squares(self.lidar2points(self.global_lidar[Lidar_index + self.lidar_Mapping["start"]-6:Lidar_index+self.lidar_Mapping["start"]+6],screen_angle))
                logger.info(f"debug: k: {k}")
                # Dist = self.safe_distance
                (x,y)= (Dist*math.cos(math.radians(screen_angle)) + 0.11,Dist*math.sin(math.radians(screen_angle)))
                logger.info(f"debug: original Position x: {x}, y: {y}")
                theta = math.atan(k)
                if k == 0:
                    if y > 0:
                        r_theta = math.pi/2
                    else:
                        r_theta = -math.pi/2
                else:
                    r_theta = math.atan(-1/k)
                logger.info(f"debug: r_theta: {math.degrees(r_theta)}")
                logger.info(f"debug: theta: {math.degrees(theta)}")
                # if Lidar_index + self.lidar_Mapping["start"] >= self.Lidar_mid:#quadrant 1
                #     tmp_y = y + self.safe_distance * math.sin(r_theta)
                #     tmp_x = x - self.safe_distance * math.cos(r_theta)
                # else:
                #     tmp_y = y - self.safe_distance * math.sin(r_theta)
                #     tmp_x = x - self.safe_distance * math.cos(r_theta)
                tmp_y = y - self.safe_distance * math.sin(r_theta)
                tmp_x = x - self.safe_distance * math.cos(r_theta)

                screen_angle = math.atan(tmp_y/tmp_x)
                Dist = math.sqrt(tmp_y**2 + tmp_x**2)
                logger.info(f"debug: new Dist: {Dist}")
                logger.info(f"debug: new screen_angle: {screen_angle}")
                logger.info(f"debug: new local position: x : {(Dist * math.cos(screen_angle)):.2f} , y : {(Dist * math.sin(screen_angle)):.2f}")
                position = Point()
                position.x = self.Pose.pose.pose.position.x + math.cos(amcl_angle+screen_angle) * Dist
                position.y = self.Pose.pose.pose.position.y + math.sin(amcl_angle+screen_angle) * Dist

                if position.x < 0.2:
                    position.x = 0.2
                elif position.x > 2.1:
                    position.x = 2.1
                if position.y < 2.0:
                    position.y = 2.0

                orientation = quaternion_from_euler(0,0,amcl_angle+r_theta)
                logger.info(f"debug: amcl_pose: {self.Pose.pose.pose.position}")
                logger.info(f"debug: local_pose: x:{(math.cos(amcl_angle+screen_angle) * Dist):.2f} , y:{(math.sin(amcl_angle+screen_angle) * Dist):.2f}")
                self.Detect.append((temp[0],position,orientation))
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
        return self.Detect[0][0],goal

def mission_start(client:SimpleActionClient,shop:str,goals:list):
    msi = Mission()
    logger.add("/home/ucar/ucar_ws/src/navigation_test/scripts/log/debug.log",
               level="INFO",
               format="{time} {level} {message}",
               filter=lambda record: "debug" in record["message"].lower())
    logger.error("-----------------------------------------------------------------")
    msi.shop = shop

    rospy.sleep(1)
    msi.visual_pub.publish(shop)
    msi.visual_handle(ttl = 0.5)
    # exit()
    # _,goal = msi.pose_cal()
    # client.send_goal(goal)
    # client.wait_for_result()
    # if client.get_result():
    #     logger.info("debug: goal reached")
    # return 0
    for goal in goals:
        if not msi.visual_handle():
            RT.rorate(-30)
            if not msi.visual_handle():
                RT.rorate(60)
                msi.visual_handle()
        if msi.Detect:
            return msi.pose_cal()
        goal.target_pose.header.stamp = rospy.Time.now()
        client.send_goal(goal)
        client.wait_for_result()
    if not msi.Detect:
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

    
def traffic_light():
    ...


    






if __name__ == "__main__":
    logger.remove()
    # logger.add("/home/ucar/ucar_ws/src/navigation_test/scripts/log/debug.log",
    #            level="INFO",
    #            format="{time} {level} {message}",
    #            filter=lambda record: "debug" in record["message"].lower())
    rospy.init_node("test")
    client = SimpleActionClient("move_base",MoveBaseAction)
    mission_start(client,"Fruit",[])

    # RT.rorate(30)
    # time.sleep(1)
    # RT.rorate(-30)
