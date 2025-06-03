#!/usr/bin/python3.7
# coding=UTF-8

import signal
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseFeedback
from std_msgs.msg import String
import actionlib
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Int8
from loguru import logger 
import std_msgs.msg
import os
import sys
import json
import dotenv
import time
import datetime
import pyaudio
import wave
import threading
# from QR_Decode import QR_Decode
from mission import msi
import socket


dotenv.load_dotenv()

# DEBUG = False
cap_flag = False
def loggers_init():
    current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parent_dir = os.path.dirname(current_dir)
    # logger.remove()
    logger.add("/home/ucar/ucar_ws/src/navigation_test/scripts/log/ucar.log", 
                format="{message}",
                level="TRACE",
                filter=lambda record: "ucar" in record["message"].lower()
                )
    logger.add("/home/ucar/ucar_ws/src/navigation_test/scripts/log/user.log", 
                format="{message}",
                level="TRACE",
                filter=lambda record: "user" in record["message"].lower()
                )
    logger.add("/home/ucar/ucar_ws/src/navigation_test/scripts/log/ucar.log",
                format="<red>{message}</red>",
                level="TRACE",
                filter=lambda record: "------------"  in record["message"].lower()
                )
    logger.add("/home/ucar/ucar_ws/src/navigation_test/scripts/log/user.log",
                format="<red>{message}</red>",
                level="ERROR",
                filter=lambda record: "------------"  in record["message"].lower()
                )
    now = time.time()
    now_datetime = datetime.datetime.fromtimestamp(now)
    formatted_time_str = now_datetime.strftime('%Y-%m-%d %H:%M:%S')
    logger.error(f"---------------------------{formatted_time_str}--------------------------------")
    logger.error("_______________#########_______________________ ")
    logger.error("______________############_____________________ ")
    logger.error("______________#############____________________ ")
    logger.error("_____________##__###########___________________ ")
    logger.error("____________###__######_#####__________________ ")
    logger.error("____________###_#######___####_________________ ")
    logger.error("___________###__##########_####________________ ")
    logger.error("__________####__###########_####_______________ ")
    logger.error("________#####___###########__#####_____________ ")
    logger.error("_______######___###_########___#####___________ ")
    logger.error("_______#####___###___########___######_________ ")
    logger.error("______######___###__###########___######_______ ")
    logger.error("_____######___####_##############__######______ ")
    logger.error("____#######__#####################_#######_____ ")
    logger.error("____#######__##############################____ ")
    logger.error("___#######__######_#################_#######___ ")
    logger.error("___#######__######_######_#########___######___ ")
    logger.error("___#######____##__######___######_____######___ ")
    logger.error("___#######________######____#####_____#####____ ")
    logger.error("____######________#####_____#####_____####_____ ")
    logger.error("_____#####________####______#####_____###______ ")
    logger.error("______#####______;###________###______#________ ")
    logger.error("________##_______####________####______________ ")

    

def time_monitor(func):
    """
    time monitor
    """
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        logger.error(f"user----Runtime:{(start_time - Global_controller.global_start_time):.2f}s, {func.__name__} spent {(end_time - start_time):.2f}s")
        print(f"{func.__name__} spent {end_time - start_time}s")
        return result
    return wrapper



class Global_controller:
    Menu = {
        "Fruit":{"apple":4,"nana":2,"melon":5},
        "Dessert":{"coke":3,"pie":10,"milk":5},
        "Vegetable":{"tom":2,"pot":5,"pep":2}
    }
    voice_path = "/home/ucar/ucar_ws/src/navigation_test/.wav"
    Voice = {
        "mission":voice_path + "/mission/mission.wav",
        "simulation":voice_path + "/simulation/",
        "spend":voice_path + "/spend/",
        "shop":voice_path + "/shop/",
        "kinds":voice_path + "/kinds/",
        "fetch":voice_path + "/fetch/get.wav",
        "finish":voice_path + "/finish/finish.wav",
        "crossing":voice_path + "/crossing/",
        "and":voice_path + "/and/and.wav"
    }
    global_start_time = time.time()



    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        self.real_shop = None
        self.virtual_shop = None
        self.config = json.load(open(dotenv.find_dotenv("config.json")))
        self.goals = []
        self.audio = None
        self.audio_player = None
        self.bill = 0
        self.local_addr = (self.config["local_ip"], self.config["local_port"])
        self.target_addr = (self.config["target_ip"], self.config["target_port"])
        self.socket_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_server.bind(self.local_addr)
        self.socket_server.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)
        self.break_pub = rospy.Publisher("/break_flag",Int8,queue_size=10)
        # self.visual_pub = rospy.Publisher("/rknn_target",String,queue_size=10)

        logger.info(f"user:socket server inited target: {self.target_addr} local: {self.local_addr}")
        self.search_goals = []
        for GOAL in self.config["goals"]:
            if "Search_goal" in GOAL.keys():
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = GOAL["Search_goal"]["position"]["x"]
                goal.target_pose.pose.position.y = GOAL["Search_goal"]["position"]["y"]
                goal.target_pose.pose.orientation.z = GOAL["Search_goal"]["orientation"]["z"]
                goal.target_pose.pose.orientation.w = GOAL["Search_goal"]["orientation"]["w"]
                tmp = [goal]
                tmp.append([item for item in GOAL["Search_goal"]["target"].values()])
                self.search_goals.append(tmp)
                self.nav_log(goal,"Search_goal")
            else:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = GOAL["Destination"]["position"]["x"]
                goal.target_pose.pose.position.y = GOAL["Destination"]["position"]["y"]
                goal.target_pose.pose.orientation.z = GOAL["Destination"]["orientation"]["z"]
                goal.target_pose.pose.orientation.w = GOAL["Destination"]["orientation"]["w"]
                self.goals.append(goal)
                self.nav_log(goal,"Destination")
        # if not DEBUG:
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.audio_player = None
        logger.info("move_base server connected")
        logger.debug("----------------------Start navigation---------------------")
        # self.sub_start = rospy.Subscriber("/angle",std_msgs.msg.Int32)
        self.visual_nav_pub = rospy.Publisher("/visual_nav",std_msgs.msg.Int32,queue_size=10)



    def signal_handler(self,signum,frame):
        logger.info("user:signal received , exit")
        exit()

    @time_monitor
    def connect(self,data):
        print("start connect")
        self.socket_server.settimeout(1)
        timeout_count = 0
        while True:
            try:
                self.socket_server.sendto(bytes(data,encoding="utf-8"), self.target_addr)
                data, _ = self.socket_server.recvfrom(10)
                if b"ACK" in data:
                    logger.info("user:socket connected")
                    break
            except socket.timeout:
                timeout_count += 1
                logger.error(f"user:socket timeout {timeout_count} times")

        
        self.socket_server.settimeout(300)
        try:
            data, _ = self.socket_server.recvfrom(20)
            self.simulink_data = data.decode('utf-8').split("|")
            logger.info(f"user:simulink data: {self.simulink_data}")
        except socket.timeout:
            logger.error("user:simulink timeout")
            return False
        return True

        


    def nav_log(self,goal,type):
        goal.target_pose.header.stamp = rospy.Time.now()
        logger.debug(f"ucar: goal: {type} x:{goal.target_pose.pose.position.x:.2f}," + \
                                          f"y:{goal.target_pose.pose.position.y:.2f}," + \
                                          f"z:{goal.target_pose.pose.orientation.z:.2f}," + \
                                          f"w:{goal.target_pose.pose.orientation.w:.2f}")
        logger.debug("----------------------------------------------------------")
    
    @time_monitor
    def navigation(self,goal,TimeOut=None):
        goal.target_pose.header.stamp = rospy.Time.now()
        self.client.send_goal(goal)
        try:
            if TimeOut is not None:
                result = self.client.wait_for_result(rospy.Duration(TimeOut))
            else:
                result = self.client.wait_for_result()
            if result:
                logger.info(f"ucar: goal {goal} reached")
            else:
                logger.error(f"ucar: goal {goal} failed")
            return result
        except rospy.exceptions.TimeoutException:
            logger.error(f"ucar: goal {goal} timeout")
            return result
    @time_monitor
    def audio_play(self):
        if self.audio_player is None:
            self.audio_player= pyaudio.PyAudio()
        paths = self.audio.split("|")
        for path in paths:
            if not os.path.exists(path):
                logger.error(f"user: audio file {path} not found")
                continue
            wf = wave.open(path, 'rb')
            if wf is None:
                return 0

            stream = self.audio_player.open(format=self.audio_player.get_format_from_width(wf.getsampwidth()),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        output=True)
            data = wf.readframes(1024)
            while data:
                stream.write(data)
                data = wf.readframes(1024)
            stream.stop_stream()
            stream.close()

            wf.close()
    def nav_test(self):
        self.audio = self.Voice["mission"]
        for num,goal in enumerate(self.goals):
            # if num == 2:
            #     continue
            # if num >3:
            #     break
            if not num == 1:
                continue
            self.navigation(goal)   
            print("goal reached")
            time.sleep(2)

            
        


'''
......................................&&.........................
....................................&&&..........................
.................................&&&&............................
...............................&&&&..............................
.............................&&&&&&..............................
...........................&&&&&&....&&&..&&&&&&&&&&&&&&&........
..................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&..............
................&...&&&&&&&&&&&&&&&&&&&&&&&&&&&&.................
.......................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&.........
...................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&...............
..................&&&   &&&&&&&&&&&&&&&&&&&&&&&&&&&&&............
...............&&&&&@  &&&&&&&&&&..&&&&&&&&&&&&&&&&&&&...........
..............&&&&&&&&&&&&&&&.&&....&&&&&&&&&&&&&..&&&&&.........
..........&&&&&&&&&&&&&&&&&&...&.....&&&&&&&&&&&&&...&&&&........
........&&&&&&&&&&&&&&&&&&&.........&&&&&&&&&&&&&&&....&&&.......
.......&&&&&&&&.....................&&&&&&&&&&&&&&&&.....&&......
........&&&&&.....................&&&&&&&&&&&&&&&&&&.............
..........&...................&&&&&&&&&&&&&&&&&&&&&&&............
................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&............
..................&&&&&&&&&&&&&&&&&&&&&&&&&&&&..&&&&&............
..............&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&....&&&&&............
...........&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&......&&&&............
.........&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&.........&&&&............
.......&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&...........&&&&............
......&&&&&&&&&&&&&&&&&&&...&&&&&&...............&&&.............
.....&&&&&&&&&&&&&&&&............................&&..............
....&&&&&&&&&&&&&&&.................&&...........................
...&&&&&&&&&&&&&&&.....................&&&&......................
...&&&&&&&&&&.&&&........................&&&&&...................
..&&&&&&&&&&&..&&..........................&&&&&&&...............
..&&&&&&&&&&&&...&............&&&.....&&&&...&&&&&&&.............
..&&&&&&&&&&&&&.................&&&.....&&&&&&&&&&&&&&...........
..&&&&&&&&&&&&&&&&..............&&&&&&&&&&&&&&&&&&&&&&&&.........
..&&.&&&&&&&&&&&&&&&&&.........&&&&&&&&&&&&&&&&&&&&&&&&&&&.......
...&&..&&&&&&&&&&&&.........&&&&&&&&&&&&&&&&...&&&&&&&&&&&&......
....&..&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&...........&&&&&&&&.....
.......&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&..............&&&&&&&....
.......&&&&&.&&&&&&&&&&&&&&&&&&..&&&&&&&&...&..........&&&&&&....
........&&&.....&&&&&&&&&&&&&.....&&&&&&&&&&...........&..&&&&...
.......&&&........&&&.&&&&&&&&&.....&&&&&.................&&&&...
.......&&&...............&&&&&&&.......&&&&&&&&............&&&...
........&&...................&&&&&&.........................&&&..
.........&.....................&&&&........................&&....
...............................&&&.......................&&......
................................&&......................&&.......
.................................&&..............................
..................................&..............................
'''

            
def main():
    loggers_init()
    GB = Global_controller()
    # rospy.wait_for_message("/start",std_msgs.msg.Int32)
    # time.sleep(3)

    logger.info(f"user: start at {time.time() - GB.global_start_time}")
    # thread = threading.Thread(target=lambda: os.system("rosnode kill /speech_command_node"))
    # thread.start()
    #--------------------------------------------------------------------------------------------------#

    GB.navigation(GB.goals[0])
    menu = msi.QR_Decode()
    logger.info(f"user: menu: {menu}")
    # GB.visual_pub.publish(menu)
    GB.audio = GB.Voice["mission"] + "|" + GB.Voice["kinds"] + f"{menu}.wav"
    # thread = threading.Thread(target=GB.audio_play)
    # thread.start()
    GB.audio_play()
    #--------------------------------------------------------------------------------------------------#


    try:
        GB.navigation(GB.goals[1])
        res = 1
        while res==1:
            
            # exit()
            (GB.real_shop,temp_goal,res) = msi.mission_start(GB.client,menu,GB.search_goals)
            GB.navigation(temp_goal)
            # if result:
            #     print(f"closer pose: ({temp_goal.x},{temp_goal.y})")
            # if GB.navigation(temp_goal,TimeOut=8) != GoalStatus.SUCCEEDED:
            #     print(f"nav Failed,goal: ({temp_goal.x},{temp_goal.y})")
            #     result += 1


        logger.info(f"user: real_shop: {GB.real_shop}")
        logger.info(f"user: temp_goal: {temp_goal}")
        GB.bill += GB.Menu[menu][GB.real_shop]
        GB.audio = GB.Voice["fetch"] + "|" + GB.Voice["shop"] + f"{GB.real_shop}.wav"
        GB.audio_play()

    except Exception as e:
        logger.error(f"user: error: {e}")

    #--------------------------------------------------------------------------------------------------#

    GB.navigation(GB.goals[2])
    GB.audio = GB.Voice["simulation"] + f"simulation-A.wav"
    GB.audio_play()
    # while not GB.connect(menu):
    #     time.sleep(1)
    # try:
    #     GB.audio = GB.Voice["simulation"] + f"simulation-{GB.simulink_data[0]}.wav"

    #     GB.virtual_shop = GB.simulink_data[1]

    #     GB.audio_play()
    #     GB.bill += GB.Menu[menu][GB.simulink_data[1]]
    #     pass
    # except Exception as e:
    #     logger.error(f"user: error: {e}")



    #--------------------------------------------------------------------------------------------------#

    GB.navigation(GB.goals[3])

    if msi.traffic_light():
        Cross = 1
        logger.info("user: crossing one is available")
        GB.audio = GB.Voice["crossing"] + f"intersection-1.wav"
        GB.audio_play()
    else:
        Cross = 2
        GB.navigation(GB.goals[4])
        logger.info("user: crossing two is available")
        GB.audio = GB.Voice["crossing"] + f"intersection-2.wav"
        GB.audio_play()
    GB.break_pub.publish(1)
    
    #--------------------------------------------------------------------------------------------------#

    if Cross == 1:
        GB.navigation(GB.goals[5])
        GB.visual_nav_pub.publish(3)
    else:
        GB.navigation(GB.goals[6])
        GB.visual_nav_pub.publish(3)
        
    rospy.wait_for_message("/visual_nav_end",std_msgs.msg.Int32)
    


    if GB.virtual_shop != GB.real_shop and GB.virtual_shop is not None:
        
        shop_path = GB.Voice["shop"] + f"{GB.real_shop}.wav"\
                    + "|" + GB.Voice["and"] \
                    + "|" + GB.Voice["shop"] + f"{GB.virtual_shop}.wav"
    else:
        shop_path = GB.Voice["shop"] + f"{GB.real_shop}.wav"
    logger.info(f"user: real_shop: {GB.real_shop}")
    logger.info(f"user: virtual_shop: {GB.virtual_shop}")
    logger.info(f"user: spend: {GB.bill}")
    GB.audio = GB.Voice["finish"] + "|" + shop_path + "|" + GB.Voice["spend"] + f"spend-{GB.bill}.wav"
    GB.audio_play()
    #--------------------------------------------------------------------------------------------------#

    GB.audio_player.terminate()
    logger.info(f"user: total time : {(time.time() - GB.global_start_time):.2f}")


def test():
    GB = Global_controller()
    GB.navigation(GB.goals[1]) 
    menu = "Fruit"
    (GB.real_shop,temp_goal) = msi.mission_start(GB.client,menu,GB.search_goals)
    logger.info(f"user: real_shop: {GB.real_shop}")
    logger.info(f"user: temp_goal: {temp_goal}")
    GB.bill += GB.Menu[menu][GB.real_shop]
    # exit()
    GB.navigation(temp_goal)
    # print(f"I have got {GB.real_shop} and spend {GB.Menu[menu][GB.real_shop]}")
    # key = input("Press Enter to continue...")
    # if key == "q":
    #     exit()
    # goal = MoveBaseGoal()
    # goal.target_pose.header.frame_id = "map"
    # goal.target_pose.header.stamp = rospy.Time.now()
    # goal.target_pose.pose.position.x = 0
    # goal.target_pose.pose.position.y = 0
    # goal.target_pose.pose.orientation.z = 0
    # goal.target_pose.pose.orientation.w = 0
    # GB.navigation(goal)




if __name__ == '__main__':
    rospy.init_node('global_controller', anonymous=True)
    # test()
    main()
    # GB = Global_controller()
    # GB.nav_test()