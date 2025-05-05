#！~/venv3.9/bin/python3

from rknnUCAR2 import inference_only
import rospy
import cv2
import time
import psutil
from std_msgs.msg import Int8,String
import signal
import os
import logging
from rknnlite.api import RKNNLite
import rospkg
# 设置ROS日志级别映射
logging.addLevelName(logging.DEBUG, 'DEBUG')
logging.addLevelName(logging.INFO, 'INFO')
logging.addLevelName(logging.WARNING, 'WARN')
logging.addLevelName(logging.ERROR, 'ERROR')
logging.addLevelName(logging.CRITICAL, 'FATAL')


local_path = rospkg.RosPack().get_path('rknn_ros')
RKNN_MODEL = local_path + '/rknn_ws/best.rknn'


class RKNN_ROS:

    Menu = {
        "Fruit":["apple","nana","melon"],
        "Dessert":["coke","milk","pie"],
        "Vegetable":["tom","pot","pep"]
    }

    def signal_handler(self):
        print("Received interrupt signal, exiting...")
        self.running = False

    def __init__(self):
        self.running = True
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # 设置环境变量来配置ROS日志
        if 'ROS_PYTHON_LOG_CONFIG_FILE' in os.environ:
            del os.environ['ROS_PYTHON_LOG_CONFIG_FILE']
        os.environ['ROSCONSOLE_FORMAT'] = '[${severity}] ${message}'
        os.environ['ROSCONSOLE_LEVEL'] = 'INFO'
        
        self.rknn = RKNNLite()
        self.detect = 0
        ret = self.rknn.load_rknn(RKNN_MODEL)
        self.counts = {
            "pie":0,
            "red":0,
            "nana":0,
            "coke":0,
            "pep":0,
            "green":0,
            "tom":0,
            "milk":0,
            "pot":0,
            "apple":0,
            "melon":0,
        }
        if ret != 0:
            print('Failed to load RKNN model!')
            exit(ret)
        try:
            ret = self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
        except Exception as e:
            print(f"Unable to use all NPU cores: {e}")
            try:
                ret = self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
            except:
                ret = self.rknn.init_runtime()
        if ret != 0:
            print('Failed to initialize runtime environment!')
            exit(ret)

    def rknn_callback(self,data):
        self.target = data.data
    
    def detect_callback(self,data):
        self.detect = data.data
        if self.detect == 1:
            print("detect")
        else:
            print("sleep")

    def inference_for_ros(self,camera_id=0,enable_debug=0):
        """Perform inference on a single image"""
        # Open camera
        rospy.init_node('rknn_ros')
        rospy.Subscriber("/rknn_target", String,self.rknn_callback)
        result_pub = rospy.Publisher("/rknn_result",String,queue_size=10)
        rospy.Subscriber("/detect",Int8,self.detect_callback)
        rospy.wait_for_message("/rknn_target", String)
        capture = cv2.VideoCapture(camera_id)


        if not capture.isOpened():
            print(f"Unable to open camera (ID: {camera_id})")
            return
        
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        rate = rospy.Rate(30)
        # None_count = 0
        while not rospy.is_shutdown():
            if self.detect == 0:
                rate.sleep()
                continue
            time_start = time.time()    
            # Read frame
            ret, frame = capture.read()
            frame = cv2.flip(frame,1)
            if not ret:
                print("Unable to get image from camera, exiting...")
                continue
            
            # Inference and draw
            _, _, class_names, scores, centers = inference_only(self.rknn, frame)
            inference_time = time.time() - time_start
            temp = None
            for cls,pos,scr in zip(class_names,centers,scores):
                if scr > 0.6:
                    if cls in self.Menu[self.target]:
                        temp = f"{cls}|{pos[0]}"
                        # None_count = 0
                        break
            if temp is not None:
                result_pub.publish(temp)
            # else:
            #     None_count += 1
            #     if None_count > 10:
            #         result_pub.publish("None")
            #         None_count = 0

            if enable_debug == 1:
                if class_names is not None:
                    for class_name in class_names:
                        if class_name in self.counts:
                            self.counts[class_name] += 1
            
                    text = f"Detected: {class_names} | Scores: {[f'{s:.3f}' for s in scores]} | Time: {inference_time:.3f}s | Counts: {self.counts}"
                    print(f"\r{text}", end='', flush=True)
                
            # Display debug info (if enabled)
            if enable_debug == 2:
                # Display memory usage
                mem = psutil.virtual_memory()
                mem_text = f"Mem: {mem.percent}%"
                # Display inference time
                time_text = f"Infer: {inference_time:.1f} s"
                print(f"\rmem: {mem_text} | time: {time_text}", end='', flush=True)
            rate.sleep()
        capture.release()

        rospy.wait_for_message("/rknn_target", String)

if __name__ == '__main__':
    rknn_ros = RKNN_ROS()
    # rknn_ros.target = "Fruit"
    rknn_ros.inference_for_ros()