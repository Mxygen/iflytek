#!~/venv3.9/bin/python3

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
import numpy as np
# 设置ROS日志级别映射
logging.addLevelName(logging.DEBUG, 'DEBUG')
logging.addLevelName(logging.INFO, 'INFO')
logging.addLevelName(logging.WARNING, 'WARN')
logging.addLevelName(logging.ERROR, 'ERROR')
logging.addLevelName(logging.CRITICAL, 'FATAL')


local_path = rospkg.RosPack().get_path('rknn_ros')
RKNN_MODEL = local_path + '/rknn_ws/best.rknn'
OBJ_THRESH = 0.5
NMS_THRESH = 0.6 
IMG_SIZE = 640  # Consider lowering to 416 or 320 for higher FPS
CLASSES = ("pie","red","nana","coke","pep","green","tom","milk","pot","apple","melon")



def nms_boxes(boxes, scores):
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep

def sigmoid(x):
    return 1 / (1 + np.exp(-x))


def xywh2xyxy(x):
    # Convert [x, y, w, h] to [x1, y1, x2, y2]
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y

def process(input, mask, anchors):
    anchors = [anchors[i] for i in mask]
    grid_h, grid_w = map(int, input.shape[0:2])

    box_confidence = sigmoid(input[..., 4])
    box_confidence = np.expand_dims(box_confidence, axis=-1)

    box_class_probs = sigmoid(input[..., 5:])

    box_xy = sigmoid(input[..., :2])*2 - 0.5

    col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
    row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
    col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    grid = np.concatenate((col, row), axis=-1)
    box_xy += grid
    box_xy *= int(IMG_SIZE/grid_h)

    box_wh = pow(sigmoid(input[..., 2:4])*2, 2)
    box_wh = box_wh * anchors

    box = np.concatenate((box_xy, box_wh), axis=-1)

    return box, box_confidence, box_class_probs


def filter_boxes(boxes, box_confidences, box_class_probs):
    boxes = boxes.reshape(-1, 4)
    box_confidences = box_confidences.reshape(-1)
    box_class_probs = box_class_probs.reshape(-1, box_class_probs.shape[-1])

    _box_pos = np.where(box_confidences >= OBJ_THRESH)
    boxes = boxes[_box_pos]
    box_confidences = box_confidences[_box_pos]
    box_class_probs = box_class_probs[_box_pos]

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)
    _class_pos = np.where(class_max_score >= OBJ_THRESH)

    boxes = boxes[_class_pos]
    classes = classes[_class_pos]
    scores = (class_max_score* box_confidences)[_class_pos]

    return boxes, classes, scores


def letterbox(im, new_shape=(640, 640), color=(0, 0, 0)):
    """Resize image while maintaining aspect ratio"""
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return im, ratio, (dw, dh)

def yolov5_post_process(input_data):
    masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
    anchors = [[10, 13], [16, 30], [33, 23], [30, 61], [62, 45],
               [59, 119], [116, 90], [156, 198], [373, 326]]

    boxes, classes, scores = [], [], []
    for input, mask in zip(input_data, masks):
        b, c, s = process(input, mask, anchors)
        b, c, s = filter_boxes(b, c, s)
        boxes.append(b)
        classes.append(c)
        scores.append(s)

    boxes = np.concatenate(boxes)
    boxes = xywh2xyxy(boxes)
    classes = np.concatenate(classes)
    scores = np.concatenate(scores)

    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]

        # 将数组转换为float64类型
        b = b.astype(np.float64)
        s = s.astype(np.float64)

        keep = nms_boxes(boxes=b, scores=s)
        # , nms_thresh=NMS_THRESH
        nboxes.append(b[keep])
        nclasses.append(c[keep])
        nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)

    return boxes, classes, scores


def preprocess_image(image):
    """Preprocess image for model input"""
    # Save a copy of the original image
    original_image = image.copy()
    
    # Resize
    img, ratio, (dw, dh) = letterbox(image, new_shape=(IMG_SIZE, IMG_SIZE))
    # Ensure correct color channels (RGB)
    if img.shape[2] == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # Convert to float32 for better NPU efficiency
    img = np.array(img, dtype='float32')
    # Expand dimensions to 4D tensor [batch, height, width, channel]
    img = np.expand_dims(img, axis=0)
    return img, ratio, (dw, dh), original_image



def inference_only(rknn, image):
    """
    Perform inference on image
    return: num_detections, boxes, class_names, scores, centers
    """
    # Preprocessing
    img, ratio, (dw, dh), original_image = preprocess_image(image)
    
    # Initialize return variables
    boxes = None
    classes = None
    scores = None
    centers = None
    class_names = None  # 初始化class_names
    
    # Inference
    try:
        # Use NPU inference
        outputs = rknn.inference(inputs=[img])
        
        # Check if outputs are valid
        if outputs is None or len(outputs) == 0:
            print("Model inference returned empty results, check model file!")
            return 0, None, None, None, None
            
        # Postprocessing
        input0_data = outputs[0]
        input1_data = outputs[1]
        input2_data = outputs[2]

        input0_data = input0_data.reshape([3, -1] + list(input0_data.shape[-2:]))
        input1_data = input1_data.reshape([3, -1] + list(input1_data.shape[-2:]))
        input2_data = input2_data.reshape([3, -1] + list(input2_data.shape[-2:]))

        input_data = list()
        input_data.append(np.transpose(input0_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input1_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input2_data, (2, 3, 0, 1)))

        boxes, classes, scores = yolov5_post_process(input_data)

        # Fix: Apply inverse transformation to remove padding and scaling effects
        if boxes is not None:
            # Adjust box coordinates to fit original image since they were detected on 640x640 image
            h, w = original_image.shape[:2]
            r_w, r_h = ratio
            
            # Convert each bounding box coordinate back to original image size
            adjusted_boxes = []
            centers = []  # 存储中心点坐标
            class_names = []  # 存储类别名称
            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = box
                
                # Remove padding
                x1 = (x1 - dw) / r_w
                y1 = (y1 - dh) / r_h
                x2 = (x2 - dw) / r_w
                y2 = (y2 - dh) / r_h
                
                # Ensure coordinates are within image boundaries
                x1 = max(0, min(w-1, x1))
                y1 = max(0, min(h-1, y1))
                x2 = max(0, min(w-1, x2))
                y2 = max(0, min(h-1, y2))
                
                # 计算中心点坐标
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                width = x2 - x1
                height = y2 - y1
                
                adjusted_boxes.append([x1, y1, x2, y2])
                centers.append([center_x, center_y, width, height])
                class_idx = classes[i]
                class_names.append(CLASSES[class_idx])
            
            boxes = np.array(adjusted_boxes)
            centers = np.array(centers)
            
        num_detections = 0 if boxes is None else len(boxes)
        
        return num_detections, boxes, class_names, scores, centers
        
    except Exception as e:
        print(f"Error during inference: {e}")
        import traceback
        traceback.print_exc()
        return 0, None, None, None, None


class RKNN_ROS:

    Menu = {
        "Fruit":["apple","nana","melon"],
        "Dessert":["coke","milk","pie"],
        "Vegetable":["tom","pot","pep"]
    }

    def signal_handler(self,signum,frame):
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
        self.target = None
        ret = self.rknn.load_rknn(RKNN_MODEL)
        self.break_flag = 0
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
        if self.detect == 1 and self.target is not None:
            print(f"detect, target: {self.target}")
            
        else:
            print(f"sleep")
    def break_flag_callback(self,data):
        self.break_flag = data.data

    def inference_for_ros(self,camera_id=0,enable_debug=0):
        """Perform inference on a single image"""
        # Open camera
        rospy.init_node('rknn_ros')
        rospy.Subscriber("/rknn_target", String,self.rknn_callback)
        result_pub = rospy.Publisher("/rknn_result",String,queue_size=10)
        rospy.Subscriber("/break_flag",Int8,self.break_flag_callback)
        rospy.wait_for_message("/rknn_target", String)
        rospy.Subscriber("/detect",Int8,self.detect_callback)
        capture = cv2.VideoCapture(camera_id)


        if not capture.isOpened():
            print(f"Unable to open camera (ID: {camera_id})")
            return
        
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        rate = rospy.Rate(30)
        # None_count = 0
        check_gray = 0
        while not rospy.is_shutdown():
            if self.break_flag == 1:
                break
            if self.detect == 0 or self.target is None:
                rate.sleep()
                continue
            time_start = time.time()    
            # Read frame
            ret, frame = capture.read()
            if np.array_equal(frame[:,:,0],frame[:,:,1]) and np.array_equal(frame[:,:,0],frame[:,:,2]) and check_gray == 0:
                print("camera is gray now waiting....")
                continue
            else:
                check_gray = 1
            
            frame = cv2.flip(frame,1)
            print(f"frame shape: {frame.shape}")
            if not ret:
                print("Unable to get image from camera, exiting...")
                continue
            
            # Inference and draw
            _, _, class_names, scores, centers = inference_only(self.rknn, frame)
            inference_time = time.time() - time_start
            temp = None
            if class_names is not None:
                for cls,pos,scr in zip(class_names,centers,scores):
                    print(f"cls: {cls}, pos: {pos}, scr: {scr}")
                    if scr > 0.6:
                        if cls in self.Menu[self.target] and self.detect == 1:
                            temp = f"{cls}|{pos[0]}"
                            print(f"temp: {temp}")
                            break
                        elif cls == "red" or cls == "green" and self.detect == 2:
                            temp = cls
                            print(f"traffic light: {temp}")
                            break
                            # None_count = 0
                       
            if temp is not None:
                result_pub.publish(temp)
                temp = None
            class_names = None
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





if __name__ == '__main__':
    rknn_ros = RKNN_ROS()
    # rknn_ros.target = "Fruit"
    rknn_ros.inference_for_ros()