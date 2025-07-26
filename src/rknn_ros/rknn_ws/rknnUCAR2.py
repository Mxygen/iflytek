#!/home/ucar/venv3.9/bin/python3
# -*- coding: utf-8 -*-

"""
RK3588 NPU-accelerated YOLOv5 inference script
Environment: debian10, Python3.9
Supports two modes: image inference and camera real-time inference
"""

import urllib
import time
import sys
import numpy as np
import cv2
import os
import argparse
from rknnlite.api import RKNNLite
import psutil  # For system information
import signal
import rospkg
# import _my_nms_module # <-- 导入 C 扩展模块


local_path = rospkg.RosPack().get_path('rknn_ros')
RKNN_MODEL = local_path + '/rknn_ws/0704ReLU.rknn'
IMG_PATH = local_path + '/rknn_ws/1.jpg'
OBJ_THRESH = 0.5
NMS_THRESH = 0.6 
IMG_SIZE = 640  # Consider lowering to 416 or 320 for higher FPS
CLASSES = ("pie","red","nana","coke","pep","green","tom","milk","pot","apple","melon")
hashMap = {"pie":214,"nana":196,"coke":195,"pep":162,"tom":214,"milk":210,"pot":107,"apple":173,"melon":178}





# Global variable to control the running state
running = True

def signal_handler(sig, frame):
    global running
    print("Interrupt received, stopping...")
    running = False
        
        
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


def draw1(image, boxes, scores, classes):
    """Draw detection boxes and labels on image"""
    for box, score, cl in zip(boxes, scores, classes):
        # x1, y1, x2, y2
        x1, y1, x2, y2 = box
        
        # print('class: {}, score: {:.2f}'.format(CLASSES[cl], score))
        # print('box coordinate [x1,y1,x2,y2]: [{:.1f}, {:.1f}, {:.1f}, {:.1f}]'.format(x1, y1, x2, y2))
        
        # Ensure coordinates are integers
        x1 = int(x1)
        y1 = int(y1)
        x2 = int(x2)
        y2 = int(y2)
        
        # Ensure coordinates are within valid range
        h, w = image.shape[:2]
        x1 = max(0, min(x1, w-1))
        y1 = max(0, min(y1, h-1))
        x2 = max(0, min(x2, w-1))
        y2 = max(0, min(y2, h-1))
        
        # Check if rectangle is valid
        if x2 <= x1 or y2 <= y1:
            print("Warning: Invalid box coordinates:", x1, y1, x2, y2)
            continue

        # Draw rectangle with OpenCV (x1,y1) is top-left, (x2,y2) is bottom-right
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw class label and confidence
        label = '{0} {1:.2f}'.format(CLASSES[cl], score)
        # Ensure label is within image and properly positioned
        text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        # Position label above the box - ensure it's visible
        text_x = x1
        text_y = y1 - 5
        # If label would go above image, place it inside the box at the top
        if text_y < 10:
            text_y = y1 + 20
        cv2.putText(image, label, (text_x, text_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)


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


def inference_and_draw(rknn, image, is_display=True, enable_debug=False):
    """Perform inference and draw results on image"""
    # Preprocessing
    img, ratio, (dw, dh), original_image = preprocess_image(image)
    height = None
    class_names = []
    adjusted_boxes = []     
    centers = []  # 存储中心点坐标
    
    try:
        # Use NPU inference
        outputs = rknn.inference(inputs=[img])
        
        # Check if outputs are valid
        if outputs is None or len(outputs) == 0:
            print("Model inference returned empty results, check model file!")
            return image, 0, None, None, None
            
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
                height = abs(y1 - y2)
            boxes = np.array(adjusted_boxes)
            if enable_debug:
                print(f"Adjusted boxes: {boxes}")

        # Use original image for drawing, not the preprocessed one
        result_img = original_image.copy()
        if boxes is not None and is_display:
            draw1(result_img, boxes, scores, classes)
            
        num_detections = 0 if boxes is None else len(boxes)

        return result_img, num_detections, boxes, classes, scores, height,class_names
        
    except Exception as e:
        print(f"Error during inference: {e}")
        import traceback
        traceback.print_exc()
        return image, 0, None, None, None
    
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

def inference_on_image(rknn, image_path, output_dir='./output'):
    """Perform inference on a single image"""
    if not os.path.exists(image_path):
        print(f"Image file {image_path} does not exist!")
        return
    
    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)
    
    # Read image
    print(f"Processing image: {image_path}")
    image = cv2.imread(image_path)
    if image is None:
        print(f"Cannot read image: {image_path}")
        return
    
    # Inference
    t1 = time.time()
    result_img, num_detections, _, _, _ = inference_and_draw(rknn, image)
    infer_time = time.time() - t1
    
    # Display inference time on result image
    time_text = f"Inference time: {infer_time:.4f}s"
    cv2.putText(result_img, time_text, (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Save result
    output_path = os.path.join(output_dir, os.path.basename(image_path))
    cv2.imwrite(output_path, result_img)
    
    print(f"Detected {num_detections} objects")
    print(f"Inference time: {infer_time:.4f}s")
    print(f"Result saved to: {output_path}")
    
    # Display result
    cv2.imshow("Detection Result", result_img)
    cv2.waitKey(0) # Wait for key press
    cv2.destroyAllWindows()

def rknn_callback(msg):
    """Callback function for rknn topic"""
    print(f"Received message: {msg.data}")


    
    

def inference_on_camera(rknn, camera_id=0):
    """Perform real-time inference on camera video stream"""
    # Open camera
    global running
    capture = cv2.VideoCapture(camera_id)
    
    # Check if camera opened successfully
    if not capture.isOpened():
        print(f"Unable to open camera (ID: {camera_id})")
        return
    
    # Set camera parameters (lower resolution for higher FPS)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print("Camera real-time inference started...")
    print("Press 'q' or 'ESC' to exit")
    
    fps = 0.0
    frame_count = 0
    process_every_n_frames = 1  # Process every nth frame, adjustable with +/-
    show_fps_every = 3  # Update FPS display more frequently (was 10)
    enable_debug = False  # Whether to show debug info
    last_result = None    # Store previous frame result
    fps_display_img = None  # Image with FPS display
    
    print(f"Current processing frequency: every {process_every_n_frames} frames")
    
    # Calculate FPS over multiple frames for more stable reading
    fps_update_interval = 10  # Update FPS calculation every 10 frames
    processing_times = []  # Store last N processing times
    max_times_to_store = 30  # Number of times to average for FPS
    height = 1
    distance = 1
    while True:
        if not running:
            break
        t1 = time.time()
        
        # Read frame
        ret, frame = capture.read()
        # print(f"frame shape: {frame.shape}")
        if not ret:
            print("Unable to get image from camera, exiting...")
            continue
        frame = cv2.flip(frame, 1)
        
        frame_count += 1

        # Always create a copy of the frame for display
        # display_frame = frame.copy()
        
        # Implement frame skipping to improve visual FPS
        if frame_count % process_every_n_frames != 0:
                
            key = cv2.waitKey(3) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                break
            elif key == ord('+') or key == ord('='):  # Increase processing frequency
                process_every_n_frames = max(1, process_every_n_frames - 1)
                print(f"Increased processing frequency: every {process_every_n_frames} frames")
            elif key == ord('-') or key == ord('_'):  # Decrease processing frequency
                process_every_n_frames += 1
                print(f"Decreased processing frequency: every {process_every_n_frames} frames")
            elif key == ord('d'):  # Toggle debug info
                enable_debug = not enable_debug
                print(f"Debug info: {'enabled' if enable_debug else 'disabled'}")
            continue
        
        # Inference and draw
        result_img, _, class_names, scores, centers ,height,names = inference_and_draw(rknn, frame)
        # print(f"\r {height}", end=' ',flush=True)
        try:
            if height:
                distance = hashMap[names[0]] / height /2
                print(f"\r {distance}", end=' ',flush=True)
        except Exception as e:
            print(e)
        cv2.imshow("Detection Result", result_img)
        inference_time = time.time() - t1
        processing_times.append(inference_time)
        # if class_names is not None:
        #     status = f"\rDetected: {class_names} | Scores: {[f'{s:.3f}' for s in scores]} | Time: {inference_time:.3f}s"
        #     print(status, end='', flush=True)
        # Calculate frame processing time
       
        # Keep only the last N times
        if len(processing_times) > max_times_to_store:
            processing_times.pop(0)
        
        # Update FPS calculation more frequently
        if len(processing_times) > 0:
            avg_time = sum(processing_times) / len(processing_times)
            fps = 1.0 / avg_time
        
        # Always update FPS display on each processed frame
        fps_text = f"FPS: {fps:.2f}"
                    
        # Display processing frequency
        freq_text = f"Processing: 1/{process_every_n_frames} frames"
        
        # Display debug info (if enabled)
        if enable_debug:
            # Display memory usage
            mem = psutil.virtual_memory()
            mem_text = f"Mem: {mem.percent}%"
            # Display inference time
            time_text = f"Infer: {inference_time*1000:.1f}ms"
        
        key = cv2.waitKey(1) & 0xFF  # Use 1ms wait for better responsiveness
        if key == ord('q') or key == 27:  # 'q' or ESC
            break
        elif key == ord('+') or key == ord('='):  # Increase processing frequency
            process_every_n_frames = max(1, process_every_n_frames - 1)
            print(f"Increased processing frequency: every {process_every_n_frames} frames")
        elif key == ord('-') or key == ord('_'):  # Decrease processing frequency
            process_every_n_frames += 1
            print(f"Decreased processing frequency: every {process_every_n_frames} frames")
        elif key == ord('d'):  # Toggle debug info
            enable_debug = not enable_debug
            print(f"Debug info: {'enabled' if enable_debug else 'disabled'}")
    
    # Release resources
    capture.release()
    cv2.destroyAllWindows()
    print("Camera real-time inference stopped")


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="RKNN YOLOv5 Inference Demo")
    
    parser.add_argument("--mode", type=str, default="camera", 
                       choices=["image", "camera"],
                       help="Inference mode: image or camera")
    
    parser.add_argument("--model", type=str, default=RKNN_MODEL,
                       help=f"RKNN model path (default: {RKNN_MODEL})")
    
    parser.add_argument("--image", type=str, default=IMG_PATH,
                       help=f"Image path for inference (default: {IMG_PATH})")
    
    parser.add_argument("--camera", type=int, default=0,
                       help="Camera ID (default: 0)")
    
    parser.add_argument("--conf", type=float, default=OBJ_THRESH,
                       help=f"Confidence threshold (default: {OBJ_THRESH})")
    
    parser.add_argument("--output", type=str, default="./output",
                       help="Output directory (default: ./output)")

    # Add new arguments for processing frequency and debug mode
    parser.add_argument("--process_freq", type=int, default=1,
                       help="Process every n frames (default: 1)")
    
    parser.add_argument("--debug", action='store_true',
                       help="Enable debug information")
    
    return parser.parse_args()

def parse_args_for_roslaunch():
    """Parse command line arguments"""
    args = {}
    args["mode"] = "camera"
    args["model"] = RKNN_MODEL
    args["image"] = IMG_PATH
    args["camera"] = 0
    args["conf"] = OBJ_THRESH
    args["output"] = "./output"
    args["process_freq"] = 1
    args["debug"] = False

    return args


def main_control():
    # 注册中断信号
    signal.signal(signal.SIGINT, signal_handler)

    # 初始化RKNN
    rknn = RKNNLite()

    # 加载模型
    print('--> Loading RKNN model:', RKNN_MODEL)
    ret = rknn.load_rknn(RKNN_MODEL)
    if ret != 0:
        print('Failed to load RKNN model!')
        exit(ret)

    # 初始化运行环境
    print('--> Initializing runtime environment (NPU acceleration)')
    try:
        ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
    except Exception as e:
        print(f"Unable to use all NPU cores: {e}")
        try:
            ret = rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        except:
            ret = rknn.init_runtime()

    if ret != 0:
        print('Failed to initialize runtime environment!')
        exit(ret)

    # 只做摄像头推理
    try:
        inference_on_camera(rknn, camera_id=0)
    except KeyboardInterrupt:
        print("User interrupted, exiting")
    finally:
        rknn.release()
        print("RKNN resources released")


if __name__ == '__main__':
    main_control()