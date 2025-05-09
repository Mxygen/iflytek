import ctypes
import cv2
import time
integral = ctypes.CDLL('/home/ucar/ucar_ws/src/visual_navigation/scripts/libintegral.so')  
integral.Canny_Method.argtypes = [ctypes.POINTER(ctypes.c_ubyte), ctypes.c_double, 
                                     ctypes.c_double, ctypes.c_int, 
                                     ctypes.c_float, ctypes.c_float]
integral.Canny_Method.restype = None













if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
    ret, frame = cap.read()

    frame = cv2.flip(frame,1)
    frame = cv2.resize(frame,(160,120))
    
    frame = frame.ctypes.data_as(ctypes.POINTER(ctypes.c_ubyte))

    linear_speed = 0.0
    orientations = 0.0
    start_time = time.time()
    integral.Canny_Method(frame,50,150,2,linear_speed,orientations)
    end_time = time.time()
    print(end_time - start_time)
    print(linear_speed," ",orientations)

