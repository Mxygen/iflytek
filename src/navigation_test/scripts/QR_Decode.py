#!/usr/bin/python3
# coding=UTF-8
import cv2
import time
from pyzbar import pyzbar
shop = ["Vegetable","Fruit","Dessert","Error"]

def QR_Decode()->str:
    if __name__ == '__main__':
        time_start = time.time()
    cap = cv2.VideoCapture(0)
    if __name__ == '__main__':
        print(f"start camera takes {time.time() - time_start} seconds")
    time_start = time.time()
    data = ""
    count = 0
    while True:
        
        ret, frame = cap.read()
        if not ret:
            continue
        frame = cv2.flip(frame, 1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # gray = cv2.equalizeHist(gray)  # 直方图均衡化
        # cv2.imshow("frame", gray)
        # cv2.waitKey(1)
        # data,bbox,_ = detector.detectAndDecode(frame)
        obj = pyzbar.decode(gray)
        # print(obj)
        # print("objects:", objects)
        data = obj[0].data.decode("utf-8")
            # bbox = obj.polygon
            # if len(bbox) == 4:
            #     bbox = np.array(bbox, dtype=np.int32)
            #     cv2.polylines(frame, [bbox], True, (0, 255, 0), 2)
            # else:
            #     continue
        # if bbox is None:
        #     continue
        count+=1
        # print("边界框:", bbox)
        if data in shop:
            # logger.DEBUG(f"QR code detected: {data}")
            if __name__ == "__main__":
                print(f"QR code detected: {data}")
                print(f"took {count} times")
            cap.release()
            return data
        if time.time() - time_start > 60:
            print("ERROR in QR_Decode")
            cap.release()
            break


if __name__ == '__main__':
    try:
        print(QR_Decode())
    except Exception as e:
        print(e)
        pass

