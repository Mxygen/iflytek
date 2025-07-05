import cv2
from pathlib import Path

current_dir = Path(__file__).parent.resolve() /'img' 

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    count = 0
    while True:
        ret,image = cap.read()
        cv2.imshow("Image",image)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
    
    while True:
        _ = input("----")
        ret,image = cap.read()
        
        cv2.imwrite(str(current_dir/f'{count}.jpg'),image)
        count += 1
        print(f"image {count} saved")




