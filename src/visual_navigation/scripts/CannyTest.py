import cv2
import numpy as np
from pynput import keyboard as kb



class Debug:

  Variables = {
    "CannyLow":50,
    "CannyHigh":90
  }


  def __init__(self,Record=False):
    self.dictKeys = []
    self.dictInit()
    self.listener = kb.Listener(on_press=self.onPress)
    self.listener.start()
    self.keyNum = 0
    self.keyNumMax = len(self.dictKeys)
    self.flag = False
    self.Record = Record
    if Record:
      self.VideoWriter = cv2.VideoWriter("output.mp4", cv2.VideoWriter_fourcc(*'mp4v'), 30, (320,240))

  def dictInit(self):
    for key in self.Variables.keys():
      self.dictKeys.append(key)

  def onPress(self,key):
    if key == kb.Key.up:
      self.Variables[self.dictKeys[self.keyNum]] += 1
    elif key == kb.Key.down:
      self.Variables[self.dictKeys[self.keyNum]] -= 1
    elif key == kb.Key.right:
      self.keyNum += 1
    elif key == kb.Key.left:
      self.keyNum -= 1
    elif key == kb.Key.space:
      self.flag = not self.flag



    if self.keyNum < 0:
      self.keyNum = self.keyNumMax - 1
    elif self.keyNum >= self.keyNumMax:
      self.keyNum = 0


    if self.Record and self.flag:
      output = f"\r{str(self.dictKeys[self.keyNum])} : {self.Variables[self.dictKeys[self.keyNum]]}" + \
              "            " + \
              "Recording..." + \
              "            "
    else:
      output = f"\r{str(self.dictKeys[self.keyNum])} : {self.Variables[self.dictKeys[self.keyNum]]}" + \
              "            " + \
              "            " + \
              "            "
    print(output,end="",flush=True)


  def mainLoop(self):

    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        # if np.array_equal(frame[:,:,0],frame[:,:,1]) and np.array_equal(frame[:,:,1],frame[:,:,2]):
        #     print("frame is gray now")
        # else:
        #     print("normal now")
        cv2.imshow("originalFrame",frame)        
        # frame = cv2.resize(frame, (160, 120))
        frame = cv2.resize(frame, (320,240))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.GaussianBlur(frame, (3,3), 1.5)
        cv2.imshow("GaussFrame",frame)
        frame = cv2.Canny(frame, self.Variables["CannyLow"], self.Variables["CannyHigh"])

        # print(frame.shape)
        cv2.imshow("frame", frame[::][120:240])
        

        if cv2.waitKey(1) & 0xFF == ord('q'):
          break
        if self.Record and self.flag:
          if len(frame.shape) == 2 or frame.shape[2] == 1:  # 灰度图
            frame_to_write = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            self.VideoWriter.write(frame_to_write)
          else:  # 彩色图
            self.VideoWriter.write(frame)
    cap.release()
    self.VideoWriter.release()
    cv2.destroyAllWindows()



if __name__ == "__main__":
  debug = Debug(Record=True)
  debug.mainLoop()