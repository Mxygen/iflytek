import rospy
from sensor_msgs.msg import LaserScan
from pynput import keyboard as kb
import math



keyPressed = 0


def callback(data):
    global keyPressed
    if not keyPressed: 
        print("Enter key pressed, saving data...")
        with open('/home/ucar/ucar_ws/src/navigation_test/scripts/lidar.txt', 'w') as file:
            for i in range(len(data.ranges)):
                file.write(f"{data.ranges[i]}" + ("\n" if i % 10 == 0 else " "))
            file.write('\n')  # 添加换行符以结束文件
        keyPressed = 1

def OnPress(key):
    print(key)
    try:
        if key == kb.Key.enter:
            global keyPressed
            keyPressed = True
    except AttributeError:
        keyPressed = None  # 如果按下的是特殊键（如Shift、Ctrl等），则设置为None

def dataCheck():
    lidar_data = []
    checkCount = 0
    infCount = 0
    targetDir = []
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

    for i in range(len(lidar_data)):
        if not lidar_data[i] == float('inf'):
            if abs((lidar_data[i] + lidar_data[(i+454)%909]) * math.cos(math.radians((i-454)*360/909))) - 6 < 0.15:
                checkCount += 1
            elif abs((lidar_data[i] + lidar_data[(i+454)%909]) * math.sin(math.radians((i-454)*360/909))) - 5 < 0.15:
                checkCount += 1
            else:
                targetDir.append(i)
        else:
            targetDir.append(i)
            infCount += 1
    tempList = []
    Dir = []
    print(targetDir)
    for i in range(len(targetDir)):
        temp = targetDir.pop(0)
        if not tempList:
            tempList.append(targetDir.pop(0))
        elif abs(tempList[-1] - temp) < 5:
            tempList.append(temp)
        else:
            if len(tempList) < 20:
                tempList.clear()
                continue
            Dir.append((tempList[0] + tempList[-1]) * 360 / 909 / 2)
            tempList.clear()

    print(f"Check count: {checkCount}")
    print(f"Inf count: {infCount}")
    print(f"Direction: {Dir}")

    


if __name__ == '__main__':
    # rospy.init_node('map_scan_node', anonymous=True)
    # rospy.Subscriber('/scan',LaserScan, callback)
    
    # Listener = kb.Listener(on_press=OnPress)
    # Listener.start()  # 启动键盘监听器
    # rospy.spin()  # 保持节点运行，等待回调函数被调用
    dataCheck()

    
        # 等待按键事件

