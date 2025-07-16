import cv2
import numpy as np

def adjust_color_temperature(frame, kelvin):
    """
    根据开尔文色温值调整视频帧的色温
    frame: 输入帧（BGR格式）
    kelvin: 色温值（1000K - 10000K）
    返回: 调整色温后的帧
    """
    # 限制色温范围
    kelvin = max(1000, min(kelvin, 10000))
    temp = kelvin / 100

    # 计算RGB色温增益（基于近似公式）
    if temp <= 66:
        red = 255
    else:
        red = temp - 60
        red = 329.698727446 * (red ** -0.1332047592)
        red = max(0, min(255, red))

    if temp <= 66:
        green = temp
        green = 99.4708025861 * (green ** 0.5) - 161.1195681661
    else:
        green = temp - 60
        green = 288.1221695283 * (green ** -0.0755148492)
    green = max(0, min(255, green))

    if temp >= 66:
        blue = 255
    else:
        blue = temp - 10
        blue = 138.5177312231 * (blue ** 0.5) - 305.0447927307
    blue = max(0, min(255, blue))

    # 归一化增益到 [0, 1]
    red_gain = red / 255.0
    green_gain = green / 255.0
    blue_gain = blue / 255.0

    # 复制输入帧
    adjusted_frame = frame.astype(float)

    # 调整每个通道的色温
    adjusted_frame[:, :, 0] *= blue_gain   # B通道
    adjusted_frame[:, :, 1] *= green_gain  # G通道
    adjusted_frame[:, :, 2] *= red_gain    # R通道

    # 裁剪到有效范围 [0, 255] 并转换回uint8
    adjusted_frame = np.clip(adjusted_frame, 0, 255).astype(np.uint8)
    return adjusted_frame

def main():
    # 初始化摄像头（默认使用设备0，通常为内置或USB摄像头）
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("错误：无法打开摄像头！")
        return

    # 设置初始色温
    kelvin = 6500
    window_name = "色温调节 - 摄像头"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 640, 480)

    print("色温调节器 (1000K - 10000K)")
    print("按 'q' 退出，按 'w' 增加1000K，按 's' 减少1000K")

    while True:
        # 读取摄像头帧
        ret, frame = cap.read()

        if not ret:
            print("错误：无法读取摄像头帧！")
            break
        # 调整帧的色温
        adjusted_frame = adjust_color_temperature(frame, kelvin)
        
        adjusted_frame = cv2.resize(adjusted_frame, (640,480))
        # 显示当前色温和帧
        cv2.putText(adjusted_frame, f"Color Temp: {kelvin}K", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow(window_name, adjusted_frame)

        # 等待按键
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('w'):
            kelvin = min(kelvin + 100, 10000)  # 增加色温
            print(f"当前色温: {kelvin}K")
        elif key == ord('s'):
            kelvin = max(kelvin - 100, 1000)   # 减少色温
            print(f"当前色温: {kelvin}K")

    # 释放摄像头并关闭窗口
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()