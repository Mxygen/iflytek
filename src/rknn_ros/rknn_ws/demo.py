import cv2

cap = cv2.VideoCapture(0)

# 定义默认参数（根据设备调整）
DEFAULT_PARAMS = {
    cv2.CAP_PROP_BRIGHTNESS: 0.5,
    cv2.CAP_PROP_CONTRAST: 0.5,
    cv2.CAP_PROP_SATURATION: 0.5,
    cv2.CAP_PROP_HUE: 0.5,
    cv2.CAP_PROP_GAIN: 0,
    cv2.CAP_PROP_EXPOSURE: -6,  # 自动曝光
    cv2.CAP_PROP_FRAME_WIDTH: 640,
    cv2.CAP_PROP_FRAME_HEIGHT: 480,
    cv2.CAP_PROP_FPS: 30,
    cv2.CAP_PROP_AUTO_WB: 1,  # 自动白平衡
}

# 恢复默认参数
for prop_id, value in DEFAULT_PARAMS.items():
    cap.set(prop_id, value)

# 验证设置结果
for prop_name, prop_id in {
    '亮度': cv2.CAP_PROP_BRIGHTNESS,
    '对比度': cv2.CAP_PROP_CONTRAST,
    '宽度': cv2.CAP_PROP_FRAME_WIDTH,
    # 添加其他需要验证的参数
}.items():
    print(f"{prop_name}: {cap.get(prop_id)}")