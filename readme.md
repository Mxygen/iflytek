#### pip 默认软连接的python2的  python也是，但不要修改，记得区分就行

更新opencv到4.7.0.72


    pip3 install numpy==1.21.1
    pip3 install opencv-python==4.7.0.72
    pip3 install --upgrade pip


    sudo apt-get update
    sudo apt-get install portaudio19-dev python3-dev
#### 安装loguru   pyzbar   pyaudio
    pip3 install loguru pyzbar pyaudio dotenv

#### python3.9下载
    wget https://www.python.org/ftp/python/3.9.0/Python-3.9.0.tgz

#### 创建ros节点需要重新设置ROS日志级别映射

    logging.addLevelName(logging.DEBUG, 'DEBUG')
    logging.addLevelName(logging.INFO,  'INFO')
    logging.addLevelName(logging.WARNING,   'WARN')
    logging.addLevelName(logging.ERROR,     'ERROR')
    logging.addLevelName(logging.CRITICAL,  'FATAL')

#### 设置环境变量来配置ROS日志   否则无法创建ros节点 rospy.init_node("")会卡死或者报错
    if 'ROS_PYTHON_LOG_CONFIG_FILE' in os.  environ:
        del os.environ  ['ROS_PYTHON_LOG_CONFIG_FILE']
    os.environ['ROSCONSOLE_FORMAT'] = '[$   {severity}] ${message}'
    os.environ['ROSCONSOLE_LEVEL'] = 'INFO'

#### 更新opencv cpp
