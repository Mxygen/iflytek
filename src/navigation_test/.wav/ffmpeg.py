#!/usr/bin/python3.7
# coding=UTF-8
import os
import wave
import pyaudio
import time
import traceback

def play_wav_file(filepath):
    """使用 pyaudio 播放单个WAV文件"""
    try:
        # 检查文件是否存在
        if not os.path.exists(filepath):
            print(f"文件不存在: {filepath}")
            return False

        # 检查文件扩展名
        if not filepath.lower().endswith('.wav'):
            print(f"不是WAV文件: {filepath}")
            return False

        # 打开WAV文件
        wf = wave.open(filepath, 'rb')

        # 创建PyAudio对象
        p = pyaudio.PyAudio()

        # 打开音频流 (使用默认设备)
        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                       channels=wf.getnchannels(),
                       rate=wf.getframerate(),
                       output=True)

        # 读取数据
        data = wf.readframes(1024)

        # 播放音频
        while data:
            stream.write(data)
            data = wf.readframes(1024)

        # 关闭流和文件
        stream.stop_stream()
        stream.close()
        p.terminate()
        wf.close()

        return True
    except Exception as e:
        # 打印更详细的错误信息
        print(f"使用 pyaudio 播放文件 {filepath} 时出错: {str(e)}")
        traceback.print_exc()
        return False

def play_wav_files(file_list):
    """按顺序播放指定的WAV文件"""
    # 顺序播放每个文件
    for wav_file in file_list:
        print(f"正在播放: {wav_file}")
        
        if play_wav_file(wav_file):
            print(f"完成播放: {wav_file}")
        else:
            print(f"播放失败: {wav_file}")
        
        # 在文件之间添加短暂延迟
        time.sleep(0.1)

def main():
    # 使用绝对路径指定要播放的文件列表
    wav_files = [
        # "C:\\Users\\mkx\\Desktop\\.wav\\本次采购任务为\\mission.wav",    # 第一个播放
    #    "C:\\Users\\mkx\\Desktop\\.wav\\种类\\sweet.wav",   # 第二个播放   # 第三个播放
        # 可以继续添加更多文件
        "/home/ucar/ucar_ws/src/navigation_test/.wav/crossing/intersection-1.wav"
    ]
    
    # 播放WAV文件
    play_wav_files(wav_files)

if __name__ == "__main__":
    main()