#!/usr/bin/python3
# coding=UTF-8

import time 
import random
import numpy as np
from pathlib import Path


localPath = Path(__file__).parent


class PID_Position:
    def __init__(self, Kp:float, Ki:float, Kd:float,Debug:bool = False):
        self.Kp:float = Kp
        self.Ki:float = Ki
        self.Kd:float = Kd
        self.Debug:bool = Debug
        self.clear()
        
    def clear(self):
        self.error:float = 0.0
        self.last_error:float = 0.0
        self.integral:float = 0.0
        self.derivative:float = 0.0
        self.output:float = 0.0

    def update(self, target_position, feedback_value):
        self.error = target_position - feedback_value
        if self.Debug:
            print(f"error: {self.error}")
        self.integral += self.error
        self.derivative = self.error - self.last_error
        self.last_error = self.error
        self.output = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative
        return self.output


class Painter:
    def __init__(self) -> None:
        import datetime
        self._Map = np.zeros((6,5))
        self.tmpList = []
        random.seed(time.time())
        now_datetime = datetime.datetime.fromtimestamp(time.time())
        self.filename = f"map_{now_datetime.strftime('%m-%d')}_{random.randint(0,10000)}.txt"


    def Save(self,verbose:bool = False) -> None:
        try:
            with open(localPath / 'maps' /self.filename, 'w') as f:
                f.write('---------------------Simple Map--------------------- \n')
                
                for i in range(len(self._Map)):
                    for j in range(len(self._Map[i])):
                        f.write('   ' + str(self._Map[i][j]))
                    f.write('\n'*3)
                if verbose:
                    ...
                    # f.write('---------------------Detailed Map--------------------- \n')
                    # for i in range(len(self._Map)):
                    #     for j in range(len(self._Map[i])):
                    #         if self._Map[i][j] == 0:
                    #             f.write(' ' + '   ')
                    #         else:
                    #             f.write(str(self._Map[i][j]) + '   ')
                    #     f.write('\n'*3)
        except Exception as e:
            print(e)

    def update(self,x:int,y:int,value:int) -> None:
        self._Map[x][y] = value

    def Exist(self,x:int,y:int) -> bool:
        return self._Map[x][y]



if __name__ == '__main__':
    painter = Painter()
    painter.Save()
    # painter.render()
    # painter.Save()








