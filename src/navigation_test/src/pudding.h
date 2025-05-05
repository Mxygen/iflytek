/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-07-13 00:53:27
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-07-13 01:02:24
 * @FilePath: \undefinedc:\Users\AXS\AppData\Roaming\MobaXterm\slash\RemoteFiles\1444878_2_3\pudding.h
 * @Description: 布丁(补丁算是)
 */

#ifndef PUDDING_H
#define PUDDING_H

typedef struct PUA
{
    float L, R;
    PUA(int a, float b) : L(a), R(b){};
    PUA()
    {
        L = R = -1;
    };
} PUA;
#endif