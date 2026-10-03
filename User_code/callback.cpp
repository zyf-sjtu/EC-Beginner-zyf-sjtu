//
// Created by zyfsj on 2026/10/3.
//
//1.把数据拷贝到rxdata
//2.简单校验frame是不是我们想要的
//3.不太可能在这个中断里面完成解包decode
//调用一个函数完成解包
#include "can.h"
#include "Motor.hpp"

Motor motor(19.0f);

extern "C" void UsercanRxMsgCallback(const uint8_t* rxdata){
    motor.canRxMsgCallback(rxdata);
}
extern "C" void UsersetTxCurrent(float amperes, uint8_t motor_id)
{
    motor.setTxCurrent(amperes,motor_id);
}


