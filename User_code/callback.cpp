//
// Created by zyfsj on 2026/10/3.
//
//1.把数据拷贝到rxdata
//2.简单校验frame是不是我们想要的
//3.不太可能在这个中断里面完成解包decode
//调用一个函数完成解包
#include "can.h"
#include "Motor.hpp"
PID ppid(50.0f,0.0f,0.0f,0.0f,1000.0f,0.1f);
PID spid(45.0f,0.0f,0.7f,100.0f,16384.0f,0.05f);

Motor motor(Motor::M2006,36.0f,Motor::POSITION_SPEED,ppid,spid);

extern "C" void UsercanRxMsgCallback(const uint8_t* rxdata){
    motor.canRxMsgCallback(rxdata);
}
extern "C" void UsersetTxCurrent(uint8_t motor_id,float angle_ref)
{
    motor.setTxIntensity(motor_id,angle_ref);
}