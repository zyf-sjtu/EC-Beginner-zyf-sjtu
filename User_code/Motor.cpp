#include <stdint.h>
#include "Motor.hpp"
#include "can_user.h"
#include "can.h"
//
// Created by zyfsj on 2026/10/3.
//


Motor::Motor(float ratio):ratio_(ratio){;}
void Motor::canRxMsgCallback(const uint8_t rxdata[8])
{
    RxDecode(rxdata);
}
float Motor::angle() const {return angle_;}
float Motor::speedRpm() const {return speedRpm_;}
float Motor::currentAmps() const {return currentA_;}
float Motor::temperatureC() const {return tempC_;}
bool Motor::hasFeedback() const {return received_;}

void Motor::setTxCurrent(float amperes, uint8_t motor_id)
{
    int16_t current=(int16_t)(amperes/20.0f*16384.0f);
    uint8_t index1=0,index2=0;
    switch (motor_id)
    {
        case 1: {index1=0; index2=1; break;}
        case 2: {index1=2; index2=3; break;}
        case 3: {index1=4; index2=5; break;}
        case 4: {index1=6; index2=7; break;}
    }
    tx_data_[index1]=(current&(0b1111111100000000))>>8;
    tx_data_[index2]=current&(0b0000000011111111);
    HAL_CAN_AddTxMessage(&hcan1,&tx_header,tx_data_,&can_tx_mailbox);
}

uint8_t* Motor::getTxData() {return tx_data_;}

void Motor::RxDecode(const uint8_t* rx_data_)
{
    received_ = true;
    ecd_angle_=(int16_t)((rx_data_[0]<<8)+rx_data_[1]);
    speedRpm_=(int16_t)((rx_data_[2]<<8)+rx_data_[3]);
    currentA_=(int16_t)((rx_data_[4]<<8)+rx_data_[5]);
    tempC_=rx_data_[6];
    angle_=ecd_angle_/kEncoderRange*360;
}
