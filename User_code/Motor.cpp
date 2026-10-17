#include <stdint.h>
#include "Motor.hpp"
#include "can_user.h"
#include "can.h"
#include <string.h>
#include <math.h>
//
// Created by zyfsj on 2026/10/3.
//

volatile float g_ff_gain=0.1*9.8*0.065/0.18;

Motor::Motor(Motortype motortype,float ratio,ControlMode mode,PID ppid,PID spid):
motor_type_(motortype),ratio_(ratio),control_mode_(mode),ppid_(ppid),spid_(spid){;}
void Motor::canRxMsgCallback(const uint8_t rxdata[8])
{
    RxDecode(rxdata);
}
float Motor::angle() const {return angle_;}
float Motor::speedRpm() const {return speedRpm_;}
float Motor::currentAmps() const {return currentA_;}
//float Motor::temperatureC() const {return tempC_;}
bool Motor::hasFeedback() const {return received_;}

void Motor::handleIntensity(float angle_ref)
{
    if (stop_flag_)
    {
        memset(tx_data_,0,sizeof(tx_data_));
    }
    feedforward_intensity_=g_ff_gain*sinf(angle_*3.14f/180.0f)*1000;
    switch (control_mode_)
    {
        case POSITION_SPEED:
            {
                float speed_ref=ppid_.calc(angle_ref,angle_)+feedforward_speed_;
                intensity_float_=spid_.calc(speed_ref,speed_)+feedforward_intensity_;
                break;
            }
    case SPEED:
        {
            intensity_float_=spid_.calc(speed_ref_,speed_) + feedforward_intensity_;
            break;
        }
        default:break;
    }
}
void Motor::setTxIntensity(uint8_t motor_id,float angle_ref)
{
    handleIntensity(angle_ref);
    int16_t intensity_int=(int16_t)intensity_float_;
    uint8_t index1=0,index2=0;
    switch (motor_id)
    {
        case 1: {index1=0; index2=1; break;}
        case 2: {index1=2; index2=3; break;}
        case 3: {index1=4; index2=5; break;}
        case 4: {index1=6; index2=7; break;}
    }
    tx_data_[index1]=(intensity_int&(0b1111111100000000))>>8;
    tx_data_[index2]=intensity_int&(0b0000000011111111);
    if (stop_flag_==true){memset(tx_data_,0,sizeof(tx_data_));}
    HAL_CAN_AddTxMessage(&hcan1,&tx_header,tx_data_,&can_tx_mailbox);
}

uint8_t* Motor::getTxData() {return tx_data_;}

void Motor::RxDecode(const uint8_t* rx_data_)
{
    ecd_angle_=(uint16_t)((rx_data_[0]<<8)+rx_data_[1]);
    speedRpm_=(int16_t)((rx_data_[2]<<8)+rx_data_[3]);
    currentA_=(int16_t)((rx_data_[4]<<8)+rx_data_[5]);
    speed_=(float)speedRpm_*6.0f/ratio_;
    if (ecd_angle_>8192||ecd_angle_<0){return;}
    //tempC_=rx_data_[6];
    if (received_==true)
    {
        delta_ = ecd_angle_ - last_ecd_;
        if (delta_ > kEncoderRange / 2)
        {
            delta_ -= kEncoderRange;
        }
        else if (delta_ < -kEncoderRange / 2)
        {
            delta_ += kEncoderRange;
        }
        total_ecd_+=delta_;
        angle_=(float)total_ecd_/(float)kEncoderRange*360.0f/ratio_;
        if (angle_>360.0f){angle_=fmodf(angle_,360.0f);}
    }
    last_ecd_=ecd_angle_;
    received_ = true;
}
