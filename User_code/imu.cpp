//
// Created by zyfsj on 2026/10/20.
//

#include "imu.h"
#include <string.h>
#include "user_code.h"
#include <cmath>

IMU::IMU(){;}
void IMU::reset()
{
    memset(acc_data_,0,6);
    memset(gyro_data_,0,6);
    ACC_Z_MSB_=0;
    ACC_Y_MSB_=0;
    ACC_X_MSB_=0;
    ACC_Y_LSB_=0;
    ACC_X_LSB_=0;
    ACC_Z_LSB_=0;
    RATE_Z_MSB_=0;
    RATE_Y_MSB_=0;
    RATE_X_MSB_=0;
    RATE_Y_LSB_=0;
    RATE_X_LSB_=0;
    RATE_Z_LSB_=0;
}
void IMU::handleSensor()
{
    decode();
    calc();
    sensorDataTransform();
}
bool IMU::hasfeedback()
{
    return received_;
}
uint8_t acc_data_[6];
uint8_t gyro_data_[6];
volatile uint8_t  ACC_Z_MSB_;
volatile uint8_t  ACC_Y_MSB_;
volatile uint8_t  ACC_X_MSB_;
volatile uint8_t  ACC_Y_LSB_;
volatile uint8_t  ACC_X_LSB_;
volatile uint8_t  ACC_Z_LSB_;
volatile uint8_t  RATE_Z_MSB_;
volatile uint8_t  RATE_Y_MSB_;
volatile uint8_t  RATE_X_MSB_;
volatile uint8_t  RATE_Y_LSB_;
volatile uint8_t  RATE_X_LSB_;
volatile uint8_t  RATE_Z_LSB_;
void IMU::decode()
{
    received_=true;
    bmi088_accel_read_reg(0x12,acc_data_,6);
    bmi088_gyro_read_reg(0x02,gyro_data_,6);
    ACC_X_LSB_=acc_data_[0];
    ACC_X_MSB_=acc_data_[1];
    ACC_Y_LSB_=acc_data_[2];
    ACC_Y_MSB_=acc_data_[3];
    ACC_Z_LSB_=acc_data_[4];
    ACC_Z_MSB_=acc_data_[5];
    RATE_X_LSB_=gyro_data_[0];
    RATE_X_MSB_=gyro_data_[1];
    RATE_Y_LSB_=gyro_data_[2];
    RATE_Y_MSB_=gyro_data_[3];
    RATE_Z_LSB_=gyro_data_[4];
    RATE_Z_MSB_=gyro_data_[5];
}
void IMU::calc()
{
    acc_x_int_=(int16_t)(ACC_X_LSB_|(ACC_X_MSB_<<8));
    acc_y_int_=(int16_t)(ACC_Y_LSB_|(ACC_Y_MSB_<<8));
    acc_z_int_=(int16_t)(ACC_Z_LSB_|(ACC_Z_MSB_<<8));
    gyro_x_int_=(int16_t)(RATE_X_LSB_|(RATE_X_MSB_<<8));
    gyro_y_int_=(int16_t)(RATE_Y_LSB_|(RATE_Y_MSB_<<8));
    gyro_z_int_=(int16_t)(RATE_Z_LSB_|(RATE_Z_MSB_<<8));
    acc_[0]=(float)acc_x_int_/65536.0f*6.0f*9.8f;
    acc_[1]=(float)acc_y_int_/65536.0f*6.0f*9.8f;
    acc_[2]=(float)acc_z_int_/65536.0f*6.0f*9.8f;
    gyro_[0]=(float)gyro_x_int_/65536.0f*2000.0f/180*3.14;
    gyro_[1]=(float)gyro_y_int_/65536.0f*2000.0f/180*3.14;
    gyro_[2]=(float)gyro_z_int_/65536.0f*2000.0f/180*3.14;
}

void IMU::sensorDataTransform(void){
    for (uint8_t i = 0; i < 3; i++) {
        gyro_sensor_[i] = gyro_[0] * R_imu_[i][0] +
                          gyro_[1] * R_imu_[i][1] +
                          gyro_[2] * R_imu_[i][2] - gyro_bias_[i];
        acc_sensor_[i] = acc_sen_ * (acc_[0] * R_imu_[i][0] +
                                         acc_[1] * R_imu_[i][1] +
                                         acc_[2] * R_imu_[i][2]) -acc_bias_[i];
    }
}

void IMU::updateQuaternion(void)
{
    float half_dt=0.5f*dt_;
    // dq = 0.5 * q ⊗ (0, w)
    dq_[0] = (-q_[1] * gyro_sensor_[0] - q_[2] * gyro_sensor_[1] - q_[3] * gyro_sensor_[2]) * half_dt;
    dq_[1] = ( q_[0] * gyro_sensor_[1] + q_[2] * gyro_sensor_[2] - q_[3] * gyro_sensor_[1]) * half_dt;
    dq_[2] = ( q_[0] * gyro_sensor_[1] - q_[1] * gyro_sensor_[2] + q_[3] * gyro_sensor_[0]) * half_dt;
    dq_[3] = ( q_[0] * gyro_sensor_[2] + q_[1] * gyro_sensor_[1] - q_[2] * gyro_sensor_[0]) * half_dt;

    for (uint8_t i = 0; i < 4; i++)
    {
        q_[i] += dq_[i];
    }

    float mod=sqrt(q_[0]*q_[0]+q_[1]*q_[1]+q_[2]*q_[2]+q_[3]*q_[3]);
    for (uint8_t i = 0; i < 4; i++)
    {
        q_[i] /= mod;
    }
}
