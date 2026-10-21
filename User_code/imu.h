//
// Created by zyfsj on 2026/10/20.
//
#include "main.h"
#ifndef IMU_IMU_H
#define IMU_IMU_H

class IMU
{
    public:
    explicit IMU(void);
    void reset(void);
    void handleSensor(void);
    void updateQuaternion(void);
    bool hasfeedback(void);
    private:
    const float R_imu_[3][3] = {{-1.0f, 0.0f, 0.0f},
                         {0.0f, 1.0f, 0.0f},
                         {0.0f, 0.0f, -1.0f}};
    bool received_=false;
    uint8_t acc_data_[6];
    uint8_t gyro_data_[6];
    volatile uint8_t  ACC_Z_MSB_=0;
    volatile uint8_t  ACC_Y_MSB_=0;
    volatile uint8_t  ACC_X_MSB_=0;
    volatile uint8_t  ACC_Y_LSB_=0;
    volatile uint8_t  ACC_X_LSB_=0;
    volatile uint8_t  ACC_Z_LSB_=0;
    volatile uint8_t  RATE_Z_MSB_=0;
    volatile uint8_t  RATE_Y_MSB_=0;
    volatile uint8_t  RATE_X_MSB_=0;
    volatile uint8_t  RATE_Y_LSB_=0;
    volatile uint8_t  RATE_X_LSB_=0;
    volatile uint8_t  RATE_Z_LSB_=0;
    volatile int16_t acc_x_int_=0, acc_y_int_=0, acc_z_int_=0;
    volatile int16_t gyro_x_int_=0, gyro_y_int_=0, gyro_z_int_=0;
    volatile float acc_[3];
    volatile float gyro_[3];
    volatile float acc_sensor_[3];
    volatile float gyro_sensor_[3];
    float acc_sen_=1.0f;
    float gyro_bias_[3];
    float acc_bias_[3];
    volatile float q_[4]={1.0f, 0.0f, 0.0f, 0.0f};
    volatile float dq_[4]={0.0f, 0.0f, 0.0f, 0.0f};
    float dt_=0.001;
    void decode(void);
    void calc(void);
    void sensorDataTransform(void);

};

#endif //IMU_IMU_H
