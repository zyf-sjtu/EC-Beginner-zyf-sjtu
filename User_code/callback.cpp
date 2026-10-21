//
// Created by zyfsj on 2026/10/20.
//

#include "callback.h"
#include "IMU.h"
#include "user_code.h"

IMU imu;
extern "C" void userCallback()
{
    imu.handleSensor();
    imu.updateQuaternion();
}
