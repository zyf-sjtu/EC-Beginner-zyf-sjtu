//
// Created by zyfsj on 2026/10/3.
//
#include <stdint.h>

#ifndef CAN_CALLBACK_H
#define CAN_CALLBACK_H

#ifdef __cplusplus

extern "C" {
#endif
void UsercanRxMsgCallback(const uint8_t* rxdata);
void UsersetTxCurrent(uint8_t motor_id,float angle_ref);
#ifdef __cplusplus
}
#endif

#endif //CAN_CALLBACK_H
