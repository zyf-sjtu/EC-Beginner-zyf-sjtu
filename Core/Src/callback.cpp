//
// Created by zyfsj on 2026/10/2.
//
#include "main.h"
#include "usart.h"

uint8_t rxmessage[1];
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart==&huart1&&(HAL_UARTEx_GetRxEventType(&huart1)==HAL_UART_RXEVENT_IDLE||HAL_UARTEx_GetRxEventType(huart)==HAL_UART_RXEVENT_TC))
    {
        HAL_UART_Transmit_IT(&huart1,rxmessage,Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rxmessage,10);
    }
}