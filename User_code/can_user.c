
#include "main.h"
CAN_TxHeaderTypeDef tx_header={
    .StdId=0x200,
    .IDE=CAN_ID_STD,
    .RTR=CAN_RTR_DATA,
    .DLC=8,
    .TransmitGlobalTime=DISABLE
};
CAN_RxHeaderTypeDef rx_header={
    .StdId=0x200,
    .ExtId=0,
    .IDE=CAN_ID_STD,
    .RTR=CAN_RTR_DATA,
    .DLC=8,
    .Timestamp=0,
    .FilterMatchIndex=0
};
CAN_FilterTypeDef can_filter_config={
    .FilterIdHigh=0x0000,
    .FilterIdLow=0x0000,
    .FilterMaskIdHigh=0x0000,
    .FilterMaskIdLow=0x0000,
    .FilterFIFOAssignment=CAN_FILTER_FIFO0,
    .FilterBank=0,
    .FilterMode=CAN_FILTERMODE_IDMASK,
    .FilterScale=CAN_FILTERSCALE_32BIT,
    .FilterActivation=ENABLE,
    .SlaveStartFilterBank=14
};
uint32_t can_tx_mailbox = 0;