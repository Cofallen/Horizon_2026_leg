#include "board2board.h"
#include "main.h"
#include "can_bsp.h"
#include "string.h"

boardRxData_t boardRxData = {0};
boardTxData_t boardTxData = {0};

// 目标值 dbus: yaw pitch s1 s2
void Board_to_board_send(boardTxData_t* send, int16_t ch2, int16_t ch3, uint8_t s1, uint8_t s2 )
{
    send->dataNeaten.ch2 = ch2;
    send->dataNeaten.ch3 = ch3;
    send->dataNeaten.s1 = s1;
    send->dataNeaten.s2 = s2;
    // canx_send_data(&hfdcan3, 0x1A, send->sendData, sizeof(send->sendData));
    canx_send_data(&hfdcan3, 0x1FF, send->sendData, sizeof(send->sendData));
    // canx_send_data(&hfdcan3, 0x200, send->sendData, sizeof(send->sendData));
}

// 上板 imu: yaw
void Board_to_board_recv(boardRxData_t* recv, uint8_t *data)
{
    // memcpy(recv, data, 8);
    uint32_t tmp = (data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24);
    recv->dataNeaten.yaw_imu = *(float*) &tmp;
}