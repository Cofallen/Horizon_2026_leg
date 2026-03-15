#include "board2board.h"
#include "main.h"
#include "can_bsp.h"
#include "string.h"

boardRxData_t boardRxData = {0};
boardTxData_t boardTxData = {0};

// 目标值 dbus: yaw pitch s1 s2
void Board_to_board_send(boardTxData_t* send, int16_t ch2, int16_t ch3, int16_t dir, uint8_t s1, uint8_t s2, float pitch)
{
    uint64_t packed = 0;
    packed |= (uint64_t)(ch2 & 0x07ff) << 0;
    packed |= (uint64_t)(ch3 & 0x07ff) << 11;
    packed |= (uint64_t)(dir & 0x07ff) << 22;
    packed |= (uint64_t)((int16_t)(pitch * 100.0f) & 0xffff) << 33;
    packed |= (uint64_t)(s1 & 0x03) << 49;
    packed |= (uint64_t)(s2 & 0x03) << 51;

    for (int i = 0; i < 8; i++) {
        send->sendData[i] = (packed >> (i * 8)) & 0xFF;
    }
    // canx_send_data(&hfdcan3, 0x1A, send->sendData, sizeof(send->sendData));
    canx_send_data(&hfdcan3, 0x1A, send->sendData, sizeof(send->sendData));
    // canx_send_data(&hfdcan3, 0x200, send->sendData, sizeof(send->sendData));
    send->dataNeaten.pitch = (int16_t)(pitch * 100.0f);     // 放缩100倍
}

// 上板 imu: yaw
void Board_to_board_recv(boardRxData_t* recv, uint8_t *data)
{
    // memcpy(recv, data, 8);
    uint32_t tmp = (data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24);
    recv->dataNeaten.yaw_imu = *(float*) &tmp;
}