#ifndef __BOARD2BOARD_H
#define __BOARD2BOARD_H

#include "main.h"

typedef union 
{
    struct __packed
    {
        int16_t ch2;
        int16_t ch3;
        uint8_t s1;
        uint8_t s2;
        int16_t pitch;
    } dataNeaten;
    uint8_t sendData[8];
}boardTxData_t;

typedef union 
{
    struct __packed
    {
        float yaw_imu;
    } dataNeaten;
    uint8_t rxData[8];
}boardRxData_t;
void Board_to_board_send(boardTxData_t* send, int16_t ch2, int16_t ch3, uint8_t s1, uint8_t s2, float pitch );
void Board_to_board_recv(boardRxData_t* recv, uint8_t *data);

extern boardRxData_t boardRxData;
extern boardTxData_t boardTxData;

#endif // !__BOARD2BOARD_H