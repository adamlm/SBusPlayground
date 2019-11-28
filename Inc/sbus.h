//
// Created by morrissettal2 on 11/27/19.
//

#ifndef SBUSPLAYGROUND_SBUS_H
#define SBUSPLAYGROUND_SBUS_H

#include <stdint.h>

#define SBUS_FRAME_SIZE 25U
#define SBUS_START_BYTE 0x0FU
#define SBUS_END_BYTE 0x00U

typedef struct {
    uint16_t channel0;
    uint16_t channel1;
    uint16_t channel2;
    uint16_t channel3;
    uint16_t channel4;
    uint16_t channel5;
    uint16_t channel6;
    uint16_t channel7;
    uint16_t channel8;
    uint16_t channel9;
    uint16_t channel10;
    uint16_t channel11;
    uint16_t channel12;
    uint16_t channel13;
    uint16_t channel14;
    uint16_t channel15;
} SBusPacketTypeDef;

void SBus_ParseByte(uint8_t byte);
void SBus_DecodeFrame(void);
uint16_t SBus_GetChannel(uint8_t channel);

#endif //SBUSPLAYGROUND_SBUS_H
