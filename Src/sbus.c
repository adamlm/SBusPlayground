//
// Created by morrissettal2 on 11/27/19.
//

#include "sbus.h"

#include <cmsis_os2.h>
#include <stm32f407xx.h>
#include <stm32f4xx_hal.h>

extern osSemaphoreId_t SBusFrameHandle;

extern UART_HandleTypeDef huart4;

static uint8_t inFrame = 0U;
static uint8_t frame[SBUS_FRAME_SIZE] = {0x00U};
static uint8_t frameIndex = 0U;

static SBusPacketTypeDef packet;

static uint16_t lastCapture = 0U;
static uint16_t now = 0U;

void SBus_ParseByte(uint8_t byte) {
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(&huart4);
        inFrame = 0U;
        frameIndex = 0;
    }

    if (!inFrame && byte == SBUS_START_BYTE) {
        frameIndex = 0U;
        inFrame = 1U;
    }

    if (inFrame) {
        frame[frameIndex++] = byte;

        if (frameIndex >= SBUS_FRAME_SIZE) {
            if (byte == SBUS_END_BYTE) {
                osSemaphoreRelease(SBusFrameHandle);
            }

            frameIndex = 0U;
            inFrame = 0U;
        }
    }
}

//void SBus_ParseByte(uint8_t byte) {
//    now = TIM2->CNT;
//    if (now < lastCapture) {
//        now += 19999;
//    }
//    if ((now - lastCapture) > 3000) {
//        frameIndex = 0;
//    }
//
//    frame[frameIndex++] = byte;
//    lastCapture = now;
//
//    if (frameIndex >= SBUS_FRAME_SIZE) {
//        if (frame[frameIndex] == SBUS_START_BYTE) {
//            osSemaphoreRelease(SBusDataHandle);
//        }
//        frameIndex = 0;
//    }
//}

void SBus_DecodeFrame(void) {
    packet.channel0 = ((frame[2] & 0x07U) << 8U) | frame[1];
    packet.channel1 = ((frame[3] & 0x3FU) << 5U) | ((frame[2] & 0xF8U) >> 3U);
    packet.channel2 = ((frame[5] & 0x01U) << 10U) | ((frame[4] & 0xFFU) << 2U) | ((frame[3] & 0xC0U) >> 6U);
    packet.channel3 = ((frame[6] & 0x0FU) << 7U) | ((frame[5] & 0xFEU) >> 1U);
    packet.channel4 = ((frame[7] & 0x7FU) << 4U) | ((frame[6] & 0xF0U) >> 4U);
    packet.channel5 = ((frame[9] & 0x03U) << 9U) | ((frame[8] & 0xFFU) << 1U) | ((frame[7] & 0x80U) >> 7U);
    packet.channel6 = ((frame[10] & 0x1FU) << 6U) | ((frame[9] & 0xFCU) >> 2U);
    packet.channel7 = ((frame[2] & 0xFFU) << 3U) | ((frame[10] & 0xE0U) >> 5U);
    packet.channel8 = ((frame[13] & 0x03U) << 8U) | frame[12];
    packet.channel9 = ((frame[14] & 0x3FU) << 5U) | ((frame[13] & 0xF8U) >> 3U);
    packet.channel10 = ((frame[16] & 0x01U) << 10U) | ((frame[15] & 0xFFU) << 2U) | ((frame[14] & 0xC0U) >> 6U);
    packet.channel11 = ((frame[17] & 0x0FU) << 7U) | ((frame[16] & 0xFEU) >> 1U);
    packet.channel12 = ((frame[18] & 0x7FU) << 4U) | ((frame[17] & 0xF0U) >> 4U);
    packet.channel13 = ((frame[20] & 0x03U) << 9U) | ((frame[19] & 0xFFU) << 1U) | ((frame[18] & 0x80U) >> 7U);
    packet.channel14 = ((frame[21] & 0x1FU) << 6U) | ((frame[20] & 0xFCU) >> 2U);
    packet.channel15 = ((frame[22] & 0xFFU) << 3U) | ((frame[21] & 0xE0U) >> 5U);
}

uint16_t SBus_GetChannel(uint8_t channel) {
    switch (channel) {
        case 0U:
            return packet.channel0;
            break;
        case 1U:
            return packet.channel1;
        default:
            return 0;
    }
//    return packet.channel0;
}
