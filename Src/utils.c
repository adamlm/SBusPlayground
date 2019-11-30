//
// Created by morrissettal2 on 11/28/19.
//

#include "utils.h"

#include <math.h>
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;

static uint32_t start;

uint16_t sbusToPwm(uint16_t val) {
    return (uint16_t )roundf((float)val*(75.0f/1639) + 294.129f);
}

void delayUs(uint32_t val) {
    start = __HAL_TIM_GET_COUNTER(&htim2);
    while (__HAL_TIM_GET_COUNTER(&htim2) - start < val);
}