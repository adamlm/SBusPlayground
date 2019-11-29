//
// Created by morrissettal2 on 11/29/19.
//

#include <stdint.h>
#include <stm32f4xx_hal.h>

extern TIM_HandleTypeDef htim2;

uint32_t runtimeCounter = 0U;

void configureTimerForRunTimeStats(void) {
    runtimeCounter = 0;
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    __HAL_TIM_ENABLE(&htim2);
}

unsigned long getRunTimeCounterValue(void) {
    return runtimeCounter;
}

void incRuntimeCounter(void) {
    runtimeCounter++;
}