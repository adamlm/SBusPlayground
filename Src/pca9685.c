//
// Created by morrissettal2 on 11/28/19.
//

#include <cmsis_os2.h>
#include <FreeRTOS.h>
#include "pca9685.h"

static uint8_t buff[5];

void PCA9685_Reset(I2C_HandleTypeDef *hi2c) {
    buff[0] = PCA9685_MODE1;
    buff[1] = MODE1_RESTART;

    HAL_I2C_Master_Transmit(hi2c, PCA9685_I2C_ADDRESS << 1U, buff, 2U, 100);
    osDelay(10 / (portTICK_RATE_MS));
}

void PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t num, uint16_t on, uint16_t off) {
    buff[0] = PCA9685_LED0_ON_L + 4 * num;
    buff[1] = on & 0xFFU;  // On
    buff[2] = on >> 8U;  // On
    buff[3] = off & 0xFFU;  // Off
    buff[4] = off >> 8U;  // Off

    HAL_I2C_Master_Transmit(hi2c, 0x40U << 1U, buff, 5U, 100U);
}

void PCA9685_SetPWMFreq(I2C_HandleTypeDef *hi2c1, float freq) {
//    void PCA9685_SetPWMFreq(I2C_HandleTypeDef *hi2c1, float *freq) {
//    uint8_t buff[2];  // Tx and Rx buffer
    uint8_t oldMode = 0x10U;

    // Range output modulation frequency is dependant on oscillator
    if (freq < 1)
        freq = 1;
    if (freq > 3500)
        freq = 3500;  // Datasheet limit is 3052=50MHz/(4*4096)

    // Calculate prescale value
    float prescaleVal = (float)((FREQUENCY_OSCILLATOR / (freq * 4096.0)) + 0.5) - 1;
    if (prescaleVal < PCA9685_PRESCALE_MIN)
        prescaleVal = PCA9685_PRESCALE_MIN;
    if (prescaleVal > PCA9685_PRESCALE_MAX)
        prescaleVal = PCA9685_PRESCALE_MAX;
    uint8_t prescale = (uint8_t)prescaleVal;

    // Read current mode
    buff[0] = PCA9685_MODE1;
    HAL_I2C_Master_Transmit(hi2c1, PCA9685_I2C_ADDRESS << 1U, buff, 1U, 100);
    HAL_I2C_Master_Receive(hi2c1, PCA9685_I2C_ADDRESS << 1U, &oldMode, 1U, 100);

    // Calculate new mode
    uint8_t newMode = (oldMode & ~MODE1_RESTART) | MODE1_SLEEP;

    // Transmit new mode
    buff[0] = PCA9685_MODE1;
    buff[1] = newMode;
    HAL_I2C_Master_Transmit(hi2c1, PCA9685_I2C_ADDRESS << 1U, buff, 2U, 100);

    // Transmit new prescale value
    buff[0] = PCA9685_PRESCALE;
    buff[1] = prescale;
    HAL_I2C_Master_Transmit(hi2c1, PCA9685_I2C_ADDRESS << 1U, buff, 2U, 100);

    // Transmit old mode
    buff[0] = PCA9685_MODE1;
    buff[1] = oldMode;
    HAL_I2C_Master_Transmit(hi2c1, PCA9685_I2C_ADDRESS << 1U, buff, 2U, 100);

    // Wait for updates to take
    osDelay(5 / (portTICK_RATE_MS));

    // This sets the MODE1 register to turn on auto increment.
    buff[0] = PCA9685_MODE1;
    buff[1] = oldMode | MODE1_RESTART | MODE1_AI;
    HAL_I2C_Master_Transmit(hi2c1, PCA9685_I2C_ADDRESS << 1U, buff, 2U, 100);
}
