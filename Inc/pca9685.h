//
// Created by morrissettal2 on 11/28/19.
//

#ifndef SBUSPLAYGROUND_PCA9685_H
#define SBUSPLAYGROUND_PCA9685_H

#include "stm32f4xx_hal.h"

// Register Addresses
#define PCA9685_MODE1 0x00U  // Mode Register 1
#define PCA9685_LED0_ON_L 0x06U  // LED0 on tick, low byte
#define PCA9685_PRESCALE 0xFEU  // Prescaler for PWM output frequency

// MODE1 Bits
#define MODE1_SLEEP 0x10U  // Low power mode. Oscillator off
#define MODE1_AI 0x20U  // Auto-Increment enabled
#define MODE1_RESTART 0x80U  // Restart enabled

// PCA9685 Defaults
#define PCA9685_I2C_ADDRESS 0x40U  // Default PCA9685 I2C Slave Address
#define FREQUENCY_OSCILLATOR 25000000  // Internal oscillator frequency in data sheet
#define PCA9685_PRESCALE_MIN 3  // Minimum prescale value
#define PCA9685_PRESCALE_MAX 255  // Maximum prescale value

// Steering Servo Defaults
#define STEER_SERVO_PIN 15U  // Servo pin on PCA9685
#define STEER_SERVO_MIN STEER_SERVO_NEUTRAL - 50U
#define STEER_SERVO_NEUTRAL 352U
#define STEER_SERVO_MAX STEER_SERVO_NEUTRAL + 25U

void PCA9685_Reset(I2C_HandleTypeDef *hi2c);
void PCA9685_SetPWMFreq(I2C_HandleTypeDef *hi2c, float freq);
void PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t num, uint16_t on, uint16_t off);

#endif //SBUSPLAYGROUND_PCA9685_H
