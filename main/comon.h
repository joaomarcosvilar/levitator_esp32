#ifndef COMON_H
#define COMON_H

#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"

#define FAN_GPIO        GPIO_NUM_27

#define I2C_NUM         I2C_NUM_0
#define I2C_SDA_GPIO    GPIO_NUM_21
#define I2C_SCL_GPIO    GPIO_NUM_22

#define PWM_CHANNEL          LEDC_CHANNEL_0
#define PWM_TIMER            LEDC_TIMER_0
#define PWM_MODE             LEDC_LOW_SPEED_MODE
#define PWM_RESOLUTION       LEDC_TIMER_13_BIT  // Resolução de 0 a 1023
#define PWM_FREQUENCY        (5000)        // 5 kHz

#endif