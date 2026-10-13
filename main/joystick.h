/**
 * Tiny-Drone-Controller Firmware
 *
 * Copyright (C) 2025-2026 Jonny Chan (LEKAIWU)
 *
 * SPDX-License-Identifier: GPL-3.0-only
 *
 * This file is part of Tiny-Drone-Controller.
 *
 * Tiny-Drone-Controller is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as published
 * by the Free Software Foundation.
 *
 * Tiny-Drone-Controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Tiny-Drone-Controller. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h" 

#define ADC_SAMPLE_COUNT_INIT        10
#define ADC_SAMPLE_COUNT             10
#define ADC_VALID_MIN                10      
#define ADC_VALID_MAX                4095//(3450+6)     

#define CONFIG_OUT_MAX_VAL           900//(900.0f)

#define SENSOR_ADC_UNIT    ADC_UNIT_1 
 
#define JOY_X_ADC_CHANNEL  ADC_CHANNEL_3    
#define JOY_Y_ADC_CHANNEL  ADC_CHANNEL_2     

typedef enum 
{
    JOY_THRUST,
    JOY_DIRECTION,
}joystick_type_t;

typedef struct 
{
    uint8_t type;
    int16_t x;          // X轴偏移量（-100 ~ 100）
    int16_t y;          // Y轴偏移量（-100 ~ 100）

    int16_t x_max;
    int16_t x_min; 
    int16_t y_max;
    int16_t y_min;

    uint8_t x_reserved;
    uint8_t y_reserved;

    uint8_t x_adc_channel;
    uint8_t y_adc_channel;
    uint16_t x_raw;     // X轴原始ADC值（0 ~ 4095）
    uint16_t y_raw;     // Y轴原始ADC值（0 ~ 4095） 
    uint16_t x_mid;     // X轴中间原始值
    uint16_t y_mid;     // Y轴中间原始值
    uint16_t dead_zone; // 死区阈值（避免漂移，建议 50~100） 
} joystick_t;
 
void joystick_init(joystick_t *joy);             // 初始化（ADC + GPIO）
void joystick_calibrate(joystick_t *joy); // 校准中间值
void joystick_read(joystick_t *joy); // 读取摇杆数据（原始值 + 处理后偏移量）  


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif//__JOYSTICK_H
