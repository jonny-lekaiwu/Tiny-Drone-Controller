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

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>    
#include "esp_log.h"  
 
 
#include "joystick.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
  
static adc_oneshot_unit_handle_t adc_handle;

void joy_stick_adc_init(uint8_t adc_unit) 
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = adc_unit,
    };

    adc_oneshot_new_unit(&init_cfg, &adc_handle);
}

void joystick_init(joystick_t *joy) 
{
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,  // 12-bit
        .atten = ADC_ATTEN_DB_11//ADC_ATTEN_DB_11         
    };

    adc_oneshot_config_channel(adc_handle, joy->x_adc_channel, &chan_cfg); 
    adc_oneshot_config_channel(adc_handle, joy->y_adc_channel, &chan_cfg); 
    
    joystick_calibrate(joy); 
}

uint16_t adc_read_filtered(adc_oneshot_unit_handle_t adc_handle, adc_channel_t channel)
{
    int samples[ADC_SAMPLE_COUNT];
    int32_t valid_sum = 0; 
    uint16_t val = 0; 

    int res = adc_oneshot_read(adc_handle, channel, (int *)&val);

    if ((0 != res) || (val>4095)) 
    {
        printf("val:%d\r\n", val);
    } 

    uint16_t min=val;
    uint16_t max=val;

    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) 
    {  
        adc_oneshot_read(adc_handle, channel, (int *)&val);
        vTaskDelay(pdMS_TO_TICKS(1));
 
        samples[i] = val; 
        valid_sum += val; 

        if (val<min)
        {
            min=val;
        }
        if (val>max)
        {
            max=val;
        }
    } 

    valid_sum-=(min+max);

    if (valid_sum<0)
    {
        valid_sum=0;
    }

    uint8_t cnt = ADC_SAMPLE_COUNT-2; 

    return valid_sum / cnt;
}

void joystick_calibrate(joystick_t *joy) 
{
    uint32_t x_sum = 0, y_sum = 0; 
    
    for (uint8_t i = 0; i<ADC_SAMPLE_COUNT_INIT; i++) 
    { 
        x_sum += adc_read_filtered(adc_handle, joy->x_adc_channel);  
        y_sum += adc_read_filtered(adc_handle, joy->y_adc_channel); 
 
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    joy->x_mid = x_sum / ADC_SAMPLE_COUNT_INIT;
    joy->y_mid = y_sum / ADC_SAMPLE_COUNT_INIT; 
}


void joystick_read(joystick_t *joy) 
{   
    joy->x_raw = adc_read_filtered(adc_handle, joy->x_adc_channel);
    
    if (joy->x_raw>4095)
    printf("joy->x_raw:%d\r\n", joy->x_raw);

    joy->y_raw = adc_read_filtered(adc_handle, joy->y_adc_channel);  
    // 2. 处理X轴偏移量（映射为-100~100，过滤死区）
    if (abs(joy->x_raw - joy->x_mid) < joy->dead_zone) 
    { 
        joy->x = 0; // 死区内视为居中
    } 
    else 
    { 
        int16_t max_val = CONFIG_OUT_MAX_VAL;

        if ((-1 != joy->x_min) && (-1 != joy->x_max))
        {
            max_val = joy->x_max;
        }

        //最右边 8 输出是-90  但要求是 90
        //最左边 4095 输出是90 但要求是 -90  
 
        if (joy->x_raw > joy->x_mid) 
        {
            joy->x = (int16_t)(((joy->x_raw - joy->x_mid) * max_val) / (ADC_VALID_MAX - joy->x_mid));
        } 
        else 
        {
            joy->x = (int16_t)(((joy->x_raw - joy->x_mid) * max_val) / joy->x_mid);
        } 

        if (joy->x_reserved)
        {
            joy->x=-joy->x;
        }
    } 
 
    if (abs(joy->y_raw - joy->y_mid) < joy->dead_zone) 
    {
        joy->y = 0;
    } 
    else 
    {
        int16_t max_val = CONFIG_OUT_MAX_VAL;

        if ((-1 != joy->y_min) && (-1 != joy->y_max))
        {
            max_val = joy->y_max;
        }

        if (joy->y_raw > joy->y_mid) 
        {
            joy->y = (int16_t)(((joy->y_raw - joy->y_mid) * max_val) / (ADC_VALID_MAX - joy->y_mid));
        } 
        else 
        {
            joy->y = (int16_t)(((joy->y_raw - joy->y_mid) * max_val) / joy->y_mid);
        }

        if (joy->y_reserved)
        {
            joy->y=-joy->y;
        }
    } 
}
