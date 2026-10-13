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

#include "app_common.h" 

#include "key_task.h"

#include "key_handle.h"
 
#define TAG "key_task"  

extern EventGroupHandle_t key_event_group;

static void key_event(key_handle_t key)
{        
    //printf("key->event:%d\r\n", key->event);

    switch (key->event)
    {    
        case SINGLE_CLICK:
        {       
            xEventGroupSetBits(key_event_group, SYS_KEY_CLICKED);
        } 
        break; 

        case LONG_PRESS_HOLD:
        {        
            xEventGroupSetBits(key_event_group, SYS_KEY_LONG_PRESSED);  
        } 
        break;  

        case DOUBLE_CLICK:
        { 
            xEventGroupSetBits(key_event_group, SYS_KEY_DOUBLE_CLICKED);  
        }
        break;

        default:break;
    }
} 

void key_task(void *parameter) 
{  
    struct key_handle sw; 

    gpio_reset_pin(CONFIG_PIN_YAM_LOCK_KEY); 

    const gpio_config_t io_key_config = 
    {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << CONFIG_PIN_YAM_LOCK_KEY) ,
        .pull_down_en = false,
        .pull_up_en = true,
    }; 

    gpio_config(&io_key_config); 

    key_handle_set_read_cb((pin_read_callback_t)gpio_get_level);   
 
    uint32_t key_mask = (KH_BIT(PRESS_DOWN)|KH_BIT(PRESS_UP)|KH_BIT(LONG_PRESS_HOLD)|KH_BIT(SINGLE_CLICK)|KH_BIT(DOUBLE_CLICK));

    key_handle_register(&sw, key_event, CONFIG_PIN_YAM_LOCK_KEY, 0, key_mask); 

    for (;;) 
    {   
        vTaskDelay(pdMS_TO_TICKS(10));

        key_handle_loop();  
    } 

    vTaskDelete(NULL);
}
