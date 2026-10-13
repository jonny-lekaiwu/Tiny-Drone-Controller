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

#include "led_task.h"
 
#define USING_GPIO_REG 1
 
#if USING_GPIO_REG
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/soc.h"
#include "rom/ets_sys.h"

#include "driver/gpio.h"
#include "driver/dedic_gpio.h"

#if CONFIG_USING_GPIO_DEDIC
dedic_gpio_bundle_handle_t bundle;  
#endif

#define LED_ACTIVE_LOW 0   // 1: 低电平点亮（常见） 0: 高电平点亮

static inline void led_gpio_init(void)
{
    // /* 1. 选择 GPIO 功能 */
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[CONFIG_PIN_LED], PIN_FUNC_GPIO);
 
    REG_CLR_BIT(GPIO_PIN_MUX_REG[CONFIG_PIN_LED], FUN_PU);
    REG_CLR_BIT(GPIO_PIN_MUX_REG[CONFIG_PIN_LED], FUN_PD);

    /* 3. 清输出寄存器 */
    REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << CONFIG_PIN_LED);

    /* 4. 使能输出（推挽） */
    REG_WRITE(GPIO_ENABLE_W1TS_REG, 1UL << CONFIG_PIN_LED);

#if LED_ACTIVE_LOW
    /* 默认关灯 */
    REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << CONFIG_PIN_LED);
#endif
    
    #if CONFIG_USING_GPIO_DEDIC
    // 建立 dedic_gpio bundle（只包含一个 GPIO）
    int gpio_array[] = { CONFIG_PIN_LED };
    dedic_gpio_bundle_config_t cfg = {
        .gpio_array = gpio_array,
        .array_size = 1,
        .flags = {
            .out_en = 1,
            .in_en  = 0,
        },
    };

    dedic_gpio_new_bundle(&cfg, &bundle);
    #endif
}
#endif

static uint8_t led_state = LS_FLASH_FAST; 

static void gpio_init(void)
{
    #if USING_GPIO_REG
    led_gpio_init();
    #else
    gpio_config_t io_config = {
        .mode = GPIO_MODE_INPUT_OUTPUT,  
    };
  
    io_config.pin_bit_mask = 1ULL << CONFIG_PIN_LED;
    gpio_config(&io_config);
    #endif
}

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static inline void gpio_set(uint8_t pin, uint8_t val)
{  
    #if USING_GPIO_REG
    if (!val)
    { 
        #if CONFIG_USING_GPIO_DEDIC 
        dedic_gpio_bundle_write(bundle, 0x1, 0x0);
        #else
        REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << pin);
        #endif
    }
    else
    {
        #if CONFIG_USING_GPIO_DEDIC 
        dedic_gpio_bundle_write(bundle, 0x1, 0x1);
        #else
        REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << pin);
        #endif
    } 
    #else
    gpio_set_level(pin, val);
    #endif 
}

void led_task(void *parameter) 
{
    uint32_t lastwake = xTaskGetTickCount();

    gpio_init();

    uint32_t tick_led = xTaskGetTickCount();
    uint8_t led_level = gpio_get_level(CONFIG_PIN_LED); 

    for(;;) 
    {
        #if 1
        uint32_t now = xTaskGetTickCount(); 

        static uint8_t led_state_last = 0xff; 

        uint8_t wf_is_connected = wifi_is_connected();
        uint8_t udp_connected = udp_is_connected();

        _sys_lock(0);
        uint8_t sys_state = sys_get_state();
        _sys_unlock();
 
        led_level = gpio_get_level(CONFIG_PIN_LED); 

        switch (led_state)
        {
            case LS_FLASH_FAST:
            {   
                if (wf_is_connected)
                {
                    if (udp_connected)
                    {
                        led_state = LS_FLASH_LOOP;
                    }
                    else
                    {
                        led_state = LS_FLASH;
                    }

                    break;
                }

                if ((now - tick_led) >= (pdMS_TO_TICKS(200)))
                {
                    tick_led = now;

                    led_level=!led_level;
 
                    gpio_set(CONFIG_PIN_LED, led_level); 
                }
            }
            break;

            case LS_FLASH:
            {
                if (wf_is_connected)
                {
                    if (udp_connected)
                    {
                        led_state = LS_FLASH_LOOP;

                        break;
                    } 
                }
                else
                {
                    led_state = LS_FLASH_FAST;

                    break;
                }

                if ((now - tick_led) >= (pdMS_TO_TICKS(1000)))
                {
                    tick_led = now;

                    led_level=!led_level;

                    gpio_set(CONFIG_PIN_LED, led_level);
                }
            }
            break;

            case LS_FLASH_LOOP:
            { 
                if (S_LOCKED == sys_state)
                {
                    led_state = LS_ON;

                    break;
                }

                if (wf_is_connected)
                {
                    if (!udp_connected)
                    {
                        led_state = LS_FLASH;

                        break;
                    } 
                }
                else
                {
                    led_state = LS_FLASH_FAST;

                    break;
                }

                uint8_t loop_cnt = sys_yaw_is_locked()?3:2;

                for (uint8_t i=0; i<loop_cnt; i++)
                {
                    gpio_set(CONFIG_PIN_LED, CONFIG_LED_ACTIVE_LEVEL);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    gpio_set(CONFIG_PIN_LED, !CONFIG_LED_ACTIVE_LEVEL);
                    vTaskDelay(pdMS_TO_TICKS(200));
                } 

                vTaskDelay(pdMS_TO_TICKS(600));
            }
            break;

            case LS_ON:
            {
                if (S_FREE_CONTROL == sys_state)
                {
                    led_state = LS_FLASH_LOOP;

                    break;
                }
                
                if ((now - tick_led) >= (pdMS_TO_TICKS(1000)))
                {
                    tick_led = now;

                    if (led_level != CONFIG_LED_ACTIVE_LEVEL)
                    {
                        gpio_set(CONFIG_PIN_LED, CONFIG_LED_ACTIVE_LEVEL);
                    } 
                }
            }
            break;

            default:break;
        } 
 
        if (led_state_last != led_state)
        { 
            led_state_last = led_state;
        }
 
        vTaskDelayUntil(&lastwake, pdMS_TO_TICKS(50));
        #else
         
        #endif
    }
}
