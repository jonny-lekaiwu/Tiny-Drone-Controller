
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
#ifndef _KEY_HANDLE_H__
#define _KEY_HANDLE_H__

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>

#define __KEY_EVENT_CALL(event)                 \
    do {                                        \
        if (key->event_callback[event]) {       \
            key->event_callback[event](key);    \
        }                                       \
    } while (0U)


#define CHECK_TICK                      1                       
#define DEBOUNCE_TICK                   (3 / CHECK_TICK)       
#define SHORT_PRESS_START_TICK          (30  / CHECK_TICK)    
#define LONG_PRESS_START_TICK           (100 / CHECK_TICK)     
#define LONG_HOLD_CYCLE_TICK            (50 / CHECK_TICK)      

#if 0
#define CHECK_TICK                      5                      // 按键状态机检查间隔时间(ms)   
#define DEBOUNCE_TICK                   (15 / CHECK_TICK)      // 消抖时间(ms) [最大40/5且DEBOUNCE_TICK < SHORT_PRESS_START_TICK/2]
#define SHORT_PRESS_START_TICK          (300  / CHECK_TICK)    // 连击态触发间隔时间(ms) [最大40 / 5]
#define LONG_PRESS_START_TICK           (1000 / CHECK_TICK)    // 长按态触发时间(ms) 
#define LONG_HOLD_CYCLE_TICK            (500 / CHECK_TICK)     // 长按态保持下连续触发长按事件的间隔(ms) 
#endif

typedef enum  
{
    INIT_NONE_STATE = 0,        
    INIT_PRESS_STATE,           
    PRESS_CHECK_STATE,           
    CONTINUOUS_PRESS_STATE,     
    LONG_PRESS_STATE,            
}key_state_t;
 
#define KH_BIT(BIT)         (1<<BIT)  

typedef enum  
{ 
    PRESS_DOWN=0,              
    PRESS_UP,                   
    SINGLE_CLICK,                
    DOUBLE_CLICK,                
    SHORT_PRESS_REPEAT,         
    LONG_PRESS_START,            
    LONG_PRESS_HOLD,            
    EVENT_SUM,                   
    NONE_PRESS,
    KEY_EVENT_MAX                   
}key_event_t;
 
struct key_handle
{
    uint16_t tick;                                          	   
    uint8_t repeat_cnt     : 4;                             	   
    uint8_t event   	   : 4;                             	   
    uint8_t state          : 3;                             	   
    uint8_t debounce_tick  : 3;                             	   
    uint8_t active_level   : 1;                                   
    uint8_t key_level      : 1;                             	   
    uint8_t pin;
    uint8_t (* pin_read)(uint8_t pin);                          
    void (* event_callback[EVENT_SUM])(struct key_handle* key);    
    struct key_handle* next;                                      
};

typedef struct key_handle *key_handle_t;
typedef void (* event_callback_t)(key_handle_t);
typedef uint8_t (* pin_read_callback_t)(uint8_t);


extern void key_handle_loop(void);
extern int8_t key_handle_set_read_cb(pin_read_callback_t set_pin_read_cb);
extern int8_t key_handle_register(struct key_handle *key, event_callback_t set_cb, uint8_t set_pin, uint8_t active_level, uint32_t key_mask);

#ifdef __cplusplus
}
#endif

#endif //