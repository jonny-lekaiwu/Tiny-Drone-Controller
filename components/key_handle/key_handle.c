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

#include "key_handle.h"

#include "string.h"

#define NULL 0
 

static key_handle_t _key_slist_head = NULL;   

static pin_read_callback_t s_pin_read_cb;
static volatile uint32_t tick_cnt = 0;

static int8_t key_handle_init(struct key_handle *key, uint8_t set_pin, uint8_t active_level)
{
    if (key == NULL)
    {
        return -1;
    }
        
    memset(key, 0, sizeof(struct key_handle)); 
    
    key->event = NONE_PRESS;
    key->active_level = active_level; 
    key->pin = set_pin;
    key->key_level = s_pin_read_cb(set_pin); 
    
    return 0;
} 

static int8_t key_event_callback_register(struct key_handle *key, uint32_t event, event_callback_t set_cb)
{
    if (key == NULL || event >= EVENT_SUM)
    {
        return -1;
    } 

    key->event_callback[event] = set_cb;

    return 0;
}

int8_t key_handle_set_read_cb(pin_read_callback_t _pin_read_cb)
{
    if (!_pin_read_cb)
    {
        return -1;
    }

    s_pin_read_cb = _pin_read_cb;

    tick_cnt = 0; 
 
    return 0;
}
 
#define ADD_KEY_HANDLE_CB(MASK, KEY, EVENT, CB)              \
    do {                                                     \
        if (KH_BIT(EVENT) & MASK) {                   \
            key_event_callback_register(KEY, EVENT, CB);     \ 
        }                                                    \
    } while (0U)

int8_t key_handle_register(struct key_handle *key, event_callback_t set_cb, uint8_t set_pin, uint8_t active_level, uint32_t key_mask)
{ 
    if (!s_pin_read_cb)
    {
        return -1;
    }

    struct key_handle *key_slist_node = _key_slist_head;  //  
    
    key_handle_init(key, set_pin, active_level); 

    for (uint8_t event_id=PRESS_DOWN; event_id<KEY_EVENT_MAX; event_id++)
    {
        ADD_KEY_HANDLE_CB(key_mask, key, event_id, set_cb); 
    }
 
    if (key == NULL)
    {
        return -1;
    } 
 
    if (_key_slist_head == NULL) 
    {
        _key_slist_head = key;
        key->next = NULL;
        
        return 0;
    }

    while (key_slist_node)
    {
        if (key_slist_node == key) return -1;     
        if (key_slist_node->next == NULL) break;  

        key_slist_node = key_slist_node->next;
    }

    key_slist_node->next = key;  
    key->next = NULL; 

    return 0;
}

static void key_handler(struct key_handle *key)
{
    uint8_t key_level_temp = s_pin_read_cb(key->pin);

    if (key->state != INIT_NONE_STATE) key->tick++;
    
    if (key_level_temp != key->key_level)
    {
        if (++(key->debounce_tick) >= DOUBLE_CLICK)
        { 
            key->key_level = key_level_temp;
            key->debounce_tick = 0;
        }
    }
    else
    {
        key->debounce_tick = 0;
    }
 
    switch (key->state)
    {
        case INIT_NONE_STATE:
        {
            if (key->key_level == key->active_level)   
            {
                key->event = (uint8_t)PRESS_DOWN;
                __KEY_EVENT_CALL(PRESS_DOWN);

                key->tick  = 0;
                key->repeat_cnt = 1;
                key->state  = INIT_PRESS_STATE;  
            }
            else
            {
                key->event = (uint8_t)NONE_PRESS;
            }    
        }
        break;

        case INIT_PRESS_STATE:
        {
            if (key->key_level != key->active_level)
            {
                key->event = (uint8_t)PRESS_UP;
                __KEY_EVENT_CALL(PRESS_UP);
                key->tick = 0;
                key->state = PRESS_CHECK_STATE; 

            }
            else if (key->tick > LONG_PRESS_START_TICK)
            { 
                key->event = (uint8_t)LONG_PRESS_START;
                __KEY_EVENT_CALL(LONG_PRESS_START);
                key->state = LONG_PRESS_STATE; 
            }
        }
        break;

        case PRESS_CHECK_STATE:
        {
            if (key->key_level == key->active_level)
            {
                key->event = (uint8_t)PRESS_DOWN;
                __KEY_EVENT_CALL(PRESS_DOWN);
                key->repeat_cnt++;
                __KEY_EVENT_CALL(SHORT_PRESS_REPEAT);
                key->tick = 0;
                key->state = CONTINUOUS_PRESS_STATE; 
            } 
            
            else if (key->tick > SHORT_PRESS_START_TICK)
            {                
                if (key->repeat_cnt == 1)
                {
                    key->event = (uint8_t)SINGLE_CLICK;
                    __KEY_EVENT_CALL(SINGLE_CLICK);
                }
                
                else if (key->repeat_cnt == 2)
                {
                    key->event = (uint8_t)DOUBLE_CLICK;
                    __KEY_EVENT_CALL(DOUBLE_CLICK);
                }
                key->state = INIT_NONE_STATE; 
            }
        }  
        break;

        case CONTINUOUS_PRESS_STATE:
        {
            if (key->key_level != key->active_level)
            {
                key->event = (uint8_t)PRESS_UP;
                __KEY_EVENT_CALL(PRESS_UP);

                if (key->tick < SHORT_PRESS_START_TICK)
                {
                    key->tick = 0;
                    key->state = PRESS_CHECK_STATE;
                }
                else
                {
                    key->state = INIT_NONE_STATE;
                }
            }
            else if (key->tick > SHORT_PRESS_START_TICK)
            {
                key->state = INIT_PRESS_STATE; 
            } 
        } 
        break;

        case LONG_PRESS_STATE:
        {
            if (key->key_level == key->active_level)
            {
                key->event = (uint8_t)LONG_PRESS_HOLD; 
                if ((key->tick % LONG_HOLD_CYCLE_TICK) == 0)     
                {
                    __KEY_EVENT_CALL(LONG_PRESS_HOLD);
                }
            }
            else
            {
                key->event = (uint8_t)PRESS_UP; 
                __KEY_EVENT_CALL(PRESS_UP);

                key->state = INIT_NONE_STATE;
            }
        }
        break;

        default:break;
    }  
}

void key_handle_loop(void)
{
    struct key_handle *key_slist_node;

    if (++tick_cnt < CHECK_TICK)
        return;
    
    for (key_slist_node = _key_slist_head; key_slist_node != NULL; key_slist_node = key_slist_node->next)
    {
        key_handler(key_slist_node);
    }

    tick_cnt = 0;
}