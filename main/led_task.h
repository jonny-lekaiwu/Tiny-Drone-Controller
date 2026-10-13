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

#ifndef LED_TASK_H
#define LED_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h" 

typedef enum 
{
    LS_FLASH_FAST,
    LS_FLASH,
    LS_FLASH_LOOP,
    LS_ON, 
}led_state_t; 

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif//LED_TASK_H
