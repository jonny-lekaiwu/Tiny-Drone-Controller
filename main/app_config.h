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

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif 

#define CONFIG_PIN_YAM_LOCK_KEY       20
#define CONFIG_PIN_LED                10
#define CONFIG_LED_ACTIVE_LEVEL       0
 
#define CONFIG_KEY_I2C_SDA            18
#define CONFIG_KEY_I2C_SCL            19 

#define CONFIG_PIN_LCD_DC             5
#define CONFIG_PIN_LCD_CS             6 
#define CONFIG_PIN_LCD_SCK            8
#define CONFIG_PIN_LCD_MOSI           7
#define CONFIG_PIN_LCD_RST            9 


#define CONFIG_NVS_SIGN_MAGIC (0xBEEFDDEE)


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif//APP_CONFIG_H
