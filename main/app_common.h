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

#ifndef APP_COMMON_H
#define APP_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif
 
#include <time.h>
#include <string.h>
#include <assert.h> 
#include <stdio.h>
#include <stdlib.h>    

#include "CJSON.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_chip_info.h" 

#include "driver/gpio.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_periph.h"

#include "wifi_task.h"   

#include "app_config.h"  

#include "web_ota.h"
 
typedef enum 
{
    S_DISCONNECTED,
    S_FREE_CONTROL,
    S_LOCKED,
    S_CONFIG,
    S_TESTING,
}system_state_t;

typedef enum 
{
  ROTATION_0,
  ROTATION_180,
  ROTATION_MAX,
}rotation_t;

typedef struct 
{
  float roll_trim;
  float pitch_trim;
  uint8_t rotation;
  uint8_t carefree;
  uint8_t reserved[62];
}sys_cfg_t;

#define SYS_KEY_CLICKED         BIT1  
#define SYS_KEY_LONG_PRESSED    BIT2    
#define SYS_KEY_DOUBLE_CLICKED  BIT3    


#define CRTP_MAX_DATA_SIZE 30

#define CRTP_HEADER(port, channel) (((port & 0x0F) << 4) | (channel & 0x0F))

#define CRTP_IS_NULL_PACKET(P) ((P.header&0xF3)==0xF3)

typedef enum {
  CRTP_PORT_CONSOLE          = 0x00,
  CRTP_PORT_PARAM            = 0x02,
  CRTP_PORT_SETPOINT         = 0x03,
  CRTP_PORT_MEM              = 0x04,
  CRTP_PORT_LOG              = 0x05,
  CRTP_PORT_LOCALIZATION     = 0x06,
  CRTP_PORT_SETPOINT_GENERIC = 0x07,
  CRTP_PORT_SETPOINT_HL      = 0x08,
  CRTP_PORT_PLATFORM         = 0x0D,
  CRTP_PORT_LINK             = 0x0F,
} CRTPPort;

enum TrajectoryCommand_e {
  COMMAND_SET_GROUP_MASK          = 0,
  COMMAND_TAKEOFF                 = 1, // Deprecated, use COMMAND_TAKEOFF_2
  COMMAND_LAND                    = 2, // Deprecated, use COMMAND_LAND_2
  COMMAND_STOP                    = 3,
  COMMAND_GO_TO                   = 4,
  COMMAND_START_TRAJECTORY        = 5,
  COMMAND_DEFINE_TRAJECTORY       = 6,
  COMMAND_TAKEOFF_2               = 7,
  COMMAND_LAND_2                  = 8,
  COMMAND_TAKEOFF_WITH_VELOCITY   = 9,
  COMMAND_LAND_WITH_VELOCITY      = 10,
};

typedef struct _CRTPPacket
{
  uint8_t size;                         //< Size of data
  union {
    struct {
      union {
        uint8_t header;                 //< Header selecting channel and port
        struct { 
          uint8_t channel     : 2;      //< Selected channel within port
          uint8_t reserved    : 2;
          uint8_t port        : 4;      //< Selected port 
        };
      };
      uint8_t data[CRTP_MAX_DATA_SIZE]; //< Data
    };
    uint8_t raw[CRTP_MAX_DATA_SIZE+1];  //< The full packet "raw"
  };
} __attribute__((packed)) CRTPPacket;

// vertical land from current x-y position to given height
struct data_land_2 {
  uint8_t groupMask;        // mask for which CFs this should apply to
  float height;             // m (absolute)
  float yaw;                // rad
  bool useCurrentYaw;       // If true, use the current yaw (ignore the yaw parameter)
  float duration;           // s (time it should take until target height is reached)
} __attribute__((packed));

struct data_takeoff_with_velocity {
  uint8_t groupMask;
  float height;
  bool heightIsRelative;
  float yaw;
  bool useCurrentYaw;
  float velocity;
} __attribute__((packed));

struct crtp_data_t
{
  float roll;       // deg
  float pitch;      // deg
  float yaw;        // deg
  uint16_t thrust;
} __attribute__((packed)); 

typedef struct
{
  uint8_t id;
  uint8_t val;
}sys_msg_t;

typedef enum 
{
    MSG_SET_STATE=1, 
    MSG_TOGGLE_YAW,
    ID_MAX,
}sys_msg_id_t;
 
extern bool _sys_lock(uint32_t timeout_ms);
extern void _sys_unlock(void);

extern uint8_t sys_get_state(void);  
extern bool sys_yaw_is_locked(void); 

extern uint8_t bind_mac[6]; 

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif//APP_COMMON_H
