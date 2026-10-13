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

#ifndef WIFI_TASK_H
#define WIFI_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_wifi.h"
#include "lwip/ip_addr.h"
#include "esp_mac.h"
#include <sys/param.h>
#include <inttypes.h>

#include "esp_system.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#define CONFIG_UDP_SERVER_ADDRESS  "192.168.26.28"
#define CONFIG_ESP_DRONE_UDP_SERVER_ADDRESS "192.168.43.42"

#define WIFI_RX_TX_PACKET_SIZE     (32)
 
typedef struct
{
  uint8_t size;
  uint8_t data[WIFI_RX_TX_PACKET_SIZE];
} udp_packet_t;

#define WIFI_STA_SCAN_MAX_CNT 20

#define MAX_LEN_IP       16
#define MAX_LEN_SSID     33
#define MAX_LEN_PASSWORD 64
#define MAX_LEN_HOSTNAME 16 //33

typedef struct 
{
    char ssid[MAX_LEN_SSID];
    char password[MAX_LEN_PASSWORD];  
} wifi_sta_t; 

typedef struct 
{
    char ssid[MAX_LEN_SSID]; 
    int8_t  rssi;   
} wifi_scan_done_ap_t;  

typedef enum 
{   
    WF_SCAN_ING,
    WF_SCAN_COMPLETED, 
    WF_SCAN_ERROR, 
    WF_SCAN_MAX,
} wifi_scan_status_t; 
    
typedef enum 
{ 
    WF_IDLE = 0,   
    WF_CONNECTING,
    WF_CONNECTED,   
    WF_DISCONNECTED,  
    WF_MAX,
} wifi_sta_status_t;  

typedef struct {
    wifi_scan_done_ap_t ap[WIFI_STA_SCAN_MAX_CNT];
    uint8_t num;
} wifi_scan_done_list_t; 
 
#define CONFIG_MAX_RECONNECT_COUNT  3 

typedef struct 
{
    volatile uint8_t sta_status;
    volatile uint8_t scan_status;  

    uint32_t last_connect_time;
    uint32_t last_scan_time; 
 
    uint8_t reconnect_count; 
 
    wifi_sta_t sta_config;
 
    wifi_scan_done_list_t scan_list; 
 
    char ip[16];
    esp_netif_t * netif_sta;
}wifi_data_t;

extern bool wifi_is_connected(void); 
extern bool udp_is_connected(void); 

extern bool udp_send_data(uint8_t *data, uint8_t size);
extern uint32_t udp_get_battery_voltage_mv(void);
extern bool udp_get_alt_hold_relative_height_mm(int32_t *height_mm);
extern bool udp_is_drone_altitude_holding(void);
extern uint8_t udp_get_drone_flight_mode(void);

extern char *wifi_get_ip_address(void);
extern bool wifi_is_valid_ipv4_address(const char *ip_address);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif//WIFI_TASK_H
