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
#include <string.h>
#include <math.h>
 
#include "esp_wifi.h"  
#include "lwip/apps/netbiosns.h"
 
#include "wifi_task.h" 
#include "app_common.h" 
  
#define WIFI_STA_DISCONNECTED      BIT1 
#define WIFI_GOT_IP                BIT2  
#define WIFI_SCAN_START            BIT3
#define WIFI_SCAN_DONE             BIT4 
#define WIFI_CONFIG_STA            BIT5 
#define WIFI_CONFIG_DISCONNECT     BIT6     

static void wifi_start_scan_and_connect(void);

static EventGroupHandle_t event_group; 

static wifi_data_t wifi = 
{
    0
};   

const char *esp_dorne_wifi_ssid="ESP-DRONE_XXXXXXXXXXXX";
const char *drone_wifi_ssid="TINY-DRONE_XXXXXXXXXXXX";
const char *drone_wifi_pwd="87654321";

#define MAX_LEN_SSID     33
#define MAX_LEN_PASSWORD 64

char *wifi_get_ip_address(void)
{
    return wifi.ip;
}

static bool string_has_content(const char *text)
{
    return text != NULL && text[0] != '\0';
} 

static bool wifi_is_valid_ssid(const char *ssid)
{
    if (!string_has_content(ssid))
    {
        return false;
    }

    if (strlen(ssid) >= MAX_LEN_SSID)
    {
        return false;
    }

    return true;
} 

bool wifi_is_valid_ipv4_address(const char *ip_address)
{
    struct in_addr ipv4_address;
    
    if (!string_has_content(ip_address))
    {
        return false;
    }
    
    size_t address_length = strlen(ip_address);

    if (address_length == strlen("0.0.0.0"))
    {
        if (strncmp("0.0.0.0", ip_address, address_length) == 0)
        {
            return false;
        }
    } 
 
    return inet_pton(AF_INET, ip_address, &ipv4_address) == 1;
} 

static void wifi_set_event_bits(EventBits_t bits)
{
    if (event_group) 
    {
        xEventGroupSetBits(event_group, bits);
    }
} 
 
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{ 
    switch (event_id) 
    {          
        case WIFI_EVENT_SCAN_DONE:
        {
            wifi_set_event_bits(WIFI_SCAN_DONE); 
        }
        break; 

        case WIFI_EVENT_STA_DISCONNECTED: 
        {  
            wifi_set_event_bits(WIFI_STA_DISCONNECTED);  
        }
        break; 

        default:
            break;
    }
    return;
}

static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{ 
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data; 
    esp_ip4addr_ntoa(&event->ip_info.ip, wifi.ip, sizeof(wifi.ip));  
    wifi_set_event_bits(WIFI_GOT_IP);  
}  

int8_t wifi_connect(const char *ssid, const char *password)
{ 
    if (!wifi_is_valid_ssid(ssid))
    { 
        return -1;
    } 

    if (!password)
    { 
        return -1;
    } 

    printf("connect to [%s]\r\n", ssid);
  
    strlcpy(wifi.sta_config.ssid, ssid, sizeof(wifi.sta_config.ssid));
 
    strlcpy(wifi.sta_config.password, password, sizeof(wifi.sta_config.password));
 
    wifi_set_event_bits(WIFI_CONFIG_STA);  
    
    return 0;
}

static int8_t wifi_sta_connect(wifi_sta_t *sta_config) 
{ 
    if (!wifi_is_valid_ssid(sta_config->ssid))
    { 
        return -1;    
    }   

    wifi_config_t config = {
        .sta.scan_method = WIFI_FAST_SCAN,
        .sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
        .sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK
    };
      
    strlcpy((char *)config.sta.ssid, sta_config->ssid, sizeof(config.sta.ssid)); 
    strlcpy((char *)config.sta.password, sta_config->password, sizeof(config.sta.password));
       
    esp_wifi_disconnect();
 
    wifi_set_event_bits(WIFI_CONFIG_DISCONNECT);  

    int res_config = esp_wifi_set_config(WIFI_IF_STA, &config);

    if (ESP_OK != res_config)
    { 
        return -1;
    }
 
    esp_err_t ret = esp_wifi_connect();  

    if (ESP_OK != ret)
    { 
    } 

    return 0; 
}

static void wifi_connect_sta(wifi_data_t *ctx, uint32_t event_bits)
{ 
    if (event_bits & WIFI_CONFIG_STA)  
    {   
        wifi_sta_t sta_config;  
 
        memcpy(&sta_config, &ctx->sta_config, sizeof(wifi_sta_t));  
         
        uint32_t tick_connect_took = (xTaskGetTickCount() - ctx->last_connect_time);
 
        int8_t res_connect = wifi_sta_connect(&sta_config); 

        printf("res_connect [%d]\r\n", res_connect);

        if (-1 != res_connect)
        { 
            ctx->sta_status = WF_CONNECTING;
            ctx->last_connect_time = xTaskGetTickCount();
        }
        else
        {
            ctx->sta_status = WF_DISCONNECTED;
        }
 
        return;
    }  

    if (event_bits & WIFI_GOT_IP)  
    {    
        ctx->sta_status = WF_CONNECTED; 
        ctx->reconnect_count = 0;  

        printf("connected\r\n");

        return;
    }

    if (event_bits & WIFI_STA_DISCONNECTED) 
    { 
        if (event_bits & WIFI_CONFIG_DISCONNECT) 
        {  
            return;
        }   
        
        ctx->sta_status = WF_DISCONNECTED;
    }  
}
  
static void sort_ap_by_rssi(wifi_scan_done_ap_t *list, int count) 
{
    if (count < 2)
    {
        return;
    }

    for (int i = 0; i < count - 1; i++) 
    {
        for (int j = 0; j < count - 1 - i; j++) 
        {
            if (list[j].rssi < list[j + 1].rssi) 
            {
                wifi_scan_done_ap_t tmp = list[j];

                list[j] = list[j + 1];
                list[j + 1] = tmp;
            }
        }
    }
}

static int ssid_exists(wifi_scan_done_ap_t *list, int count, const char *ssid) 
{
    for (int i = 0; i < count; i++) 
    {
        if (strcmp((const char *)list[i].ssid, (const char *)ssid) == 0) 
        {
            return 1;
        }
    }
    return 0;
}

static bool string_is_valid_utf8(const char *str, size_t len)
{
    size_t i = 0;
    while (i < len) {
        uint8_t c = (uint8_t)str[i];
        if (c < 0x80) i++;
        else if ((c >> 5) == 0x6 && i + 1 < len &&
                 (str[i+1] & 0xC0) == 0x80) i += 2;
        else if ((c >> 4) == 0xE && i + 2 < len &&
                 (str[i+1] & 0xC0) == 0x80 &&
                 (str[i+2] & 0xC0) == 0x80) i += 3;
        else if ((c >> 3) == 0x1E && i + 3 < len &&
                 (str[i+1] & 0xC0) == 0x80 &&
                 (str[i+2] & 0xC0) == 0x80 &&
                 (str[i+3] & 0xC0) == 0x80) i += 4;
        else return false;
    }
    return true;
} 
 
#include "lwip/sockets.h"
#include "lwip/inet.h"
static int udp_sock = -1; 
static QueueHandle_t udp_data_tx;
static volatile uint32_t battery_voltage_mv;
static volatile int32_t alt_hold_relative_height_mm;
static volatile uint8_t drone_flight_mode;
static volatile bool drone_altitude_holding;

#define CRTP_INFO_PORT             8
#define CRTP_INFO_BATTERY_CHANNEL  1
#define CRTP_BATTERY_VOLTAGE_ID    0
#define DRONE_FLIGHT_MODE_ALTHOLD  1U
#define DRONE_FLIGHT_MODE_POSHOLD  2U

uint32_t udp_get_battery_voltage_mv(void)
{
    return battery_voltage_mv;
}

uint8_t udp_get_drone_flight_mode(void)
{
    return drone_flight_mode;
}

bool udp_is_drone_altitude_holding(void)
{
    return drone_flight_mode == DRONE_FLIGHT_MODE_ALTHOLD &&
           drone_altitude_holding;
}

bool udp_get_alt_hold_relative_height_mm(int32_t *height_mm)
{
    const bool show_height =
        drone_flight_mode == DRONE_FLIGHT_MODE_ALTHOLD ||
        drone_flight_mode == DRONE_FLIGHT_MODE_POSHOLD;
    if (height_mm == NULL || !show_height) {
        if (height_mm != NULL) {
            *height_mm = 0;
        }
        return false;
    }

    *height_mm = alt_hold_relative_height_mm;
    return true;
}

static uint8_t udp_compute_checksum(const void *data, size_t len)
{
    const uint8_t *bytes = data;
    int i;
    unsigned char cksum = 0;

    for (i = 0; i < len; i++) {
        cksum += *(bytes++);
    }

    return cksum;
}

bool udp_send_data(uint8_t *data, uint8_t size)
{
    udp_packet_t outStage = {0};
    outStage.size = size;
    memcpy(outStage.data, data, size);

    #if 0
    printf("\npack size = %d\n", size);
    for (size_t i = 0; i < size; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    #endif
 
    return (xQueueSend(udp_data_tx, &outStage, portMAX_DELAY) == pdTRUE);
};

static void udp_close(void)
{
    battery_voltage_mv = 0;
    alt_hold_relative_height_mm = 0;
    drone_flight_mode = 0;
    drone_altitude_holding = false;

    if (udp_sock >= 0) 
    {
        shutdown(udp_sock, 0);
        close(udp_sock);
        udp_sock = -1;   
    }
} 

static void udp_receive_crtp_packets(void)
{
    uint8_t rx_data[WIFI_RX_TX_PACKET_SIZE];
    struct sockaddr_in source_addr = {0};
    socklen_t source_addr_len = sizeof(source_addr);
    int len;

    while ((len = recvfrom(udp_sock,
                           rx_data,
                           sizeof(rx_data),
                           MSG_DONTWAIT,
                           (struct sockaddr *)&source_addr,
                           &source_addr_len)) > 0)
    {
        /* One header byte, five battery payload bytes and one checksum. */
        if (len < 7)
        { 
            continue;
        }

        uint8_t received_cksum = rx_data[len - 1];
        if (received_cksum != udp_compute_checksum(rx_data, len - 1))
        { 
            continue;
        }

        uint8_t header = rx_data[0];
        uint8_t port = (header >> 4) & 0x0F;
        uint8_t channel = header & 0x03;
 
        if (port == CRTP_INFO_PORT &&
            channel == CRTP_INFO_BATTERY_CHANNEL &&
            rx_data[1] == CRTP_BATTERY_VOLTAGE_ID)
        {
            float battery_voltage;
            memcpy(&battery_voltage, &rx_data[2], sizeof(battery_voltage));

            if (battery_voltage >= 0.0f && battery_voltage <= 10.0f)
            {
                uint32_t battery_mv = (uint32_t)(battery_voltage * 1000.0f + 0.5f);
                battery_voltage_mv = battery_mv;
                //printf("battery voltage: %lu mV\r\n", (unsigned long)battery_mv);
            }

            /* Optional extension: float relative height in metres followed
             * by the public FlightMode value. Internal takeoff/landing state
             * is deliberately not exposed over this compatibility packet. */
            if (len >= 12) {
                float relative_height_m;
                memcpy(&relative_height_m, &rx_data[6], sizeof(relative_height_m));
                const uint8_t flight_mode = rx_data[10];
                if (flight_mode <= 3U && isfinite(relative_height_m) &&
                    relative_height_m >= -10.0f && relative_height_m <= 100.0f) {
                    alt_hold_relative_height_mm =
                        (int32_t)(relative_height_m * 1000.0f);
                    drone_flight_mode = flight_mode;
                    drone_altitude_holding =
                        (len >= 13 && rx_data[11] != 0U);
                } else {
                    alt_hold_relative_height_mm = 0;
                    drone_flight_mode = 0;
                    drone_altitude_holding = false;
                }
            }
        }
    } 
}

static void udp_server_tx_task(void *pvParameters)
{
    udp_packet_t out_packet = {0};

    static uint8_t wifi_is_connected_last = 0;
    static uint8_t err_cnt = 0;  
    
    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(2390),
        .sin_addr.s_addr = inet_addr(CONFIG_UDP_SERVER_ADDRESS),
    };

    while (1) 
    {
        vTaskDelay(pdMS_TO_TICKS(10));

        uint8_t wf_is_connected = wifi_is_connected();

        if (wf_is_connected == false) 
        { 
            if (udp_sock >= 0)
            {
                udp_close();
            }
            
            continue;
        }

        if (wf_is_connected)
        {
            if (udp_sock<0)
            {
                udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP); 
            } 
        }

        if ((xQueueReceive(udp_data_tx, &out_packet, pdMS_TO_TICKS(20)) == pdTRUE))
        { 
            wifi_ap_record_t connected_ap = {0};
            if (esp_wifi_sta_get_ap_info(&connected_ap) != ESP_OK)
            {
                continue;
            }

            /* Select from the actual AP so reconnects and network changes
             * cannot leave packets targeting the previous drone. */
            const char *server_address = CONFIG_UDP_SERVER_ADDRESS;
            if (strlen((const char *)connected_ap.ssid) == strlen(esp_dorne_wifi_ssid) &&
                strncmp((const char *)connected_ap.ssid, esp_dorne_wifi_ssid,
                        strlen("ESP-DRONE_")) == 0)
            {
                server_address = CONFIG_ESP_DRONE_UDP_SERVER_ADDRESS;
            }
            dest_addr.sin_addr.s_addr = inet_addr(server_address);

            out_packet.data[out_packet.size] = udp_compute_checksum(out_packet.data, out_packet.size);
            out_packet.size += 1;

            int err = sendto(udp_sock, out_packet.data, out_packet.size, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) 
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                 
                continue;
            }
            
            #if 0
            printf("\nSend size = %d checksum = %02X\n", out_packet.size, out_packet.data[out_packet.size - 1]);
            for (size_t i = 0; i < out_packet.size; i++) {
                printf("%02X ", out_packet.data[i]);
            }
            printf("\n");
            #endif
        }

        udp_receive_crtp_packets();
    }
}

#include "web_ota.h"

static httpd_handle_t ota_server = NULL;

void wifi_task(void *pvParameter) 
{  
    event_group = xEventGroupCreate();     
  
    memset(&wifi.sta_config, 0, sizeof(wifi_sta_t));  
    wifi.reconnect_count = 0; 
    wifi.scan_status = WF_SCAN_MAX;

    esp_netif_init();
    esp_event_loop_create_default();
 
    wifi.netif_sta = esp_netif_create_default_wifi_sta(); 

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = 1;  
    esp_wifi_init(&cfg);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
      
    udp_data_tx = xQueueCreate(16, sizeof(udp_packet_t));
   
    xTaskCreate(udp_server_tx_task, "udp_tx", 2048, NULL, 8, NULL);

    wifi_start_scan_and_connect();  
    
    while (1)
    {  
        uint32_t now = xTaskGetTickCount();

        EventBits_t _bits = xEventGroupWaitBits(event_group, ( WIFI_STA_DISCONNECTED | WIFI_GOT_IP | WIFI_CONFIG_DISCONNECT | 
                           WIFI_SCAN_START | WIFI_SCAN_DONE |
                           WIFI_CONFIG_STA  ), 
                                                     pdTRUE,   
                                                     pdFALSE,  
                                                     pdMS_TO_TICKS(10));
          
        if (_bits & WIFI_SCAN_START)
        {  
            int res_scan = esp_wifi_scan_start(NULL, true);
    
            if (0 != res_scan)
            {  
                wifi.scan_status = WF_SCAN_ERROR;  
            } 
            else
            {
                wifi.scan_status = WF_SCAN_ING; 
                wifi.last_scan_time = xTaskGetTickCount(); 
            }  
        }   
        else if (_bits & WIFI_SCAN_DONE)
        {  
            uint16_t sta_number = 0;
            wifi_ap_record_t *ap_list_buffer = NULL;

            wifi_scan_done_ap_t ap_temp[WIFI_STA_SCAN_MAX_CNT]={0};

            esp_wifi_scan_get_ap_num(&sta_number);

            if (sta_number > WIFI_STA_SCAN_MAX_CNT)
                sta_number = WIFI_STA_SCAN_MAX_CNT;
    
            ap_list_buffer = malloc(sta_number * sizeof(wifi_ap_record_t));
            
            if (ap_list_buffer != NULL) 
            { 
                memset(ap_list_buffer, 0, sta_number * sizeof(wifi_ap_record_t));
                memset(ap_temp, 0, sizeof(wifi_scan_done_ap_t)*sta_number);

                if (esp_wifi_scan_get_ap_records(&sta_number, ap_list_buffer) == ESP_OK) 
                {        
                    uint16_t valid_cnt = 0;

                    for (uint16_t i=0; i<sta_number; i++) 
                    {
                        wifi_scan_done_ap_t *p_ap = &ap_temp[i];
                        wifi_ap_record_t *p_record = &ap_list_buffer[i];
    
                        if (!string_has_content((const char *)p_record->ssid))
                        { 
                            continue;
                        } 

                        if (!string_is_valid_utf8((const char *)p_record->ssid, strlen((const char *)p_record->ssid)))
                        {  
                            continue;
                        } 

                        #if 0
                        if (strlen((char *)p_record->ssid) != strlen(drone_wifi_ssid))
                        { 
                            continue;
                        } 

                        if (strncmp(drone_wifi_ssid, (char *)p_record->ssid, 10))
                        { 
                            continue;
                        } 
                        #else
                        if ((strlen((char *)p_record->ssid) != strlen(drone_wifi_ssid)) 
                            && (strlen((char *)p_record->ssid) != strlen(esp_dorne_wifi_ssid)))
                        { 
                            continue;
                        } 

                        if ((strncmp(drone_wifi_ssid, (char *)p_record->ssid, 10))
                            && (strncmp(esp_dorne_wifi_ssid, (char *)p_record->ssid, 10)))
                        { 
                            continue;
                        } 
                        #endif

                        strlcpy(p_ap->ssid, (char *)p_record->ssid, sizeof(p_record->ssid)); 
                        p_ap->rssi = p_record->rssi; 
    
                        valid_cnt++;
    
                        if (valid_cnt)
                        { 
                            if (!strncmp(drone_wifi_ssid, (char *)p_record->ssid, 10))
                            {
                                wifi_connect(p_ap->ssid, drone_wifi_pwd); 
                            }
                            else
                            {
                                wifi_connect(p_ap->ssid, "12345678"); 
                            } 

                            wifi.last_connect_time = xTaskGetTickCount(); 

                            break;
                        }
                    }  

                    wifi.scan_list.num = valid_cnt; 
                }

                free(ap_list_buffer); 
            }    

            esp_wifi_clear_ap_list();  
    
            wifi.scan_status = WF_SCAN_COMPLETED; 
        }    
        else if (_bits & WIFI_STA_DISCONNECTED) 
        {   
            wifi.sta_status = WF_DISCONNECTED;
            if (ota_server)
            {
                stop_web_ota_server(ota_server);

                ota_server = NULL;
            }
        }  
        else if (_bits & WIFI_CONFIG_DISCONNECT) 
        {   
            
        }  
        else if (_bits & WIFI_GOT_IP) 
        {   
            wifi.sta_status = WF_CONNECTED; 
            wifi.reconnect_count = 0;  

            printf("connected\r\n");
            if (NULL == ota_server)
            {
                ota_server = start_web_ota_server();
            }
        }  
        else if (_bits & WIFI_CONFIG_STA)  
        {   
            wifi_sta_t sta_config;  
    
            memcpy(&sta_config, &wifi.sta_config, sizeof(wifi_sta_t));  
            
            uint32_t tick_connect_took = (xTaskGetTickCount() - wifi.last_connect_time);
    
            int8_t res_connect = wifi_sta_connect(&sta_config); 

            printf("res_connect [%d]\r\n", res_connect);

            if (-1 != res_connect)
            { 
                wifi.sta_status = WF_CONNECTING;
                wifi.last_connect_time = xTaskGetTickCount();
            }
            else
            {
                wifi.sta_status = WF_DISCONNECTED;
            } 
        }  

        uint32_t tick_now = xTaskGetTickCount(); 
    
        if (wifi.scan_status == WF_SCAN_ING 
            && (tick_now - wifi.last_scan_time) > pdMS_TO_TICKS(30000)) 
        { 
            wifi.scan_status = WF_SCAN_ERROR; 
        }
        
        uint32_t tick_connect_took = (tick_now - wifi.last_connect_time);

        if (tick_connect_took > pdMS_TO_TICKS(10000)) 
        {
            wifi.last_connect_time = tick_now; 
    
            bool can_reconnect = (wifi.sta_status != WF_CONNECTED); 

            if (can_reconnect) 
            {  
                if (wifi.scan_status != WF_SCAN_ING)
                { 
                    wifi_start_scan_and_connect();
                }  
            }  
        } 
    } 
}

bool wifi_is_connected(void)
{ 
    if (WF_CONNECTED == wifi.sta_status)
    {
        return true;
    }

    return false; 
} 

bool udp_is_connected(void)
{
    if (udp_sock>=0)
    {
        return true;
    }

    return false;
}  

static void wifi_start_scan_and_connect(void)
{
    /*扫描连接 ESP-DRONE_XXXXXXXXXXXX*/ 
     
    if (wifi.scan_status == WF_SCAN_ING)
    {
        return;
    }  

    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
        wifi_set_event_bits(WIFI_SCAN_START); 

    printf("wifi scan started\r\n");
}  
