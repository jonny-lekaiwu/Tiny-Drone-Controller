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
#include "wifi_task.h"  

#include "joystick.h" 
 
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_io_spi.h" 
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_types.h"
#include "esp_lcd_panel_st7789.h"

#include "esp_log.h"
#include "esp_heap_caps.h"

#include "driver/spi_master.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h" 

 

#define LCD_PIXEL_CLOCK_HZ (60 * 1000 * 1000)
#define LCD_HOST SPI2_HOST
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8
#define BITS_PER_PIXEL 16  

#define LCD_H_RES 320
#define LCD_V_RES 240
 
#define LCD_BUF_NUM     3
#define LCD_BUF_LINES   64

#define LCD_TEST_SCREEN_COUNT       3
#define LCD_TEST_HOOK_LENGTH        24
#define LCD_TEST_HOOK_THICKNESS     3

#define FONT_W     8
#define FONT_H     16 
 
#define LCD_JOY_STICK_RAW_TEXT_Y     (28)
#define LCD_STATUS_TEXT_Y            (0)
#define LCD_YAW_LOCK_TEXT_X          (0)
#define LCD_CAREFREE_TEXT_X          (LCD_H_RES - 8 * FONT_W)
#define LCD_FLIGHT_MODE_TEXT_Y       (LCD_V_RES - FONT_H)
#define LCD_BATTERY_TEXT_X           ((LCD_H_RES - 6 * FONT_W) / 2)
#define LCD_BATTERY_TEXT_Y           (LCD_V_RES - FONT_H)
#define LCD_ALTITUDE_TEXT_X          (LCD_BATTERY_TEXT_X + 7 * FONT_W)
#define LCD_ALTITUDE_TEXT_Y          (LCD_V_RES - FONT_H)

#define  MAX_YAW_ANGLE 360


static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;


EventGroupHandle_t key_event_group;  

#define TAG "main_task" 

static void log_internal_heap(const char *stage)
{
    const uint32_t caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    printf("[HEAP] %s: free=%u, min=%u, largest=%u\r\n",
           stage,
           (unsigned)heap_caps_get_free_size(caps),
           (unsigned)heap_caps_get_minimum_free_size(caps),
           (unsigned)heap_caps_get_largest_free_block(caps));
}

int nvs_data_write(const char * key, const void * w_data, size_t w_len) ;
int nvs_data_read(const char * key, void * r_data, size_t r_len) ;

extern void wifi_task(void *pvParameter); 
extern void led_task(void *parameter);
extern void key_task(void *pvParameter);

extern void joy_stick_adc_init(uint8_t adc_unit); 
 

uint8_t *lcd_get_free_buffer(void);
void lcd_release_buffer(uint8_t *buf); 
void lcd_set_text_osd(uint8_t *fb,
                      int fb_width,
                      int block_lines,
                      int block_y,
                      int text_x,
                      int text_y,
                      const char *text);
void lcd_set_text_pos(uint16_t x,
                      uint16_t y,
                      const char *text);
static void lcd_draw_test_pattern(uint8_t screen_index);
static void lcd_queue_solid_rect(uint16_t x,
                                 uint16_t y,
                                 uint16_t width,
                                 uint16_t height,
                                 uint16_t color);

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

typedef void ut_queue_t;  

ut_queue_t *_queue_create(size_t length, size_t itemSize)
{ 
    return (ut_queue_t *)xQueueCreate(length, itemSize);
}

static bool _queue_send(ut_queue_t *queue, const void *item, int timeout_ms)
{
    if (!queue)
    {
        return false;
    }
    int ticks = timeout_ms < 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xQueueSend((QueueHandle_t)queue, item, ticks) == pdTRUE;
}

static bool _queue_receive(ut_queue_t *queue, void *out, int timeout_ms)
{
    if (!queue)
    {
        return false;
    }
    int ticks = timeout_ms < 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xQueueReceive((QueueHandle_t)queue, out, ticks) == pdTRUE;
}
 
joystick_t joy_thrust = 
{
    .type = JOY_THRUST,  
    .x_adc_channel = ADC_CHANNEL_2,//yaw
    .y_adc_channel = ADC_CHANNEL_3,//thrust
    .dead_zone = 5,
    .x_min = -MAX_YAW_ANGLE,
    .x_max = MAX_YAW_ANGLE,
    .y_min = -1,
    .y_max = -1,
    .x_reserved = 1,
    .y_reserved = 1,
}; 

joystick_t joy_control = 
{
    .type = JOY_DIRECTION, 
    .x_adc_channel = ADC_CHANNEL_1,//roll
    .y_adc_channel = ADC_CHANNEL_4,//pitch
    .dead_zone = 5,
    .x_min = -15,
    .x_max = 15,
    .y_min = -15,
    .y_max = 15,
    .x_reserved = 0,
    .y_reserved = 1,
};  

static sys_cfg_t sys_cfg = {0}; 
static uint32_t tick_lock = 0;

static SemaphoreHandle_t _mux; 

bool _sys_lock(uint32_t timeout_ms)
{  
    if (NULL == _mux)
    { 
        _mux = xSemaphoreCreateMutex(); 
    }

    const TickType_t timeout_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    return xSemaphoreTake(_mux, timeout_ticks) == pdTRUE; 
}

void _sys_unlock(void)
{   
    xSemaphoreGive(_mux);  
} 

static volatile uint8_t system_state = S_DISCONNECTED; 
static CRTPPacket drone_packet;

static volatile bool unlocked = false;  
static volatile bool yaw_locked = true; 
static volatile bool video_frame_received = false;
static volatile TickType_t last_video_frame_tick = 0;
static volatile uint32_t last_video_pipeline_ms = 0;

#define VIDEO_FRAME_ACTIVE_TIMEOUT_MS  500

static bool video_frame_is_active(void)
{
    return video_frame_received &&
           (xTaskGetTickCount() - last_video_frame_tick <=
            pdMS_TO_TICKS(VIDEO_FRAME_ACTIVE_TIMEOUT_MS));
}

uint8_t sys_get_state(void)
{
    return system_state;
}   

#define CONFIG_BEGINNER_THROTTLE_CURVE   1

#define THROTTLE_INPUT_MIN               200U
#define THROTTLE_INPUT_MAX               4095U
#define THROTTLE_OUTPUT_MIN              0U
#define THROTTLE_OUTPUT_MAX              55000U

/*
 * Three-point beginner curve:
 *   200  ->     0
 *   2500 -> 33000
 *   3600 -> 45000
 *   4095 -> 55000
 *
 * The middle segment is intentionally flatter for finer control around the
 * usual hover range. The final segment becomes steeper again so full thrust
 * authority is unchanged.
 */
#define THROTTLE_CURVE_INPUT_LOW         2500U
#define THROTTLE_CURVE_OUTPUT_LOW        33000U
#define THROTTLE_CURVE_INPUT_HIGH        3600U
#define THROTTLE_CURVE_OUTPUT_HIGH       45000U

static uint16_t map_throttle_segment(uint16_t input,
                                     uint16_t input_min,
                                     uint16_t input_max,
                                     uint16_t output_min,
                                     uint16_t output_max)
{
    const uint32_t input_range = input_max - input_min;
    const uint32_t output_range = output_max - output_min;
    const uint32_t scaled = (uint32_t)(input - input_min) * output_range +
                            input_range / 2U;

    return (uint16_t)(scaled / input_range + output_min);
}

uint16_t thrust_value_round(uint16_t input) 
{
    const uint16_t input_min = THROTTLE_INPUT_MIN;
    const uint16_t input_max = THROTTLE_INPUT_MAX;
    const uint16_t output_min = THROTTLE_OUTPUT_MIN;
    const uint16_t output_max = THROTTLE_OUTPUT_MAX;

    if (input >= 60000)
    {
        return 0;
    }
    
    if (input <= input_min) return output_min;
    if (input >= input_max) return output_max;
    
#if CONFIG_BEGINNER_THROTTLE_CURVE
    if (input <= THROTTLE_CURVE_INPUT_LOW)
    {
        return map_throttle_segment(input,
                                    input_min,
                                    THROTTLE_CURVE_INPUT_LOW,
                                    output_min,
                                    THROTTLE_CURVE_OUTPUT_LOW);
    }

    if (input <= THROTTLE_CURVE_INPUT_HIGH)
    {
        return map_throttle_segment(input,
                                    THROTTLE_CURVE_INPUT_LOW,
                                    THROTTLE_CURVE_INPUT_HIGH,
                                    THROTTLE_CURVE_OUTPUT_LOW,
                                    THROTTLE_CURVE_OUTPUT_HIGH);
    }

    return map_throttle_segment(input,
                                THROTTLE_CURVE_INPUT_HIGH,
                                input_max,
                                THROTTLE_CURVE_OUTPUT_HIGH,
                                output_max);
#else
    return map_throttle_segment(input,
                                input_min,
                                input_max,
                                output_min,
                                output_max);
#endif
}

bool sys_yaw_is_locked(void)
{
    return yaw_locked;
}

#define MAX_ROLL_ANGLE        15.0f
#define MAX_PITCH_ANGLE       15.0f
#define MAX_TRIM_ANGLE        5.0f 
#define XMODE_MAX_YAW_ANGLE   90.0f 

#define CONFIG_TRIM_STEP     0.1f
#define ROLL_TRIM_TRIGGER  (MAX_ROLL_ANGLE / 3.0f)
#define PITCH_TRIM_TRIGGER (MAX_PITCH_ANGLE / 3.0f)
#define CONFIG_TRIM_REPEAT_DELAY_MS   400
#define CONFIG_TRIM_REPEAT_PERIOD_MS  150

static float constrain_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static void update_drone_packet(CRTPPacket *pack)
{
    pack->size = sizeof(struct crtp_data_t)+1; 
    pack->header = CRTP_HEADER(CRTP_PORT_SETPOINT, 0x00); 
    pack->reserved = sys_cfg.carefree;
    struct crtp_data_t *out_values = (struct crtp_data_t *)pack->data;
        
    #if 0
    out_values->roll = joy_control.x;
    out_values->pitch = joy_control.y; 
    #else
    out_values->roll = constrain_float(joy_control.x + sys_cfg.roll_trim, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
    out_values->pitch = constrain_float(joy_control.y + sys_cfg.pitch_trim, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);
    #endif

    if (!sys_cfg.carefree)
    {
        out_values->yaw = yaw_locked?0:constrain_float(joy_thrust.x, -XMODE_MAX_YAW_ANGLE, XMODE_MAX_YAW_ANGLE); 
    }
    else
    {
        out_values->yaw = yaw_locked?0:joy_thrust.x; 
    } 

    out_values->thrust = thrust_value_round(joy_thrust.y_raw); 
}

static void update_land_packet(CRTPPacket *pack)
{ 
    pack->size = sizeof(struct data_land_2) + 1;
    pack->header = CRTP_HEADER(CRTP_PORT_SETPOINT_HL, 0x00);

    pack->data[0] = COMMAND_LAND_2;

    struct data_land_2 *out_values = (struct data_land_2 *)&pack->data[1];

    out_values->groupMask = 0;
 
    out_values->height = 0.0f;
 
    out_values->yaw = 0.0f;
    out_values->useCurrentYaw = true;
 
    out_values->duration = 3.0f;
}

static void update_takeoff_packet(CRTPPacket *pack)
{
    pack->size = sizeof(struct data_takeoff_with_velocity) + 1;
    pack->header = CRTP_HEADER(CRTP_PORT_SETPOINT_HL, 0x00);
    pack->data[0] = COMMAND_TAKEOFF_WITH_VELOCITY;

    struct data_takeoff_with_velocity *out_values =
        (struct data_takeoff_with_velocity *)&pack->data[1];
    out_values->groupMask = 0;
    /* The drone treats this packet as a safe ALTHOLD takeoff request. These
     * fields keep the packet compatible with the standard high-level format. */
    out_values->height = 0.4f;
    out_values->heightIsRelative = true;
    out_values->yaw = 0.0f;
    out_values->useCurrentYaw = true;
    out_values->velocity = 0.16f;
}

static void update_landing_keepalive_packet(CRTPPacket *pack)
{
    pack->header = CRTP_HEADER(CRTP_PORT_SETPOINT, 0x00);
    pack->reserved = sys_cfg.carefree;

    pack->size = sizeof(struct crtp_data_t);

    struct crtp_data_t *values =  (struct crtp_data_t *)pack->data;

    values->roll = 0.0f;
    values->pitch = 0.0f;
    values->yaw = 0.0f;

    /*
     * 保持非零，表示仍处于飞行控制状态。
     * 实际推力会被转换成Hover高度控制。
     */
    values->thrust = 1;
}

void joystick_task(void *parameter) 
{  
    joy_stick_adc_init(SENSOR_ADC_UNIT);

    joystick_init(&joy_thrust);
    joystick_init(&joy_control); 

    for (;;)
    {   
        joystick_read(&joy_thrust);
        joystick_read(&joy_control);
 
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

const uint8_t font16_table [] = 
{
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*" ",0*/
    0x00,0x00,0x00,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x10,0x10,0x00,0x00,/*"!",1*/
    0x00,0x12,0x24,0x24,0x48,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*""",2*/
    0x00,0x00,0x00,0x12,0x12,0x12,0x7E,0x24,0x24,0x24,0x7E,0x24,0x24,0x24,0x00,0x00,/*"#",3*/
    0x00,0x00,0x08,0x3C,0x4A,0x4A,0x48,0x38,0x0C,0x0A,0x0A,0x4A,0x4A,0x3C,0x08,0x08,/*"$",4*/
    0x00,0x00,0x00,0x44,0xA4,0xA8,0xA8,0xB0,0x54,0x1A,0x2A,0x2A,0x4A,0x44,0x00,0x00,/*"%",5*/
    0x00,0x00,0x00,0x30,0x48,0x48,0x48,0x50,0x6E,0xA4,0x94,0x98,0x89,0x76,0x00,0x00,/*"&",6*/
    0x00,0x60,0x20,0x20,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"'",7*/
    0x00,0x02,0x04,0x08,0x08,0x10,0x10,0x10,0x10,0x10,0x10,0x08,0x08,0x04,0x02,0x00,/*"(",8*/
    0x00,0x40,0x20,0x10,0x10,0x08,0x08,0x08,0x08,0x08,0x08,0x10,0x10,0x20,0x40,0x00,/*")",9*/
    0x00,0x00,0x00,0x00,0x10,0x10,0xD6,0x38,0x38,0xD6,0x10,0x10,0x00,0x00,0x00,0x00,/*"*",10*/
    0x00,0x00,0x00,0x00,0x00,0x08,0x08,0x08,0x7F,0x08,0x08,0x08,0x00,0x00,0x00,0x00,/*"+",11*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x20,0x20,0x40,/*",",12*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"-",13*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x60,0x00,0x00,/*".",14*/
    0x00,0x00,0x02,0x04,0x04,0x04,0x08,0x08,0x10,0x10,0x10,0x20,0x20,0x40,0x40,0x00,/*"/",15*/
    0x00,0x00,0x00,0x18,0x24,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x24,0x18,0x00,0x00,/*"0",16*/
    0x00,0x00,0x00,0x08,0x38,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x3E,0x00,0x00,/*"1",17*/
    0x00,0x00,0x00,0x3C,0x42,0x42,0x42,0x02,0x04,0x08,0x10,0x20,0x42,0x7E,0x00,0x00,/*"2",18*/
    0x00,0x00,0x00,0x3C,0x42,0x42,0x02,0x04,0x18,0x04,0x02,0x42,0x42,0x3C,0x00,0x00,/*"3",19*/
    0x00,0x00,0x00,0x04,0x0C,0x0C,0x14,0x24,0x24,0x44,0x7F,0x04,0x04,0x1F,0x00,0x00,/*"4",20*/
    0x00,0x00,0x00,0x7E,0x40,0x40,0x40,0x78,0x44,0x02,0x02,0x42,0x44,0x38,0x00,0x00,/*"5",21*/
    0x00,0x00,0x00,0x18,0x24,0x40,0x40,0x5C,0x62,0x42,0x42,0x42,0x22,0x1C,0x00,0x00,/*"6",22*/
    0x00,0x00,0x00,0x7E,0x42,0x04,0x04,0x08,0x08,0x10,0x10,0x10,0x10,0x10,0x00,0x00,/*"7",23*/
    0x00,0x00,0x00,0x3C,0x42,0x42,0x42,0x24,0x18,0x24,0x42,0x42,0x42,0x3C,0x00,0x00,/*"8",24*/
    0x00,0x00,0x00,0x38,0x44,0x42,0x42,0x42,0x46,0x3A,0x02,0x02,0x24,0x18,0x00,0x00,/*"9",25*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,/*":",26*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x10,/*";",27*/
    0x00,0x00,0x00,0x02,0x04,0x08,0x10,0x20,0x40,0x20,0x10,0x08,0x04,0x02,0x00,0x00,/*"<",28*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,/*"=",29*/
    0x00,0x00,0x00,0x40,0x20,0x10,0x08,0x04,0x02,0x04,0x08,0x10,0x20,0x40,0x00,0x00,/*">",30*/
    0x00,0x00,0x00,0x3C,0x42,0x42,0x62,0x04,0x08,0x08,0x08,0x00,0x18,0x18,0x00,0x00,/*"?",31*/
    0x00,0x00,0x00,0x38,0x44,0x5A,0xAA,0xAA,0xAA,0xAA,0xAA,0x5C,0x42,0x3C,0x00,0x00,/*"@",32*/
    0x00,0x00,0x00,0x10,0x10,0x18,0x28,0x28,0x24,0x3C,0x44,0x42,0x42,0xE7,0x00,0x00,/*"A",33*/
    0x00,0x00,0x00,0xF8,0x44,0x44,0x44,0x78,0x44,0x42,0x42,0x42,0x44,0xF8,0x00,0x00,/*"B",34*/
    0x00,0x00,0x00,0x3E,0x42,0x42,0x80,0x80,0x80,0x80,0x80,0x42,0x44,0x38,0x00,0x00,/*"C",35*/
    0x00,0x00,0x00,0xF8,0x44,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x44,0xF8,0x00,0x00,/*"D",36*/
    0x00,0x00,0x00,0xFC,0x42,0x48,0x48,0x78,0x48,0x48,0x40,0x42,0x42,0xFC,0x00,0x00,/*"E",37*/
    0x00,0x00,0x00,0xFC,0x42,0x48,0x48,0x78,0x48,0x48,0x40,0x40,0x40,0xE0,0x00,0x00,/*"F",38*/
    0x00,0x00,0x00,0x3C,0x44,0x44,0x80,0x80,0x80,0x8E,0x84,0x44,0x44,0x38,0x00,0x00,/*"G",39*/
    0x00,0x00,0x00,0xE7,0x42,0x42,0x42,0x42,0x7E,0x42,0x42,0x42,0x42,0xE7,0x00,0x00,/*"H",40*/
    0x00,0x00,0x00,0x7C,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00,/*"I",41*/
    0x00,0x00,0x00,0x3E,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x88,0xF0,/*"J",42*/
    0x00,0x00,0x00,0xEE,0x44,0x48,0x50,0x70,0x50,0x48,0x48,0x44,0x44,0xEE,0x00,0x00,/*"K",43*/
    0x00,0x00,0x00,0xE0,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x42,0xFE,0x00,0x00,/*"L",44*/
    0x00,0x00,0x00,0xEE,0x6C,0x6C,0x6C,0x6C,0x6C,0x54,0x54,0x54,0x54,0xD6,0x00,0x00,/*"M",45*/
    0x00,0x00,0x00,0xC7,0x62,0x62,0x52,0x52,0x4A,0x4A,0x4A,0x46,0x46,0xE2,0x00,0x00,/*"N",46*/
    0x00,0x00,0x00,0x38,0x44,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x44,0x38,0x00,0x00,/*"O",47*/
    0x00,0x00,0x00,0xFC,0x42,0x42,0x42,0x42,0x7C,0x40,0x40,0x40,0x40,0xE0,0x00,0x00,/*"P",48*/
    0x00,0x00,0x00,0x38,0x44,0x82,0x82,0x82,0x82,0x82,0x82,0xB2,0x4C,0x38,0x06,0x00,/*"Q",49*/
    0x00,0x00,0x00,0xFC,0x42,0x42,0x42,0x7C,0x48,0x48,0x44,0x44,0x42,0xE3,0x00,0x00,/*"R",50*/
    0x00,0x00,0x00,0x3E,0x42,0x42,0x40,0x20,0x18,0x04,0x02,0x42,0x42,0x7C,0x00,0x00,/*"S",51*/
    0x00,0x00,0x00,0xFE,0x92,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x38,0x00,0x00,/*"T",52*/
    0x00,0x00,0x00,0xE7,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x3C,0x00,0x00,/*"U",53*/
    0x00,0x00,0x00,0xE7,0x42,0x42,0x44,0x24,0x24,0x28,0x28,0x18,0x10,0x10,0x00,0x00,/*"V",54*/
    0x00,0x00,0x00,0xD6,0x54,0x54,0x54,0x54,0x54,0x6C,0x28,0x28,0x28,0x28,0x00,0x00,/*"W",55*/
    0x00,0x00,0x00,0xE7,0x42,0x24,0x24,0x18,0x18,0x18,0x24,0x24,0x42,0xE7,0x00,0x00,/*"X",56*/
    0x00,0x00,0x00,0xEE,0x44,0x44,0x28,0x28,0x10,0x10,0x10,0x10,0x10,0x38,0x00,0x00,/*"Y",57*/
    0x00,0x00,0x00,0x7E,0x84,0x04,0x08,0x08,0x10,0x20,0x20,0x42,0x42,0xFC,0x00,0x00,/*"Z",58*/
    0x00,0x1E,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x1E,0x00,/*"[",59*/
    0x00,0x00,0x40,0x20,0x20,0x20,0x10,0x10,0x10,0x08,0x08,0x04,0x04,0x04,0x02,0x02,/*"\",60*/
    0x00,0x78,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x78,0x00,/*"]",61*/
    0x00,0x18,0x24,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"^",62*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,/*"_",63*/
    0x00,0x60,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"`",64*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x44,0x0C,0x34,0x44,0x4C,0x36,0x00,0x00,/*"a",65*/
    0x00,0x00,0x00,0x00,0xC0,0x40,0x40,0x58,0x64,0x42,0x42,0x42,0x64,0x58,0x00,0x00,/*"b",66*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x22,0x40,0x40,0x40,0x22,0x1C,0x00,0x00,/*"c",67*/
    0x00,0x00,0x00,0x00,0x06,0x02,0x02,0x3E,0x42,0x42,0x42,0x42,0x46,0x3B,0x00,0x00,/*"d",68*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x42,0x42,0x7E,0x40,0x42,0x3C,0x00,0x00,/*"e",69*/
    0x00,0x00,0x00,0x00,0x0C,0x12,0x10,0x7C,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00,/*"f",70*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x44,0x44,0x38,0x40,0x3C,0x42,0x42,0x3C,/*"g",71*/
    0x00,0x00,0x00,0x00,0xC0,0x40,0x40,0x5C,0x62,0x42,0x42,0x42,0x42,0xE7,0x00,0x00,/*"h",72*/
    0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x70,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00,/*"i",73*/
    0x00,0x00,0x00,0x0C,0x0C,0x00,0x00,0x1C,0x04,0x04,0x04,0x04,0x04,0x04,0x44,0x78,/*"j",74*/
    0x00,0x00,0x00,0x00,0xC0,0x40,0x40,0x4E,0x48,0x50,0x70,0x48,0x44,0xEE,0x00,0x00,/*"k",75*/
    0x00,0x00,0x00,0x10,0x70,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00,/*"l",76*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x49,0x49,0x49,0x49,0x49,0xED,0x00,0x00,/*"m",77*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDC,0x62,0x42,0x42,0x42,0x42,0xE7,0x00,0x00,/*"n",78*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x42,0x42,0x42,0x42,0x42,0x3C,0x00,0x00,/*"o",79*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xD8,0x64,0x42,0x42,0x42,0x64,0x58,0x40,0xE0,/*"p",80*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1A,0x26,0x42,0x42,0x42,0x26,0x1A,0x02,0x07,/*"q",81*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xEE,0x32,0x20,0x20,0x20,0x20,0xF8,0x00,0x00,/*"r",82*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x42,0x40,0x3C,0x02,0x42,0x7C,0x00,0x00,/*"s",83*/
    0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x7C,0x10,0x10,0x10,0x10,0x12,0x0C,0x00,0x00,/*"t",84*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC6,0x42,0x42,0x42,0x42,0x46,0x3B,0x00,0x00,/*"u",85*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xEE,0x44,0x44,0x28,0x28,0x10,0x10,0x00,0x00,/*"v",86*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDB,0x89,0x4A,0x5A,0x54,0x24,0x24,0x00,0x00,/*"w",87*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x76,0x24,0x18,0x18,0x18,0x24,0x6E,0x00,0x00,/*"x",88*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE7,0x42,0x24,0x24,0x18,0x18,0x10,0x10,0x60,/*"y",89*/
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x44,0x08,0x10,0x10,0x22,0x7E,0x00,0x00,/*"z",90*/
    0x00,0x03,0x04,0x04,0x04,0x04,0x04,0x04,0x08,0x04,0x04,0x04,0x04,0x04,0x03,0x00,/*"{",91*/
    0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,/*"|",92*/
    0x00,0xC0,0x20,0x20,0x20,0x20,0x20,0x20,0x10,0x20,0x20,0x20,0x20,0x20,0xC0,0x00,/*"}",93*/
    0x20,0x5A,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"~",94*/
};

#define LCD_TEXT_SLOT_NUM  6

typedef struct
{
    bool used;
    uint16_t x;
    uint16_t y;
    uint16_t previous_width;
} lcd_text_slot_t;

static lcd_text_slot_t text_slots[LCD_TEXT_SLOT_NUM];

static lcd_text_slot_t *lcd_get_text_slot(uint16_t x, uint16_t y)
{
    lcd_text_slot_t *free_slot = NULL;

    for (int i = 0; i < LCD_TEXT_SLOT_NUM; i++)
    {
        if (text_slots[i].used)
        {
            if (text_slots[i].x == x && text_slots[i].y == y)
            {
                return &text_slots[i];
            }
        }
        else if (!free_slot)
        {
            free_slot = &text_slots[i];
        }
    }

    if (free_slot)
    {
        free_slot->used = true;
        free_slot->x = x;
        free_slot->y = y;
        free_slot->previous_width = 0;
    }

    return free_slot;
}

void lcd_reset_text_slot(void)
{
    memset(text_slots, 0, sizeof(text_slots)); 
} 

static void lcd_clear_text_line(uint16_t y)
{
    if (y >= LCD_V_RES || y + FONT_H > LCD_V_RES)
        return;

    lcd_queue_solid_rect(0, y, LCD_H_RES, FONT_H, 0x0000);

    /* The whole row is now blank, so discard every cached text slot on it. */
    for (int i = 0; i < LCD_TEXT_SLOT_NUM; i++)
    {
        if (text_slots[i].used && text_slots[i].y == y)
        {
            memset(&text_slots[i], 0, sizeof(text_slots[i]));
        }
    }
}

void lcd_reset_text(void)
{ 
    lcd_clear_text_line(LCD_STATUS_TEXT_Y);
    lcd_clear_text_line(LCD_JOY_STICK_RAW_TEXT_Y);
} 

void lcd_fill_screen(uint16_t color)
{ 
    lcd_queue_solid_rect(0, 0, LCD_H_RES, LCD_V_RES, color);

    lcd_reset_text_slot();
} 

/* 240*240*2 = 115200 Bytes */
static uint8_t *framebuffer = NULL;

static void lcd_spi_init(void)
{
    spi_bus_config_t bus_cfg = {
        .sclk_io_num = CONFIG_PIN_LCD_SCK,
        .mosi_io_num = CONFIG_PIN_LCD_MOSI,
        .miso_io_num = -1,

        .quadwp_io_num = -1,
        .quadhd_io_num = -1,

        .max_transfer_sz = LCD_H_RES * LCD_V_RES * 2,
    };

    ESP_ERROR_CHECK(spi_bus_initialize( LCD_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
} 

typedef struct
{
    uint8_t *buf;

    uint16_t x;
    uint16_t y;

    uint16_t width;
    uint16_t height;

    bool frame_end;
    TickType_t frame_start_tick;

} lcd_flush_msg_t;

typedef struct
{
    uint8_t *buf;
    bool frame_end;
    TickType_t frame_start_tick;
} lcd_inflight_msg_t;

extern QueueHandle_t lcd_queue;
extern QueueHandle_t free_buf_queue;
extern QueueHandle_t lcd_inflight_queue;

void framebuffer_init(void);

uint8_t *lcd_get_free_buffer(void);
void lcd_release_buffer(uint8_t *buf);

QueueHandle_t lcd_queue;
QueueHandle_t free_buf_queue;
QueueHandle_t lcd_inflight_queue;

static uint8_t *lcd_buf_pool[LCD_BUF_NUM];

void framebuffer_init(void)
{
    for (int i = 0; i < LCD_BUF_NUM; i++)
    {
        lcd_buf_pool[i] = heap_caps_malloc(LCD_H_RES *LCD_BUF_LINES *2,MALLOC_CAP_DMA);

        if (!lcd_buf_pool[i])
        {
            printf("alloc fb[%d] fail\r\n",i);

            while (1)
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }

        ESP_LOGI(TAG, "fb[%d]=%p", i, lcd_buf_pool[i]);

        xQueueSend(free_buf_queue, &lcd_buf_pool[i], portMAX_DELAY);
    }

    log_internal_heap("after LCD DMA buffers");
}

uint8_t *lcd_get_free_buffer(void)
{
    uint8_t *buf = NULL;

    xQueueReceive(free_buf_queue, &buf, portMAX_DELAY);

    return buf;
}

void lcd_release_buffer(uint8_t *buf)
{
    xQueueSend(free_buf_queue, &buf, portMAX_DELAY);
}

static bool lcd_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                 esp_lcd_panel_io_event_data_t *edata,
                                 void *user_ctx)
{
    lcd_inflight_msg_t msg = {0};
    BaseType_t task_woken = pdFALSE;

    if (xQueueReceiveFromISR(lcd_inflight_queue, &msg, &task_woken) == pdTRUE)
    {
        if (msg.frame_end)
        {
            last_video_pipeline_ms =
                (uint32_t)((xTaskGetTickCountFromISR() -
                            msg.frame_start_tick) * portTICK_PERIOD_MS);
        }

        xQueueSendFromISR(free_buf_queue, &msg.buf, &task_woken);
    }

    return task_woken == pdTRUE;
}

static void lcd_queue_solid_rect(uint16_t x,
                                 uint16_t y,
                                 uint16_t width,
                                 uint16_t height,
                                 uint16_t color)
{
    while (height > 0)
    {
        const uint16_t block_height =
            height > LCD_BUF_LINES ? LCD_BUF_LINES : height;
        uint16_t *buf = (uint16_t *)lcd_get_free_buffer();

        for (uint32_t i = 0; i < (uint32_t)width * block_height; i++)
        {
            buf[i] = color;
        }

        lcd_flush_msg_t msg =
        {
            .buf = (uint8_t *)buf,
            .x = x,
            .y = y,
            .width = width,
            .height = block_height,
        };

        xQueueSend(lcd_queue, &msg, portMAX_DELAY);
        y += block_height;
        height -= block_height;
    }
}

static void lcd_draw_horizontal_line(uint16_t x,
                                     uint16_t y,
                                     uint16_t length,
                                     uint16_t thickness,
                                     uint16_t color)
{
    lcd_queue_solid_rect(x, y, length, thickness, color);
}

static void lcd_draw_vertical_line(uint16_t x,
                                   uint16_t y,
                                   uint16_t length,
                                   uint16_t thickness,
                                   uint16_t color)
{
    lcd_queue_solid_rect(x, y, thickness, length, color);
}

static void lcd_draw_test_pattern(uint8_t screen_index)
{
    static const uint16_t background_colors[LCD_TEST_SCREEN_COUNT] =
    {
        0x00F8, /* red   */
        0xE007, /* green */
        0x1F00, /* blue  */
    };
    const uint16_t hook_color = 0xFFFF;

    if (screen_index >= LCD_TEST_SCREEN_COUNT)
    {
        return;
    }

    /* Background is also submitted through lcd_queue in 16-line blocks. */
    lcd_queue_solid_rect(0,
                         0,
                         LCD_H_RES,
                         LCD_V_RES,
                         background_colors[screen_index]);

    /* Top-left and top-right corners. */
    lcd_draw_horizontal_line(0, 0,
                             LCD_TEST_HOOK_LENGTH,
                             LCD_TEST_HOOK_THICKNESS,
                             hook_color);
    lcd_draw_vertical_line(0, 0,
                           LCD_TEST_HOOK_LENGTH,
                           LCD_TEST_HOOK_THICKNESS,
                           hook_color);
    lcd_draw_horizontal_line(LCD_H_RES - LCD_TEST_HOOK_LENGTH, 0,
                             LCD_TEST_HOOK_LENGTH,
                             LCD_TEST_HOOK_THICKNESS,
                             hook_color);
    lcd_draw_vertical_line(LCD_H_RES - LCD_TEST_HOOK_THICKNESS, 0,
                           LCD_TEST_HOOK_LENGTH,
                           LCD_TEST_HOOK_THICKNESS,
                           hook_color);

    /* Bottom-left and bottom-right corners. */
    lcd_draw_horizontal_line(0, LCD_V_RES - LCD_TEST_HOOK_THICKNESS,
                             LCD_TEST_HOOK_LENGTH,
                             LCD_TEST_HOOK_THICKNESS,
                             hook_color);
    lcd_draw_vertical_line(0, LCD_V_RES - LCD_TEST_HOOK_LENGTH,
                           LCD_TEST_HOOK_LENGTH,
                           LCD_TEST_HOOK_THICKNESS,
                           hook_color);
    lcd_draw_horizontal_line(LCD_H_RES - LCD_TEST_HOOK_LENGTH,
                             LCD_V_RES - LCD_TEST_HOOK_THICKNESS,
                             LCD_TEST_HOOK_LENGTH,
                             LCD_TEST_HOOK_THICKNESS,
                             hook_color);
    lcd_draw_vertical_line(LCD_H_RES - LCD_TEST_HOOK_THICKNESS,
                           LCD_V_RES - LCD_TEST_HOOK_LENGTH,
                           LCD_TEST_HOOK_LENGTH,
                           LCD_TEST_HOOK_THICKNESS,
                           hook_color);

    lcd_reset_text_slot();
}

static void lcd_flush_task(void *arg)
{
    lcd_flush_msg_t msg;

    while (1)
    {
        if (xQueueReceive(
                lcd_queue,
                &msg,
                portMAX_DELAY))
        {
            lcd_inflight_msg_t inflight =
            {
                .buf = msg.buf,
                .frame_end = msg.frame_end,
                .frame_start_tick = msg.frame_start_tick,
            };

            xQueueSend(lcd_inflight_queue, &inflight, portMAX_DELAY);

            esp_lcd_panel_draw_bitmap(panel_handle,
                msg.x,msg.y,
                msg.x + msg.width,msg.y + msg.height,
                msg.buf);
        }
    }
}

static void lcd_init(void)
{
    lcd_spi_init();

    esp_lcd_panel_io_spi_config_t io_config = 
    {
        .dc_gpio_num = CONFIG_PIN_LCD_DC,
        .cs_gpio_num = CONFIG_PIN_LCD_CS, 
        .pclk_hz = LCD_PIXEL_CLOCK_HZ, 
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS, 
        .spi_mode = 0, 
        .trans_queue_depth = 10,
        .on_color_trans_done = lcd_color_trans_done,
    };

    ESP_ERROR_CHECK(
        esp_lcd_new_panel_io_spi(
            (esp_lcd_spi_bus_handle_t)LCD_HOST,
            &io_config,
            &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = CONFIG_PIN_LCD_RST,

        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,

        .bits_per_pixel = BITS_PER_PIXEL,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle,&panel_config,&panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));

    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 0));
    
    esp_lcd_panel_swap_xy(panel_handle, true);

    /*使能sway后，xy传入的参数交换了*/

    if (ROTATION_0 == sys_cfg.rotation)
    {
        esp_lcd_panel_mirror(panel_handle, true, false);
    }
    else
    {
        esp_lcd_panel_swap_xy(panel_handle, true); 
        esp_lcd_panel_mirror(panel_handle, false, true);
    } 
 
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true)); 

    framebuffer_init(); 
    
    xTaskCreate(lcd_flush_task,"lcd_flush",4096,NULL,6,NULL);

    #if LCD_TEST_ENABLE 

    xTaskCreate(
        test_lcd_task_s,
        "lcd_test",
        4096,
        NULL,
        5,
        NULL);  
    
    #endif  
} 

#if 0
[HEAP] after task creation: free=160924, min=160924, largest=114688
wifi scan started
[HEAP] after LCD DMA buffers: free=101380, min=101380, largest=73728
[HEAP] after JPEG receive buffer: free=76188, min=76188, largest=53248
#endif

#define VIDEO_PORT         5000
#define MAX_PACKET_SIZE    1400

#define JPEG_MAX_SIZE      (16 * 1024)

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "esp_jpeg_common.h"
#include "esp_jpeg_dec.h"

typedef struct
{
    uint32_t frame_id;
    uint32_t total_size;
    uint16_t total_packets;
    uint16_t seq_num;
    uint16_t data_len;
} __attribute__((packed)) packet_header_t;

static esp_err_t jpeg_decode_to_lcd(
    uint8_t *jpeg_data,
    uint32_t jpeg_size,
    TickType_t frame_start_tick)
{
    esp_err_t decode_result = ESP_OK;

    jpeg_error_t ret;

    jpeg_dec_config_t config = DEFAULT_JPEG_DEC_CONFIG();

    config.output_type = JPEG_PIXEL_FORMAT_RGB565_BE;

    config.block_enable = true;

    jpeg_dec_handle_t jpeg_dec = NULL;

    ret = jpeg_dec_open(
        &config,
        &jpeg_dec);

    if (ret != JPEG_ERR_OK)
    {
        printf("open dec failed\r\n");

        return ESP_FAIL;
    }

    jpeg_dec_io_t io = {0};

    jpeg_dec_header_info_t info = {0};

    io.inbuf = jpeg_data;
    io.inbuf_len = jpeg_size;

    ret = jpeg_dec_parse_header(
        jpeg_dec,
        &io,
        &info);

    if (ret != JPEG_ERR_OK)
    { 
        printf("dec ret:%d\r\n", ret);

        jpeg_dec_close(jpeg_dec);
        return ESP_FAIL;
    }

    if (info.width != LCD_H_RES || info.height != LCD_V_RES)
    {
        ESP_LOGE(TAG, "Unsupported JPEG size: %ux%u (LCD: %ux%u)",
                 info.width, info.height, LCD_H_RES, LCD_V_RES);
        jpeg_dec_close(jpeg_dec);
        return ESP_ERR_INVALID_SIZE;
    }
 
    int outbuf_len = 0;

    ret = jpeg_dec_get_outbuf_len(jpeg_dec, &outbuf_len);

    if ((ret != JPEG_ERR_OK) ||
        (outbuf_len <= 0))
    {
        printf("get_outbuf ret:%d\r\n", ret);
        
        jpeg_dec_close(jpeg_dec);
        return ESP_FAIL;
    }

    uint8_t *block_buf = jpeg_calloc_align(outbuf_len, 16); 

    if (!block_buf)
    { 
        printf("calloc block_buf failed\r\n");

        jpeg_dec_close(jpeg_dec);
        return ESP_FAIL;
    }

    io.outbuf = block_buf;

    int process_count = 0; 

    ret = jpeg_dec_get_process_count(jpeg_dec, &process_count);

    if ((ret != JPEG_ERR_OK) ||
        (process_count <= 0))
    {
        printf("get_process ret:%d, process_count:%d\r\n", ret, process_count);

        jpeg_free_align(block_buf);
        jpeg_dec_close(jpeg_dec);
        return ESP_FAIL;
    }

    /* Build one immutable OSD snapshot per JPEG frame. The previous code did
     * the same locks, telemetry reads and snprintf calls once per decoder
     * block (normally 15 times for a 320x240 frame). */
    char frame_text[64] = {0};

    _sys_lock(0);
    const uint8_t frame_sys_state = sys_get_state();
    _sys_unlock();

    if (frame_sys_state == S_LOCKED)
    {
        snprintf(frame_text, sizeof(frame_text), "Waiting for unlocking");
    }
    else if (last_video_pipeline_ms > 0)
    {
        snprintf(frame_text, sizeof(frame_text), "frame:%lums",
                 (unsigned long)last_video_pipeline_ms);
    }
    else
    {
        snprintf(frame_text, sizeof(frame_text), "frame:--ms");
    }

    char battery_text[16] = {0};
    char altitude_text[16] = {0};
    const bool frame_wifi_connected = wifi_is_connected();
    if (frame_wifi_connected)
    {
        const uint32_t battery_mv = udp_get_battery_voltage_mv();
        if (battery_mv > 0)
        {
            const uint32_t battery_tenths = (battery_mv + 50U) / 100U;
            snprintf(battery_text, sizeof(battery_text), "%lu.%luV",
                     (unsigned long)(battery_tenths / 10U),
                     (unsigned long)(battery_tenths % 10U));
        }
        else
        {
            snprintf(battery_text, sizeof(battery_text), "--.-V");
        }

        int32_t relative_height_mm = 0;
        if (udp_get_alt_hold_relative_height_mm(&relative_height_mm))
        {
            snprintf(altitude_text, sizeof(altitude_text), "%.1fm%s",
                     relative_height_mm / 1000.0f,
                     udp_is_drone_altitude_holding() ? "H" : "");
        }
    }

    const bool show_yaw_locked =
        frame_sys_state == S_FREE_CONTROL && yaw_locked;
    const bool show_carefree =
        frame_sys_state == S_FREE_CONTROL && sys_cfg.carefree;

    int current_y = 0;

    for (int i = 0; i < process_count; i++)
    {
        ret = jpeg_dec_process(jpeg_dec, &io);

        if (ret != JPEG_ERR_OK)
        {
            ESP_LOGE(TAG, "JPEG block decode failed: ret=%d, block=%d/%d",
                     ret, i + 1, process_count);
            decode_result = ESP_FAIL;
            break;
        }

        int block_lines = io.out_size / (info.width * 2);

        const size_t lcd_buf_size = LCD_H_RES * LCD_BUF_LINES * sizeof(uint16_t);
        if (io.out_size <= 0 ||
            (size_t)io.out_size > lcd_buf_size ||
            block_lines <= 0 ||
            current_y + block_lines > LCD_V_RES)
        {
            ESP_LOGE(TAG, "Invalid JPEG block: size=%d, lines=%d, y=%d",
                     io.out_size, block_lines, current_y);
            decode_result = ESP_ERR_INVALID_SIZE;
            break;
        }

        //printf("block lines:%d\r\n", block_lines);
 
        uint8_t *fb = lcd_get_free_buffer();
        memcpy(fb, block_buf, io.out_size);

        if (current_y < FONT_H && current_y + block_lines > 0)
        {
            lcd_set_text_osd(fb, info.width, block_lines,
                             current_y, 0, 0, frame_text);
        }

        const bool overlaps_bottom_osd =
            current_y < LCD_FLIGHT_MODE_TEXT_Y + FONT_H &&
            current_y + block_lines > LCD_FLIGHT_MODE_TEXT_Y;

        if (overlaps_bottom_osd)
        {
            if (battery_text[0] != '\0')
            {
                lcd_set_text_osd(fb,
                                 info.width,
                                 block_lines,
                                 current_y,
                                 LCD_BATTERY_TEXT_X,
                                 LCD_BATTERY_TEXT_Y,
                                 battery_text);
            }

            if (altitude_text[0] != '\0')
            {
                lcd_set_text_osd(fb,
                                 info.width,
                                 block_lines,
                                 current_y,
                                 LCD_ALTITUDE_TEXT_X,
                                 LCD_ALTITUDE_TEXT_Y,
                                 altitude_text);
            }

            if (show_yaw_locked)
            {
                lcd_set_text_osd(fb,
                                 info.width,
                                 block_lines,
                                 current_y,
                                 LCD_YAW_LOCK_TEXT_X,
                                 LCD_FLIGHT_MODE_TEXT_Y,
                                 "yawlocked");
            }

            if (show_carefree)
            {
                lcd_set_text_osd(fb,
                                 info.width,
                                 block_lines,
                                 current_y,
                                 LCD_CAREFREE_TEXT_X,
                                 LCD_FLIGHT_MODE_TEXT_Y,
                                 "carefree");
            }
        }

        lcd_flush_msg_t msg =
        {
            .buf = fb,
            .x = 0,
            .y = current_y,
            .width = info.width,
            .height = block_lines,
            .frame_end = (current_y + block_lines >= info.height),
            .frame_start_tick = frame_start_tick,
        };

        xQueueSend(lcd_queue, &msg, portMAX_DELAY);

        current_y += block_lines;
    }

    jpeg_free_align(block_buf);

    jpeg_dec_close(jpeg_dec);

    if (decode_result == ESP_OK)
    {
        last_video_frame_tick = xTaskGetTickCount();
        video_frame_received = true;
    }

    return decode_result;
}

void lcd_set_text_osd(uint8_t *fb,
                      int fb_width,
                      int block_lines,
                      int block_y,
                      int text_x,
                      int text_y,
                      const char *text)
{ 
    if (!fb || !text)
        return;

    uint16_t *dst = (uint16_t *)fb;

    int len = strlen(text);
    int text_width = len * FONT_W;

    for (int ch = 0; ch < len; ch++)
    {
        uint8_t c = (uint8_t)text[ch];

        if (c < 0x20 || c > 0x7E)
            continue;

        const uint8_t *glyph =
            &font16_table[(c - 0x20) * FONT_H];

        for (int row = 0; row < FONT_H; row++)
        {
            uint8_t bits = glyph[row];

            int screen_y = text_y + row;

            if (ROTATION_180 == sys_cfg.rotation)
            {
                screen_y = text_y + FONT_H - 1 - row;
            } 
            
            /*
             * 判断这一行是否在当前JPEG块内
             */
            if (screen_y < block_y ||
                screen_y >= block_y + block_lines)
            {
                continue;
            }

            int dst_y = screen_y - block_y;

            for (int col = 0; col < FONT_W; col++)
            {
                if (!(bits & (0x80 >> col)))
                    continue;

                int dst_x = text_x + ch * FONT_W + col;

                if (ROTATION_180 == sys_cfg.rotation)
                {
                    dst_x = text_x + text_width - 1 - (ch * FONT_W + col);
                }  

                if (dst_x >= 0 &&
                    dst_x < fb_width &&
                    dst_y >= 0 &&
                    dst_y < block_lines)
                {
                    dst[dst_y * fb_width + dst_x] = 0xFFFF;
                }
            }
        }
    } 
}
 
static void lcd_draw_string_block_pos(uint16_t *fb,
                                      int buffer_width,
                                      int block_start,
                                      int block_lines,
                                      int text_offset_x,
                                      const char *text,
                                      uint16_t color)
{
    if (!fb || !text)
        return;

    int len = strlen(text);
    int text_width = len * FONT_W;

    for (int ch = 0; ch < len; ch++)
    {
        uint8_t c = (uint8_t)text[ch];

        if (c < 0x20 || c > 0x7E)
            continue;

        const uint8_t *glyph =
            &font16_table[(c - 0x20) * FONT_H];

        for (int row = 0; row < FONT_H; row++)
        {
            uint8_t bits = glyph[row];

            int text_y = row;

            if (ROTATION_180 == sys_cfg.rotation)
            {
                text_y = FONT_H - 1 - row;
            } 

            if (text_y < block_start ||
                text_y >= block_start + block_lines)
            {
                continue;
            }

            int dst_y = text_y - block_start;
 
            for (int col = 0; col < FONT_W; col++)
            {
                if (!(bits & (0x80 >> col)))
                    continue;
                    
                int dst_x = text_offset_x + ch * FONT_W + col;

                if (ROTATION_180 == sys_cfg.rotation)
                {
                    dst_x = text_offset_x + text_width - 1 - (ch * FONT_W + col);
                } 

                if (dst_x >= 0 &&
                    dst_x < buffer_width &&
                    dst_y >= 0 &&
                    dst_y < block_lines)
                {
                    fb[dst_y * buffer_width + dst_x] = color;
                }
            } 
        }
    }
}

void lcd_set_text_pos(uint16_t x, uint16_t y, const char *text)
{
    if (!text)
        return;

    if (x >= LCD_H_RES ||
        y >= LCD_V_RES ||
        y + FONT_H > LCD_V_RES)
    {
        return;
    }

    lcd_text_slot_t *slot = lcd_get_text_slot(x, y);

    if (!slot)
        return;

    uint16_t current_width = strlen(text) * FONT_W;

    if (current_width > LCD_H_RES - x)
        current_width = LCD_H_RES - x;

    /*
     * 使用新旧文字中较大的宽度作为清除区域。
     */
    uint16_t area_width = current_width;

    if (slot->previous_width > area_width)
        area_width = slot->previous_width;

    /*
     * text为空字符串，而且之前也没有内容。
     */
    if (area_width == 0)
        return;

    /*
     * 本次刷新完成后，记录当前文字宽度。
     */
    slot->previous_width = current_width;

    for (int block_start = 0;
         block_start < FONT_H;
         block_start += LCD_BUF_LINES)
    {
        int block_lines = FONT_H - block_start;

        if (block_lines > LCD_BUF_LINES)
            block_lines = LCD_BUF_LINES;

        uint16_t *buf =
            (uint16_t *)lcd_get_free_buffer();

        if (!buf)
            return;

        /*
         * 自动清除新旧文字覆盖的完整区域。
         */
        memset(buf, 0, area_width * block_lines * sizeof(uint16_t));

        int text_offset_x = 0;

        if (ROTATION_180 == sys_cfg.rotation)
        {
            text_offset_x = area_width - current_width;
        } 

        lcd_draw_string_block_pos(
            buf,
            area_width,
            block_start,
            block_lines,
            text_offset_x,
            text,
            0xFFFF);

        lcd_flush_msg_t msg =
        {
            .buf    = (uint8_t *)buf,
            .width  = area_width,
            .height = block_lines,
        };

        msg.x = x;
        msg.y = y + block_start;

        if (ROTATION_180 == sys_cfg.rotation)
        {
            msg.x = LCD_H_RES - x - area_width;

            msg.y = LCD_V_RES - y - FONT_H + block_start;
        } 

        if (xQueueSend(
                lcd_queue,
                &msg,
                portMAX_DELAY) != pdTRUE)
        {
            lcd_release_buffer((uint8_t *)buf);

            return;
        }
    }
}

static bool jpeg_check_basic(const uint8_t *data, uint32_t size)
{
    if (!data || size < 4)
        return false;

    /* JPEG SOI */
    if (data[0] != 0xFF || data[1] != 0xD8)
        return false;

    /* JPEG EOI */
    if (data[size - 2] != 0xFF || data[size - 1] != 0xD9)
        return false;

    return true;
}
 
#define JPEG_PACKET_PAYLOAD_SIZE    (MAX_PACKET_SIZE - sizeof(packet_header_t))
#define MAX_JPEG_PACKETS            64

static bool jpeg_check_markers(const uint8_t *data, uint32_t size)
{
    if (!data || size < 4)
        return false;

    if (data[0] != 0xFF || data[1] != 0xD8)
        return false;

    if (data[size - 2] != 0xFF || data[size - 1] != 0xD9)
        return false;

    return true;
}

static void jpeg_decoder_task(void *arg)
{
    lcd_init();

    lcd_fill_screen(0x0000);
    lcd_set_text_pos(0, LCD_STATUS_TEXT_Y, "Connecting WiFi...");

    uint8_t *jpeg_buffer = malloc(JPEG_MAX_SIZE);
    uint8_t *packet_bitmap = calloc(MAX_JPEG_PACKETS, 1);

    log_internal_heap("after JPEG receive buffer");

    if (!jpeg_buffer || !packet_bitmap)
    {
        ESP_LOGE(TAG, "JPEG buffer allocation failed");

        free(jpeg_buffer);
        free(packet_bitmap);

        vTaskDelete(NULL);
        return;
    }

    int sock = -1;
    bool wifi_last = false;

    uint32_t current_frame_id = UINT32_MAX;
    uint16_t expected_packets = 0;
    uint16_t received_packets = 0;
    uint32_t frame_size = 0;
    TickType_t frame_tick = 0;

    while (1)
    {
        bool wifi = wifi_is_connected();

        _sys_lock(0);
        uint8_t decoder_sys_state = sys_get_state();
        _sys_unlock();

        /* Do not let incoming video overwrite the LCD timing test pattern. */
        if (S_TESTING == decoder_sys_state)
        {
            if (sock >= 0)
            {
                close(sock);
                sock = -1;
            }

            wifi_last = false;
            current_frame_id = UINT32_MAX;
            expected_packets = 0;
            received_packets = 0;
            frame_size = 0;
            memset(packet_bitmap, 0, MAX_JPEG_PACKETS);

            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (!wifi)
        {
            if (wifi_last)
            {
                wifi_last = false;

                if (sock >= 0)
                {
                    close(sock);
                    sock = -1;
                }

                current_frame_id = UINT32_MAX;
                expected_packets = 0;
                received_packets = 0;
                frame_size = 0;

                memset(packet_bitmap, 0, MAX_JPEG_PACKETS);

                lcd_fill_screen(0x0000);
                lcd_set_text_pos(0, LCD_STATUS_TEXT_Y, "Connecting WiFi...");

                printf("wifi disconnected\r\n");
            }

            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (!wifi_last)
        {
            wifi_last = true;

            _sys_lock(0);
            uint8_t sys_state = sys_get_state();
            _sys_unlock();

            if (S_LOCKED == sys_state)
                lcd_set_text_pos(0, LCD_STATUS_TEXT_Y, "Waiting for unlocking");
            else
                lcd_set_text_pos(0, LCD_STATUS_TEXT_Y, "Waiting Video...");

            struct sockaddr_in listen_addr = {
                .sin_family = AF_INET,
                .sin_port = htons(VIDEO_PORT),
                .sin_addr.s_addr = htonl(INADDR_ANY),
            };

            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

            if (sock < 0)
            {
                printf("socket failed\r\n");

                wifi_last = false;

                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            struct timeval timeout = {
                .tv_sec = 0,
                .tv_usec = 100000,
            };

            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

            if (bind(sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0)
            {
                printf("bind failed\r\n");

                close(sock);

                sock = -1;
                wifi_last = false;

                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }

        uint8_t packet[MAX_PACKET_SIZE];

        struct sockaddr_in src_addr;
        socklen_t addr_len = sizeof(src_addr);

        int len = recvfrom(sock, packet, sizeof(packet), 0, (struct sockaddr *)&src_addr, &addr_len);

        if (len < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                continue;

            printf("recv error:%d\r\n", errno);

            close(sock);

            sock = -1;
            wifi_last = false;

            continue;
        }

        if (len == sizeof(packet_header_t))
        {
            packet_header_t *header = (packet_header_t *)packet;

            if (header->frame_id == UINT32_MAX)
            {
                printf("Camera Connect Failed\r\n");
                lcd_set_text_pos(0, LCD_STATUS_TEXT_Y, "Camera Connect Failed");
            }

            continue;
        }

        if (len < sizeof(packet_header_t))
            continue;

        packet_header_t *header = (packet_header_t *)packet;
        uint8_t *payload = packet + sizeof(packet_header_t);

        int actual_payload_len = len - sizeof(packet_header_t);

        /*
         * 先检查数据包基本字段。
         */
        if (header->total_size == 0 ||
            header->total_size > JPEG_MAX_SIZE ||
            header->total_packets == 0 ||
            header->total_packets > MAX_JPEG_PACKETS)
        {
            continue;
        }

        /*
         * total_size与total_packets必须匹配。
         */
        uint16_t calculated_packets =  (header->total_size + JPEG_PACKET_PAYLOAD_SIZE - 1) / JPEG_PACKET_PAYLOAD_SIZE;

        if (header->total_packets != calculated_packets)
            continue;

        /*
         * UDP实际收到的负载长度必须与包头一致。
         */
        if (header->data_len == 0 ||
            header->data_len > JPEG_PACKET_PAYLOAD_SIZE ||
            actual_payload_len != header->data_len)
        {
            continue;
        }

        /*
         * 收到新帧，丢弃上一帧未完成的数据。
         */
        if (header->frame_id != current_frame_id)
        {
            if (current_frame_id != UINT32_MAX &&
                received_packets != expected_packets)
            {
                ESP_LOGD(TAG, "Drop incomplete frame:%lu, packets:%u/%u",
                         (unsigned long)current_frame_id,
                         received_packets,
                         expected_packets);
            }

            current_frame_id = header->frame_id;
            expected_packets = header->total_packets;
            received_packets = 0;
            frame_size = header->total_size;
            frame_tick = xTaskGetTickCount();

            memset(packet_bitmap, 0, MAX_JPEG_PACKETS);
        }

        /*
         * 同一帧内的参数必须保持一致。
         */
        if (header->total_size != frame_size ||
            header->total_packets != expected_packets ||
            header->seq_num >= expected_packets)
        {
            continue;
        }

        uint32_t offset = header->seq_num * JPEG_PACKET_PAYLOAD_SIZE;

        if (offset >= frame_size)
            continue;

        uint32_t remaining = frame_size - offset;

        uint16_t expected_data_len =
            remaining > JPEG_PACKET_PAYLOAD_SIZE ?
            JPEG_PACKET_PAYLOAD_SIZE :
            remaining;

        /*
         * 非末包必须是完整负载长度，末包必须等于剩余长度。
         */
        if (header->data_len != expected_data_len)
            continue;

        if (offset + header->data_len > frame_size)
            continue;

        /*
         * 重复包直接忽略。
         */
        if (packet_bitmap[header->seq_num])
            continue;

        /*
         * 所有检查通过后再写数据、设置位图。
         */
        memcpy(jpeg_buffer + offset, payload, header->data_len);

        packet_bitmap[header->seq_num] = 1;
        received_packets++;

        if (received_packets == expected_packets)
        {
            TickType_t cost = xTaskGetTickCount() - frame_tick;

            if (!jpeg_check_markers(jpeg_buffer, frame_size))
            {
                printf("Invalid JPEG markers, frame:%lu, size:%lu, cost:%lu ms",
                         (unsigned long)current_frame_id,
                         (unsigned long)frame_size,
                         (unsigned long)(cost * portTICK_PERIOD_MS));
            }
            else
            {
                esp_err_t decode_ret = jpeg_decode_to_lcd(
                    jpeg_buffer,
                    frame_size,
                    frame_tick);

                if (decode_ret != ESP_OK)
                {
                    printf("JPEG decode failed, frame:%lu, size:%lu, err:%s\r\n",
                           (unsigned long)current_frame_id,
                           (unsigned long)frame_size,
                           esp_err_to_name(decode_ret));
                }
            }

            current_frame_id = UINT32_MAX;
            expected_packets = 0;
            received_packets = 0;
            frame_size = 0;

            memset(packet_bitmap, 0, MAX_JPEG_PACKETS);
        }
    }
} 

int nvs_data_read(const char * key, void * r_data, size_t r_len) 
{
    nvs_handle_t r_handle;
    esp_err_t err = ESP_OK;

    // Open
    err = nvs_open("trim", NVS_READWRITE, &r_handle);
    if (err != ESP_OK) return err;

    // Read
    err = nvs_get_blob(r_handle, key, r_data, &r_len);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(r_handle);
    return ESP_OK;
}

int nvs_data_write(const char * key, const void * w_data, size_t w_len) 
{
    nvs_handle_t w_handle;
    esp_err_t err = ESP_OK;

    // Open
    err = nvs_open("trim", NVS_READWRITE, &w_handle);
    if (err != ESP_OK) return err;

    // Write
    err = nvs_set_blob(w_handle, key, w_data, w_len);
    if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(w_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(w_handle);
    return ESP_OK;
}  

static void sys_save_cfg(void)
{
    nvs_data_write("sys", &sys_cfg, sizeof(sys_cfg_t)); 

    uint32_t magic = CONFIG_NVS_SIGN_MAGIC;
    
    int save_err = nvs_data_write("magic", &magic, sizeof(uint32_t)); 
}

static void cmd_tx_task(void *pvParameters)
{
    uint32_t now = xTaskGetTickCount();

    tick_lock = now;
 
    uint32_t tick_send = now; 
    uint32_t tick_print = now;  
      
    uint32_t magic = 0;
    int err = nvs_data_read("magic", &magic, sizeof(uint32_t));
    printf("read: err: %d  magic: 0x[%lx]\r\n", err, magic);

    char value_str[64]={0};

    bool need_write = false;

    if (magic != CONFIG_NVS_SIGN_MAGIC) 
    {
        magic = CONFIG_NVS_SIGN_MAGIC;

        sys_cfg.roll_trim = 0.0f;
        sys_cfg.pitch_trim = 0.0f; 
        sys_cfg.rotation = ROTATION_0; 
        sys_cfg.carefree = 1;

        nvs_data_write("sys", &sys_cfg, sizeof(sys_cfg_t));  
        
        need_write = true;
    }
    else
    {
        nvs_data_read("sys", &sys_cfg, sizeof(sys_cfg_t));   

        snprintf(value_str, sizeof(value_str), "roll trim:%.1f, pitch trim:%0.1f", sys_cfg.roll_trim, sys_cfg.pitch_trim);   

        if (sys_cfg.rotation >= ROTATION_MAX)
        {
            sys_cfg.rotation = ROTATION_0; 

            need_write = true;
        }

        if (sys_cfg.carefree > 1)
        {
            sys_cfg.carefree = 1;

            need_write = true;
        }

        if (sys_cfg.roll_trim > 5.0f)
        {
            sys_cfg.roll_trim=0.0f;

            need_write = true;
        }

        if (sys_cfg.pitch_trim > 5.0f)
        {
            sys_cfg.pitch_trim=0.0f;

            need_write = true;
        }
 
        printf("%s, rotatioin:%d, carefree:%d\r\n", value_str, sys_cfg.rotation, sys_cfg.carefree);  
    } 

    if (need_write)
    {
        sys_save_cfg();
    }

    static bool landing = false;
    uint8_t test_screen = 0;

    for (;;)
    {  
        static uint8_t system_state_last = -1;

        now = xTaskGetTickCount();  

        uint8_t wf_is_connected = wifi_is_connected();

        _sys_lock(0);
        uint8_t sys_state = sys_get_state();
        _sys_unlock();
 
        /* JPEG frames carry their own transparent OSD. Only use the direct
         * LCD path when no video is arriving, otherwise its black clearing
         * rectangle would cover part of the image. */
        if (!video_frame_is_active())
        {
            char battery_text[16] = "";
            if (wf_is_connected)
            {
                const uint32_t battery_mv = udp_get_battery_voltage_mv();
                if (battery_mv > 0)
                {
                    const uint32_t battery_tenths = (battery_mv + 50U) / 100U;
                    snprintf(battery_text, sizeof(battery_text), "%lu.%luV",
                             (unsigned long)(battery_tenths / 10U),
                             (unsigned long)(battery_tenths % 10U));
                }
                else
                {
                    snprintf(battery_text, sizeof(battery_text), "---V");
                }
            }

            lcd_set_text_pos(LCD_BATTERY_TEXT_X,
                             LCD_FLIGHT_MODE_TEXT_Y,
                             battery_text);

            char altitude_text[16] = "";
            int32_t relative_height_mm = 0;
            if (wf_is_connected &&
                udp_get_alt_hold_relative_height_mm(&relative_height_mm))
            {
                snprintf(altitude_text, sizeof(altitude_text), "%.1fm%s",
                         relative_height_mm / 1000.0f,
                         udp_is_drone_altitude_holding() ? "H" : "");
            }
            lcd_set_text_pos(LCD_ALTITUDE_TEXT_X,
                             LCD_ALTITUDE_TEXT_Y,
                             altitude_text);

            const bool show_flight_state = sys_state == S_FREE_CONTROL;
            lcd_set_text_pos(LCD_YAW_LOCK_TEXT_X,
                             LCD_FLIGHT_MODE_TEXT_Y,
                             show_flight_state && yaw_locked ?
                                 "yawlocked" : "");
            lcd_set_text_pos(LCD_CAREFREE_TEXT_X,
                             LCD_FLIGHT_MODE_TEXT_Y,
                             show_flight_state && sys_cfg.carefree ?
                                 "carefree" : "");
        }

        /* Always consume every key event. The current system state decides
         * whether an event is actionable, so events cannot leak into a later
         * state and trigger an unexpected delayed operation. */
        EventBits_t key_bits = xEventGroupWaitBits(
            key_event_group,
            SYS_KEY_CLICKED | SYS_KEY_DOUBLE_CLICKED | SYS_KEY_LONG_PRESSED,
            pdTRUE,
            pdFALSE,
            0);

        if (key_bits)
        {
            //printf("key bits:%x, state:%u\r\n", (unsigned int)key_bits, (unsigned int)sys_state);
        } 

        switch (sys_state)
        { 
            case S_CONFIG:
            {
                if (key_bits & SYS_KEY_CLICKED)
                {
                    sys_cfg.carefree = !sys_cfg.carefree;
                }

                lcd_set_text_pos(0, LCD_STATUS_TEXT_Y,  sys_cfg.carefree ? "carefree" : "");

                if (key_bits & SYS_KEY_LONG_PRESSED)
                {
                    _sys_lock(0);
                    system_state = S_LOCKED;
                    _sys_unlock();

                    sys_save_cfg(); 

                    break; 
                }

                if (key_bits & SYS_KEY_DOUBLE_CLICKED)
                {  
                    if (ROTATION_180==sys_cfg.rotation)
                    {
                        sys_cfg.rotation = ROTATION_0;
                    }
                    else
                    {
                        sys_cfg.rotation = ROTATION_180;
                    } 

                    lcd_fill_screen(0x0000);
                }

                static int8_t last_roll_dir = 0;
                static int8_t last_pitch_dir = 0;
                static TickType_t roll_press_tick = 0;
                static TickType_t pitch_press_tick = 0;
                static TickType_t roll_repeat_tick = 0;
                static TickType_t pitch_repeat_tick = 0;

                TickType_t now = xTaskGetTickCount();
                int8_t roll_dir = joy_control.x > ROLL_TRIM_TRIGGER ? 1 : (joy_control.x < -ROLL_TRIM_TRIGGER ? -1 : 0);
                int8_t pitch_dir = joy_control.y > PITCH_TRIM_TRIGGER ? 1 : (joy_control.y < -PITCH_TRIM_TRIGGER ? -1 : 0);

                // joy_control.x 对应 roll
                if (roll_dir == 0) 
                {
                    last_roll_dir = 0;
                } 
                else if (roll_dir != last_roll_dir) 
                {
                    sys_cfg.roll_trim = constrain_float(sys_cfg.roll_trim + roll_dir * CONFIG_TRIM_STEP, -MAX_TRIM_ANGLE, MAX_TRIM_ANGLE);
                    roll_press_tick = now;
                    roll_repeat_tick = now;
                    last_roll_dir = roll_dir;
                } 
                else if ((now - roll_press_tick) >= pdMS_TO_TICKS(CONFIG_TRIM_REPEAT_DELAY_MS) &&
                        (now - roll_repeat_tick) >= pdMS_TO_TICKS(CONFIG_TRIM_REPEAT_PERIOD_MS)) 
                {
                    sys_cfg.roll_trim = constrain_float(sys_cfg.roll_trim + roll_dir * CONFIG_TRIM_STEP, -MAX_TRIM_ANGLE, MAX_TRIM_ANGLE);
                    roll_repeat_tick = now;
                }

                // joy_control.y 对应 pitch
                if (pitch_dir == 0) {
                    last_pitch_dir = 0;
                } else if (pitch_dir != last_pitch_dir) {
                    sys_cfg.pitch_trim = constrain_float(sys_cfg.pitch_trim + pitch_dir * CONFIG_TRIM_STEP, -MAX_TRIM_ANGLE, MAX_TRIM_ANGLE);
                    pitch_press_tick = now;
                    pitch_repeat_tick = now;
                    last_pitch_dir = pitch_dir;
                } else if ((now - pitch_press_tick) >= pdMS_TO_TICKS(CONFIG_TRIM_REPEAT_DELAY_MS) &&
                        (now - pitch_repeat_tick) >= pdMS_TO_TICKS(CONFIG_TRIM_REPEAT_PERIOD_MS)) {
                    sys_cfg.pitch_trim = constrain_float(sys_cfg.pitch_trim + pitch_dir * CONFIG_TRIM_STEP, -MAX_TRIM_ANGLE, MAX_TRIM_ANGLE);
                    pitch_repeat_tick = now;
                }
 
                char show_cfg[64] = {0};

                snprintf(show_cfg, sizeof(show_cfg), "roll trim:(%+.1f) pitch trim:(%+.1f), %d", sys_cfg.roll_trim, sys_cfg.pitch_trim, (ROTATION_180==sys_cfg.rotation)?180:0);
                lcd_set_text_pos(0, LCD_JOY_STICK_RAW_TEXT_Y, show_cfg);

                char show_ip[64] = {0};

                char *check_ip = wifi_get_ip_address();

                if (wifi_is_valid_ipv4_address(check_ip))
                { 
                    snprintf(show_ip, sizeof(show_ip), "ip:%s", check_ip);
                    lcd_set_text_pos(168+8, LCD_STATUS_TEXT_Y, show_ip);
                } 
            }
            break;

            case S_DISCONNECTED:
            {
                if (key_bits & SYS_KEY_LONG_PRESSED)
                {
                    _sys_lock(0);
                    system_state = S_TESTING;
                    _sys_unlock(); 

                    /* Allow the decoder receive timeout and pending LCD
                     * transfers to finish before drawing the timing image. */
                    vTaskDelay(pdMS_TO_TICKS(150));

                    test_screen = 0;
                    lcd_draw_test_pattern(test_screen);

                    break; 
                } 
                
                if (wf_is_connected)
                {
                    _sys_lock(0);
                    if (unlocked)
                    {
                        system_state = S_FREE_CONTROL;
                    }
                    else
                    {
                        system_state = S_LOCKED;
                    } 
                    _sys_unlock();
                }
            }
            break;

            case S_FREE_CONTROL:
            {   
                if (key_bits & SYS_KEY_CLICKED)
                {
                    yaw_locked = !yaw_locked;
                }

                if (key_bits & SYS_KEY_DOUBLE_CLICKED)
                {
                    landing = !landing;
                    //printf("landing:%d\r\n", landing);

                    if (landing)
                    {
                        update_land_packet(&drone_packet);
                        udp_send_data(drone_packet.raw, drone_packet.size + 1);
                    }
                }

                if (key_bits & SYS_KEY_LONG_PRESSED)
                { 
                    _sys_lock(0);
                    system_state = S_CONFIG;
                    _sys_unlock(); 
                }

                tick_send = now;

                if (!wf_is_connected)
                {
                    _sys_lock(0);
                    system_state = S_DISCONNECTED; 
                    _sys_unlock();
                }
                else
                { 
                    #if 0
                    header:30, port:03, channel:00
                    thrust 0-65000
                    yaw -90-90 右边是90 左边是-90
                    x pitch -15 to 15 上方是-15，下方是15
                    y roll -15 to 15 左边是-15 右边是15  
                    #endif 
 
                    update_drone_packet(&drone_packet); 

                    udp_send_data(drone_packet.raw, drone_packet.size+1);   
                }
            }
            break;

            case S_LOCKED:
            {
                bool toggle_lock = ((joy_control.x_raw > 4000) &&((joy_thrust.x_raw>4000) && (joy_thrust.x_raw<4096)));

                if (!unlocked)
                { 
                    char show_x_raw[64]={0};

                    snprintf(show_x_raw, sizeof(show_x_raw), "l:(%d,%d)  r:(%d,%d) fw:%.8s", joy_thrust.x_raw, joy_thrust.y_raw, joy_control.x_raw, joy_control.y_raw, CONFIG_FIRMWARE_VERSION);

                    lcd_set_text_pos(0, LCD_JOY_STICK_RAW_TEXT_Y, show_x_raw);  

                    if (toggle_lock)
                    {
                        unlocked = true; 
                    }
                }
                else
                { 
                    if (!wf_is_connected)
                    {
                        _sys_lock(0);
                        system_state = S_DISCONNECTED;
                        _sys_unlock();
                    }
                    else
                    { 
                        if (unlocked)
                        {
                            _sys_lock(0);
                            system_state = S_FREE_CONTROL; 
                            _sys_unlock();
                            
                            lcd_reset_text();
                        } 
                    }
                }   
            }
            break;

            case S_TESTING:
            {
                if (key_bits & SYS_KEY_CLICKED)
                {
                    if (++test_screen < LCD_TEST_SCREEN_COUNT)
                    {
                        lcd_draw_test_pattern(test_screen);
                    }
                    else
                    {
                        test_screen = 0;

                        _sys_lock(0);
                        system_state = S_DISCONNECTED;
                        _sys_unlock();

                        lcd_fill_screen(0x0000);
                        lcd_set_text_pos(0, LCD_STATUS_TEXT_Y, "Connecting WiFi...");
                    }
                }
            }
            break;

            default:break;
        }

        _sys_lock(0);

        if (system_state_last != system_state)
        { 
            if (S_CONFIG == system_state_last)
            {
                printf("config trim\r\n"); 
                
                lcd_reset_text();
            }

            if (S_DISCONNECTED == system_state)
            {
                unlocked = false; 
            } 

            system_state_last = system_state;  

            printf("new state:%d\r\n", system_state); 
        } 

        _sys_unlock(); 
 
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{   
    log_internal_heap("boot");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
 
    key_event_group = xEventGroupCreate();   

    free_buf_queue = xQueueCreate( LCD_BUF_NUM, sizeof(uint8_t *));

    lcd_queue = xQueueCreate( LCD_BUF_NUM, sizeof(lcd_flush_msg_t));

    lcd_inflight_queue = xQueueCreate(LCD_BUF_NUM,
                                      sizeof(lcd_inflight_msg_t));

    log_internal_heap("after app queues");
 
    xTaskCreate(wifi_task, "wifi_task", 4096, NULL, 3, NULL); 
    xTaskCreate(led_task, "led_task", 2048, NULL, 3, NULL); 
    /* Key scanning is latency-sensitive and must be able to preempt the
     * CPU-heavy software JPEG decoder on this single-core ESP32-C3. */
    xTaskCreate(key_task, "key_task", 2048+1024, NULL, 5, NULL);
    xTaskCreate(joystick_task, "joystick", 2048, NULL, 10, NULL); 
    /* Video decoding is background work. Keeping it below control, key and
     * Wi-Fi tasks prevents a continuous video stream from starving them. */
    xTaskCreate(jpeg_decoder_task, "jpeg_dec", 4096*4, NULL, 2, NULL);
    xTaskCreate(cmd_tx_task, "cmd_tx", 4096*2, NULL, 7, NULL);  

    log_internal_heap("after task creation");
}
