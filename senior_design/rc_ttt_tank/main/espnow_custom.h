#ifndef ESP_NOW_CUSTOM_H
#define ESP_NOW_CUSTOM_H
#define CONFIG_FREERTOS_ENABLE_BACKWARD_COMPATIBILITY 1

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
//#include "driver/adc.h"
//#include <esp_adc_cal.h>

#include "sdkconfig.h"
#include "espnow_basic_config.h"

static EventGroupHandle_t s_evt_group;

//Global Queue Handlers 
/*static QueueHandle_t firing_queue;
static QueueHandle_t turret_queue;*/

typedef struct {
    esp_now_recv_info_t sender_mac_addr[ESP_NOW_ETH_ALEN];
    my_data_t data;
    int len;
} recv_packet_t;

// Initialize turret angle to midpoint (135 degrees) (22?)
static int angle = (MIN_DUTY_CYCLE + MAX_DUTY_CYCLE) / 2;
static uint32_t rotate_turret = 0;
//static uint32_t rotation_angle[11] = {10,12,14,16,19,22,24,26,28,30,32}; //min = 10, max = 32

#define MY_ESPNOW_WIFI_MODE WIFI_MODE_STA
#define MY_ESPNOW_WIFI_IF   ESP_IF_WIFI_STA

//Prototypes
/**************************************************
* Title:	target_tracking_task
* Summary:	Controls IR Sensors and emitters and send data to turret task based if 
            there is an IR signal detected. Rotates the Turret Servo Motor. 
            Based on Target Tracking Input Data from the IR Detectors based on if 
            they are receiving IR light or not from the other tank's emitters.
* Param:
* Return:
**************************************************/
void target_tracking_task(void *pvParameter);

/**************************************************
* Title: turret_rotation
* Summary: rotates the turret servo motor based on input from the sensors and emitters
* Param:
* Return:
**************************************************/
void turret_rotation(int angle);

/**************************************************
* Title: angle_adjustment
* Summary: adjusts the angle of the turret based on sensor data in order to maximize sensor reading
* Param:
* Return:
**************************************************/
void angle_adjustment(int channels, float v_data[channels], int v_sensor[channels], int adc_channels[channels]);

/**************************************************
* Title:	firing_task
* Summary:	function activates the firing mechanism micro servo motor
            Which rotates forward (180 Deg) then back (0 Deg) and pushes 
            out the nerf dart using the 3D printed rack and pinion
* Param:
* Return:
**************************************************/
void firing_task(void *pvParameter);

/**************************************************
* Title:	recv_cb
* Summary:	call back function called when esp_now messages are received
*			interprets data received in data packet from remote
			moves the tank according to that data, activates flywheels
			or fires turret
* Param:
* Return:
**************************************************/
void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len);

/**************************************************
* Title: send_espnow_data
* Summary: sends an esp_now message to the remote
* Param:
* Return:
**************************************************/
esp_err_t send_espnow_data(void);

/**************************************************
* Title:	packet_sent_cb
* Summary:	function that send the status of the espnow message 
            if it was a success or a failure
* Param:
* Return:
**************************************************/
void packet_sent_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

/**************************************************
* Title: init_espnow_slave
* Summary: initializes the wireless messaging of the tank
* Param:
* Return:
**************************************************/
void init_espnow_slave(void);

/**************************************************
* Title: config_gpio_pins
* Summary: set up gpio pins for the tank esp32
* Param:
* Return:
**************************************************/
void config_gpio_pins(void);

/**************************************************
* Title: adc_turret_init
* Summary: initialize adc channels and turret LEDC channel
* Param:
* Return:
**************************************************/
void adc_turret_init();

#endif //ESP_NOW_CUSTOM_H