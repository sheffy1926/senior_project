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
#include "esp_adc/adc_oneshot.h"

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

#define MY_ESPNOW_WIFI_MODE WIFI_MODE_STA
#define MY_ESPNOW_WIFI_IF   ESP_IF_WIFI_STA

//Prototypes
/**************************************************
* Title:	target_tracking_task
* Summary:	Controls IR Sensors and Emitters and send data to turret 
            task based if there is an IR signal detected.
* Param:
* Return:
**************************************************/
void target_tracking_task(void *pvParameter);

/**************************************************
* Title:	turret_task
* Summary:	Rotates the Turret Servo Motor. Based on Target Tracking Input Data
* Param:
* Return:
**************************************************/
void turret_task(void *pvParameter);

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

#endif //ESP_NOW_CUSTOM_H