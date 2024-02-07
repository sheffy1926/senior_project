#ifndef REMOTE_H
#define REMOTE_H
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
#include "esp_sleep.h"
#include "esp_random.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "espnow_basic_config.h"

QueueHandle_t button_queue;
bool led_state = FALSE;

typedef struct{
	int button_pin;
} button_event_t;

typedef struct {
    esp_now_recv_info_t sender_mac_addr[ESP_NOW_ETH_ALEN];
    my_data_t data;
    int len;
} recv_packet_t;

static EventGroupHandle_t s_evt_group;

#define MY_ESPNOW_WIFI_MODE WIFI_MODE_STA
#define MY_ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
// #define MY_ESPNOW_WIFI_MODE WIFI_MODE_AP
// #define MY_ESPNOW_WIFI_IF   ESP_IF_WIFI_AP

/**************************************************
* Title:	init_gpio
* Summary:	Initialize GPIO pins to correct configuration
            of modes for input and output pins 
* Param:
* Return:
**************************************************/
static void init_gpio(void);

/**************************************************
* Title:	firing_buttons
* Summary:	Function handles firing button presses and activates 
            the Flywheels LED and the Busy Firing LED
* Param:
* Return:
**************************************************/
void firing_buttons(int fire_but_level);

/**************************************************
* Title:	button_task
* Summary:	Task handles GPIO button interrupts 
* Param:
* Return:
**************************************************/
void button_task(void *args);

/**************************************************
* Title:	gpio_isr_handler
* Summary:	Handles the isr whenever any GPIO interrupt occurs
* Param:
* Return:
**************************************************/
static void IRAM_ATTR gpio_isr_handler(void* arg);

/**************************************************
* Title:	init_espnow_master
* Summary:	initializes the wireless messaging of the remote
* Param:
* Return:
**************************************************/
static void init_espnow_master(void);

/**************************************************
* Title:	packet_sent_cb
* Summary:	function that send the status of the espnow message 
            if it was a success or a failure
* Param:
* Return:
**************************************************/
static void packet_sent_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

/**************************************************
    * Title: send_espnow_data
    * Summary: sends an esp_now message to the tank
    * Param:
    * Return:
    **************************************************/
esp_err_t send_espnow_data(void);

/**************************************************
* Title:	recv_cb
* Summary:	call_back for when espnow messages are received, 
			activates flywheels and fires turret
* Param:	mac addr-> mac address of the sender (tank) 
*			data-> data packet of my_data_t
*			len -> len of message received
* Return:	none
**************************************************/
void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len);

#endif 