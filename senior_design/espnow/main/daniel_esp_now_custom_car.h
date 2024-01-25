#ifndef ESP_NOW_CUSTOM_H
#define ESP_NOW_CUSTOM_H
#define CONFIG_FREERTOS_ENABLE_BACKWARD_COMPATIBILITY 1

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "driver/gpio.h"


#include "sdkconfig.h"

#include "espnow_basic_config.h"

#define MY_ESPNOW_WIFI_MODE WIFI_MODE_STA
#define MY_ESPNOW_WIFI_IF   ESP_IF_WIFI_STA



/**************************************************
* Title:	recv_cb
* Summary:	call back function called when esp_now messages are recieved
*			interprits data recieved in data packet from remote
			moves the car according to that data, or shoots laser
* Param:
* Return:
**************************************************/
void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len);



/**************************************************
* Title: send_espnow_data
* Summary: sends an esp_now message to the score_board
* Param:
* Return:
**************************************************/
esp_err_t send_espnow_data(my_data_t data);


/**************************************************
* Title: init_espnow_master
* Summary: initializes the wireless messaging of the car
* Param:
* Return:
**************************************************/
void init_espnow_master(void);


#endif //ESP_NOW_CUSTOM_H