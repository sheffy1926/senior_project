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
#include "driver/rmt.h"

#include "espnow_custom.h"
#include "sdkconfig.h"

#include "espnow_basic_config.h"

#define OUT_PIN_SEL ((1ULL<<RF_PIN) | (1ULL<<RB_PIN) | (1ULL<<LF_PIN) | (1ULL<<LB_PIN) | (1ULL<<FIRE_PIN) | 
		(1ULL<<FW_PIN) | (1ULL<<PWM_FIRING_SERVO) | (1ULL<<PWM_TURRET_SERVO) | (1ULL<<IR_S_1) | (1ULL<<IR_EMIT))

static const char *TAG = "tank";

//Initialize target tracking sensors and emitters
static void target_tracking_init(){
	//initialize Target Tracking Sensors and Emitters
	rmt_config_t rmt_tx_config = {
		.channel = RMT_TX_CHANNEL,
		.gpio_num = IR_EMIT,
		.clk_div = 150,
		.rmt_mode = RMT_MODE_TX,
		.tx_config = {
			.tankrier_freq_hz = 38000,
			.tankrier_duty_percent = 50,
			.tankrier_level = RMT_TANKRIER_LEVEL_HIGH,
			.tankrier_en = true,
			.loop_en = false,
			.idle_level = RMT_IDLE_LEVEL_LOW,
			.idle_output_en = true,
		},
		.mem_block_num = 1,
	};

	rmt_config_t rmt_rx_config = {
		.channel = RMT_RX_CHANNEL,
		.gpio_num = IR_S_1,
		.clk_div = 150,
		.rmt_mode = RMT_MODE_RX,
		.rx_config = {
			.tankrier_freq_hz = 38000,
			.tankrier_duty_percent = 50,
			.tankrier_level = RMT_TANKRIER_LEVEL_HIGH,
			.rm_tankrier = true,
			.filter_ticks_thresh = 100,
			.filter_en = true,
			.idle_threshold = 12000,
		},
		.mem_block_num = 1,
	};

	//create target tracking task
	xTaskCreate(target_tracking_task, "target_tracking_task", 2048, NULL, tskIDLE_PRIORITY, NULL);

}

//Monitor target tracking sensors and interpret data to determine target direction
int target_tracking_task(void) {
	int target_direction;

	return target_direction;
}

//Initialize turret rotation servo motor
static void turret_rotation_init(){

	xTaskCreate(turret_rotation_task, "turret_rotation_task", 2048, NULL, tskIDLE_PRIORITY, NULL);
}

//Rotate turret according to target tracking sensor data
void turret_rotation_task (int target_direction){

}

void app_main(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that yo_want to set
    io_conf.pin_bit_mask = OUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

	//init espnow
    init_espnow_master();

	vTaskDelay(2000 /portTICK_PERIOD_MS);

	//init target tracking and turret rotation
	target_tracking_init();
	turret_rotation_init();

	ESP_LOGI(TAG, "before main loop");
	while(1){
		//gpio_set_level(RF_PIN, 0);
		//gpio_set_level(RB_PIN, 0);
		//gpio_set_level(LF_PIN, 0);
		//gpio_set_level(LB_PIN, 0);
		vTaskDelay(750 / portTICK_PERIOD_MS);	
	}

}