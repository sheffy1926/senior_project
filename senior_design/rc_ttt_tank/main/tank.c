#define CONFIG_FREERTOS_ENABLE_BACKWARD_COMPATIBILITY 1

/****************************************************
 * Tank DONE List:
 * 	Update PINS to be the correct mode. LEDs as outputs (manually)
 * 	Update ESP LOG Message to determine what LEDs are working correctly/receiving signal from espnow
 *  configure driving button input to turn on LEDs while button is being pressed in test configuration
 *  reconfigure firing input to activate firing servo motor and then turn its position to start each time (PWM Signal)
 *  Once test configuration works, pass PWM signal from remote to provide acceleration curve for the motors 
 *  3D Designed Firing Mechanism, Magazine Holder
 *  Research IR Sensors and Emitters
 *  3D Design the Tank's Base, Shell, Turret Connection, Barrel, Turret Base 
 * 	Design Custom PCB for Tank
 * 	Write code to power emitters and sensors
 *  Configure IR Sensors to detect IR Emitter output
 * Tank TODO List:
 * 	1: Reconfigure flywheel input to toggle on flywheels by flipping a transistor
 * 	2: Reconfigure IR Sensors to detect emitters and determine which sensors is receiving the strongest signal 
 * 	3: Apply this detection sensing into rotational position and send PWM signal to turret motor to rotate
****************************************************/

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
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
#include "driver/adc.h"
#include <esp_adc_cal.h>
//#include "esp_adc/adc_oneshot.h"
//#include "esp_adc/adc_continous.h"
//#include esp_adc/adc_cali.h 
//#include esp_adc/adc_cali_scheme.h"

#include "espnow_custom.h"
#include "sdkconfig.h"
#include "espnow_basic_config.h"

#define DRIVE_PIN_SEL ((1ULL<<RF_IN2_PIN) | (1ULL<<RB_IN1_PIN) | (1ULL<<LB_IN4_PIN) | (1ULL<<LF_IN3_PIN));
#define OUT_PIN_SEL ((1ULL<<TURRET_PIN) | (1ULL<<FW_NMOS) | (1ULL<<FIRE_PIN) | (1ULL<<IR_EMITS_NMOS) | (1ULL<<IR_S_NMOS));
#define IN_PIN_SEL ((1ULL<<IR_S_1) | (1ULL<<IR_S_2) | (1ULL<<IR_S_3_CENTER) | (1ULL<<IR_S_4) | (1ULL<<IR_S_5) | (1ULL<<IR_S_6));
static const char *TAG = "tank";

/**************************************************
* Title:	target_tracking_task
* Summary:	Controls IR Sensors and emitters and send data to turret 
            task based if there is an IR signal detected.
* Param:
* Return:
**************************************************/
void target_tracking_task(void *pvParameter) {
    //ADC1_CHANNEL_0
    int i = 0;
    int adc_channels[5] = {ADC1_CHANNEL_3,ADC1_CHANNEL_6,ADC1_CHANNEL_7,ADC1_CHANNEL_4,ADC1_CHANNEL_5};
    int raw_data[5] = {0};
    float vol_data[5] = {0};
    //float sort_vol[5] = {0};
    
    while (1) {
        for (i = 0; i < 5; i++){
            // Read ADC value from IR Photodiode
            //Vout = Dout * Vmax / Dmax = convert raw data to voltage
            raw_data[i] = adc1_get_raw(adc_channels[i]); 
            vol_data[i] = (raw_data[i] / (float)ADC_MAX_VALUE) * MAX_VOLTAGE;

            // Print the ADC value to esp_log
            ESP_LOGI(TAG, "IR_S_%d Raw Data: %d, Voltage: %f", i+1, raw_data[i], vol_data[i]);
            // Delay for 0.5 seconds
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
        // Read ADC2 value from IR Photodiode on ADC2_CHANNEL_6
        /*esp_err_t r = adc2_get_raw(ADC2_CHANNEL_6, ADC_WIDTH, &s7_raw);
        if (r == ESP_OK) {
            //Vout = Dout * Vmax / Dmax = convert raw data to voltage 
            s7_v = (s7_raw / (float)ADC_MAX_VALUE) * MAX_VOLTAGE;
            // Print the ADC value to esp_log
            ESP_LOGI(TAG, "IR_S_7 Raw Data: %d, Voltage: %f\n", s7_raw, s7_v);
        } else if (r == ESP_ERR_TIMEOUT) {
            ESP_LOGI(TAG, "ADC2 used by Wi-Fi.\n");
        }
        // Delay for 0.5 seconds
        vTaskDelay(500 / portTICK_PERIOD_MS);*/

        //Sorting algorithm based on IR sensor data

    }
}

/**************************************************
* Title: config_gpio_pins
* Summary: set up gpio pins for the tank esp32
* Param:
* Return:
**************************************************/
void config_gpio_pins(void){
    //zero-initialize the config structure.
    gpio_config_t d_conf = {};
    //disable interrupt
    d_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    d_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    d_conf.pin_bit_mask = DRIVE_PIN_SEL;
    //disable pull-down mode
    d_conf.pull_down_en = 1;
    //disable pull-up mode
    d_conf.pull_up_en = 0;
    //configure output GPIO pins with the given settings
    gpio_config(&d_conf);

	//zero-initialize the config structure.
    gpio_config_t o_conf = {};
    //disable interrupt
    o_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    o_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    o_conf.pin_bit_mask = OUT_PIN_SEL;
    //disable pull-down mode
    o_conf.pull_down_en = 1;
    //disable pull-up mode
    o_conf.pull_up_en = 0;
    //configure output GPIO pins with the given settings
    gpio_config(&o_conf);

    gpio_config_t i_conf = {};
    //disable interrupt
    i_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins
    i_conf.pin_bit_mask = IN_PIN_SEL;
    //set as input mode
    i_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    i_conf.pull_up_en = 1;
    //interrupt on both edges
    i_conf.intr_type = GPIO_INTR_ANYEDGE;	
    //configure input GPIO pins with the given settings
    gpio_config(&i_conf);

	//Initialize gpio pins to off
	gpio_set_level(FW_NMOS, 0);
	gpio_set_level(RB_IN1_PIN, 0);
    gpio_set_level(RF_IN2_PIN, 0);
	gpio_set_level(LF_IN3_PIN, 0);
    gpio_set_level(LB_IN4_PIN, 0);
    gpio_set_level(IR_EMITS_NMOS, 0);
    gpio_set_level(IR_S_NMOS,0);
    gpio_set_level(TURRET_PIN,DUTY_CENTER);
}

void adc_init(){
    // Configure ADC 1
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11); //IR_S_1
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); //IR_S_2
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11); //IR_S_3
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11); //IR_S_4
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11); //IR_S_5
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); //IR_S_6
    gpio_set_level(IR_EMITS_NMOS, 1);   //Turn IR Emitters on/off
    // Configure ADC 2
    //adc2_config_channel_atten(ADC2_CHANNEL_6, ADC_ATTEN_DB_11); //IR_S_7
}

void app_main(void){
	//Set up gpio pins to correct modes
    config_gpio_pins();
    //Initialize ADCs
    adc_init();

	//for some reason just having this makes it faster
	//!note I would prefer not to have it
    s_evt_group = xEventGroupCreate();
    assert(s_evt_group);

	//init espnow
    init_espnow_slave();
	vTaskDelay(2000 /portTICK_PERIOD_MS);

	//init target tracking, turret rotation, firing task
	//xTaskCreate(firing_task, "firing_task", 1024, NULL, 5, NULL);
    //xTaskCreate(turret_task, "turret_task", 2048, NULL, 5, NULL);
	xTaskCreate(target_tracking_task, "target_tracking_task", 1024, NULL, 5, NULL);

	while(1){
		vTaskDelay(250 / portTICK_PERIOD_MS);
	}
}
