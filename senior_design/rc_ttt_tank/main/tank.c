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
 * Tank TODO List:
 * 	1: Reconfigure flywheel input to toggle on flywheels by flipping a transistor
 * 	2: Once Target Tracking Method is determined and ordered write code to power emitters and sensors
 * 	3: Configure IR Sensors to detect IR Emitter output (Hopefully using analog IR sensors not digital)
 * 	4: Reconfigure IR Sensors to detect emitters and determine which sensors is receiving the strongest signal 
 * 	5: Apple this detection sensing into rotational position and send PWM signal to turret motor to rotate
 * 
 * 	3D Design the Tank's Base, Shell, Turret Connection, Barrel, Turret Base 
 * 	Design Custom PCB for Tank 
 * 	Buy more components to have all of the necessary parts to complete the tank maybe buy more 3D filament 
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
#include "esp_adc/adc_oneshot.h"

#include "espnow_custom.h"
#include "sdkconfig.h"
#include "espnow_basic_config.h"

//| (1ULL<<IR_EMITS_NMOS) | (1ULL<<IR_S_NMOS)
#define OUT_PIN_SEL ((1ULL<<RF_IN2_PIN) | (1ULL<<RB_IN1_PIN) | (1ULL<<LB_IN4_PIN) | (1ULL<<LF_IN3_PIN) | (1ULL<<TURRET_PIN) | (1ULL<<FW_PIN) | (1ULL<<FIRE_PIN) | (1ULL<<IR_EMIT))
//#define IN_PIN_SEL ((1ULL<<IR_S_1) | (1ULL<<IR_S_2) | (1ULL<<IR_S_3) | (1ULL<<IR_S_4) | (1ULL<<IR_S_5) | (1ULL<<IR_S_6) | (1ULL<<IR_S_7))
static const char *TAG = "tank";
adc_oneshot_unit_handle_t adc2_handle;

/**************************************************
* Title:	target_tracking_task
* Summary:	Controls IR Sensors and emitters and send data to turret 
            task based if there is an IR signal detected.
* Param:
* Return:
**************************************************/
void target_tracking_task(void *pvParameter) {
    int s1_raw = 0;
    float s1_v = 0;

    // Configure ADC
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_0,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_6, &config));
    gpio_set_level(IR_EMIT,1); //Turn IR Emitter on 

    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC_CHANNEL_6, &s1_raw));
        s1_v = (s1_raw / (float)ADC_MAX_VALUE) * MAX_VOLTAGE;
        ESP_LOGI(TAG, "ADC2 Channel[6] Raw Data: %d, Voltage: %f", s1_raw, s1_v);

        //Vout = Dout * Vmax / Dmax = convert raw data to voltage 

        // Read ADC value from IR_S_1
       // sv_1 = adc2_get_raw(ADC2_CHANNEL_6);

        // Print the ADC value to esp_log
        //ESP_LOGI("target_tracking_task", "ADC Value (IR_S_1): %d", sv_1);

        // Delay for 0.5 seconds
        vTaskDelay(500 / portTICK_PERIOD_MS);
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
    i_conf.pin_bit_mask = IR_S_1;
    //set as input mode
    i_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    i_conf.pull_up_en = 1;
    //interrupt on both edges
    i_conf.intr_type = GPIO_INTR_ANYEDGE;	
    //configure input GPIO pins with the given settings
    gpio_config(&i_conf);

	//Initialize gpio pins to off
	gpio_set_level(FW_PIN, 0);
	gpio_set_level(RB_IN1_PIN, 0);
    gpio_set_level(RF_IN2_PIN, 0);
	gpio_set_level(LF_IN3_PIN, 0);
    gpio_set_level(LB_IN4_PIN, 0);
    gpio_set_level(IR_EMIT, 0);
}

void adc_init(){
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));
}

void app_main(void){
	//Set up gpio pins to correct modes
    config_gpio_pins();
    //Initialize ADCs
    adc_init();

	// for some reason just having this makes it faster
	//!note I would prefer not to have it
    s_evt_group = xEventGroupCreate();
    assert(s_evt_group);

	//init espnow
    init_espnow_slave();
	vTaskDelay(2000 /portTICK_PERIOD_MS);

	//init target tracking and turret rotation
	//turret_queue = xQueueCreate(8,sizeof(uint32_t));*/

	xTaskCreate(firing_task, "firing_task", 1024, NULL, 5, NULL);
    xTaskCreate(turret_task, "turret_task", 2048, NULL, 5, NULL);
	xTaskCreate(target_tracking_task, "target_tracking_task", 2048, NULL, 5, NULL);

	while(1){
		vTaskDelay(750 / portTICK_PERIOD_MS);
	}
}
