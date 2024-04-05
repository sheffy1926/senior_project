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
 * 	Reconfigure IR Sensors to detect emitters and determine which sensor is receiving the strongest signal 
 * 	Apply this detection sensing into rotational position and send signal to turret motor to rotate
 * Tank TODO List:
 * 	1: Reconfigure flywheel input to toggle on flywheels by flipping a transistor
****************************************************/

#include <stdlib.h>
#include <stdint.h>
#include "inttypes.h"
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

//(1ULL<<IR_S_1) | (1ULL<<IR_S_2)
#define DRIVE_PIN_SEL ((1ULL<<RF_IN2_PIN) | (1ULL<<RB_IN1_PIN) | (1ULL<<LB_IN4_PIN) | (1ULL<<LF_IN3_PIN));
#define OUT_PIN_SEL ((1ULL<<TURRET_PIN) | (1ULL<<FW_NMOS) | (1ULL<<FIRE_PIN) | (1ULL<<IR_EMITS_NMOS) | (1ULL<<IR_S_NMOS));
#define IN_PIN_SEL ((1ULL<<IR_S_3) | (1ULL<<IR_S_4) | (1ULL<<IR_S_5) | (1ULL<<IR_S_6) | (1ULL<<IR_S_7));
static const char *TAG = "tank";

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

/**************************************************
* Title: adc_turret_init
* Summary: initialize adc channels and turret LEDC channel
* Param:
* Return:
**************************************************/
void turret_init(){
    ledc_timer_config_t timer_conf1;
    timer_conf1.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf1.timer_num = TURRET_PWM_TIMER;
	timer_conf1.duty_resolution = DUTY_RESOLUTION;
    timer_conf1.freq_hz = PWM_FREQUENCY;
    ledc_timer_config(&timer_conf1);

    ledc_channel_config_t ledc_conf1;
    ledc_conf1.gpio_num = TURRET_PIN;
    ledc_conf1.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_conf1.channel = TURRET_PWM_CHANNEL;
    ledc_conf1.intr_type = LEDC_INTR_DISABLE;
    ledc_conf1.timer_sel = TURRET_PWM_TIMER;
    ledc_conf1.duty = 256;
    ledc_conf1.hpoint = 0;
    ledc_channel_config(&ledc_conf1);  
}

void app_main(void){
	//Set up gpio pins to correct modes
    config_gpio_pins();
    //Initialize ADCs
    turret_init();

	//for some reason just having this makes it faster
	//!note I would prefer not to have it
    s_evt_group = xEventGroupCreate();
    assert(s_evt_group);

	//init espnow
    init_espnow_slave();
	vTaskDelay(2000 /portTICK_PERIOD_MS);

	//init target tracking, turret rotation, firing task
	xTaskCreate(firing_task, "firing_task", 1024, NULL, 5, NULL);
	//xTaskCreate(target_tracking_task, "target_tracking_task", 2048, NULL, 5, NULL);

	while(1){
		vTaskDelay(250 / portTICK_PERIOD_MS);
	}
}
