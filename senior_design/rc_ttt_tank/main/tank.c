#define CONFIG_FREERTOS_ENABLE_BACKWARD_COMPATIBILITY 1

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "driver/gpio.h"

#include "espnow_custom.h"
#include "sdkconfig.h"

#include "espnow_basic_config.h"

/****************************************************
 * Tank TODO List:
 * 	Update ESP LOG Message to determine what LEDs are working correctly/receiving signal from espnow
 * 	Make only feedback sent over espnow to remote be if the firing mechanism is in action? (maybe)
 * 	Update PINS to be the correct mode. LEDs as outputs (manually)
 * 
 * 	First: configure driving button input to turn on LEDs while button is being pressed in test configuration
 * 	Second: configure flywheel input LED to be toggled on/off by button press (test)
 * 	Third: configure firing input LED each time button is released for a short duration (test)
 * 	Fourth: test configuration works, pass PWM signal from remote to provide acceleration curve for the motors 
 * 	Fifth: reconfigure flywheel input to toggle on flywheels by flipping a transistor? (Need to research this more)
 * 	Sixth: reconfigure firing input to activate firing servo motor and then turn its position to start each time (PWM Signal)
 * 
 * 	Seventh: Research IR Sensors and Emitters
 * 	Eighth: Once Target Tracking Method is determined and ordered write code to power emitters and sensors
 * 	Ninth: Configure IR Sensors to detect IR Emitter output (Hopefully using analog IR sensors not digital)
 * 	Tenth: Reconfigure IR Sensors to detect emitters and determine which sensors is receiving the strongest signal 
 * 	Eleventh: Apple this detect sensing into rotational position and send PWM signal to turret motor to rotate
 * 
 * 	3D Design the Tank's Base, Shell, Turret Connection ,Barrel, Turret Base, Dart Magazine Holder, Firing Mechanism 
 * 	Design Custom PCB for Tank 
 * 	Buy more components to have all of the necessary parts to complete the tank except maybe the nerf gun, transistors and the 
 * 		target tracking sensors/emitters, maybe buy more 3D filament 
****************************************************/

//#define OUT_PIN_SEL ((1ULL<<RF_PIN) | (1ULL<<RB_PIN) | (1ULL<<LF_PIN) | (1ULL<<LB_PIN) | (1ULL<<TURRET_PIN) | (1ULL<<FIRE_PIN) | (1ULL<<FW_PIN) | (1ULL<<IR_EMIT))
#define OUT_PIN_SEL ((1ULL<<R_LED_R) | (1ULL<<R_LED_G) | (1ULL<<R_LED_B) | (1ULL<<L_LED_R) | (1ULL<<L_LED_G) | (1ULL<<L_LED_B) | (1ULL<<FIRE_PIN) | (1ULL<<FW_PIN))
//#define IN_PIN_SEL ( (1ULL<<IR_S_1) )
static const char *TAG = "tank";

//Initialize target tracking sensors and emitters
/*static void target_tracking_init(){

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

}*/

void app_main(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
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
	//target_tracking_init();
	//turret_rotation_init();

	//Initialize test LEDs to off (1 is off for RGB LEDs)
	gpio_set_level(R_LED_R, 1);
	gpio_set_level(R_LED_G, 1);
	gpio_set_level(R_LED_B, 1);
	gpio_set_level(L_LED_R, 1);
	gpio_set_level(L_LED_G, 1);
	gpio_set_level(L_LED_B, 1);	

	gpio_set_level(FIRE_PIN, 0);
	gpio_set_level(FW_PIN, 0);

	ESP_LOGI(TAG, "before main loop");
	while(1){
		//gpio_set_level(RF_PIN, 0);
		//gpio_set_level(RB_PIN, 0);
		//gpio_set_level(LF_PIN, 0);
		//gpio_set_level(LB_PIN, 0);
		vTaskDelay(750 / portTICK_PERIOD_MS);
	}
}
