#define CONFIG_FREERTOS_ENABLE_BACKWARD_COMPATIBILITY 1

/****************************************************
 * Tank DONE List:
 * 	Update PINS to be the correct mode. LEDs as outputs (manually)
 * 	Update ESP LOG Message to determine what LEDs are working correctly/receiving signal from espnow
 * Tank TODO List:
 * 	1: configure driving button input to turn on LEDs while button is being pressed in test configuration
 * 	2: Once test configuration works, pass PWM signal from remote to provide acceleration curve for the motors 
 * 	3: reconfigure flywheel input to toggle on flywheels by flipping a transistor? (Need to research this more)
 * 	4: reconfigure firing input to activate firing servo motor and then turn its position to start each time (PWM Signal)
 * 
 * 	5: Research IR Sensors and Emitters
 * 	6: Once Target Tracking Method is determined and ordered write code to power emitters and sensors
 * 	7: Configure IR Sensors to detect IR Emitter output (Hopefully using analog IR sensors not digital)
 * 	8: Reconfigure IR Sensors to detect emitters and determine which sensors is receiving the strongest signal 
 * 	9: Apple this detection sensing into rotational position and send PWM signal to turret motor to rotate
 * 
 * 	3D Design the Tank's Base, Shell, Turret Connection ,Barrel, Turret Base, Dart Magazine Holder, Firing Mechanism 
 * 	Design Custom PCB for Tank 
 * 	Buy more components to have all of the necessary parts to complete the tank except maybe the nerf gun, 
 * 		transistors and the target tracking sensors/emitters, maybe buy more 3D filament 
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

#include "espnow_custom.h"
#include "sdkconfig.h"
#include "espnow_basic_config.h"

//#define OUT_PIN_SEL ((1ULL<<RF_PIN) | (1ULL<<RB_PIN) | (1ULL<<LF_PIN) | (1ULL<<LB_PIN) | (1ULL<<TURRET_PIN) | (1ULL<<FIRE_PIN) | (1ULL<<FW_PIN) | (1ULL<<IR_EMIT))
#define OUT_PIN_SEL ((1ULL<<R_LED_R) | (1ULL<<R_LED_G) | (1ULL<<R_LED_B) | (1ULL<<L_LED_R) | (1ULL<<L_LED_G) | (1ULL<<L_LED_B) | (1ULL<<FIRE_PIN) | (1ULL<<FW_PIN))
//#define IN_PIN_SEL ( (1ULL<<IR_S_1) | (1ULL<<IR_S_2) | (1ULL<<IR_S_3) | (1ULL<<IR_S_4) )
static const char *TAG = "tank";

//Initialize target tracking sensors and emitters
/*static void target_tracking_init(){
}

//Monitor target tracking sensors and interpret data to determine target direction
int target_tracking_task(void) {
	int target_direction;
	return target_direction;
}

//Initialize turret rotation servo motor
static void turret_rotation_init(){
}

//Rotate turret according to target tracking sensor data
void turret_rotation_task (int target_direction){
}*/

void firing_task(void *pvParameter) {
    esp_rom_gpio_pad_select_gpio(FIRE_SERVO_PIN);
    gpio_set_direction(FIRE_SERVO_PIN, GPIO_MODE_OUTPUT);

    ledc_timer_config_t timer_conf;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num = SERVO_PWM_TIMER;
	timer_conf.duty_resolution = DUTY_RESOLUTION;
    timer_conf.freq_hz = PWM_FREQUENCY;
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ledc_conf;
    ledc_conf.gpio_num = FIRE_SERVO_PIN;
    ledc_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_conf.channel = SERVO_PWM_CHANNEL;
    ledc_conf.intr_type = LEDC_INTR_DISABLE;
    ledc_conf.timer_sel = SERVO_PWM_TIMER;
    ledc_conf.duty = 4;
    ledc_conf.hpoint = 0;
    ledc_channel_config(&ledc_conf);

    while (1) {
        // Wait until the GPIO pin controlling the servo motor is pulled low
		if(gpio_get_level(FIRE_SERVO_PIN) == 1){
			vTaskDelay(50 / portTICK_PERIOD_MS);
			gpio_set_level(FIRE_SERVO_PIN,0);
			// Rotate the servo forward (180 degrees)
			ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL, DUTY_MAX);
			ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL);
			vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for 0.5 seconds

			// Rotate the servo back to the starting position (0 degrees)
			ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL, DUTY_MIN);
			ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL);
			vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for 0.5 seconds
		}
    }
}

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

	//configure input GPIO pins (IR Sensors)
	/*gpio_reset_pin(IN_PIN_SEL);
	gpio_set_pull_mode(IN_PIN_SEL,GPIO_PULLUP_ONLY);
	gpio_set_direction(IN_PIN_SEL,GPIO_MODE_INPUT);*/
}

void app_main(void){
	//Set up gpio pins to correct modes
    config_gpio_pins();

	// for some reason just having this makes it faster
	//!note I would prefer not to have it
    s_evt_group = xEventGroupCreate();
    assert(s_evt_group);

	//init espnow
    init_espnow_slave();
	vTaskDelay(2000 /portTICK_PERIOD_MS);

	//Initialize test LEDs to off (1 is off for RGB LEDs)
	gpio_set_level(R_LED_R, OFF);
	gpio_set_level(R_LED_G, OFF);
	gpio_set_level(R_LED_B, OFF);
	gpio_set_level(L_LED_R, OFF);
	gpio_set_level(L_LED_G, OFF);
	gpio_set_level(L_LED_B, OFF);	

	gpio_set_level(FIRE_PIN, 0);
	gpio_set_level(FW_PIN, 0);
	gpio_set_level(FIRE_SERVO_PIN,0);

	//init target tracking and turret rotation
	//target_tracking_init();
	//turret_rotation_init();

	xTaskCreate(firing_task, "firing_task", 256, NULL, 3, NULL);
	//xTaskCreate(target_tracking_task, "target_tracking_task", 2048, NULL, 2, NULL);
	//xTaskCreate(turret_rotation_task, "turret_rotation_task", 2048, NULL, 1, NULL);

	while(1){
		//gpio_set_level(RF_PIN, 0);
		//gpio_set_level(RB_PIN, 0);
		//gpio_set_level(LF_PIN, 0);
		//gpio_set_level(LB_PIN, 0);

		vTaskDelay(750 / portTICK_PERIOD_MS);
	}
}
