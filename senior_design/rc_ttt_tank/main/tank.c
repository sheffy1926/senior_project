#define CONFIG_FREERTOS_ENABLE_BACKWARD_COMPATIBILITY 1

/****************************************************
 * Tank DONE List:
 * 	Update PINS to be the correct mode. LEDs as outputs (manually)
 * 	Update ESP LOG Message to determine what LEDs are working correctly/receiving signal from espnow
 *  configure driving button input to turn on LEDs while button is being pressed in test configuration
 *  reconfigure firing input to activate firing servo motor and then turn its position to start each time (PWM Signal)
 *  Once test configuration works, pass PWM signal from remote to provide acceleration curve for the motors 
 *  3D Designed Firing Mechanism, Magazine Holder
 * Tank TODO List:
 * 	1: Reconfigure flywheel input to toggle on flywheels by flipping a transistor? 
 * 	2: Research IR Sensors and Emitters
 * 	3: Once Target Tracking Method is determined and ordered write code to power emitters and sensors
 * 	4: Configure IR Sensors to detect IR Emitter output (Hopefully using analog IR sensors not digital)
 * 	5: Reconfigure IR Sensors to detect emitters and determine which sensors is receiving the strongest signal 
 * 	6: Apple this detection sensing into rotational position and send PWM signal to turret motor to rotate
 * 
 * 	3D Design the Tank's Base, Shell, Turret Connection ,Barrel, Turret Base 
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

//| (1ULL<<IR_EMIT)
#define OUT_PIN_SEL ((1ULL<<RB_IN1_PIN) | (1ULL<<RF_IN2_PIN) | (1ULL<<LF_IN3_PIN) | (1ULL<<LB_IN4_PIN) | (1ULL<<TURRET_PIN) | (1ULL<<FW_PIN) | (1ULL<<FIRE_SERVO_PIN))
//#define IN_PIN_SEL ( (1ULL<<IR_S_1) | (1ULL<<IR_S_2) | (1ULL<<IR_S_3) | (1ULL<<IR_S_4) )
static const char *TAG = "tank";

/**************************************************
* Title:	target_tracking_task
* Summary:	Controls IR Sensors and emitters and send data to turret 
            task based if there is an IR signal detected.
* Param:
* Return:
**************************************************/
/*void target_tracking_task(void *pvParameter){

	while(1){

	}
}*/

/**************************************************
* Title:	driving_pwm_init
* Summary:	initialize pwm channels and signals for driving DC motors 
* Param:
* Return:
**************************************************/
void driving_pwm_init(void){
	// Configure PWM timer
    ledc_timer_config_t timer_conf2 = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = DRIVING_PWM_TIMER,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf2);

    // Configure PWM channels for right motor
    ledc_channel_config_t rb_in1_conf = {
        .gpio_num = RB_IN1_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = RB_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = DRIVING_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&rb_in1_conf);

    ledc_channel_config_t rf_in2_conf = {
        .gpio_num = RF_IN2_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = RF_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = DRIVING_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&rf_in2_conf);

    // Configure PWM channels for left motor
    ledc_channel_config_t lf_in3_conf = {
        .gpio_num = LF_IN3_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LF_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = DRIVING_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&lf_in3_conf);

    ledc_channel_config_t lb_in4_conf = {
        .gpio_num = LB_IN4_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LB_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = DRIVING_PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&lb_in4_conf);

    // Start the PWM generator
    ledc_fade_func_install(0);
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

	//Initialize gpio pins to off
	gpio_set_level(FW_PIN, 0);
	gpio_set_level(RB_IN1_PIN, 0);
	gpio_set_level(RF_IN2_PIN, 0);
	gpio_set_level(LF_IN3_PIN, 0);
	gpio_set_level(LB_IN4_PIN, 0);

	//configure input GPIO pins (IR Sensors)
	/*gpio_reset_pin(IN_PIN_SEL);
	gpio_set_pull_mode(IN_PIN_SEL,GPIO_PULLUP_ONLY);
	gpio_set_direction(IN_PIN_SEL,GPIO_MODE_INPUT);*/
}

void app_main(void){
	//Set up gpio pins to correct modes
    config_gpio_pins();
	driving_pwm_init();

	// for some reason just having this makes it faster
	//!note I would prefer not to have it
    s_evt_group = xEventGroupCreate();
    assert(s_evt_group);

	//init espnow
    init_espnow_slave();
	vTaskDelay(2000 /portTICK_PERIOD_MS);

	//init target tracking and turret rotation
	/*r_motor_queue = xQueueCreate(64, sizeof(uint32_t));
	l_motor_queue = xQueueCreate(64, sizeof(uint32_t));
	firing_queue = xQueueCreate(1,sizeof(uint32_t));
	turret_queue = xQueueCreate(8,sizeof(uint32_t));*/

	xTaskCreate(firing_task, "firing_task", 4096, NULL, 5, NULL);
	xTaskCreate(driving_task, "driving_task", 4096, NULL, 5, NULL);
	xTaskCreate(turret_task, "turret_task", 4096, NULL, 5, NULL);
	//xTaskCreate(target_tracking_task, "target_tracking_task", 4096, NULL, 3, NULL);

	while(1){
		vTaskDelay(750 / portTICK_PERIOD_MS);
	}
}
