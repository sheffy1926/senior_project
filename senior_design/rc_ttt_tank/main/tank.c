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
* Title:	turret_task
* Summary:	Rotates the Turret Servo Motor. Based on Target Tracking Input Data
* Param:
* Return:
**************************************************/
void turret_task(void *pvParameter){
	esp_rom_gpio_pad_select_gpio(TURRET_PIN);

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
    ledc_conf1.duty = 4;
    ledc_conf1.hpoint = 0;
    ledc_channel_config(&ledc_conf1);

	static uint32_t rotate_turret = 1;

	while (1) {
		//Wait for a message to be received from firing_task for a fire button press
		/*if(uxQueueMessagesWaiting(turret_queue) > 0){
			if(xQueueReceive(turret_queue,rotate_turret,25)== pdTRUE){}
		}*/
        // Wait until the GPIO pin controlling the servo motor is pulled low
		if(rotate_turret == 1){
			ESP_LOGI(TAG, "Turret Servo Activated");
			vTaskDelay(10 / portTICK_PERIOD_MS);
            for(int i = 0; i < 2; i++){
                // Rotate the servo forward (270 degrees)
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, TURRET_PWM_CHANNEL, DUTY_MAX_TURRET);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, TURRET_PWM_CHANNEL);
                vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for 1 second

                // Rotate the servo back to the starting position (0 degrees)
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, TURRET_PWM_CHANNEL, DUTY_MIN_TURRET);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, TURRET_PWM_CHANNEL);
                vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for 10 milliseconds
            }
		}
    }
}

/**************************************************
* Title:	firing_task
* Summary:	function activates the firing mechanism micro servo motor
            Which rotates forward (180 Deg) then back (0 Deg) and pushes 
            out the nerf dart using the 3D printed rack and pinion 
* Param:
* Return:
**************************************************/
void firing_task(void *pvParameter) {
    esp_rom_gpio_pad_select_gpio(FIRE_SERVO_PIN);

    ledc_timer_config_t timer_conf0;
    timer_conf0.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf0.timer_num = SERVO_PWM_TIMER;
	timer_conf0.duty_resolution = DUTY_RESOLUTION;
    timer_conf0.freq_hz = PWM_FREQUENCY;
    ledc_timer_config(&timer_conf0);

    ledc_channel_config_t ledc_conf0;
    ledc_conf0.gpio_num = FIRE_SERVO_PIN;
    ledc_conf0.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_conf0.channel = SERVO_PWM_CHANNEL;
    ledc_conf0.intr_type = LEDC_INTR_DISABLE;
    ledc_conf0.timer_sel = SERVO_PWM_TIMER;
    ledc_conf0.duty = 4;
    ledc_conf0.hpoint = 0;
    ledc_channel_config(&ledc_conf0);

	static uint32_t fire_servo = 0;

    while (1) {
		//Wait for a message to be received from firing_task for a fire button press
		if(uxQueueMessagesWaiting(firing_queue) > 0){
			if(xQueueReceive(firing_queue,fire_servo,25)== pdTRUE){}
		}
        // Wait until the GPIO pin controlling the servo motor is pulled low
		if(fire_servo == 1){
			ESP_LOGI(TAG, "Firing Servo Activated");
			vTaskDelay(10 / portTICK_PERIOD_MS);
            for(int i = 0; i < 2; i++){
                // Rotate the servo forward (180 degrees)
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL, DUTY_MAX_FIRE);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL);
                vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for 1 second

                // Rotate the servo back to the starting position (0 degrees)
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL, DUTY_MIN_FIRE);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL);
                vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for 10 milliseconds
            }
		}
    }
}

/**************************************************
* Title:	driving_task
* Summary:	function activates the driving motors of the tank using the h-bridge,
            it drives the motors clockwise or counterclock wise while a driving 
            button is being press on the remote
* Param:
* Return:
**************************************************/
void driving_task(void *pvParameter) {
	static uint32_t right_drive = 0; //1 = Right Forward, 2 = Right Back
	static uint32_t left_drive = 0; //1 = Left Forward, 2 = Left Back
	
	while(1){
		//Wait for a message to be received from driving_task right driving button presses
		if(uxQueueMessagesWaiting(r_motor_queue) > 0){
			if(xQueueReceive(r_motor_queue,right_drive,25)== pdTRUE){}
		}

		//Wait for a message to be received from driving_task left driving button presses
		if(uxQueueMessagesWaiting(l_motor_queue) > 0){
			if(xQueueReceive(l_motor_queue,left_drive,25)== pdTRUE){}
		}

		//Right Forward Motion
		if (right_drive = 1){
			ESP_LOGI(TAG, "Right Forward Activated");
			for (int duty = 0; duty <= 1023; duty += 100) {
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, RF_PWM_CHANNEL, duty);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, RF_PWM_CHANNEL);
                vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust acceleration
            }
		}
		//Right Backwards Motion
		else if (right_drive == 2){
			ESP_LOGI(TAG, "Right Back Activated");
			for (int duty = 0; duty <= 1023; duty += 100) {
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, RB_PWM_CHANNEL, duty);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, RB_PWM_CHANNEL);
                vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust acceleration
            }
		}
		//Right Motor Off
		else if (right_drive == 0){
			ESP_LOGI(TAG, "Right Motor Off");
			ledc_set_duty(LEDC_HIGH_SPEED_MODE, RB_PWM_CHANNEL, 0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, RB_PWM_CHANNEL);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, RF_PWM_CHANNEL, 0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, RF_PWM_CHANNEL);
		}

		//Left Forwards Motion
		if (left_drive == 1){
			ESP_LOGI(TAG, "Left Forward Activated");
			for (int duty = 0; duty <= 1023; duty += 100) {
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, LF_PWM_CHANNEL, duty);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, LF_PWM_CHANNEL);
                vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust acceleration
            }
		}
		//Left Backwards Motion 
		else if (left_drive == 2){
			ESP_LOGI(TAG, "Left Back Activated");
			for (int duty = 0; duty <= 1023; duty += 100) {
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, LB_PWM_CHANNEL, duty);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, LB_PWM_CHANNEL);
                vTaskDelay(100 / portTICK_PERIOD_MS); // Adjust acceleration
            }
		}
		//Left Motor Off
		else if (left_drive == 0){
			ESP_LOGI(TAG, "Left Motor Off");
			ledc_set_duty(LEDC_HIGH_SPEED_MODE, LF_PWM_CHANNEL, 0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LF_PWM_CHANNEL);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LB_PWM_CHANNEL, 0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LB_PWM_CHANNEL);
		}
		vTaskDelay(10 / portTICK_PERIOD_MS); // Adjust control frequency
	}
}

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
	r_motor_queue = xQueueCreate(4, sizeof(uint32_t));
	l_motor_queue = xQueueCreate(4, sizeof(uint32_t));
	firing_queue = xQueueCreate(1,sizeof(uint32_t));
	turret_queue = xQueueCreate(8,sizeof(uint32_t));

	xTaskCreate(firing_task, "firing_task", 4096, NULL, 5, NULL);
	xTaskCreate(driving_task, "driving_task", 4096, NULL, 5, NULL);
	xTaskCreate(turret_task, "turret_task", 4096, NULL, 5, NULL);
	//xTaskCreate(target_tracking_task, "target_tracking_task", 4096, NULL, 3, NULL);

	while(1){

		vTaskDelay(750 / portTICK_PERIOD_MS);
	}
}
