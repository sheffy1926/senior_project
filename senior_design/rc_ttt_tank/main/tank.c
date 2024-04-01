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
* Summary:	Controls IR Sensors and emitters and send data to turret task based if 
            there is an IR signal detected. Rotates the Turret Servo Motor. 
            Based on Target Tracking Input Data from the IR Detectors based on if 
            they are receiving IR light or not from the other tank's emitters.
* Param:
* Return:
**************************************************/
void target_tracking_task(void *pvParameter) {
    //,ADC1_CHANNEL_0
    int channels = 5;
    int adc_channels[5] = {ADC1_CHANNEL_3,ADC1_CHANNEL_6,ADC1_CHANNEL_7,ADC1_CHANNEL_4,ADC1_CHANNEL_5};
    int raw_data[5] = {0};
    float v_data[5] = {0};
    float v_sensor[5] = {1,2,3,4,5};

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
    // Initialize turret angle to midpoint (135 degrees) (22?)
    int angle = (MIN_DUTY_CYCLE + MAX_DUTY_CYCLE) / 2;
    
    while (1) {
        //If the flywheels are activated, then the turret and sensors are active for sensing and rotating
        if(rotate_turret == 1){
            for (int i = 0; i < channels; i++){
                //Read ADC value from IR Photodiode
                //Vout = Dout * Vmax / Dmax = convert raw data to voltage
                raw_data[i] = adc1_get_raw(adc_channels[i]); 
                v_data[i] = (raw_data[i] / (float)ADC_MAX_VALUE) * MAX_VOLTAGE;
                v_sensor[i] = i+1;

                //Print the ADC value to esp_log
                ESP_LOGI(TAG, "IR_S_%d Raw Data: %d, Voltage: %f", i+1, raw_data[i], v_data[i]);
                //Delay for 0.1 seconds
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
            //Read ADC2 value from IR Photodiode on ADC2_CHANNEL_6
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

            //Bubble Sort algorithm to sort v_data and v_sensor in descending order
            for (int i = 0; i < channels - 1; i++) {
                for (int j = 0; j < channels - i - 1; j++) {
                    if (v_data[j] < v_data[j + 1]) {
                        //Swap v_data[j] and v_data[j + 1]
                        float temp = v_data[j];
                        v_data[j] = v_data[j + 1];
                        v_data[j + 1] = temp;
                        //Swap v_sensor[j] and v_sensor[j + 1]
                        int n = v_sensor[j];
                        v_sensor[j] = v_sensor[j + 1];
                        v_sensor[j + 1] = n;
                    }
                }
            }

            //Adjust angle of turret based on what sensor has the highest voltage
            int angle_adjustment = 0;
            if (v_data[0] >= MIN_VOLTAGE && v_data[0] <= MAX_VOLTAGE) {
                if (v_sensor[0] == 1) {
                    if (v_sensor[1] == 2){
                        angle_adjustment = 1;   // Rotate turret clockwise
                    }
                    else if (v_sensor[1] == 5){
                        angle_adjustment = -1;  // Rotate turret counter clockwise
                    }
                    else {
                        angle_adjustment = 1;   // Rotate turret clockwise
                    }
                } else if (v_sensor[0] == 2){
                    angle_adjustment = 1;       // Rotate turret clockwise
                } else if (v_sensor[0] == 4){
                    angle_adjustment = -1;      // Rotate turret counter clockwise
                } else if (v_sensor[0] == 5){
                    if (v_sensor[1] == 4){
                        angle_adjustment = -1;  // Rotate turret counter clockwise
                    }
                    else if (v_sensor[1] == 1){
                        angle_adjustment = 1;   // Rotate turret clockwise
                    }
                    else {
                        angle_adjustment = -1;  // Rotate turret counter clockwise
                    }
                //If the center sensor has the highest value make minor adjustments to fine tune the direction of the turret.
                } else if (v_sensor[0] == 3) {
                    // Rotate one way to see if the value goes up, then rotate back if it goes down
                    int original_angle = angle;
                    angle += angle_adjustment * DUTY_CYCLE_STEP;
                    for(int i = 0; i < 2; i++){
                        //Rotate the turret 
                        ledc_set_duty(LEDC_HIGH_SPEED_MODE, TURRET_PWM_CHANNEL, angle);
                        ledc_update_duty(LEDC_HIGH_SPEED_MODE, TURRET_PWM_CHANNEL);
                        vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for 0.5 seconds
                    }

                    // If value increases, continue rotating in the same direction
                    int new_sensor_value = adc1_get_raw(adc_channels[2]);
                    float new_sensor_voltage = (new_sensor_value / (float)ADC_MAX_VALUE) * MAX_VOLTAGE;
                    //If the voltage got worse rotate back to the original angle
                    if (new_sensor_voltage < v_data[0]) {
                        if (angle_adjustment == 1){
                            angle_adjustment = -1;
                        }
                        else {
                            angle_adjustment = 1;
                        }
                        angle = original_angle;
                    }
                }
                // Update turret angle within limits
                angle += angle_adjustment * DUTY_CYCLE_STEP;
                if (angle < MIN_DUTY_CYCLE) {
                    angle = MIN_DUTY_CYCLE;
                } else if (angle > MAX_DUTY_CYCLE) {
                    angle = MAX_DUTY_CYCLE;
                }
            }
            //If no sensor is receiving a high enough value return to the starting position
            else {
                angle = (MIN_DUTY_CYCLE + MAX_DUTY_CYCLE) / 2;
            }

            // Set the duty cycle of the servo motor based on turret angle
            for(int i = 0; i < 2; i++){
                //Rotate the turret 
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, TURRET_PWM_CHANNEL, angle);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, TURRET_PWM_CHANNEL);
                vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for 0.5 seconds
            }

        }
        //If the flywheels are off, then turn the sensors and rotation off
        else {
            angle = (MIN_DUTY_CYCLE + MAX_DUTY_CYCLE) / 2;

            // Set the duty cycle of the servo motor based on turret angle
            for(int i = 0; i < 2; i++){
                //Rotate the turret 
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, TURRET_PWM_CHANNEL, angle);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, TURRET_PWM_CHANNEL);
                vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for 0.5 seconds
            }
        }
        //Delay for 0.5 seconds
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
