#ifndef ESPNOW_BASIC_CONFIG_H
#define ESPNOW_BASIC_CONFIG_H

#include <inttypes.h>
#include <stdbool.h>

#define LINK	1
#define ZELDA	2
//change this value to compile for different tank remote pairs
#define TANK_REMOTE_PAIR LINK

#define ON 0
#define OFF 1
#define TRUE 1
#define FALSE 0
#define DEBOUNCE_DELAY_MS 30

//Tank Pins for Motors, Firing Mechanism, Flywheels, 
//PWM Signals, IR Sensors & Emitters
#define RB_IN1_PIN 5
#define RF_IN2_PIN 18
#define LF_IN3_PIN 19
#define LB_IN4_PIN 21 
#define TURRET_PIN 13
#define FIRE_SERVO_PIN 15 
#define FW_PIN 12
/* #define IR_S_1 14
#define IR_S_2 27
#define IR_S_3 26
#define IR_S_4 25
#define IR_S_5 33
#define IR_S_6 32
#define IR_S_7 35
#define IR_EMIT 4 */

//Remote Pins for Buttons and LEDs
#define RF_BUT 12
#define RB_BUT 13
#define LF_BUT 15
#define LB_BUT 4
#define FIRE_BUT 14
#define FW_BUT 5
#define FIRE_LED 25
#define FW_LED 18

//Firing Servo Variables
#define SERVO_PWM_CHANNEL   	LEDC_CHANNEL_0
#define SERVO_PWM_TIMER     	LEDC_TIMER_0
#define DUTY_RESOLUTION 		8
#define PWM_FREQUENCY      		50 	// Hz
#define DUTY_MIN_FIRE       	8   // 8% duty cycle (0 degrees)
#define DUTY_MAX_FIRE       	32  // 32% duty cycle (180 degrees)

//Turret Servo Variables
#define TURRET_PWM_CHANNEL  	LEDC_CHANNEL_1
#define TURRET_PWM_TIMER    	LEDC_TIMER_1
#define DUTY_MIN_TURRET       	8   // 8% duty cycle (0 degrees)
#define DUTY_1					16
#define DUTY_2 					24
#define DUTY_MAX_TURRET       	32  // 32% duty cycle (270 degrees)
//#define SERVO_MIN_PULSEWIDTH   	550  // Minimum pulse width in microseconds
//#define SERVO_MAX_PULSEWIDTH   	2450 // Maximum pulse width in microseconds
//#define SERVO_MAX_DEGREE       	270  // Maximum angle in degrees

//message types
#define TANK_COMMAND  	1
//#define FIRE_COMMAND  	2

// Define the structure of your data
typedef struct __attribute__((packed)) {
	//message type
	uint8_t message_type;

	//commands to the tank
    bool rf;
    bool rb;
    bool lf;
    bool lb;
	
	//commands to fire the turret
	bool fire_turret;
	bool activate_fw;
	bool fw_led;

	//uint8_t tank_id;
} my_data_t;

//LINK
#define PAIR_1_TANK_MAC		{0x48, 0xE7, 0x29, 0xB6, 0x76, 0x9C} 
#define PAIR_1_REMOTE_MAC	{0x48, 0xE7, 0x29, 0x96, 0x9C, 0xEC} 
#define PAIR_1_REMOTE_MAC_ARR	(uint8_t[])PAIR_1_REMOTE_MAC

//ZELDA
#define PAIR_2_TANK_MAC		{0x48, 0xE7, 0x29, 0xB6, 0x70, 0xB4}
#define PAIR_2_REMOTE_MAC	{0x48, 0xE7, 0x29, 0xB5, 0xB2, 0x98}
#define PAIR_2_REMOTE_MAC_ARR	(uint8_t[])PAIR_2_REMOTE_MAC

#if TANK_REMOTE_PAIR == LINK
	#define TANK_MAC	PAIR_1_TANK_MAC	 
	#define REMOTE_MAC	PAIR_1_REMOTE_MAC
	//#define TANK_ID 1
	//#define REMOTE_ID 1

#elif TANK_REMOTE_PAIR == ZELDA
	#define TANK_MAC	PAIR_2_TANK_MAC	 
	#define REMOTE_MAC	PAIR_2_REMOTE_MAC
	//#define TANK_ID 2
	//#define REMOTE_ID 2
#endif

#define MY_ESPNOW_PMK "pmk1234567890123"
#define MY_ESPNOW_CHANNEL 1
// #define MY_ESPNOW_ENABLE_LONG_RANGE 1
#define MY_SLAVE_DEEP_SLEEP_TIME_MS 1000

#endif // ESPNOW_BASIC_CONFIG_H