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
#define TEST_LED 4			//Test LED Pin If Needed
#define RB_IN1_PIN 5		//Right Back Pin
#define RF_IN2_PIN 18		//Right Forward Pin
#define LF_IN3_PIN 19		//Left Forward Pin
#define LB_IN4_PIN 21 		//Left Back Pin
#define TURRET_PIN 13		//Turret Servo Motor Pin
#define FIRE_PIN 15 		//Firing Mechanism Servo Motor Pin 
#define FW_NMOS 12			//Pin to activate Flywheels NMOS Low-Side Switch

#define IR_EMITS_NMOS 22 	//Pin to activate Emitters NMOS Low-Side Switch
#define IR_S_NMOS 23		//Pin to activate Detectors NMOS Low-Side Switch
//#define IR_S_1 14 		//				- ADC 2_6 (14)
#define IR_S_2 36 			//-ADC 1_0 36	- ADC 2_8 (25)
#define IR_S_3 39 			//-ADC 1_3 39 	- ADC 2_9 (26)
#define IR_S_4 34 			//-ADC 1_6 34   - ADC 2_7 (27)
#define IR_S_5 32 			//ADC 1_4 32
#define IR_S_6 33 			//ADC 1_5 33
#define IR_S_7 35 			//ADC 1_7 35 			

//Remote Pins for Buttons and LEDs
#define RF_BUT 12			//Right Forward Driving Button
#define RB_BUT 13			//Right Back Driving Button
#define LF_BUT 15			//Left Forward Driving Button
#define LB_BUT 4			//Left Back Driving Button
#define FIRE_BUT 14			//Activate Firing Mechanism Button
#define FW_BUT 5			//Toggle Flywheels Button
#define FIRE_LED 25			//Firing Status LED Pin
#define FW_LED 18			//Flywheel Status LED Pin

//Firing Servo Variables
#define SERVO_PWM_CHANNEL   	LEDC_CHANNEL_0
#define SERVO_PWM_TIMER     	LEDC_TIMER_0
#define DUTY_RESOLUTION 		8
#define PWM_FREQUENCY      		50 	// Hz
#define DUTY_MIN_FIRE       	8   // 8% duty cycle (0 degrees)
#define DUTY_MAX_FIRE       	20  // 20% duty cycle (90 degrees on this servo)
//Turret Servo Variables
#define TURRET_PWM_CHANNEL  	LEDC_CHANNEL_1
#define TURRET_PWM_TIMER    	LEDC_TIMER_1
#define DUTY_CYCLE_STEP			1
#define MIN_DUTY_CYCLE 			10
#define DUTY_CENTER				22	//(135 degrees - Centered) (MIN + MAX) / 2 
#define MAX_DUTY_CYCLE			32

#define ADC_WIDTH    	ADC_WIDTH_BIT_12
#define ADC_ATTEN    	ADC_ATTEN_DB_11 	// Attenuation level for ADC
#define V_REF    		1100        	 	// Default voltage reference (millivolts)
#define ADC_MAX_VALUE 	4095
#define MAX_VOLTAGE 	3.3
#define MIN_VOLTAGE		0.5

//message types
#define TANK_COMMAND  	1

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
} my_data_t;

//LINK
#define PAIR_1_TANK_MAC		{0xD8, 0x13, 0x2A, 0x7F, 0x9B, 0x34} 
#define PAIR_1_REMOTE_MAC	{0x48, 0xE7, 0x29, 0x96, 0x9C, 0xEC} 
#define PAIR_1_REMOTE_MAC_ARR	(uint8_t[])PAIR_1_REMOTE_MAC

//ZELDA
#define PAIR_2_TANK_MAC		{0xD8, 0x13, 0x2A, 0x7F, 0x9C, 0xFC}
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