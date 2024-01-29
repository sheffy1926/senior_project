#ifndef ESPNOW_BASIC_CONFIG_H
#define ESPNOW_BASIC_CONFIG_H

#include <inttypes.h>
#include <stdbool.h>

#define LINK	1
#define ZELDA	2
//change this value to compile for different tank remote pairs
#define TANK_REMOTE_PAIR LINK

//Pin Layout for Test Configuration 
#define R_LED_R 13
#define R_LED_G 12
#define R_LED_B 14
#define L_LED_R 2
#define L_LED_G 4
#define L_LED_B 5

//Tank Pins for Motors, Firing Mechanism, Flywheels, 
//PWM Signals, IR Sensors & Emitters
//#define RF_PIN 12
//#define RB_PIN 13
//#define LF_PIN 2
//#define LB_PIN 4 
//#define TURRET_PIN 19
#define FIRE_PIN 18
#define FW_PIN 27
/* #define IR_S_1 22
#define IR_S_2 23
#define IR_S_3 34
#define IR_S_4 35
#define IR_EMIT 21 */

//#define RMT_TX_CHANNEL RMT_CHANNEL_0
//#define RMT_RX_CHANNEL RMT_CHANNEL_2

//Remote Pins for Buttons and LEDs
#define RF_BUT 12
#define RB_BUT 13
#define LF_BUT 2
#define LB_BUT 4
#define FIRE_BUT 14
#define FW_BUT 5
#define FIRE_LED 18
#define FW_LED 25

#define ON 0
#define OFF 1
#define TRUE 1
#define FALSE 0

//message types
#define TANK_COMMAND  	1
#define FIRE_COMMAND  	2

// Define the structure of your data
typedef struct __attribute__((packed)) {
	//message type
	uint8_t message_type;

	//commands to the tank
	//use if message type is TANK_COMMAND
    bool rf;
    bool rb;
    bool lf;
    bool lb;
	
	//commands to fire the turret
	//use if message type is FIRE_COMMAND
	bool fire_turret;
	bool activate_fw;
	bool turret_firing;
	bool fw_active;	

	//uint8_t tank_id;
} my_data_t;

//LINK
#define PAIR_1_TANK_MAC		{0x48, 0xC7, 0x29, 0xB6, 0x76, 0x9C}
#define PAIR_1_REMOTE_MAC	{0x48, 0xE7, 0x29, 0x96, 0x9C, 0xEC}
#define PAIR_1_REMOTE_MAC_ARR	(uint8_t[])PAIR_1_REMOTE_MAC

//ZELDA
#define PAIR_2_TANK_MAC		{0xf4, 0x12, 0xfa, 0x1b, 0x8a, 0x30}
#define PAIR_2_REMOTE_MAC	{0x48, 0x27, 0xe2, 0xad, 0x03, 0x60}
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