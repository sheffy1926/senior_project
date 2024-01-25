#ifndef ESPNOW_BASIC_CONFIG_H
#define ESPNOW_BASIC_CONFIG_H

#include <inttypes.h>
#include <stdbool.h>

#define INKY	1
#define BLINKY	2
//change this value to compile for different care remote pairs
#define CAR_REMOTE_PAIR INKY

#define RF_PIN 12
#define RB_PIN 13
#define LF_PIN 2
#define LB_PIN 4
#define FIRE_PIN 14
#define FLY_WHEEL_PIN 5
#define PWM_PIN 19

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_RX_CHANNEL RMT_CHANNEL_2

#define RF_BUT 12
#define RB_BUT 13
#define LF_BUT 2
#define LB_BUT 4
#define FIRE_BUT 14
#define FLY_WHEEL_BUT 5

//message types
#define CAR_COMMAND  	1
#define TURRET_COMMAND  2

// Define the structure of your data
typedef struct __attribute__((packed)) {
	//message type
	uint8_t message_type;

	//commands to the car
	//use if message type is CAR_COMMAND
    bool rf;
    bool rb;
    bool lf;
    bool lb;
	bool fire_turret;

	//hit report 
	//use if mssage type is TURRET_COMMAND
	uint8_t turret_shooting;
	uint8_t flywheel_activated;

	uint8_t car_id;
} my_data_t;

//INKY
#define PAIR_1_CAR_MAC		{0xec, 0xda, 0x3b, 0x0f, 0x20, 0x44}
#define PAIR_1_REMOTE_MAC	{0x34, 0x85, 0x18, 0x23, 0x7e, 0xb0}
#define PAIR_1_REMOTE_MAC_ARR	(uint8_t[])PAIR_1_REMOTE_MAC

//BLINKY
#define PAIR_2_CAR_MAC		{0xf4, 0x12, 0xfa, 0x1b, 0x8a, 0x30}
#define PAIR_2_REMOTE_MAC	{0x48, 0x27, 0xe2, 0xad, 0x03, 0x60}
#define PAIR_2_REMOTE_MAC_ARR	(uint8_t[])PAIR_2_REMOTE_MAC

#if CAR_REMOTE_PAIR == INKY
	#define CAR_MAC		PAIR_1_CAR_MAC	 
	#define REMOTE_MAC	PAIR_1_REMOTE_MAC
	#define CAR_ID 1
	#define REMOTE_ID 1

#elif CAR_REMOTE_PAIR == BLINKY
	#define CAR_MAC		PAIR_2_CAR_MAC	 
	#define REMOTE_MAC	PAIR_2_REMOTE_MAC
	#define CAR_ID 2
	#define REMOTE_ID 2
#endif

#define MY_ESPNOW_PMK "pmk1234567890123"
#define MY_ESPNOW_CHANNEL 1

// #define MY_ESPNOW_ENABLE_LONG_RANGE 1

#define MY_SLAVE_DEEP_SLEEP_TIME_MS 1000

#endif // ESPNOW_BASIC_CONFIG_H