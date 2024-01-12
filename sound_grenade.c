#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define RED_1 13
#define GREEN_1 12
#define BLUE_1 14
#define RED_2 2
#define GREEN_2 4
#define BLUE_2 5
#define RED_3 18
#define GREEN_3 19
#define BLUE_3 21
#define RED_4 27
#define GREEN_4 26
#define BLUE_4 25
#define BUTTON_GO 22
#define BUTTON_RESET 23
#define SOUND_BOARD 32
#define DEBOUNCE_DELAY 50
#define ON 0
#define OFF 1

void initialize();

typedef enum {
    IDLE,
    PRESSED,
    RELEASED,
} button_state_t;

void initialize() {
    //Set gpio pin directions for RGB LEDs
    gpio_reset_pin(RED_1);
    gpio_set_direction(RED_1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GREEN_1);
    gpio_set_direction(GREEN_1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(BLUE_1);
    gpio_set_direction(BLUE_1, GPIO_MODE_OUTPUT);

    gpio_reset_pin(RED_2);
    gpio_set_direction(RED_2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GREEN_2);
    gpio_set_direction(GREEN_2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(BLUE_2);
    gpio_set_direction(BLUE_2, GPIO_MODE_OUTPUT);

    gpio_reset_pin(RED_3);
    gpio_set_direction(RED_3, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GREEN_3);
    gpio_set_direction(GREEN_3, GPIO_MODE_OUTPUT);
    gpio_reset_pin(BLUE_3);
    gpio_set_direction(BLUE_3, GPIO_MODE_OUTPUT);

    gpio_reset_pin(RED_4);
    gpio_set_direction(RED_4, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GREEN_4);
    gpio_set_direction(GREEN_4, GPIO_MODE_OUTPUT);
    gpio_reset_pin(BLUE_4);
    gpio_set_direction(BLUE_4, GPIO_MODE_OUTPUT);

    // Initialize LEDs to off
    gpio_set_level(RED_1, OFF);
    gpio_set_level(GREEN_1, OFF);
    gpio_set_level(BLUE_1, OFF);

    gpio_set_level(RED_2, OFF);
    gpio_set_level(GREEN_2, OFF);
    gpio_set_level(BLUE_2, OFF);

    gpio_set_level(RED_3, OFF);
    gpio_set_level(GREEN_3, OFF);
    gpio_set_level(BLUE_3, OFF);

    gpio_set_level(RED_4, OFF);
    gpio_set_level(GREEN_4, OFF);
    gpio_set_level(BLUE_4, OFF);

	// Enable pull-up resistors for the buttons
    gpio_set_direction(BUTTON_GO, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON_RESET, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BUTTON_RESET, GPIO_PULLUP_ONLY);

	//Initialize Sound Board Pin 
	gpio_reset_pin(SOUND_BOARD);
	gpio_set_pull_mode(SOUND_BOARD, GPIO_PULLUP_ONLY);
    gpio_set_direction(SOUND_BOARD, GPIO_MODE_OUTPUT);
    gpio_set_level(SOUND_BOARD, OFF);
}

void app_main(void)
{
    initialize();

    button_state_t buttonGoState = IDLE;
    TickType_t buttonGoPressTime = 0;

    while (1) {
        //Track Go Button Current Level Pressed or Unpressed
        int buttonGoLevel = gpio_get_level(BUTTON_GO);

		//Switch Case for Go Button State
        switch (buttonGoState) {
			//Wait For Go Button Press
            case IDLE:
                if (buttonGoLevel == 0) {
                    buttonGoState = PRESSED;
                    buttonGoPressTime = xTaskGetTickCount();
                }
                break;
			//If pressed make sure debouce delay is passed before moving to released state
            case PRESSED:
                if (buttonGoLevel == 1) {
                    if ((xTaskGetTickCount() - buttonGoPressTime) >= pdMS_TO_TICKS(DEBOUNCE_DELAY)) {
                        // Debounce delay passed, transition to RELEASED state
                        buttonGoState = RELEASED;
                    }
                } else {
                    // Button released before debounce delay, return to IDLE state
                    buttonGoState = IDLE;
                }
                break;
			//Released state triggers FLashing LEDs and Sound 
            case RELEASED:
				//Start 4 Second Timer 
				vTaskDelay(4000 / portTICK_PERIOD_MS);
				//Activate Sound Board to Play Grenade Sounds 
	            gpio_set_level(SOUND_BOARD, ON);
                // Perform LED flashing 
                gpio_set_level(RED_1, ON);
                gpio_set_level(GREEN_1, ON);
                gpio_set_level(BLUE_1, ON);

                gpio_set_level(RED_2, ON);
                gpio_set_level(GREEN_2, ON);
                gpio_set_level(BLUE_2, ON);

                gpio_set_level(RED_3, ON);
                gpio_set_level(GREEN_3, ON);
                gpio_set_level(BLUE_3, ON);

                gpio_set_level(RED_4, ON);
                gpio_set_level(GREEN_4, ON);
                gpio_set_level(BLUE_4, ON);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                gpio_set_level(SOUND_BOARD, OFF);
                
                int N = 100;
                for (int i = 0; i < 10; i++) {
                    gpio_set_level(RED_1, ON);
                    gpio_set_level(GREEN_1, ON);
                    gpio_set_level(BLUE_1, ON);

                    gpio_set_level(RED_2, ON);
                    gpio_set_level(GREEN_2, ON);
                    gpio_set_level(BLUE_2, ON);

                    gpio_set_level(RED_3, ON);
                    gpio_set_level(GREEN_3, ON);
                    gpio_set_level(BLUE_3, ON);

                    gpio_set_level(RED_4, ON);
                    gpio_set_level(GREEN_4, ON);
                    gpio_set_level(BLUE_4, ON);
                    vTaskDelay(N / portTICK_PERIOD_MS);
                    gpio_set_level(RED_1, OFF);
                    gpio_set_level(GREEN_1, OFF);
                    gpio_set_level(BLUE_1, OFF);

                    gpio_set_level(RED_2, OFF);
                    gpio_set_level(GREEN_2, OFF);
                    gpio_set_level(BLUE_2, OFF);

                    gpio_set_level(RED_3, OFF);
                    gpio_set_level(GREEN_3, OFF);
                    gpio_set_level(BLUE_3, OFF);

                    gpio_set_level(RED_4, OFF);
                    gpio_set_level(GREEN_4, OFF);
                    gpio_set_level(BLUE_4, OFF);
                    vTaskDelay(N / portTICK_PERIOD_MS);
                    N = N + 20;
                }
                gpio_set_level(RED_1, ON);
                gpio_set_level(GREEN_1, ON);
                gpio_set_level(BLUE_1, ON);

                gpio_set_level(RED_2, ON);
                gpio_set_level(GREEN_2, ON);
                gpio_set_level(BLUE_2, ON);
                
                gpio_set_level(RED_3, ON);
                gpio_set_level(GREEN_3, ON);
                gpio_set_level(BLUE_3, ON);

                gpio_set_level(RED_4, ON);
                gpio_set_level(GREEN_4, ON);
                gpio_set_level(BLUE_4, ON);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                gpio_set_level(RED_1, ON);
                gpio_set_level(GREEN_1, OFF);
                gpio_set_level(BLUE_1, OFF);

                gpio_set_level(RED_2, ON);
                gpio_set_level(GREEN_2, OFF);
                gpio_set_level(BLUE_2, OFF);

                gpio_set_level(RED_3, ON);
                gpio_set_level(GREEN_3, OFF);
                gpio_set_level(BLUE_3, OFF);

                gpio_set_level(RED_4, ON);
                gpio_set_level(GREEN_4, OFF);
                gpio_set_level(BLUE_4, OFF);
				//Reset State Machine 
                buttonGoState = IDLE;
                break;
            default:
				buttonGoState = IDLE;
                break;
        }
        // Reset LEDs for another activation of the sound grenade
        if (gpio_get_level(BUTTON_RESET) == 0) {
            initialize();
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
