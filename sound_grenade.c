#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
//#include "esp_log.h"

#define BLINK_LED1 2
#define BLINK_LED2 13
#define BLINK_LED3 12
#define BLINK_LED4 14
#define BUTTON_GO 22
#define BUTTON_RESET 23
#define SOUND_BOARD 4
#define DEBOUNCE_DELAY 50

typedef enum {
    IDLE,
    PRESSED,
    RELEASED,
} button_state_t;

void app_main(void)
{
    //char *outTaskName = pcTaskGetName(NULL);
    //ESP_LOGI(outTaskName, "Blinking LED!\n");
	//Initialize LED Pins
    gpio_reset_pin(BLINK_LED1);
    gpio_set_direction(BLINK_LED1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(BLINK_LED2);
    gpio_set_direction(BLINK_LED2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(BLINK_LED3);
    gpio_set_direction(BLINK_LED3, GPIO_MODE_OUTPUT);
    gpio_reset_pin(BLINK_LED4);
    gpio_set_direction(BLINK_LED4, GPIO_MODE_OUTPUT);
	// Initialize LEDs to off
    gpio_set_level(BLINK_LED1, 0);
    gpio_set_level(BLINK_LED2, 0);
    gpio_set_level(BLINK_LED3, 0);
    gpio_set_level(BLINK_LED4, 0);
	// Enable pull-up resistors for the buttons
    gpio_set_direction(BUTTON_GO, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON_RESET, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BUTTON_RESET, GPIO_PULLUP_ONLY);
	//Initialize Sound Board Pin 
	gpio_reset_pin(SOUND_BOARD);
	gpio_set_pull_mode(SOUND_BOARD, GPIO_PULLUP_ONLY);
    gpio_set_direction(SOUND_BOARD, GPIO_MODE_OUTPUT);

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
				/*gpio_set_level(SOUND_BOARD, 0);
				vTaskDelay(200 / portTICK_PERIOD_MS);
				gpio_set_level(SOUND_BOARD, 1);*/

                // Perform LED flashing 
				gpio_set_level(BLINK_LED1, 1);
                gpio_set_level(BLINK_LED2, 1);
                gpio_set_level(BLINK_LED3, 1);
                gpio_set_level(BLINK_LED4, 1);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                int N = 100;
                for (int i = 0; i < 10; i++) {
                    gpio_set_level(BLINK_LED1, 1);
                    gpio_set_level(BLINK_LED2, 1);
                    gpio_set_level(BLINK_LED3, 1);
                    gpio_set_level(BLINK_LED4, 1);
                    vTaskDelay(N / portTICK_PERIOD_MS);
                    gpio_set_level(BLINK_LED1, 0);
                    gpio_set_level(BLINK_LED2, 0);
                    gpio_set_level(BLINK_LED3, 0);
                    gpio_set_level(BLINK_LED4, 0);
                    vTaskDelay(N / portTICK_PERIOD_MS);
                    N = N + 20;
                }
                gpio_set_level(BLINK_LED1, 1);
                gpio_set_level(BLINK_LED2, 1);
                gpio_set_level(BLINK_LED3, 1);
                gpio_set_level(BLINK_LED4, 1);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                //gpio_set_level(BLINK_LED1, 0);
                //gpio_set_level(BLINK_LED2, 0);
                //gpio_set_level(BLINK_LED3, 0);
                //gpio_set_level(BLINK_LED4, 0);
				//Reset State Machine 
                buttonGoState = IDLE;
                break;
            default:
				buttonGoState = IDLE;
                break;
        }
        // Reset LEDs for another activation of the sound grenade
        if (gpio_get_level(BUTTON_RESET) == 0) {
            gpio_set_level(BLINK_LED1, 0);
            gpio_set_level(BLINK_LED2, 0);
            gpio_set_level(BLINK_LED3, 0);
            gpio_set_level(BLINK_LED4, 0);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
