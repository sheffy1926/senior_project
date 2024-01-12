#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "driver/gpio.h"

#define RESOLUTION_HZ   10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_GPIO_NUM    5  //LED Strip GPIO pin
#define LED_NUMBERS     60  //Number of LEDs
#define BUTTON_GO       22  //GPIO for Go Button
#define BUTTON_RESET    23  //GPIO for Reset Button
#define SOUND_BOARD     12  //GPIO for Sound Board
#define DEBOUNCE_DELAY  50
#define ON              0
#define OFF             1

void initialize();
void reset();

static uint8_t led_strip_pixels[LED_NUMBERS * 3];

typedef enum {
    IDLE,
    PRESSED,
    RELEASED,
} button_state_t;

void initialize() {
	//Enable pull-up resistors for the buttons
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

void reset(){
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;

    //Create RMT TX channel
    rmt_channel_handle_t led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = LED_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    rmt_new_tx_channel(&tx_chan_config, &led_chan);
    //Install led strip encoder
    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RESOLUTION_HZ,
    };
    rmt_new_led_strip_encoder(&encoder_config, &led_encoder);
    //Enable RMT TX channel
    rmt_enable(led_chan);
    //Start LED rainbow chase
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };

    //Turn LEDs Off 
    for (int j = 0; j < LED_NUMBERS; j ++) {
        // Build RGB pixels
        led_strip_pixels[j * 3 + 0] = red;
        led_strip_pixels[j * 3 + 1] = green;
        led_strip_pixels[j * 3 + 2] = blue;
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
    gpio_set_level(SOUND_BOARD, OFF);
}

void app_main(void)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;

    //Create RMT TX channel
    rmt_channel_handle_t led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = LED_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    rmt_new_tx_channel(&tx_chan_config, &led_chan);
    //Install led strip encoder
    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RESOLUTION_HZ,
    };
    rmt_new_led_strip_encoder(&encoder_config, &led_encoder);
    //Enable RMT TX channel
    rmt_enable(led_chan);
    //Start LED rainbow chase
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };

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
				//Start 4 Second Timer ---------------------------------------------------------------
				vTaskDelay(4000 / portTICK_PERIOD_MS);
                //Activate Sound Board to Play Grenade Sounds
                gpio_set_level(SOUND_BOARD, ON);
                vTaskDelay(100 / portTICK_PERIOD_MS);
                gpio_set_level(SOUND_BOARD, OFF);
                //Flash the LEDs 
                int N = 100;
                for (int i = 0; i < 10; i++) {
                    for (int j = i; j < LED_NUMBERS; j ++) {
                        // Build RGB pixels
                        green = 255;
                        blue = 255;
                        red = 255;
                        led_strip_pixels[j * 3 + 0] = green;
                        led_strip_pixels[j * 3 + 2] = red;
                        led_strip_pixels[j * 3 + 1] = blue;
                    }
                    //LEDs On - Flush RGB values to LEDs
                    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
                    vTaskDelay(N / portTICK_PERIOD_MS);
                    //LEDs Off
                    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
                    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
                    vTaskDelay(N / portTICK_PERIOD_MS);
                    N = N + 20;
                }
                vTaskDelay(10 / portTICK_PERIOD_MS);

                //LEDS to Red
                 for (int i = 0; i < 3; i++) {
                    for (int j = i; j < LED_NUMBERS; j++) {
                        // Build RGB pixels
                        red = 255;
                        green = 0;
                        blue = 0;
                        led_strip_pixels[j * 3 + 0] = green;
                        led_strip_pixels[j * 3 + 1] = red;
                        led_strip_pixels[j * 3 + 2] = blue;
                    }
                    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
                }
				//Reset State Machine 
                buttonGoState = IDLE;
                break;
            default:
				buttonGoState = IDLE;
                break;
        }
        // Reset LEDs for another activation of the LEDs and Sound Board
        if (gpio_get_level(BUTTON_RESET) == 0) {
            reset();
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}