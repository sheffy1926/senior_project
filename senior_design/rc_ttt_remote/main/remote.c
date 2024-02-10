#define CONFIG_FREERTOS_ENABLE_BACKWARD_COMPATIBILITY 1

/****************************************************
 * Remote DONE List:
 * 	Update ESP LOG Message to determine what buttons/LEDs are working correctly
 * 	Make it so Flywheel active LED is turned on directly from remote button press
 *  configure flywheel input LED to be toggled on/off by button press (test)
 * 	configure firing input LED each time button is pressed for short duration (test)
 * Remote TODO List:
 * 	1: configure driving button input to turn on LEDs while button is being pressed in test configuration

 * 	4: test configuration works, develop PWM signal to make a acceleration function for driving input to the motors 
 * 	5: reconfigure flywheel input to toggle on flywheels by flipping a transistor? (Need to research this more)
 * 	6: reconfigure firing input to activate firing servo motor each time button is released (PWM Signal)
 * 
 * 	3D Design Remote. With Grips? 
 * 	Design Custom PCB for Remote
 * 	Buy RC battery for remote or just use 9V batteries?
****************************************************/

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_efuse.h"
#include "esp_sleep.h"
#include "esp_random.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "remote.h"
#include "espnow_basic_config.h"

#define IN_PIN_SEL ((1ULL<<RF_BUT) | (1ULL<<RB_BUT) | (1ULL<<LF_BUT) | (1ULL<<LB_BUT))
#define FIRE_PIN_SEL ((1ULL<<FIRE_BUT) | (1ULL<<FW_BUT))
#define OUT_PIN_SEL ((1ULL<<FW_LED) | (1ULL<<FIRE_LED))

static const char *TAG = "remote";

/**************************************************
* Title:	recv_cb
* Summary:	call_back for when espnow messages are received, 
			activates flywheels and fires turret
* Param:	mac addr-> mac address of the sender (tank) 
*			data-> data packet of my_data_t
*			len -> len of message received
* Return:	none
**************************************************/
void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len){
	ESP_LOGI(TAG, "tank message received");
    
    if(len != sizeof(my_data_t))
    {
        ESP_LOGE(TAG, "Unexpected data length: %d != %u", len, sizeof(my_data_t));
        return;
    }

	//move the tank accordingly
	my_data_t *packet = data; //!note this line generates a warning. it works fine though
								//because we checked the length above

	if(packet->message_type != FIRE_COMMAND){
		ESP_LOGE(TAG, "wrong message_type received from tank");
	} /*else{
		gpio_set_level(FW_LED, packet->fw_active);
		gpio_set_level(FIRE_LED, packet->turret_firing);
	}*/
	return;
}

/**************************************************
    * Title: send_espnow_data
    * Summary: sends an esp_now message to the tank
    * Param:
    * Return:
    **************************************************/
esp_err_t send_espnow_data(void){
    const uint8_t destination_mac[] = TANK_MAC;
	static my_data_t data;

    // Convert MAC address to string
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
             destination_mac[0], destination_mac[1], destination_mac[2],
             destination_mac[3], destination_mac[4], destination_mac[5]);

    // Log the MAC address we are sending data to
    ESP_LOGI(TAG, "Sending data to MAC: %s", mac_str);

	//populate data
	data.message_type = TANK_COMMAND;
	data.rf = !(gpio_get_level(RF_BUT));
	data.rb = !(gpio_get_level(RB_BUT));
	data.lf = !(gpio_get_level(LF_BUT));
	data.lb = !(gpio_get_level(LB_BUT));

	//populate data
	data.message_type = FIRE_COMMAND;
    if (led_state == TRUE){
	    data.fire_turret = !(gpio_get_level(FIRE_BUT));
    }
	data.activate_fw = !(gpio_get_level(FW_BUT));
    
    if((gpio_get_level(RF_BUT)) == 0){	
        ESP_LOGI(TAG, "RF Button Press");} 
    if((gpio_get_level(RB_BUT)) == 0){
		ESP_LOGI(TAG, "RB Button Press");}
    if((gpio_get_level(LF_BUT)) == 0){
		ESP_LOGI(TAG, "LF Button Press");}
    if((gpio_get_level(LB_BUT)) == 0){
		ESP_LOGI(TAG, "LB Button Press");}
    if((gpio_get_level(FIRE_BUT)) == 0){
		ESP_LOGI(TAG, "FIRE Button Press");}
    if((gpio_get_level(FW_BUT)) == 0){
		ESP_LOGI(TAG, "FW Button Press");}

    // Send it
    esp_err_t err = esp_now_send(destination_mac, (uint8_t*)&data, sizeof(data));
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "Send error (%d)", err);
        return ESP_FAIL;
    }
	//ESP_LOGI(TAG, "Remote Data Sent!");
    return ESP_OK;
}

/**************************************************
* Title:	packet_sent_cb
* Summary:	function that send the status of the espnow message 
            if it was a success or a failure
* Param:
* Return:
**************************************************/
static void packet_sent_cb(const uint8_t *mac_addr, esp_now_send_status_t status){
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }
    assert(status == ESP_NOW_SEND_SUCCESS || status == ESP_NOW_SEND_FAIL);
    xEventGroupSetBits(s_evt_group, BIT(status));
}

/**************************************************
* Title:	init_espnow_master
* Summary:	initializes the wireless messaging of the remote
* Param:
* Return:
**************************************************/
static void init_espnow_master(void){
    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(MY_ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    vTaskDelay(500 / portTICK_PERIOD_MS);
#if MY_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(MY_ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
    ESP_ERROR_CHECK( esp_now_init() );
    // Define a receive function pointer using the typedef
    esp_now_recv_cb_t recv_cb_ptr = &recv_cb;
    ESP_ERROR_CHECK( esp_now_register_recv_cb(recv_cb_ptr) );
	ESP_LOGD(TAG,"send_cb registered, attempting to register recv_cb for remote");
	
    ESP_ERROR_CHECK( esp_now_register_send_cb(packet_sent_cb) );
    ESP_LOGD(TAG,"attempting to register send_cb for remote");
    ESP_ERROR_CHECK( esp_now_set_pmk((const uint8_t *)MY_ESPNOW_PMK) );

    // Alter this if you want to specify the gateway mac, enable encyption, etc
    const esp_now_peer_info_t broadcast_destination = {
        .peer_addr = TANK_MAC,
        .channel = MY_ESPNOW_CHANNEL,
        .ifidx = MY_ESPNOW_WIFI_IF
    };
    ESP_ERROR_CHECK( esp_now_add_peer(&broadcast_destination) );
}

/**************************************************
* Title:	gpio_isr_handler
* Summary:	Handles the isr whenever any GPIO interrupt occurs
* Param:
* Return:
**************************************************/
static void IRAM_ATTR gpio_isr_handler(void* arg){
	int button_pin = (int)arg;
	button_event_t button_event = {
		.button_pin = button_pin,
	};
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendFromISR(button_queue, &button_event, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**************************************************
* Title:	button_task
* Summary:	Task handles GPIO button interrupts 
* Param:
* Return:
**************************************************/
void button_task(void *args) {
	button_event_t button_event;
    int fire_but_level;
    TickType_t last_press_time = 0;

	while(1){
		if(xQueueReceive(button_queue, &button_event, portMAX_DELAY) == pdTRUE){
            // Perform debouncing for FIRE_BUT and FW_BUT buttons
            if (button_event.button_pin == FIRE_BUT || button_event.button_pin == FW_BUT) {
                TickType_t current_time = xTaskGetTickCount();
                if ((current_time - last_press_time) < pdMS_TO_TICKS(DEBOUNCE_DELAY_MS)) {
                    // Debounce period not elapsed yet, ignore this press
                    continue;
                }
                last_press_time = current_time;
                
                if(button_event.button_pin == FIRE_BUT){
                    fire_but_level = 1;
                    firing_buttons(fire_but_level);
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    fire_but_level = 0;
                }
                else if (button_event.button_pin == FW_BUT){
                    fire_but_level = 2;
                    firing_buttons(fire_but_level);
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                    fire_but_level = 0;
                }
                fire_but_level = 0;
                firing_buttons(fire_but_level);
            }
            //Send Button Press Data through ESPNOW
			send_espnow_data();
            
		} 
	}
}

/**************************************************
* Title:	firing_buttons
* Summary:	Function handles firing button presses and activates 
            the Flywheels LED and the Busy Firing LED
* Param:
* Return:
**************************************************/
void firing_buttons(int fire_but_level){
    //Toggle Flywheel LED each time button is pushed
    if(fire_but_level == 2){
        vTaskDelay(10 / portTICK_PERIOD_MS);
        led_state = !led_state;
        gpio_set_level(FW_LED, led_state);
        ESP_LOGI(TAG,"Flywheel Button Pressed");
    }
    //Turn on FIRE LED for 1 sec each time it is pressed 
    if (led_state == TRUE){
        if(fire_but_level == 1){
            vTaskDelay(10 / portTICK_PERIOD_MS);
            gpio_set_level(FIRE_LED, 1);
            ESP_LOGI(TAG,"Fire Button Pressed");
            vTaskDelay(1500 / portTICK_PERIOD_MS);
            gpio_set_level(FIRE_LED, 0);
        }
        else{
            gpio_set_level(FIRE_LED, 0);
        }
    }
}

/**************************************************
* Title:	init_gpio
* Summary:	Initialize GPIO pins to correct configuration
            of modes for input and output pins 
* Param:
* Return:
**************************************************/
static void init_gpio(void){
    //zero-initialize the config structure.
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

    //zero-initialize the config structure.
	gpio_config_t f_conf = {};
	//disable interrupt
	f_conf.intr_type = GPIO_INTR_DISABLE;
	//bit mask of the pins
	f_conf.pin_bit_mask = FIRE_PIN_SEL;
	//set as input mode
	f_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	f_conf.pull_up_en = 1;
    //interrupt on rising edge
    f_conf.intr_type = GPIO_INTR_NEGEDGE;	
	//configure input GPIO pins with the given settings
	gpio_config(&f_conf);

    //configure output GPIO pins (LEDs)
    //zero-initialize the config structure.
	gpio_config_t o_conf = {};
	//disable interrupt
	o_conf.intr_type = GPIO_INTR_DISABLE;
	//bit mask of the pins
	o_conf.pin_bit_mask = OUT_PIN_SEL;
	//set as output mode
	o_conf.mode = GPIO_MODE_OUTPUT;
	//enable pull-down mode
	o_conf.pull_down_en = 1;
	//configure output GPIO pins with the given settings
	gpio_config(&o_conf);
}

void app_main(void){
	// for some reason just having this makes it faster
	//!note I would prefer not to have it
    s_evt_group = xEventGroupCreate();
    assert(s_evt_group);

    //Initalize GPIO Pins to correct config modes 
    init_gpio();

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(RF_BUT,   gpio_isr_handler, (void*) RF_BUT   );
    gpio_isr_handler_add(RB_BUT,   gpio_isr_handler, (void*) RB_BUT   );
    gpio_isr_handler_add(LF_BUT,   gpio_isr_handler, (void*) LF_BUT   );
    gpio_isr_handler_add(LB_BUT,   gpio_isr_handler, (void*) LB_BUT   );
    gpio_isr_handler_add(FIRE_BUT, gpio_isr_handler, (void*) FIRE_BUT );
    gpio_isr_handler_add(FW_BUT,   gpio_isr_handler, (void*) FW_BUT   );

    //Initalize Status LEDs to off 
	gpio_set_level(FIRE_LED, 0);
	gpio_set_level(FW_LED, 0);

    init_espnow_master();
	ESP_LOGD(TAG, "remote esp initialization complete");

	button_queue = xQueueCreate(20, sizeof(button_event_t));
	xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
    
	while(1){
		send_espnow_data();
		vTaskDelay(750 / portTICK_PERIOD_MS);
	}
}