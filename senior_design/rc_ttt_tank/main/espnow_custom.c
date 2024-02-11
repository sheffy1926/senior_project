#include "espnow_custom.h"

//extern static const char *TAG;
static const char *TAG = "tank_espnow_custom";

static uint32_t fw_state = 0;
static uint32_t fire_servo = 0;

void firing_task(void *pvParameter) {
    esp_rom_gpio_pad_select_gpio(FIRE_SERVO_PIN);
    gpio_set_direction(FIRE_SERVO_PIN, GPIO_MODE_OUTPUT);

    ledc_timer_config_t timer_conf;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num = SERVO_PWM_TIMER;
	timer_conf.duty_resolution = DUTY_RESOLUTION;
    timer_conf.freq_hz = PWM_FREQUENCY;
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ledc_conf;
    ledc_conf.gpio_num = FIRE_SERVO_PIN;
    ledc_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_conf.channel = SERVO_PWM_CHANNEL;
    ledc_conf.intr_type = LEDC_INTR_DISABLE;
    ledc_conf.timer_sel = SERVO_PWM_TIMER;
    ledc_conf.duty = 4;
    ledc_conf.hpoint = 0;
    ledc_channel_config(&ledc_conf);

    while (1) {
        // Wait until the GPIO pin controlling the servo motor is pulled low
		if(fire_servo == 1){
			ESP_LOGI(TAG, "Firing Servo Activated");
			vTaskDelay(10 / portTICK_PERIOD_MS);
            for(int i = 0; i < 2; i++){
                // Rotate the servo forward (180 degrees)
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL, DUTY_MAX);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL);
                vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for 1 second

                // Rotate the servo back to the starting position (0 degrees)
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL, DUTY_MIN);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL);
                vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for 10 milliseconds
            }
            gpio_set_level(FIRE_PIN, 0);
            fire_servo = 0;
		}
    }
}

/**************************************************
* Title:	recv_cb
* Summary:	call back function called when esp_now messages are received
*			interprets data received in data packet from remote
			moves the tank according to that data, activates flywheels,
            or fires turret
* Param:
* Return:
**************************************************/
void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len){
    if(len != sizeof(my_data_t))
    {
        ESP_LOGE(TAG, "Unexpected data length: %d != %u", len, sizeof(my_data_t));
        return;
    }

    //move the tank, activate flywheels or fire turret accordingly
	my_data_t *packet = data; //!note this line generates a warning. it works fine though
							  //because we checked the length above

    // Convert MAC address to string
    /*char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);

    ESP_LOGI(TAG, "Receiving Remote data from MAC: %s", mac_str);*/
    ESP_LOGI(TAG, "message_type: %d", packet->message_type);
    //ESP_LOGI(TAG, "rf: %d", packet->rf);
    //ESP_LOGI(TAG, "rb: %d", packet->rb);
    //ESP_LOGI(TAG, "lf: %d", packet->lf);
    //ESP_LOGI(TAG, "lb: %d", packet->lb);
    ESP_LOGI(TAG, "fire_turret: %d", packet->fire_turret);
    ESP_LOGI(TAG, "activate_fw: %d", packet->activate_fw);

	if(packet->message_type != TANK_COMMAND){
		ESP_LOGE(TAG, "wrong message_type received from remote");
	} 
    else if(packet->message_type == TANK_COMMAND){
		/*gpio_set_level(L_IN1_PIN, packet->rf);
		gpio_set_level(L_IN2_PIN, packet->rb);
		gpio_set_level(R_IN3_PIN, packet->lf);
		gpio_set_level(R_IN4_PIN, packet->lb);*/

        //Right Forward turn LED to White
        if (packet->rf == 1){
            gpio_set_level(R_LED_R, ON);
            gpio_set_level(R_LED_G, ON);
            gpio_set_level(R_LED_B, ON);
        }
        else if (packet->rf == 0){
            gpio_set_level(R_LED_R, OFF);
            gpio_set_level(R_LED_G, OFF);
            gpio_set_level(R_LED_B, OFF);
        }

        //Right Back turn LED to Red
        if (packet->rb == 1){
            gpio_set_level(R_LED_R, ON);
            gpio_set_level(R_LED_G, OFF);
            gpio_set_level(R_LED_B, OFF);
        }
        else if (packet->rb == 0){
            gpio_set_level(R_LED_R, OFF);
        }

        //Left Forward turn LED to White
        if (packet->lf == 1){
            gpio_set_level(L_LED_R, ON);
            gpio_set_level(L_LED_G, ON);
            gpio_set_level(L_LED_B, ON);
        }
        else if (packet->lf == 0){
            gpio_set_level(L_LED_R, OFF);
            gpio_set_level(L_LED_G, OFF);
            gpio_set_level(L_LED_B, OFF);
        }

        //Left Back turn LED to Red
        if (packet->lb == 1){
            gpio_set_level(L_LED_R, ON);
            gpio_set_level(L_LED_G, OFF);
            gpio_set_level(L_LED_B, OFF);
        }
        else if (packet->lb == 0){
            gpio_set_level(L_LED_R, OFF);
        }

        //Activate Firing Servo and LED if Flywheels are active
        if(fw_state == 1){
            if(packet->fire_turret == 1){
                gpio_set_level(FIRE_PIN, 1);
                fire_servo = 1;
            }
        }

        //Toggle FW LED
        if (packet->activate_fw == 1){
            fw_state = ! fw_state;
            gpio_set_level(FW_PIN, fw_state);
        }
	}
	return;
}

/**************************************************
* Title: send_espnow_data
* Summary: sends an esp_now message to the remote
* Param:
* Return:
**************************************************/
esp_err_t send_espnow_data(void){

    const uint8_t destination_mac[] = REMOTE_MAC;
    static my_data_t data;

    //populate data
	/*data.message_type = FIRE_COMMAND;
	data.fw_active = !(gpio_get_level(FW_PIN));
	data.turret_firing = !(gpio_get_level(FIRE_PIN));*/

    // Convert MAC address to string
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
             destination_mac[0], destination_mac[1], destination_mac[2],
             destination_mac[3], destination_mac[4], destination_mac[5]);

    // Log the MAC address we are sending data to
    ESP_LOGI(TAG, "Send Tank data to MAC: %s", mac_str);

    // Send it
    esp_err_t err = esp_now_send(destination_mac, (uint8_t*)&data, sizeof(data));
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "Send error (%d)", err);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sent!");
    return ESP_OK;
}

/**************************************************
* Title:	packet_sent_cb
* Summary:	function that send the status of the espnow message 
            if it was a success or a failure
* Param:
* Return:
**************************************************/
void packet_sent_cb(const uint8_t *mac_addr, esp_now_send_status_t status){
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }
    assert(status == ESP_NOW_SEND_SUCCESS || status == ESP_NOW_SEND_FAIL);
    xEventGroupSetBits(s_evt_group, BIT(status));
}

/**************************************************
* Title: init_espnow_slave
* Summary: initializes the wireless messaging of the tank
* Param:
* Return:
**************************************************/
void init_espnow_slave(void){
    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_ERROR_CHECK( esp_netif_init());
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
    ESP_LOGD(TAG,"Attempting to register recv_cb for tank");

    ESP_ERROR_CHECK( esp_now_register_send_cb(packet_sent_cb) );
    ESP_LOGD(TAG,"Attempting to register send_espnow_data for tank");
    ESP_ERROR_CHECK( esp_now_set_pmk((const uint8_t *)MY_ESPNOW_PMK));

    const esp_now_peer_info_t broadcast_destination = {
        .peer_addr = REMOTE_MAC,
        .channel = MY_ESPNOW_CHANNEL,
        .ifidx = MY_ESPNOW_WIFI_IF
    };
    ESP_ERROR_CHECK( esp_now_add_peer(&broadcast_destination) );
}
