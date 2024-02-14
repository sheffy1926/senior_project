#include "espnow_custom.h"

//extern static const char *TAG;
static const char *TAG = "tank_espnow_custom";

static uint32_t fw_state = 0;

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
    static uint32_t fire_servo = 0;	
    static uint32_t right_drive = 0;
	static uint32_t left_drive = 0;

    if(len != sizeof(my_data_t))
    {
        ESP_LOGE(TAG, "Unexpected data length: %d != %u", len, sizeof(my_data_t));
        return;
    }

    //move the tank, activate flywheels or fire turret accordingly
	my_data_t *packet = data; //!note this line generates a warning. it works fine though
							  //because we checked the length above

    ESP_LOGI(TAG, "message_type: %d", packet->message_type);
    ESP_LOGI(TAG, "rf: %d", packet->rf);
    ESP_LOGI(TAG, "rb: %d", packet->rb);
    ESP_LOGI(TAG, "lf: %d", packet->lf);
    ESP_LOGI(TAG, "lb: %d", packet->lb);
    //ESP_LOGI(TAG, "fire_turret: %d", packet->fire_turret);
    //ESP_LOGI(TAG, "activate_fw: %d", packet->activate_fw);

	if(packet->message_type != TANK_COMMAND){
		ESP_LOGE(TAG, "wrong message_type received from remote");
	} 
    else if(packet->message_type == TANK_COMMAND){
        /********************************************************************/
        //Right and Left Motor Button Press Monitoring 
        //If both right buttons are not pressed/pressed turn motor off
        if ((packet->rf == 0) && (packet->rb == 0)){
            right_drive = 0;
            xQueueSendToBack(r_motor_queue,right_drive,25);
        } 
        else if ((packet->rf == 1) && (packet->rb == 1)){
            right_drive = 0;
            xQueueSendToBack(r_motor_queue,right_drive,25);
        } 
        //Right Forward Button Pressed 
        else if (packet->rf == 1){
            right_drive = 1;
            xQueueSendToBack(r_motor_queue,right_drive,25);
        }
        //Right Back Button Pressed
        else if (packet->rb == 1){
            right_drive = 2;
            xQueueSendToBack(r_motor_queue,right_drive,25);
        }

        //If both left buttons are not pressed/pressed turn motor off
        if ((packet->lf == 0) && (packet->lb == 0)){
            left_drive = 0;
            xQueueSendToBack(l_motor_queue,left_drive,25);
        } 
        else if ((packet->lf == 1) && (packet->lb == 1)){
            left_drive = 0;
            xQueueSendToBack(l_motor_queue,left_drive,25);
        }
        //Left Forward Button Pressed 
        else if (packet->lf == 1){
            left_drive = 1;
            xQueueSendToBack(l_motor_queue,left_drive,25);
        }
        //Left Back Button Pressed
        else if (packet->lb == 1){
            left_drive = 2;
            xQueueSendToBack(l_motor_queue,left_drive,25);
        }
        /********************************************************************/
        //Fire Button Monitoring
        //Activate Firing Servo and LED if Flywheels are active
        if(fw_state == 1){
            if(packet->fire_turret == 1){
                fire_servo = 1;
                xQueueSendToBack(firing_queue,fire_servo,25);
            }
            else{
                fire_servo = 0;
                xQueueSendToBack(firing_queue,fire_servo,25);
            }
        }
        /********************************************************************/
        //Flywheel Button Monitoring
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
