#include "espnow_custom.h"

//extern static const char *TAG;
static const char *TAG = "tank_espnow_custom";

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

	if((packet->message_type != TANK_COMMAND) && (packet->message_type != FIRE_COMMAND)){
		ESP_LOGE(TAG, "wrong message_type received from remote");
	} 
    else if(packet->message_type == TANK_COMMAND){
		/*gpio_set_level(RF_PIN, packet->rf);
		gpio_set_level(RB_PIN, packet->rb);
		gpio_set_level(LF_PIN, packet->lf);
		gpio_set_level(LB_PIN, packet->lb);*/

        //Right Forward turn LED to White
        gpio_set_level(R_LED_R, packet->rf);
		gpio_set_level(R_LED_G, packet->rf);
		gpio_set_level(R_LED_B, packet->rf);

        //Right Back turn LED to Red
        gpio_set_level(R_LED_R, packet->rb);
		gpio_set_level(R_LED_G, OFF);
		gpio_set_level(R_LED_B, OFF);

        //Left Forward turn LED to White
		gpio_set_level(L_LED_R, packet->lf);
        gpio_set_level(L_LED_G, packet->lf);
        gpio_set_level(L_LED_B, packet->lf);

        //Left Back turn LED to Red
        gpio_set_level(L_LED_R, packet->lb);
		gpio_set_level(L_LED_G, OFF);
		gpio_set_level(L_LED_B, OFF);

        ESP_LOGI(TAG, "Remote Driving Data Received!");
    }
    else if (packet->message_type == FIRE_COMMAND){
		gpio_set_level(FIRE_PIN, packet->fire_turret);
		gpio_set_level(FW_PIN, packet->activate_fw);

        ESP_LOGI(TAG, "Remote Firing Data Received!");
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
