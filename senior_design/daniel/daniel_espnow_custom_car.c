#include "esp_now_custom.h"
#include "driver/rmt.h"


//extern static const char *TAG;

static const char *TAG = "esp_custom";
extern rmt_item32_t items [8];

typedef struct {
    uint8_t sender_mac_addr[ESP_NOW_ETH_ALEN];
    my_data_t data;
} recv_packet_t;


/**************************************************
* Title:	recv_cb
* Summary:	call back function called when esp_now messages are recieved
*			interprets data received in data packet from remote
			moves the tank according to that data, activates flywheels,
            or fires turret
* Param:
* Return:
**************************************************/
void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    
    if(len != sizeof(my_data_t))
    {
        ESP_LOGE(TAG, "Unexpected data length: %d != %u", len, sizeof(my_data_t));
        return;
    }

	//move the tank accordingly
	my_data_t *packet = data; //!note this line generates a warning. it works fine though
								//because we checked the length above

	if(packet->message_type != TANK_COMMAND){
		ESP_LOGE(TAG, "wrong message_type recieved");
	} else{
		gpio_set_level(RF_PIN, packet->rf);
		gpio_set_level(RB_PIN, packet->rb);
		gpio_set_level(LF_PIN, packet->lf);
		gpio_set_level(LB_PIN, packet->lb);
		/*if(packet->shoot_laser){
			ESP_LOGI(TAG, "shooting laser");
			//fire the laser
			rmt_write_items(RMT_TX_CHANNEL, items, 8, true);
			
		}*/
	}

    //ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
	return;
}

/**************************************************
* Title: send_espnow_data
* Summary: sends an esp_now message to the score_board
* Param:
* Return:
**************************************************/
esp_err_t send_espnow_data(my_data_t data)
{
    const uint8_t destination_mac[] = REMOTE_MAC;

    // Send it
    esp_err_t err = esp_now_send(destination_mac, (uint8_t*)&data, sizeof(data));
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "Send error (%x)", err);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sent!");
    return ESP_OK;
}

/**************************************************
* Title: init_espnow_master
* Summary: initializes the wireless messaging of the tank
* Param:
* Return:
**************************************************/
void init_espnow_master(void)
{
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
#if MY_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(MY_ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(recv_cb) );
    ESP_ERROR_CHECK( esp_now_set_pmk((const uint8_t *)MY_ESPNOW_PMK) );

    const esp_now_peer_info_t broadcast_destination = {
        .peer_addr = REMOTE_MAC,
        .channel = MY_ESPNOW_CHANNEL,
        .ifidx = MY_ESPNOW_WIFI_IF
    };
    ESP_ERROR_CHECK( esp_now_add_peer(&broadcast_destination) );
}
