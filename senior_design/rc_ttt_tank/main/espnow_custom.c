#include "espnow_custom.h"

//extern static const char *TAG;
static const char *TAG = "tank_espnow_custom";

//static uint32_t fw_state = 0;
static uint32_t fire_servo = 0;
static uint32_t rotate_turret = 4;
static uint32_t rotation_angle[7] = {8,14,20,25,28,30,32}; //min = 8, max = 32

/**************************************************
* Title:	turret_task
* Summary:	Rotates the Turret Servo Motor. Based on Target Tracking Input Data
            from the IR Detectors based on if they are receiving IR light or not
            from the other tank's emitters. 
* Param:
* Return:
**************************************************/
void turret_task(void *pvParameter){

    ledc_timer_config_t timer_conf1;
    timer_conf1.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf1.timer_num = TURRET_PWM_TIMER;
	timer_conf1.duty_resolution = DUTY_RESOLUTION;
    timer_conf1.freq_hz = PWM_FREQUENCY;
    ledc_timer_config(&timer_conf1);

    ledc_channel_config_t ledc_conf1;
    ledc_conf1.gpio_num = TURRET_PIN;
    ledc_conf1.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_conf1.channel = TURRET_PWM_CHANNEL;
    ledc_conf1.intr_type = LEDC_INTR_DISABLE;
    ledc_conf1.timer_sel = TURRET_PWM_TIMER;
    ledc_conf1.duty = 256;
    ledc_conf1.hpoint = 0;
    ledc_channel_config(&ledc_conf1);

    while (1) {
        //Rotate the turret servo if IR sensors detect target tank in range 
        //ESP_LOGI(TAG, "Turret Servo Activated");
        for(int i = 0; i < 2; i++){
            //Rotate the servo forward (180 degrees)
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, TURRET_PWM_CHANNEL, rotation_angle[rotate_turret]);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, TURRET_PWM_CHANNEL);
            vTaskDelay(250 / portTICK_PERIOD_MS); // Wait for 0.25 seconds
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Release for 10 milliseconds
	}
}

/**************************************************
* Title:	firing_task
* Summary:	function activates the firing mechanism micro servo motor
            Which rotates forward (180 Deg) then back (0 Deg) and pushes 
            out the nerf dart using the 3D printed rack and pinion 
* Param:
* Return:
**************************************************/
void firing_task(void *pvParameter) {
    esp_rom_gpio_pad_select_gpio(FIRE_PIN);

    ledc_timer_config_t timer_conf0;
    timer_conf0.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf0.timer_num = SERVO_PWM_TIMER;
	timer_conf0.duty_resolution = DUTY_RESOLUTION;
    timer_conf0.freq_hz = PWM_FREQUENCY;
    ledc_timer_config(&timer_conf0);

    ledc_channel_config_t ledc_conf0;
    ledc_conf0.gpio_num = FIRE_PIN;
    ledc_conf0.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_conf0.channel = SERVO_PWM_CHANNEL;
    ledc_conf0.intr_type = LEDC_INTR_DISABLE;
    ledc_conf0.timer_sel = SERVO_PWM_TIMER;
    ledc_conf0.duty = 256;
    ledc_conf0.hpoint = 0;
    ledc_channel_config(&ledc_conf0);

    while (1) {
		//Activate the firing mechanism servo motor if the fire button is pressed
		if(fire_servo == 1){
			ESP_LOGI(TAG, "Firing Servo Activated");
			vTaskDelay(10 / portTICK_PERIOD_MS);
            for(int i = 0; i < 2; i++){
                // Rotate the servo forward (180 degrees)
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL, DUTY_MAX_FIRE);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL);
                vTaskDelay(500 / portTICK_PERIOD_MS); // Wait for 0.5 seconds

                // Rotate the servo back to the starting position (0 degrees)
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL, DUTY_MIN_FIRE);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO_PWM_CHANNEL);
                vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for 10 milliseconds
            }
            fire_servo = 0;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Release for 10 milliseconds
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
    //static uint32_t fire_servo = 0;	

    if(len != sizeof(my_data_t))
    {
        ESP_LOGE(TAG, "Unexpected data length: %d != %u", len, sizeof(my_data_t));
        return;
    }
    //move the tank, activate flywheels or fire turret accordingly
	my_data_t *packet = data; //!note this line generates a warning. it works fine though
							  //because we checked the length above

    //ESP_LOGI(TAG, "message_type: %d", packet->message_type);
    //ESP_LOGI(TAG, "rf: %d", packet->rf);
    //ESP_LOGI(TAG, "rb: %d", packet->rb);
    //ESP_LOGI(TAG, "lf: %d", packet->lf);
    //ESP_LOGI(TAG, "lb: %d", packet->lb);
    //ESP_LOGI(TAG, "fire_turret: %d", packet->fire_turret);
    //ESP_LOGI(TAG, "activate_fw: %d", packet->activate_fw);

	if(packet->message_type != TANK_COMMAND){
		ESP_LOGE(TAG, "wrong message_type received from remote");
        //Flywheel Button Toggling - Toggle FW LED
        gpio_set_level(FW_NMOS, 0);
        gpio_set_level(FIRE_PIN, 0);
        //If no message was received turn off the driving pins 
        gpio_set_level(RF_IN2_PIN, 0);
        gpio_set_level(RB_IN1_PIN, 0);
        gpio_set_level(LF_IN3_PIN, 0);
        gpio_set_level(LB_IN4_PIN, 0);
	} 
    else if(packet->message_type == TANK_COMMAND){
        //Driving Buttons Activation
        gpio_set_level(RF_IN2_PIN, packet->rf);
        gpio_set_level(RB_IN1_PIN, packet->rb);
        gpio_set_level(LF_IN3_PIN, packet->lf);
        gpio_set_level(LB_IN4_PIN, packet->lb);

        gpio_set_level(FW_NMOS, packet->fw_led);         //Flywheel Button Toggling - Toggle FW LED
        gpio_set_level(IR_EMITS_NMOS, packet->fw_led);   //Turn IR Emitters on/off
        gpio_set_level(IR_S_NMOS, packet->fw_led);       //Turn IR Detectors on/off
        gpio_set_level(FIRE_PIN, packet->fire_turret);   //Activate Firing Mechanism

        //Fire Button Monitoring
        //Activate Firing Servo and LED if Flywheels are active
        if(packet->fw_led == 1){
            if(packet->fire_turret == 1){
                //fire_servo = 1;
                if (rotate_turret >= 7){
                    rotate_turret = 0;
                }
                else {
                    rotate_turret++;
                }
                vTaskDelay(5 / portTICK_PERIOD_MS); // Release for 5 milliseconds
            }
            else {
                //gpio_set_level(IR_EMITS_NMOS,0);  //Turn IR Emitters off
                //gpio_set_level(IR_S_NMOS,0);      //Turn IR Detectors off
                //fire_servo = 0;
            }
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
	data.fw_active = !(gpio_get_level(FW_NMOS));
	data.firing_led = !(gpio_get_level(FIRE_PIN));

    // Convert MAC address to string
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
             destination_mac[0], destination_mac[1], destination_mac[2],
             destination_mac[3], destination_mac[4], destination_mac[5]);

    // Log the MAC address we are sending data to
    ESP_LOGI(TAG, "Send Tank data to MAC: %s", mac_str);*/

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
