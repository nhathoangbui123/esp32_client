#include "esp_log.h"

#include "espnow_basic_config.h"

static const char *TAG = "My_Master";

// Your task to handle received my_data_t
void my_data_receive(const uint8_t *sender_mac_addr, const my_data_t *data)
{
    ESP_LOGI(TAG, "Data from: LY - %u, RX - %u, TRIM1 - %u, TRIM2- %u, TRIM3 - %u", 
                data->LY, 
                data->RX,
                data->TRIM1,
                data->TRIM2,
                data->TRIM3);
}