#include "tcp_task.h"

void tcp_client_task(void *pvParameters);

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(&tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
}
