#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "sdkconfig.h"

#include "servo.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "espnow_basic_config.h"

#define SSID                "NhatHoang"
#define PASS                "01217818548"

#define MY_ESPNOW_WIFI_MODE WIFI_MODE_STA
#define MY_ESPNOW_WIFI_IF   ESP_IF_WIFI_STA

#define GPIO_LEFT               4
#define GPIO_RIGHT              5  
#define GPIO_PWM_OUT            32 
#define GPIO_PWM_SERVO          GPIO_NUM_18 
static const char *TAG =        "ESP32-RF-CAR";

static xQueueHandle s_recv_queue;

uint16_t LY=0;
uint16_t RX=0;
uint16_t TRIM1=0;
uint16_t TRIM2=0;
uint16_t TRIM3=0;

uint16_t convT1=0;
uint16_t convT2=0;
uint16_t convT3=0;

uint8_t dir=0,turn=0;

float speed=0;
uint32_t servo=0;
typedef struct {
    uint8_t sender_mac_addr[ESP_NOW_ETH_ALEN];
    my_data_t data;
} recv_packet_t;

void my_data_receive(const uint8_t *sender_mac_addr, const my_data_t *data);

void servo_task(void *pvParameters);
void motor_task(void *pvParameters);
void logic_motor_task(void *pvParameters);
void logic_servo_task(void *pvParameters);
void servo_task(void *pvParameters){

	attach(GPIO_PWM_SERVO,400,2600,LEDC_CHANNEL_0,LEDC_TIMER_0);
    write_val(0);	
    vTaskDelay(500/ portTICK_RATE_MS);
	while (1){
			write_val(100);
			vTaskDelay(10 / portTICK_RATE_MS);
	}
    ESP_LOGI("Servo Task", "Stopping task ...");
    vTaskDelay(10/ portTICK_PERIOD_MS); 
    vTaskDelete(0);
}
static void mcpwm_gpio_initialize()
{
    //PWM wave
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM_OUT);
    gpio_set_level(GPIO_PWM_OUT, 0);
        
    gpio_pad_select_gpio(GPIO_LEFT);
    gpio_set_direction(GPIO_LEFT, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_LEFT, 0);
    
    gpio_pad_select_gpio(GPIO_RIGHT);
    gpio_set_direction(GPIO_RIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_RIGHT, 0);
}
/* Forward */
static void motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);    
    gpio_set_level(GPIO_LEFT, 0);
    gpio_set_level(GPIO_RIGHT, 1);    
}
/* Backward */
static void motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);    
    gpio_set_level(GPIO_LEFT, 1);
    gpio_set_level(GPIO_RIGHT, 0); 
}
/* Stop */
static void motor_stop (mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle)
{
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    gpio_set_level(GPIO_LEFT, 0);
    gpio_set_level(GPIO_RIGHT, 0); 
}

void motor_task(void *pvParameters){
    mcpwm_gpio_initialize();
    ESP_LOGI("Motor Task", "Config PWM....");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    while (1) {
        switch (dir)
        {
            case 1:
                ESP_LOGI("Motor Task", "motor_forward");
                motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, speed);
                
                break;
            case 2:
                ESP_LOGI("Motor Task", "motor_backward");
                motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, speed);
                break;
            default:
                ESP_LOGI("Motor Task", "motor_stop");
                motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0, speed);
                break;
        }
        vTaskDelay(10/ portTICK_PERIOD_MS);

    }
    ESP_LOGI("Motor Task", "Stopping task ...");
    vTaskDelay(10/ portTICK_PERIOD_MS); 
    vTaskDelete(0);
}
static void queue_process_task(void *p)
{
    static recv_packet_t recv_packet;

    ESP_LOGI(TAG, "Listening");
    for(;;)
    {
        if(xQueueReceive(s_recv_queue, &recv_packet, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        // Refer to user function
        my_data_receive(recv_packet.sender_mac_addr, &recv_packet.data);
        LY=recv_packet.data.LY;
        RX=recv_packet.data.RX;
        TRIM1=recv_packet.data.TRIM1;
        TRIM2=recv_packet.data.TRIM2;
        TRIM3=recv_packet.data.TRIM3;
        ESP_LOGI(TAG, "Data: LY - %u, RX - %u, TRIM1 - %u, TRIM2- %u, TRIM3 - %u", 
            LY, 
            RX,
            TRIM1,
            TRIM2,
            TRIM3);
    }
}

static void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    static recv_packet_t recv_packet;

    ESP_LOGI(TAG, "%d bytes incoming from " MACSTR, len, MAC2STR(mac_addr));
    
    if(len != sizeof(my_data_t))
    {
        ESP_LOGE(TAG, "Unexpected data length: %d != %u", len, sizeof(my_data_t));
        return;
    }

    memcpy(&recv_packet.sender_mac_addr, mac_addr, sizeof(recv_packet.sender_mac_addr));
    memcpy(&recv_packet.data, data, len);
    if (xQueueSend(s_recv_queue, &recv_packet, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Queue full, discarded");
        return;
    }
}

static void init_espnow_master(void)
{
    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    
    ESP_ERROR_CHECK( esp_netif_init());
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = SSID,
            .password =PASS,
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(MY_ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );

    ESP_LOGI("Wifi", "wifi_init_sta finished.");
    ESP_ERROR_CHECK( esp_wifi_connect());
    ESP_LOGI("Wifi", "esp_wifi_connect finished.");
#if MY_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(MY_ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(recv_cb) );
    ESP_ERROR_CHECK( esp_now_set_pmk((const uint8_t *)MY_ESPNOW_PMK) );
}
void logic_motor_task(void *pvParameters)
{
    while (1){
        convT1=(float)(TRIM1*100)/4095;
        speed=convT1;
        ESP_LOGI("logic_motor_task", "speed motor set = %0.2f",speed);

        if(RX>2100){
            dir=1;
            ESP_LOGI("logic_motor_task", "motor_forward");
            vTaskDelay(10/ portTICK_PERIOD_MS);
            continue;
        }
        else if(RX<1700){
            dir=2;
            ESP_LOGI("logic_motor_task", "motor_backward");
            vTaskDelay(10/ portTICK_PERIOD_MS);
            continue;
        }
        else{
            dir=0;
            ESP_LOGI("logic_motor_task", "motor_stop");
            vTaskDelay(10/ portTICK_PERIOD_MS);
            continue;
        }
        vTaskDelay(10/ portTICK_PERIOD_MS); 
    } 
}
void logic_servo_task(void *pvParameters)
{
    while (1){
        convT2=(float)(TRIM2*100)/4095;
        convT3=(float)(TRIM3*100)/4095;

        servo=(uint32_t)convT2;
        ESP_LOGI("logic_servo_task", "servo = %u",servo);

        if(LY>2100){
            turn=1;
            ESP_LOGI("logic_servo_task", "turn left");
            vTaskDelay(10/ portTICK_PERIOD_MS);
            continue;
        }
        else if(LY<1700){
            turn=2;
            ESP_LOGI("logic_servo_task", "turn right");
            vTaskDelay(10/ portTICK_PERIOD_MS);
            continue;
        }
        else{
            turn=0;
            ESP_LOGI("logic_servo_task", "middle");
            vTaskDelay(10/ portTICK_PERIOD_MS);
            continue;
        }

        vTaskDelay(10/ portTICK_PERIOD_MS); 
    } 
}
void app_main(void)
{
    ESP_LOGI("app_main", "nvs_flash_init");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI("app_main", "init_espnow_master");
    init_espnow_master();

    ESP_LOGI("app_main", "xQueueCreate");
    s_recv_queue = xQueueCreate(10, sizeof(recv_packet_t));
    assert(s_recv_queue);

    ESP_LOGI("app_main", "queue_process_task");
    BaseType_t err = xTaskCreate(queue_process_task, "recv_task", 8192, NULL, 4, NULL);
    assert(err == pdPASS);
    
    ESP_LOGI("app_main", "motor_task");
    err = xTaskCreate(&motor_task, "motor_task", 4096, NULL, 2, NULL);
    assert(err == pdPASS);

    ESP_LOGI("app_main", "servo_task");
    err = xTaskCreate(&servo_task, "servo_task", 4096, NULL, 2, NULL);
    assert(err == pdPASS);

    ESP_LOGI("app_main", "logic_motor_task");
    err = xTaskCreate(&logic_motor_task, "logic_motor_task", 2048, NULL, 3, NULL);
    assert(err == pdPASS);

    ESP_LOGI("app_main", "logic_servo_task");
    err = xTaskCreate(&logic_servo_task, "logic_servo_task", 2048, NULL, 3, NULL);
    assert(err == pdPASS);
}