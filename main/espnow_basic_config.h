#ifndef ESPNOW_BASIC_CONFIG_H
#define ESPNOW_BASIC_CONFIG_H

#include <inttypes.h>
#include <stdbool.h>

// Define the structure of your data
typedef struct __attribute__((packed)) {
    uint16_t LY;
    uint16_t RX;
    uint16_t TRIM1;
    uint16_t TRIM2;
    uint16_t TRIM3;
} my_data_t;

// Destination MAC address
// The default address is the broadcast address, which will work out of the box, but the slave will assume every tx succeeds.
// Setting to the master's address will allow the slave to determine if sending succeeded or failed.
//   note: with default config, the master's WiFi driver will log this for you. eg. I (721) wifi:mode : sta (12:34:56:78:9a:bc)
//c4:4f:33:75:b3:11
//0c:dc:7e:61:54:30
#define MY_RECEIVER_MAC {0x0C, 0xDC, 0x7E, 0x61, 0x54, 0x30}

#define MY_ESPNOW_PMK "pmk1234567890123"
#define MY_ESPNOW_CHANNEL 1

// #define MY_ESPNOW_ENABLE_LONG_RANGE 1

#define MY_SLAVE_DEEP_SLEEP_TIME_MS 10000

#endif // ESPNOW_BASIC_CONFIG_H