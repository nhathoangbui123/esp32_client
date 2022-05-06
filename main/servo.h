#ifndef _SERVO_CONTROL_H_
#define _SERVO_CONTROL_H_

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/ledc.h>


double getDutyByPercentage(double percentage);
double getDutyByuS(double uS);

void attach(gpio_num_t pin, unsigned int minuS, unsigned int maxuS, ledc_channel_t ledcChannel, ledc_timer_t ledcTimer);
void writeMicroSeconds(unsigned int uS);
void write_val(unsigned int value);
void detach();
#endif