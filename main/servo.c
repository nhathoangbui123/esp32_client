#include "servo.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/ledc.h>

const int _freqHz = 50;

ledc_channel_t _ledcChannel;
unsigned int _min;
unsigned int _max;
ledc_timer_bit_t _timerResolution;

double getDutyByPercentage(double percentage){
	if (percentage <= 0){
		return 0;
	}

	if (percentage > 100){
		percentage = 100;
	}
	return (percentage / 100.0) * ((2<<(_timerResolution-1))-1);
}

double getDutyByuS(double uS){
	return getDutyByPercentage(((uS * 100.0)/(1000000/_freqHz)));
}

void attach(gpio_num_t pin, unsigned int minuS, unsigned int maxuS, ledc_channel_t ledcChannel, ledc_timer_t ledcTimer){
	_min = minuS;
	_max = maxuS;
	_ledcChannel = ledcChannel;
	_timerResolution = LEDC_TIMER_16_BIT;

	ledc_timer_config_t timer_conf;
	timer_conf.duty_resolution 	= _timerResolution;
	timer_conf.freq_hz    		= _freqHz;
	timer_conf.speed_mode 		= LEDC_HIGH_SPEED_MODE;
	timer_conf.timer_num  		= ledcTimer;
	timer_conf.clk_cfg          = LEDC_AUTO_CLK;
	ledc_timer_config(&timer_conf);

	ledc_channel_config_t ledc_conf;
	ledc_conf.channel		= _ledcChannel;
	ledc_conf.duty			= 0;
	ledc_conf.gpio_num		= (int)pin;
	ledc_conf.intr_type		= LEDC_INTR_DISABLE;
	ledc_conf.speed_mode	= LEDC_HIGH_SPEED_MODE;
	ledc_conf.timer_sel		= ledcTimer;
	ledc_conf.hpoint        = 0;
	ledc_channel_config(&ledc_conf);
}

void detach(){
	ledc_stop(LEDC_HIGH_SPEED_MODE, _ledcChannel, 0);
}

void writeMicroSeconds(unsigned int uS){
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, _ledcChannel, getDutyByuS(uS));
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, _ledcChannel);
}

void write_val(unsigned int value) {
	// 0 = MinServoAngle ; 180 = Max ServoAngle;
	int scale = (value - 0) * (_max - _min) / (180 - 0) + _min;
	writeMicroSeconds(scale);
}