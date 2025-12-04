/****************************************************************************
 *
 *   Copyright (C) 2012, 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file drv_pwm_servo.c
 *
 * Servo driver supporting PWM servos connected to STM32 timer blocks.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that
 * have output pins, does not require an interrupt.
 */

#include <px4_platform_common/px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <nuttx/queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>

#include <px4_arch/io_timer.h>

#include "esp32_ledc.h"

typedef uint16_t	servo_position_t;

struct pwm_info_s pwm_info0;
struct pwm_lowerhalf_s *pwm0;

#if defined(CONFIG_ESP32_LEDC_TIM1)
struct pwm_lowerhalf_s *pwm1;
struct pwm_info_s pwm_info1;
#endif

int up_pwm_servo_set(unsigned channel, servo_position_t value)
{
	//syslog(LOG_INFO, "[ESP PWM] PWM set ch: %d value:%d\n", channel,value);
	if (channel < CONFIG_ESP32_LEDC_TIM0_CHANNELS) {
		if (pwm_info0.frequency == 15000 || pwm_info0.frequency > 400)
		{
			pwm_info0.channels[channel].duty = (value*400)/(1000000/65535);
			return OK;
		}
		pwm_info0.channels[channel].duty = (value*pwm_info0.frequency)/(1000000/65535);
		return OK;

	}
#if defined(CONFIG_ESP32_LEDC_TIM1)
	else if (channel < (CONFIG_ESP32_LEDC_TIM0_CHANNELS + CONFIG_ESP32_LEDC_TIM1_CHANNELS)) {
		channel -= CONFIG_ESP32_LEDC_TIM0_CHANNELS; // adjust channel number for second timer
		if (pwm_info1.frequency == 15000 || pwm_info1.frequency > 400)
		{
			pwm_info1.channels[channel].duty = (value*400)/(1000000/65535);
			return OK;
		}
		pwm_info1.channels[channel].duty = (value*pwm_info1.frequency)/(1000000/65535);
		return OK;
	}
#endif
return PX4_ERROR;

}

servo_position_t up_pwm_servo_get(unsigned channel)
{

	//syslog(LOG_INFO, "[ESP PWM]PWM get ch duty: %d\n", channel);
	if (channel < CONFIG_ESP32_LEDC_TIM0_CHANNELS) {
		return pwm_info0.channels[channel].duty;
	}
#if defined(CONFIG_ESP32_LEDC_TIM1)
	else if (channel < (CONFIG_ESP32_LEDC_TIM0_CHANNELS + CONFIG_ESP32_LEDC_TIM1_CHANNELS)) {
		channel -= CONFIG_ESP32_LEDC_TIM0_CHANNELS; // adjust channel number for second timer
		return pwm_info1.channels[channel].duty;
	}
#endif

return PX4_ERROR;
}

int up_pwm_servo_init(uint32_t channel_mask)
{
	syslog(LOG_INFO, "[ESP PWM] init channel_mask: %02lX\n", channel_mask);

	// int ret = 0;
	if (channel_mask & io_timer_get_group(0)) {
  	pwm0 = esp32_ledc_init(0);
  	if (!pwm0)
    	{
      		syslog(LOG_ERR, "[ESP PWM][boot] Failed to get the LEDC PWM 0 lower half\n");
    	}


	pwm0->ops->setup(pwm0);

	pwm_info0.frequency=400;
	for (int i = 0; i < CONFIG_ESP32_LEDC_TIM0_CHANNELS; i++) {
		pwm_info0.channels[i].duty=0;
	}

	pwm0->ops->start(pwm0,&pwm_info0);

	syslog(LOG_INFO, "[ESP PWM] SYSPWM INIT OK, group 0 , channel mask: %02lX\n", channel_mask);


	//return channel_mask;
}

#ifdef CONFIG_ESP32_LEDC_TIM1
	if (channel_mask & io_timer_get_group(1)) {
		pwm1 = esp32_ledc_init(1);
		if (!pwm1)
		{
			syslog(LOG_ERR, "[[ESP PWM]] Failed to get the LEDC PWM 1 lower half\n");
			return -ENODEV;
		}

		pwm1->ops->setup(pwm1);

		pwm_info1.frequency=400;
		for (int i = 0; i < CONFIG_ESP32_LEDC_TIM1_CHANNELS; i++) {
		pwm_info1.channels[i].duty=0;
	}

		pwm1->ops->start(pwm1,&pwm_info1);

		syslog(LOG_INFO, "[ESP PWM] SYSPWM INIT OK, group 1 , channel mask: %02lX\n", channel_mask);
		//return channel_mask;
	}
#endif

return channel_mask;
}


void up_pwm_servo_deinit(uint32_t channel_mask)
{
	/* disable the timers */
	up_pwm_servo_arm(false, channel_mask);
}

int up_pwm_servo_set_rate_group_update(unsigned group, unsigned rate)
{
	syslog(LOG_INFO, "[ESP PWM] group update group: %d rate:%d\n", group,rate);

	if(group == 0)
	{
		if (rate == 0){
			pwm_info0.frequency = 15000;
			return OK;
		}
		pwm_info0.frequency = rate;
		return OK;
	}
	#ifdef CONFIG_ESP32_LEDC_TIM1
	else if (group == 1)
	{
		if (rate == 0){
			pwm_info1.frequency = 15000;
			return OK;
		}
		pwm_info1.frequency = rate;
		return OK;
	}
	#endif
	return ERROR;
}

void up_pwm_update(unsigned channels_mask)
{
	//syslog(LOG_INFO, "[ESP PWM] up_pwm_update channels_mask: %02X\n", channels_mask);

	if (channels_mask & io_timer_get_group(0)) {
		pwm0->ops->start(pwm0,&pwm_info0);
	}
#if defined(CONFIG_ESP32_LEDC_TIM1)
	if (channels_mask & io_timer_get_group(1)) {
		pwm1->ops->start(pwm1,&pwm_info1);
	}
#endif

}

uint32_t up_pwm_servo_get_rate_group(unsigned group)
{
	syslog(LOG_INFO, "[ESP PWM] up_pwm_servo_get_rate_group: %d\n", group);
	if(group == 0)
		return  io_timer_get_group(0);
	if (group == 1)
		return io_timer_get_group(1);
	return 0;
}

void
up_pwm_servo_arm(bool armed, uint32_t channel_mask)
{
	syslog(LOG_INFO, "[ESP PWM] up_pwm_servo_arm armed:%d channel_mask:%02lX\n", armed,channel_mask);

	if (channel_mask & io_timer_get_group(0)) {
		if (armed) {
			pwm0->ops->start(pwm0,&pwm_info0);
		} else {
			pwm0->ops->stop(pwm0);
		}
	}
#ifdef CONFIG_ESP32_LEDC_TIM1
	if (channel_mask & io_timer_get_group(1)) {
		if (armed) {
			pwm1->ops->start(pwm1,&pwm_info1);
		} else {
			pwm1->ops->stop(pwm1);
		}
	}
#endif
}
