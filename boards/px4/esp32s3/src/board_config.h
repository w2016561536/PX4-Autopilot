/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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

/**
 * @file board_config.h
 *
 * PX4FMUv4 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */

#define GPIO_LED_RED                 (GPIO_OUTPUT|11)
#define GPIO_LED_GREEN               (GPIO_OUTPUT|12)
#define GPIO_LED_BLUE                (GPIO_OUTPUT|5)
// #define GPIO_LED_SAFETY              GPIO_LED_BLUE

#define BOARD_HAS_CONTROL_STATUS_LEDS 1
// #define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_LED        LED_GREEN
#define BOARD_ARMED_STATE_LED  LED_RED


//#define GPIO_SENSORS_3V3_EN           (GPIO_OUTPUT | 12)

#define HRT_TIMER                    0  /* use timer 3 for the HRT */

#define BOARD_SPI_BUS_MAX_BUS_ITEMS 2
//#define PRINTF_LOG


/**
 * ADC channels:
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver.
 */
#define ADC_5V_RAIL_SENSE_IO		36
#define ADC_BATTERY_VOLTAGE_CHANNEL_IO	39
#define ADC_BATTERY_CURRENT_CHANNEL_IO	34

#define ADC_5V_RAIL_SENSE            	0
#define ADC_BATTERY_VOLTAGE_CHANNEL    	3
#define ADC_BATTERY_CURRENT_CHANNEL    	6

#define ADC_CHANNELS \
(	(1 << ADC_5V_RAIL_SENSE) 		|\
 	(1 << ADC_BATTERY_VOLTAGE_CHANNEL) 	|\
  	(1 << ADC_BATTERY_CURRENT_CHANNEL))

#define ADC_V5_V_FULL_SCALE (7.17f)

//#define RC_SERIAL_PORT		"/dev/ttyS2"

#define GPIO_HEATER_OUTPUT   /* PA8 */ (GPIO_OUTPUT| 46)
#define HEATER_OUTPUT_EN(on_true)      px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))

/* AUX PWMs
 */
#define DIRECT_PWM_OUTPUT_CHANNELS	4

#define BOARD_ENABLE_CONSOLE_BUFFER

/* Power supply control and monitoring GPIOs. */
//#define GPIO_VDD_BRICK_VALID         (GPIO_INPUT|GPIO_PULLUP|32)
//#define GPIO_VDD_USB_VALID           (GPIO_INPUT|GPIO_PULLUP|35)


#define BOARD_ADC_USB_CONNECTED      1//(px4_arch_gpioread(GPIO_VDD_USB_VALID))
#define BOARD_ADC_BRICK_VALID        1//(px4_arch_gpioread(GPIO_VDD_BRICK_VALID))
#define BOARD_ADC_USB_VALID          1//(px4_arch_gpioread(GPIO_VDD_USB_VALID))



__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void esp32s3_spiinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>


#endif /* __ASSEMBLY__ */

__END_DECLS
