/****************************************************************************
 *
 *   Copyright (C) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file _main.cpp
 * @author w2016561536
 *
 * Driver for the IP5109 connected via I2C.
 */

#include <px4_platform_common/module.h>
#include <lib/parameters/param.h>
#include <cmath>

#include "esp32s3_gpio.h"
#include "hardware/esp32s3_gpio_sigmap.h"

int inv_uart1 = 0;
int inv_uart2 = 0;

extern "C" int esp_inv_ctrl_main(int argc, char *argv[])
{

	param_get(param_find("UART1_RX_INV"), &inv_uart1);
	param_get(param_find("UART2_RX_INV"), &inv_uart2);

	if (inv_uart1)
	esp32s3_gpio_matrix_in(CONFIG_ESP32S3_UART1_RXPIN, U1RXD_IN_IDX, 1);

	if (inv_uart2)
	esp32s3_gpio_matrix_in(CONFIG_ESP32S3_UART2_RXPIN, U2RXD_IN_IDX, 1);

	printf("ENABLE_INVERTER_FOR_UART1_RX: %d\n", inv_uart1);
	printf("ENABLE_INVERTER_FOR_UART2_RX: %d\n", inv_uart2);
	return 0;
}
