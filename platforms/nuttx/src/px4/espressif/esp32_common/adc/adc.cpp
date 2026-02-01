/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <board_config.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/param.h>
#include <debug.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <px4_arch/adc.h>
#include <px4_platform_common/log.h>
#include <nuttx/analog/adc.h>
#include <nuttx/spinlock.h>
#include <syslog.h>
#include <nuttx/semaphore.h>
#include <nuttx/irq.h>
#include <errno.h>


#include "xtensa.h"
#include "hardware/esp32_efuse.h"
#include "hardware/esp32_sens.h"
#include "hardware/esp32_gpio_sigmap.h"
#include "hardware/esp32_rtc_io.h"

struct adc_dev_s *adcdev;

#define CONFIG_ESPRESSIF_ADC_1

#ifdef CONFIG_ESPRESSIF_ADC_1
#define ADC_1_MAX_CHANNELS 8
#define ADC_2_MAX_CHANNELS 10
static const uint8_t g_chanlist_adc1[ADC_1_MAX_CHANNELS] =
{
#ifdef CONFIG_ESPRESSIF_ADC_1_CH0
  1,
#endif
#ifdef CONFIG_ESPRESSIF_ADC_1_CH1
  2,
#endif
#ifdef CONFIG_ESPRESSIF_ADC_1_CH2
  3,
#endif
#ifdef CONFIG_ESPRESSIF_ADC_1_CH3
  4,
#endif
#ifdef CONFIG_ESPRESSIF_ADC_1_CH4
  5,
#endif
#ifdef CONFIG_ESPRESSIF_ADC_1_CH5
  6,
#endif
#ifdef CONFIG_ESPRESSIF_ADC_1_CH6
  7,
#endif
#ifdef CONFIG_ESPRESSIF_ADC_1_CH7
  8,
#endif
};
#endif

extern "C"{
uint32_t esp_adc_oneshot_read_now(struct adc_dev_s *dev, uint8_t channel_id);
struct adc_dev_s *esp_adc_initialize(int adc_num,
                                     const uint8_t *channel_list);
}

int px4_arch_adc_init(uint32_t base_address) {


  adcdev = (adc_dev_s*)kmm_malloc(sizeof(struct adc_dev_s));
  if (adcdev == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to allocate adc_dev_s instance\n");
      return PX4_ERROR;
    }

  memset(adcdev, 0, sizeof(struct adc_dev_s));

  adcdev = esp_adc_initialize(1, g_chanlist_adc1);

	syslog(LOG_INFO, "INFO: ESP32 ADC drvier ready\n");

	if (adcdev == NULL)
    	{
    	  syslog(LOG_ERR, "ERROR: Failed to initialize ADC %d\n", 1);
    	  return PX4_ERROR;
    	}

	syslog(LOG_INFO, "INFO: ESP32S3 ADC INIT, BOARD ADC START\n");
	return PX4_OK ;

}


void px4_arch_adc_uninit(uint32_t base_address) {
	// nothing to do
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel) {

	if (adcdev == NULL) {
		return UINT32_MAX;
	}
	if( channel == ADC_BATTERY_CURRENT_CHANNEL){
		return 3000; // not supported
	}
        if (channel != ADC_BATTERY_VOLTAGE_CHANNEL) {
		return UINT32_MAX;
	}
	return esp_adc_oneshot_read_now(adcdev, channel);

}

float px4_arch_adc_reference_v() {
	return 3.1f; // TODO: provide true vref
}

uint32_t px4_arch_adc_temp_sensor_mask() {
	return 0;
}

uint32_t px4_arch_adc_dn_fullcount() {
	return 1 << 12; // 12 bit ADC
}


