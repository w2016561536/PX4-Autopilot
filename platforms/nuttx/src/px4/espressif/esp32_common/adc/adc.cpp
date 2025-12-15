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
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_efuse.h"
#include "hardware/esp32s3_sens.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "hardware/regi2c_ctrl.h"
#include "hardware/regi2c_saradc.h"
#include "hardware/esp32s3_rtc_io.h"



/* ADC calibration max count */

#define ADC_CAL_CNT_MAX         (32)

/* ADC calibration max value */

#define ADC_CAL_VAL_MAX         (4096 - 1)

/* ADC calibration sampling channel */

#define ADC_CAL_CHANNEL         (0xf)

/* ADC max value mask */

#define ADC_VAL_MASK            (0xfff)

#define ADC_CAL_BASE_REG        EFUSE_RD_SYS_PART1_DATA0_REG

#define ADC_CAL_VER_OFF         (128)
#define ADC_CAL_VER_LEN         (2)

#define ADC_CAL_DATA_COMP       (1550)

#define ADC_CAL_VOL_LEN         (8)

/* ADC input voltage attenuation, this affects measuring range */

#define ADC_ATTEN_DB_0          (0)     /* Vmax = 950 mV  */
#define ADC_ATTEN_DB_2_5        (1)     /* Vmax = 1250 mV */
#define ADC_ATTEN_DB_6          (2)     /* Vmax = 1750 mV */
#define ADC_ATTEN_DB_12         (3)     /* Vmax = 3100 mV */

/* ADC attenuation */

#if defined(CONFIG_ESP32S3_ADC_VOL_950)
#  define ADC_ATTEN_DEF         ADC_ATTEN_DB_0
#  define ADC_CAL_DATA_LEN        (8)

#  define ADC_CAL_DATA_OFF      (149)
#  define ADC_CAL_VOL_OFF       (201)

#  define ADC_CAL_VOL_DEF       (488)
#elif defined(CONFIG_ESP32S3_ADC_VOL_1250)
#  define ADC_ATTEN_DEF         ADC_ATTEN_DB_2_5
#  define ADC_CAL_DATA_LEN        (6)

#  define ADC_CAL_DATA_OFF      (157)
#  define ADC_CAL_VOL_OFF       (209)

#  define ADC_CAL_VOL_DEF       (641)
#elif defined(CONFIG_ESP32S3_ADC_VOL_1750)
#  define ADC_ATTEN_DEF         ADC_ATTEN_DB_6
#  define ADC_CAL_DATA_LEN        (6)

#  define ADC_CAL_DATA_OFF      (163)
#  define ADC_CAL_VOL_OFF       (217)

#  define ADC_CAL_VOL_DEF       (892)
#elif defined(CONFIG_ESP32S3_ADC_VOL_3100)
#  define ADC_ATTEN_DEF         ADC_ATTEN_DB_12
#  define ADC_CAL_DATA_LEN        (6)

#  define ADC_CAL_DATA_OFF      (169)
#  define ADC_CAL_VOL_OFF       (225)

#  define ADC_CAL_VOL_DEF       (1592)
#endif

#define setbits(bs, a)     modifyreg32(a, 0, bs)
#define resetbits(bs, a)   modifyreg32(a, bs, 0)


struct adc_chan_s
{
  uint32_t ref;           /* Reference count */

  const uint8_t channel;  /* Channel number */
  const uint8_t pin;      /* GPIO pin number */

  const struct adc_callback_s *cb;  /* Upper driver callback */
};


static spinlock_t g_modifyreg_lock = SP_UNLOCKED;
void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits)
{
  irqstate_t flags;
  uint32_t   regval;

  flags   = spin_lock_irqsave(&g_modifyreg_lock);
  regval  = getreg32(addr);
  regval &= ~clearbits;
  regval |= setbits;
  putreg32(regval, addr);
  spin_unlock_irqrestore(&g_modifyreg_lock, flags);
}

#include "esp32s3_adc.h"

#define ADC_1_MAX_CHANNELS 8
struct adc_dev_s *adcdev;

static inline void adc_samplecfg(int channel)
{
  uint32_t regval;

  /* set (Frequency division) (inversion adc) */

  regval = getreg32(SENS_SAR_READER1_CTRL_REG);
  regval &= ~(SENS_SAR1_CLK_DIV_M);
  regval |= (1 << SENS_SAR1_CLK_DIV_S);
  putreg32(regval, SENS_SAR_READER1_CTRL_REG);

  /* Enable ADC1, its sampling attenuation */

  regval = getreg32(SENS_SAR_ATTEN1_REG);
  regval &= ~(ADC_ATTEN_DEF << (channel * 2));
  regval |= ADC_ATTEN_DEF << (channel * 2);
  putreg32(regval, SENS_SAR_ATTEN1_REG);

  /* Enable ADC1, its sampling channel and attenuation */

  regval  = getreg32(SENS_SAR_MEAS1_CTRL2_REG);
  regval &= ~(SENS_SAR1_EN_PAD_M | SENS_SAR1_EN_PAD_FORCE_M |
              SENS_MEAS1_START_FORCE_M);
  regval |= ((1 << channel) << SENS_SAR1_EN_PAD_S) |
              SENS_SAR1_EN_PAD_FORCE | SENS_MEAS1_START_FORCE;
  putreg32(regval, SENS_SAR_MEAS1_CTRL2_REG);
}


static uint32_t adc_read_work(struct adc_dev_s *dev)
{
  // int ret;
  //int32_t adc;
  struct adc_chan_s *priv = (struct adc_chan_s *)dev->ad_priv;

  irqstate_t flags = px4_enter_critical_section();

  adc_samplecfg(priv->channel);
  uint32_t regval;

  /* Trigger ADC1 sampling */

  setbits(SENS_MEAS1_START_SAR, SENS_SAR_MEAS1_CTRL2_REG);

  /* Wait until ADC1 sampling is done */
	const hrt_abstime now = hrt_absolute_time();
  do
    {
      regval = getreg32(SENS_SAR_MEAS1_CTRL2_REG);
	if ((hrt_absolute_time() - now) > 50) {
			px4_leave_critical_section(flags);
			return UINT32_MAX;
		}
    }
  while (!(regval & SENS_MEAS1_DONE_SAR_M));

  regval = getreg32(SENS_SAR_MEAS1_CTRL2_REG) & ADC_VAL_MASK;
  /* Disable ADC sampling */

  resetbits(SENS_MEAS1_START_SAR, SENS_SAR_MEAS1_CTRL2_REG);

//   adc = (int32_t)(regval * (UINT16_MAX * ADC_CAL_VOL_DEF / 2098) /
//                   UINT16_MAX);

//    PX4_INFO("channel: %" PRIu8 ", voltage: %" PRIu32 " mV\n", priv->channel,
// 	   (uint32_t)adc);

  px4_leave_critical_section(flags);
  return regval;
}


int px4_arch_adc_init(uint32_t base_address) {

	adcdev = (adc_dev_s *)kmm_malloc(sizeof(struct adc_dev_s));
  	if (adcdev == NULL)
  	  {
  	    syslog(LOG_ERR, "ERROR: Failed to allocate adc_dev_s instance\n");
  	    return PX4_ERROR;
  	  }

	esp32s3_adc_init(ADC_BATTERY_VOLTAGE_CHANNEL,adcdev);

	syslog(LOG_INFO, "INFO: ESP32S3 ADC drvier ready\n");

	if (adcdev == NULL)
    	{
    	  syslog(LOG_ERR, "ERROR: Failed to initialize ADC %d\n", 1);
    	  return PX4_ERROR;
    	}



	/* 提供给驱动注册的回调表 */
	// static const struct adc_callback_s g_adc_cb = {
	//   .au_receive = esp_adc_receive,
	// };
	// if (adcdev->ad_ops->ao_bind(adcdev, &g_adc_cb) != OK) {
	// 	syslog(LOG_ERR, "ERROR: Failed to bind ADC callbacks\n");
	// 	return PX4_ERROR;
	// }

	if (adcdev->ad_ops->ao_setup(adcdev) != OK) {
		syslog(LOG_ERR, "ERROR: Failed to setup ADC %d\n", 1);
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
	return adc_read_work(adcdev);

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


