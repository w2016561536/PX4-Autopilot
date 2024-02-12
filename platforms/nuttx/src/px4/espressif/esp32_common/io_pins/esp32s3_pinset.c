#include <px4_platform_common/px4_config.h>
#include <systemlib/px4_macros.h>

#include <arch/board/board.h>
#include <arch/chip/chip.h>
#include <px4_arch/micro_hal.h>
#include <errno.h>
#include <stdio.h>
#include <syslog.h>

#define ESP32S3_NGPIOS       49 /* GPIO0-44 */

int px4_esp32s3_configgpio(uint32_t pinset)
{
	if ((pinset & GPIO_NUM_MASK) >= ESP32S3_NGPIOS) {
		return -EINVAL;
	}
	return esp32s3_configgpio((int)(pinset & GPIO_NUM_MASK),(uint16_t)(pinset >> GPIO_SET_SHIFT));
}

int px4_esp32s3_unconfiggpio(uint32_t pinset)
{
	return px4_esp32s3_configgpio((pinset & GPIO_NUM_MASK) | GPIO_INPUT | GPIO_OPEN_DRAIN);
}

/****************************************************************************
 * Name: esp32s3_gpiosetevent
 *
 * Description:
 *   Sets/clears GPIO based event and interrupt triggers.
 *
 * Input Parameters:
 *  - pinset:      GPIO pin configuration
 *  - risingedge:  Enables interrupt on rising edges
 *  - fallingedge: Enables interrupt on falling edges
 *  - event:       Generate event when set
 *  - func:        When non-NULL, generate interrupt
 *  - arg:         Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure indicating the
 *   nature of the failure.
 *
 ****************************************************************************/
int esp32s3_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge,
                       bool event, xcpt_t func, void *arg)
{
	uint32_t pin = pinset & GPIO_NUM_MASK;
	int irq = ESP32S3_PIN2IRQ(pin);

	if(event == true)
	{
		int ret = irq_attach(irq, func, arg);
		if (ret < 0)
		{
		syslog(LOG_ERR, "ERROR: irq_attach() failed: %d\n", ret);
		return ret;
		}

		if(risingedge == true && fallingedge == true)
			esp32s3_gpioirqenable(irq, CHANGE);
		else if(risingedge == true && fallingedge == false)
			esp32s3_gpioirqenable(irq, RISING);
		else if(risingedge == false && fallingedge == true)
			esp32s3_gpioirqenable(irq, FALLING);
	}
	else{
		esp32s3_gpioirqdisable(irq);
	}

	//syslog(LOG_INFO, "esp32s3_gpiosetevent: %d\n", ret);
  	return OK;
}
