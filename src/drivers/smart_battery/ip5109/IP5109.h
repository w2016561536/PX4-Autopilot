/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include <stdint.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/battery_status.h>
#include <uORB/Publication.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#define IP5109_ADDR 0x58

#define	SYS_CTL0 0x01
#define SYS_CTL0_CLR 0xE1
#define ENABLE_FLASH_LIGHT 16
#define LIGHT_ENABLE 8
#define BOOST_ENABLE 4
#define CHARGE_ENABLE 2

#define SYS_CTL1 0x02
#define SYS_CTL1_CLR 0xFC
#define ENABLE_LOW_LOAD_AUTO_OFF 2
#define ENABLE_AUTO_ON_LOAD_DETECT 1

#define SYS_CTL2 0x0c
#define SYS_CTL2_CLR 0x3
#define LOW_LOAD_THRESHOLD(x) (((uint8_t)x / 12) << 3) //unit: mA

#define SYS_CTL3 0X03
#define SYS_CTL3_CLR 0x1F
#define LONG_PRESSED_TIME(x) (((uint8_t)(x - 1)) << 6) //x in range of 1 to 4
#define ENABLE_DOUBLE_SHORT_CLICK 32

#define SYS_CTL5 0x07
#define SYS_CL5_CLR 0xBC
#define NTC_ENABLE 64

#define CHARGER_CTL1 0x22
#define CHARGER_CTL1_CLR 0xF3
#define UNDER_VOLTAGE_THRESHOLD_4v53 0
#define UNDER_VOLTAGE_THRESHOLD_4v63 4
#define UNDER_VOLTAGE_THRESHOLD_4v73 8
#define UNDER_VOLTAGE_THRESHOLD_4v83 12

#define CHARGER_CTL2 0x24
#define CHARGER_CLT2_CLR 0x99
#define BATTERY_4V2 0
#define BATTERY_4V3 32
#define BATTERY_4V35 64
#define BATTERY_OVERVOLTAGE_0MV 0
#define BATTERY_OVERVOLTAGE_14MV 2
#define BATTERY_OVERVOLTAGE_28MV 4
#define BATTERY_OVERVOLTAGE_42MV 6

#define CHG_DIG_CTL4 0x26
#define CHG_DIG_CTL4_CLR 0xBF
#define ENABLE_EXTERNEL_VSETPIN 64

#define CHG_DIG_CTL5 0x25
#define CHG_DIG_CTL5_CLR 0xE0
#define CHARGE_CURRENT_SET(x)  ((uint8_t)((float)x * 10))

#define BATVADC_DAT0 0xa2
#define BATVADC_DAT1 0xA3

#define BATIADC_DAT0 0xa4
#define BATIADC_DAT1 0xa5


using namespace time_literals;

/*
 * This driver configure IP5109 into 4 channels with gnd as baseline.
 * Start each sample cycle by setting sample channel.
 * PGA set to 6.144V
 * SPS set to 256
 * Valid output ranges from 0 to 32767 on each channel.
 */
class IP5109 : public device::I2C, public I2CSPIDriver<IP5109>
{
public:
	IP5109(const I2CSPIDriverConfig &config);
	~IP5109() override;

	int init() override;

	static void print_usage();

	void RunImpl();

	int probe() override;

protected:

	void print_status() override;

	void exit_and_cleanup() override;

private:

	uORB::Publication<battery_status_s>		_to_battery_report{ORB_ID(battery_status)};

	static const hrt_abstime	SAMPLE_INTERVAL{1000_ms};

	battery_status_s _battery_report{};

	perf_counter_t			_cycle_perf;

	int     _channel_cycle_count{0};

	bool    _reported_ready_last_cycle{false};

	//float low_voltage_warn;

	int readReg(uint8_t addr, uint8_t *buf, size_t len);

	int writeReg(uint8_t addr, uint8_t *buf, size_t len);

	int modifyReg(uint8_t addr, uint8_t set_flag, uint8_t clear_flag);

	int setReg(uint8_t addr, uint8_t reset_flag ,uint8_t set_flag);

	float getVoltage();

	float getCurrent();

	int checkReg(uint8_t addr, uint8_t reset_flag, uint8_t set_flag, uint8_t clear_flag);
};
