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

#include "IP5109.h"
#include <px4_platform_common/module.h>
#include <lib/parameters/param.h>
#include <drivers/drv_adc.h>
#include <cmath>

float low_voltage_warn;

IP5109::IP5109(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": ip5109_bat"))
{
}

IP5109::~IP5109()
{
	ScheduleClear();
	perf_free(_cycle_perf);
}

void IP5109::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();	// nothing to do
}

void IP5109::RunImpl()
{
	if (should_exit()) {
		PX4_INFO("stopping");
		return;	// stop and return immediately to avoid unexpected schedule from stopping procedure
	}

	perf_begin(_cycle_perf);

	float battery_voltage = getVoltage();

	if (std::isnan(battery_voltage)) {
		PX4_ERR("IP5109: failed to read battery voltage");
		perf_end(_cycle_perf);
		return;
	}

	float battery_current = getCurrent();
	if (std::isnan(battery_current)) {
		PX4_ERR("IP5109: failed to read battery current");
		perf_end(_cycle_perf);
		return;
	}


	_battery_report.timestamp = hrt_absolute_time();

	_battery_report.id = 1;

	_battery_report.connected = true;

	_battery_report.voltage_cell_v[0] = battery_voltage / 1000.0f;

	// Convert millivolts to volts.
	_battery_report.voltage_v = battery_voltage / 1000.0f;
	_battery_report.voltage_filtered_v = _battery_report.voltage_v;

	_battery_report.current_a = (battery_current/ 1000.0f);
	_battery_report.current_filtered_a = _battery_report.current_a;

	_battery_report.current_average_a = _battery_report.current_a;

	// Read run time to empty (minutes).
	_battery_report.time_remaining_s = _battery_report.voltage_v / 4.2f * 5;

	// Read average time to empty (minutes).
	_battery_report.average_time_to_empty = _battery_report.voltage_v / 4.2f * 5;

	// Calculate total discharged amount in mah.
	_battery_report.discharged_mah = 2000;

	// Normalize 0.0 to 1.0
	if (battery_voltage - 3200 > 1000) {
		_battery_report.remaining = 1;

	} else if (battery_voltage - 3200 < 0) {
		_battery_report.remaining = 0;

	} else{
	_battery_report.remaining = (battery_voltage - 3200) / (4200 - 3200);
	}

	// Read battery temperature and covert to Celsius.
	_battery_report.temperature = 30;

	// Only publish if no errors.

	_battery_report.capacity = 2000;
	_battery_report.cycle_count = 1;
	_battery_report.serial_number = 11451;
	_battery_report.max_cell_voltage_delta = 1.5;
	_battery_report.cell_count = 1;
	_battery_report.state_of_health = 100;

	// TODO: This critical setting should be set with BMS info or through a paramter
	// Setting a hard coded BATT_CELL_VOLTAGE_THRESHOLD_FAILED may not be appropriate
	//if (_lifetime_max_delta_cell_voltage > BATT_CELL_VOLTAGE_THRESHOLD_FAILED) {
	//	_battery_report.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	if (battery_voltage < low_voltage_warn * 1000.0f) {
		_battery_report.warning = battery_status_s::BATTERY_WARNING_LOW;
	}

	//_battery_report.interface_error = perf_event_count(_interface->_interface_errors);

	_to_battery_report.publish(_battery_report);


	perf_end(_cycle_perf);
}

void IP5109::print_usage()
{
	PRINT_MODULE_USAGE_NAME("ip5109", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(IP5109_ADDR);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void IP5109::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
}

extern "C" int ip5109_main(int argc, char *argv[])
{
	using ThisDriver = IP5109;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = IP5109_ADDR;

	param_get(param_find("LOW_VOLTAGE_WARN"), &low_voltage_warn);

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_BAT_DEVTYPE_BATMON_IP5109);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
