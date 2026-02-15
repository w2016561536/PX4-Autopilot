/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/actuator_outputs.h>
#include "motor.hpp"

using namespace time_literals;

class BrushedServo : public ModuleBase<BrushedServo>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	BrushedServo();
	~BrushedServo() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _adc_report_sub{this, ORB_ID(adc_report)};        // subscription that schedules WorkItemExample when updated
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};

	// Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamBool<px4::params::SERVO_DRV_EN>) _param_servo_drv_en,
		(ParamBool<px4::params::SERVO1_INV>) _param_servo1_inv,
		(ParamBool<px4::params::SERVO2_INV>) _param_servo2_inv,
		(ParamFloat<px4::params::SERVO1_OFFSET>) _param_servo1_offset,
		(ParamFloat<px4::params::SERVO2_OFFSET>) _param_servo2_offset,
		(ParamInt<px4::params::SERVO1_BIND>) _param_servo1_bind,
		(ParamInt<px4::params::SERVO2_BIND>) _param_servo2_bind,
		(ParamInt<px4::params::S1_VOLTAGE_MIN>) _param_s1_voltage_min,
		(ParamInt<px4::params::S1_VOLTAGE_MAX>) _param_s1_voltage_max,
		(ParamInt<px4::params::S2_VOLTAGE_MIN>) _param_s2_voltage_min,
		(ParamInt<px4::params::S2_VOLTAGE_MAX>) _param_s2_voltage_max,
		(ParamInt<px4::params::SERVO_MAX_OUTPUT>) _param_servo_max_output,
		(ParamFloat<px4::params::SERVO_KP>) _param_servo_kp,
		(ParamFloat<px4::params::SERVO_KI>) _param_servo_ki,
		(ParamFloat<px4::params::SERVO_KD>) _param_servo_kd
	)


	Motor motor1;
	Motor motor2;
};
