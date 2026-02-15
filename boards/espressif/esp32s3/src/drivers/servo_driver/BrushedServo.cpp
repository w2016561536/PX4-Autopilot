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

#include "BrushedServo.hpp"

BrushedServo::BrushedServo() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

BrushedServo::~BrushedServo()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool BrushedServo::init()
{
	parameter_update_s pupdate;
	_parameter_update_sub.copy(&pupdate);

	updateParams();

	// check if SERVO_DRV_EN is set, if not, warn and exit
	if (_param_servo_drv_en.get() == false) {
		PX4_INFO("SERVO_DRV_EN is not set, exiting");
		return false;
	}

	// initialize ledc controller for pwm output
	up_pwm_servo_init(0xF0); // use last 4 channels of pwm

	// set pwm rate to 20khz
	up_pwm_servo_set_rate_group_update(2, 20000); // set group 0 to 20kHz

	// start output at 0
	for (int i = 0; i < 4; ++i) {
		up_pwm_servo_set(4 + i, 0); // set all channels to neutral
	}

	// update channels
	up_pwm_update(0xF0);

	// initialize motor object
	motor1.SetTorqueLimit((float)_param_servo_max_output.get() / 100.0f);
	motor1.mechanicalAngleMin = 500;
	motor1.mechanicalAngleMax = 2500;
	motor1.adcValAtAngleMin = _param_s1_voltage_min.get();
	motor1.adcValAtAngleMax = _param_s1_voltage_max.get();
	motor1.dce.kp = _param_servo_kp.get();
	motor1.dce.ki = _param_servo_ki.get();
	motor1.dce.kv = 0;
	motor1.dce.kd = _param_servo_kd.get();
	motor1.dce.setPointPos = 1500 + (float)_param_servo1_offset.get() / 100.0f * 2000.0f; // neutral position

	// initialize motor object
	motor2.SetTorqueLimit((float)_param_servo_max_output.get() / 100.0f);
	motor2.mechanicalAngleMin = 500;
	motor2.mechanicalAngleMax = 2500;
	motor2.adcValAtAngleMin = _param_s2_voltage_min.get();
	motor2.adcValAtAngleMax = _param_s2_voltage_max.get();
	motor2.dce.kp = _param_servo_kp.get();
	motor2.dce.ki = _param_servo_ki.get();
	motor2.dce.kv = 0;
	motor2.dce.kd = _param_servo_kd.get();
	motor2.dce.setPointPos = 1500 + (float)_param_servo2_offset.get() / 100.0f * 2000.0f; // neutral position

	// execute Run() on every adc_report publication
	if (!_adc_report_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// alternatively, Run on fixed interval
	// ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

	return true;
}

void BrushedServo::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)

		motor1.SetTorqueLimit((float)_param_servo_max_output.get() / 100.0f);
		motor1.adcValAtAngleMin = _param_s1_voltage_min.get();
		motor1.adcValAtAngleMax = _param_s1_voltage_max.get();
		motor1.dce.kp = _param_servo_kp.get();
		motor1.dce.ki = _param_servo_ki.get();
		motor1.dce.kd = _param_servo_kd.get();

		motor2.SetTorqueLimit((float)_param_servo_max_output.get() / 100.0f);
		motor2.adcValAtAngleMin = _param_s2_voltage_min.get();
		motor2.adcValAtAngleMax = _param_s2_voltage_max.get();
		motor2.dce.kp = _param_servo_kp.get();
		motor2.dce.ki = _param_servo_ki.get();
		motor2.dce.kd = _param_servo_kd.get();
	}

	// Example
	//  grab latest adc data
	if (_adc_report_sub.updated()) {
		adc_report_s adc_report;

		if (_adc_report_sub.copy(&adc_report)) {
			// DO WORK
			int servo_1_adc_value = 0;
			int servo_2_adc_value = 0;

			for (int i = 0; i < 12; i++) {
				// findout two servo feedback voltage from adc report
				if (adc_report.channel_id[i] == ADC_SERVO_1_CHANNEL) {
					servo_1_adc_value = adc_report.raw_data[i];

				} else if (adc_report.channel_id[i] == ADC_SERVO_2_CHANNEL) {
					servo_2_adc_value = adc_report.raw_data[i];
				}
			}

			// get actuator output and update motor setpoint
			actuator_outputs_s actuator_outputs;

			_actuator_outputs_sub.updated(); // check if there is new actuator output, if not, use the last one

			if (_actuator_outputs_sub.copy(&actuator_outputs)) {
				float servo_1_output = actuator_outputs.output[_param_servo1_bind.get()];
				float servo_2_output = actuator_outputs.output[_param_servo2_bind.get()];
				motor1.dce.setPointPos = servo_1_output + (float)_param_servo1_offset.get() / 100.0f * 2000.0f;
				motor2.dce.setPointPos = servo_2_output + (float)_param_servo2_offset.get() / 100.0f * 2000.0f;
			}

			// update motor state with adc feedback
			motor1.angle = motor1.mechanicalAngleMin +
				       (motor1.mechanicalAngleMax - motor1.mechanicalAngleMin) *
				       ((float) servo_1_adc_value - (float) motor1.adcValAtAngleMin) /
				       ((float) motor1.adcValAtAngleMax - (float) motor1.adcValAtAngleMin);

			// Calculate PID
			motor1.CalcDceOutput(motor1.angle, 0);

			motor2.angle = motor2.mechanicalAngleMin +
				       (motor2.mechanicalAngleMax - motor2.mechanicalAngleMin) *
				       ((float) servo_2_adc_value - (float) motor2.adcValAtAngleMin) /
				       ((float) motor2.adcValAtAngleMax - (float) motor2.adcValAtAngleMin);

			// Calculate PID
			motor2.CalcDceOutput(motor2.angle, 0);

			// set pwm output with pid output
			if ((motor1.dce.output >= 0) ^ (_param_servo1_inv.get() == false)) {
				up_pwm_servo_set(4, motor1.dce.output > 2500 ? 2500 : motor1.dce.output); // motor 1_A forward
				up_pwm_servo_set(5, 0); // motor 1_B

			} else {
				up_pwm_servo_set(4, 0); // motor 1_A
				up_pwm_servo_set(5, -motor1.dce.output > 2500 ? 2500 : -motor1.dce.output); // motor 1_B reverse
			}

			if ((motor2.dce.output >= 0) ^ (_param_servo2_inv.get() == false)) {
				up_pwm_servo_set(6, motor2.dce.output > 2500 ? 2500 : motor2.dce.output); // motor 2_A forward
				up_pwm_servo_set(7, 0); // motor 2_B

			} else {
				up_pwm_servo_set(6, 0); // motor 2_A
				up_pwm_servo_set(7, -motor2.dce.output > 2500 ? 2500 : -motor2.dce.output); // motor 2_B reverse
			}

			// update pwm output
			up_pwm_update(0xF0);

		}
	}

	perf_end(_loop_perf);
}

int BrushedServo::task_spawn(int argc, char *argv[])
{
	BrushedServo *instance = new BrushedServo();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int BrushedServo::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int BrushedServo::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BrushedServo::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is a servo driver for brushed motors.
It has 2 channels.
Channel 1 feedback is on marco ADC_SERVO_1_CHANNEL,
and channel 2 feedback is on marco ADC_SERVO_2_CHANNEL.
It will take last 4 channels of ledc for pwm output, and the channel assignment is as follows:
- ledc channel 4: motor 1_A output
- ledc channel 5: motor 1_B output
- ledc channel 6: motor 2_A output
- ledc channel 7: motor 2_B output
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("brushed_servo", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int brushed_servo_main(int argc, char *argv[])
{
	return BrushedServo::main(argc, argv);
}
