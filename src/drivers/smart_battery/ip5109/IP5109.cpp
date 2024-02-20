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

#include "IP5109.h"
#include <cassert>

int IP5109::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}

	// set charge current to 0.5A
	setReg(CHG_DIG_CTL5, CHG_DIG_CTL5_CLR, CHARGE_CURRENT_SET(0.5));

	// Enable boost and charge, disable light
	modifyReg(SYS_CTL0, BOOST_ENABLE | CHARGE_ENABLE, LIGHT_ENABLE);

	// Disable auto low load off and enable auto on load detect
	modifyReg(SYS_CTL1, ENABLE_AUTO_ON_LOAD_DETECT, ENABLE_LOW_LOAD_AUTO_OFF);

	// Set low load threshold to 100mA
	setReg(SYS_CTL2, SYS_CTL2_CLR, LOW_LOAD_THRESHOLD(100));

	// disable double click off
	modifyReg(SYS_CTL3, 0, ENABLE_DOUBLE_SHORT_CLICK);

	// disable ntc control
	modifyReg(SYS_CTL5, 0, NTC_ENABLE);

	setReg(CHARGER_CTL2, CHARGER_CLT2_CLR, BATTERY_4V2 | BATTERY_OVERVOLTAGE_28MV);

	ret |= checkReg(CHG_DIG_CTL5, CHG_DIG_CTL5_CLR, CHARGE_CURRENT_SET(0.5),0);
	ret |= checkReg(SYS_CTL0, SYS_CTL0_CLR, BOOST_ENABLE | CHARGE_ENABLE, LIGHT_ENABLE);
	ret |= checkReg(SYS_CTL1, SYS_CTL1_CLR, ENABLE_AUTO_ON_LOAD_DETECT, ENABLE_LOW_LOAD_AUTO_OFF);
	ret |= checkReg(SYS_CTL2, SYS_CTL2_CLR, LOW_LOAD_THRESHOLD(100), 0);
	ret |= checkReg(SYS_CTL3, SYS_CTL3_CLR, 0, ENABLE_DOUBLE_SHORT_CLICK);
	ret |= checkReg(SYS_CTL5, SYS_CL5_CLR, 0, NTC_ENABLE);
	ret |= checkReg(CHARGER_CTL2, CHARGER_CLT2_CLR, BATTERY_4V2 | BATTERY_OVERVOLTAGE_28MV, 0);

	if (ret == PX4_OK) ScheduleOnInterval(SAMPLE_INTERVAL , SAMPLE_INTERVAL );
	else PX4_ERR("IP5109 init failed (%i)", ret);

	return ret;
}

int IP5109::probe()
{
	uint8_t buf;
	int ret = readReg(SYS_CTL1, &buf, 1);

	if (ret != PX4_OK) {
		PX4_ERR("readReg failed (%i)", ret);
		return ret;
	}

	if (!(buf & SYS_CTL1_CLR & (BOOST_ENABLE | CHARGE_ENABLE))) {
		PX4_ERR("IP5109 not found");
		return PX4_ERROR;
	}

	return PX4_OK;
}


int IP5109::readReg(uint8_t addr, uint8_t *buf, size_t len)
{
	return transfer(&addr, 1, buf, len);
}

int IP5109::writeReg(uint8_t addr, uint8_t *buf, size_t len)
{
	uint8_t buffer[len + 1];
	buffer[0] = addr;
	memcpy(buffer + 1, buf, sizeof(uint8_t)*len);
	return transfer(buffer, len + 1, nullptr, 0);
}

int IP5109::modifyReg(uint8_t addr, uint8_t set_flag, uint8_t clear_flag){
	uint8_t origin_data;
	readReg(addr, &origin_data, 1);
	origin_data |= set_flag;
	origin_data &= ~clear_flag;
	return writeReg(addr, &origin_data, 1);
}

int IP5109::setReg(uint8_t addr, uint8_t reset_flag ,uint8_t set_flag){
	uint8_t origin_data;
	readReg(addr, &origin_data, 1);
	origin_data &= reset_flag;
	origin_data |= set_flag;
	return writeReg(addr, &origin_data, 1);
}

int IP5109::checkReg(uint8_t addr, uint8_t reset_flag, uint8_t set_flag, uint8_t clear_flag){
	uint8_t origin_data;
	readReg(addr, &origin_data, 1);
	if((origin_data & reset_flag) == (set_flag & (~clear_flag)))
	{
		return PX4_OK;
	}
	PX4_ERR("IP5109 Register Check FAIL: addr: 0x%02x , read: 0x%02x , should: 0x%02x",
		addr, origin_data & reset_flag, set_flag & (~clear_flag));
	return PX4_ERROR;
}

float IP5109::getVoltage()
{
	uint8_t BATVADC_VALUE_high;
	uint8_t BATVADC_VALUE_low;
	int ret = PX4_OK;
	ret |= readReg(BATVADC_DAT1, &BATVADC_VALUE_high, 1);
	ret |= readReg(BATVADC_DAT0, &BATVADC_VALUE_low, 1);
	if(ret != PX4_OK)
	{
		PX4_ERR("IP5109: read voltage failed (%i)", ret);
		return std::sqrt(-1.0f);
	}
	float BATVOL;
 	if((BATVADC_VALUE_high & 0x20)==0x20)//补码
	{
		BATVOL = 2600-((~BATVADC_VALUE_low)+(~(BATVADC_VALUE_high & 0x1F))*256+1)*0.26855;
 	}
	else //原码
	{
		BATVOL = 2600+(BATVADC_VALUE_low+BATVADC_VALUE_high*256)*0.26855; //mv 为单位
	}
	return BATVOL;
}

float IP5109::getCurrent()
{
	uint8_t BATIADC_VALUE_high;
	uint8_t BATIADC_VALUE_low;
	int ret = PX4_OK;
	ret |= readReg(BATIADC_DAT1, &BATIADC_VALUE_high, 1);
	ret |= readReg(BATIADC_DAT0, &BATIADC_VALUE_low, 1);
	if(ret != PX4_OK)
	{
		PX4_ERR("IP5109: read current failed (%i)", ret);
		return std::sqrt(-1.0f);
	}
	float BATCUR;
 	if((BATIADC_VALUE_high & 0x20)==0x20)//负值
 	{
 		char a=~BATIADC_VALUE_low;
 		char b=(~(BATIADC_VALUE_high & 0x1F) & 0x1f);
 		int c=b*256+a+1;
		BATCUR=-c*0.745985;
 		//BATCUR=-(int)(((~BATIADC_VALUE_low)+(~(BATIADC_VALUE_high & 0x1F))*256+1)*0.745985);
	}
	else //正值
 	{
 		BATCUR= (BATIADC_VALUE_high*256+BATIADC_VALUE_low)*0.745985; //mA 为单位
 	}
	return BATCUR;
}
