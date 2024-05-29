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
#include <lib/parameters/param.h>

float charge_current = 0.5f;
int enable_boost = 0;

int IP5109::init()
{
	int ret = I2C::init();


	if (ret != PX4_OK) {
		return ret;
	}

	ret = PX4_OK;
	ret |= probe();

	if (ret != PX4_OK) {
		PX4_ERR("IP5109 probe failed (%i)", ret);
		return ret;
	}

	param_get(param_find("BAT_CHG_CURRENT"), &charge_current);
	param_get(param_find("ENABLE_BOOST"), &enable_boost);

	// set charge current
	setReg(CHG_DIG_CTL5, CHG_DIG_CTL5_CLR, CHARGE_CURRENT_SET(charge_current));

	// Enable charge, disable light
	if (enable_boost)
		modifyReg(SYS_CTL0, CHARGE_ENABLE | BOOST_ENABLE, LIGHT_ENABLE | ENABLE_FLASH_LIGHT );
	else
		modifyReg(SYS_CTL0, CHARGE_ENABLE, LIGHT_ENABLE | ENABLE_FLASH_LIGHT | BOOST_ENABLE);

	// Disable auto low load off and auto on load detect
	modifyReg(SYS_CTL1, 0, ENABLE_LOW_LOAD_AUTO_OFF | ENABLE_AUTO_ON_LOAD_DETECT);

	ret |= checkReg(CHG_DIG_CTL5, CHG_DIG_CTL5_CLR, CHARGE_CURRENT_SET(charge_current),0);

	if (enable_boost)
		ret |= checkReg(SYS_CTL0, SYS_CTL0_CLR, CHARGE_ENABLE | BOOST_ENABLE, LIGHT_ENABLE | ENABLE_FLASH_LIGHT );
	else
		ret |= checkReg(SYS_CTL0, SYS_CTL0_CLR, CHARGE_ENABLE, LIGHT_ENABLE | ENABLE_FLASH_LIGHT | BOOST_ENABLE);

	ret |= checkReg(SYS_CTL1, SYS_CTL1_CLR, 0, ENABLE_LOW_LOAD_AUTO_OFF | ENABLE_AUTO_ON_LOAD_DETECT);

	if (ret == PX4_OK) ScheduleOnInterval(SAMPLE_INTERVAL , SAMPLE_INTERVAL );
	else PX4_ERR("IP5109 init failed (%i)", ret);

	return ret;
}

int IP5109::probe()
{
	uint8_t buf;
	int ret = readReg(0, &buf, 1);

	if (ret != PX4_OK) {
		PX4_ERR("readReg failed (%i)", ret);
		return ret;
	}

	if (buf != IP5109_WHO_AM_I) {
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
	uint8_t buffer[2];
	buffer[0] = addr;
	memcpy(buffer + 1, buf, 1);
	return transfer(buffer, 2, nullptr, 0);
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
	if((origin_data & (~reset_flag)) == (set_flag & (~clear_flag)))
	{
		return PX4_OK;
	}
	PX4_ERR("IP5109 Register Check FAIL: addr: 0x%02x , read: 0x%02x , should: 0x%02x",
		addr, origin_data & (~reset_flag), set_flag & (~clear_flag));
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
		BATVOL = 2600-((float)(~BATVADC_VALUE_low)+((float)(~(BATVADC_VALUE_high & 0x1F)))*256+1)*0.26855f;
 	}
	else //原码
	{
		BATVOL = 2600+(((float)BATVADC_VALUE_low)+((float)BATVADC_VALUE_high)*256)*0.26855f; //mv 为单位
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
 		int c=(int)b*256+(int)a+1;
		BATCUR=-(float)c*0.745985f;
 		//BATCUR=-(int)(((~BATIADC_VALUE_low)+(~(BATIADC_VALUE_high & 0x1F))*256+1)*0.745985);
	}
	else //正值
 	{
 		BATCUR= (((float)BATIADC_VALUE_high*256+(float)BATIADC_VALUE_low))*0.745985f; //mA 为单位
 	}
	return BATCUR;
}
