/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <uORB/topics/sensor_optical_flow.h>

// Data Format for Flow3953 3901U
// ===============================
// 9 bytes total per message:
// 1) Header (0xFE)
// 2) Number of data bytes (0x04)
// 3) X motion low byte
// 4) X motion high byte
// 5) Y motion low byte
// 6) Y motion high byte
// 7) Checksum (byte3+...+byte6)
// 8) Quality
// 9) Footer (0xAA)

enum Flow3953_PARSE_STATE {
	Flow3953_PARSE_STATE0_UNSYNC = 0,
	Flow3953_PARSE_STATE1_HEADER,
	Flow3953_PARSE_STATE2_NBYTES,
	Flow3953_PARSE_STATE3_XM_L,
	Flow3953_PARSE_STATE4_XM_H,
	Flow3953_PARSE_STATE5_YM_L,
	Flow3953_PARSE_STATE6_YM_H,
	Flow3953_PARSE_STATE7_HIGHT_LOW,
	Flow3953_PARSE_STATE8_HIGHT_HIGH,
	Flow3953_PARSE_STATE9_CHECKSUM,
	Flow3953_PARSE_STATE10_QUALITY,
	Flow3953_PARSE_STATE11_FOOTER
};

bool flow3953_parse(char c, char *parserbuf, unsigned *parserbuf_index, enum Flow3953_PARSE_STATE *state,
		     sensor_optical_flow_s *report, uint16_t *distance_cm);
