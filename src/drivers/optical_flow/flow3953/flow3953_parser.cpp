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

/**
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 *
 * Declarations of parser for the ThoneFlow-3901U optical flow sensor
 */

#include "flow3953_parser.h"
#include <string.h>
#include <stdlib.h>

//#define Flow3953_DEBUG

bool flow3953_parse(char c, char *parserbuf, unsigned *parserbuf_index, enum Flow3953_PARSE_STATE *state,
		     sensor_optical_flow_s *flow, uint16_t *distance_cm)
{
	bool parsed_packet = false;

	switch (*state) {
	case Flow3953_PARSE_STATE11_FOOTER:
		if (c == 0xFE) {
			*state = Flow3953_PARSE_STATE1_HEADER;

		} else {
			*state = Flow3953_PARSE_STATE0_UNSYNC;
		}

		break;

	case Flow3953_PARSE_STATE0_UNSYNC:
		if (c == 0xFE) {
			*state = Flow3953_PARSE_STATE1_HEADER;
		}

		break;

	case Flow3953_PARSE_STATE1_HEADER:
		if (c == 0x04) {
			*state = Flow3953_PARSE_STATE2_NBYTES;

		} else {
			*state = Flow3953_PARSE_STATE0_UNSYNC;
		}

		break;

	case Flow3953_PARSE_STATE2_NBYTES:
		*state = Flow3953_PARSE_STATE3_XM_L;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case Flow3953_PARSE_STATE3_XM_L:
		*state = Flow3953_PARSE_STATE4_XM_H;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case Flow3953_PARSE_STATE4_XM_H:
		*state = Flow3953_PARSE_STATE5_YM_L;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case Flow3953_PARSE_STATE5_YM_L:
		*state = Flow3953_PARSE_STATE6_YM_H;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;
	case Flow3953_PARSE_STATE6_YM_H:
		*state = Flow3953_PARSE_STATE7_HIGHT_LOW;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;
	case Flow3953_PARSE_STATE7_HIGHT_LOW:
		*state = Flow3953_PARSE_STATE8_HIGHT_HIGH;
		parserbuf[*parserbuf_index] = c;
		(*parserbuf_index)++;

		break;

	case Flow3953_PARSE_STATE8_HIGHT_HIGH: {
			unsigned char cksm = 0;

			// Calculate checksum over motion values
			for (int i = 0; i < 6; i++) {
				cksm += parserbuf[i];
			}

			if (c == cksm) {
				// Checksum valid, populate sensor report
				int16_t delta_x = uint16_t(parserbuf[1]) << 8 | parserbuf[0];
				int16_t delta_y = uint16_t(parserbuf[3]) << 8 | parserbuf[2];
				uint16_t distance_data = uint16_t(parserbuf[5]) << 8 | parserbuf[4];
				flow->distance_m = (float)distance_data * 1e-2f;
				flow->distance_available = true;

				flow->pixel_flow[0] = static_cast<float>(delta_x) * (3.52e-3f);
				flow->pixel_flow[1] = static_cast<float>(delta_y) * (3.52e-3f);
				*state = Flow3953_PARSE_STATE9_CHECKSUM;

			} else {
				*state = Flow3953_PARSE_STATE0_UNSYNC;
			}

			*parserbuf_index = 0;
		}

		break;

	case Flow3953_PARSE_STATE9_CHECKSUM:
		*state = Flow3953_PARSE_STATE10_QUALITY;
		flow->quality = uint8_t(c);

		break;

	case Flow3953_PARSE_STATE10_QUALITY:
		if (c == 0xAA) {
			*state = Flow3953_PARSE_STATE11_FOOTER;
			parsed_packet = true;

		} else {
			*state = Flow3953_PARSE_STATE0_UNSYNC;
		}

		break;

	}

	return parsed_packet;
}
