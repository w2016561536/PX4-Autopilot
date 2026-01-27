/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_board_wlan.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <syslog.h>
#include <debug.h>

#include <nuttx/wireless/wireless.h>

#include "esp32s3_spiflash.h"
#include "espressif/esp_wlan.h"
#include "netutils/netlib.h"
#include "netutils/dhcpd.h"
#include "esp32s3_board_wlan.h"
#include "esp_wifi_types.h"
#include "esp_mac.h"

extern int wapi_main(int argc, char *argv[]);
extern esp_err_t esp_read_mac(uint8_t *mac, esp_mac_type_t type);
extern esp_err_t esp_wifi_set_protocol(wifi_interface_t ifx, uint8_t protocol_bitmap);
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_wlan_init
 *
 * Description:
 *   Configure the wireless subsystem.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_wlan_init(void)
{
  int ret = OK;
/*

#ifdef ESP32S3_WLAN_HAS_STA
  ret = esp32s3_wlan_sta_initialize();
  if (ret)
    {
      printf("ERROR: Failed to initialize Wi-Fi station\n");
      return ret;
    }
#endif*/

  ret = esp_wlan_softap_initialize();
  if (ret)
    {
      wlerr("ERROR: Failed to initialize Wi-Fi softAP\n");
      return ret;
    }

  // force to use 802.11b
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B );

  netlib_ifup("wlan0");

  //dhcpd_start("wlan0");

  return ret;
}
