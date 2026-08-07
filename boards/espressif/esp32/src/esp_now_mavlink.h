/****************************************************************************
 * include/nuttx/wireless/esp_now_mavlink.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_ESP_NOW_MAVLINK_H
#define __INCLUDE_NUTTX_WIRELESS_ESP_NOW_MAVLINK_H

#include <nuttx/config.h>

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MAVLINK_ESPNOW_KEY_LEN 16
#define MAVLINK_ESPNOW_ADDR_LEN 6

/* The PMK is copied by mavlink_espnow_register(); the caller may erase its
 * buffer after the function returns.  peer_addr may be a unicast address or
 * ff:ff:ff:ff:ff:ff for broadcast.  No LMK/data-frame encryption is used.
 */

struct mavlink_espnow_config_s
{
  uint8_t peer_addr[MAVLINK_ESPNOW_ADDR_LEN];
  uint8_t pmk[MAVLINK_ESPNOW_KEY_LEN];
  uint8_t channel;                  /* 0: use the current Wi-Fi channel */
};

/****************************************************************************
 * Name: mavlink_espnow_register
 *
 * Description:
 *   Initialize a minimal station radio, then register a stream-oriented
 *   ESP-NOW character device.  No esp-netif or NuttX network device is
 *   created.  This driver owns the process-wide Wi-Fi/ESP-NOW instances and
 *   callbacks.
 *
 * Input Parameters:
 *   devpath - Character device path, normally "/dev/mavlink0".
 *   config  - Remote peer or broadcast address, plus a 16-byte PMK.
 *
 * Returned Value:
 *   Zero on success or a negated errno value on failure.
 ****************************************************************************/

int mavlink_espnow_register(const char *devpath,
                            const struct mavlink_espnow_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_ESP_NOW_MAVLINK_H */
