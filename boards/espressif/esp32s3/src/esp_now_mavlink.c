/****************************************************************************
 * drivers/wireless/esp_now_mavlink.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <poll.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/tioctl.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>
#include <esp_now_mavlink.h>

/* The MAVLink payload is byte aligned.  Force generated accessors to use
 * byte copies on ESP32/Xtensa instead of alignment-increasing pointer casts.
 */

#define MAVLINK_ALIGNED_FIELDS 0

#if defined(__GNUC__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#endif
#include <mavlink/common/mavlink.h>
#if defined(__GNUC__)
#  pragma GCC diagnostic pop
#endif

#include "esp_now.h"
#include "esp_wifi.h"

/****************************************************************************
 * Driver Configuration
 ****************************************************************************/

#define MAVESPNOW_RXBUFSIZE       4096
#define MAVESPNOW_NPOLLWAITERS    2
#define MAVESPNOW_TXTIMEOUT_MS    1000
#define MAVESPNOW_WIFI_RXBUFS     4
#define MAVESPNOW_WIFI_TXBUFS     4
#define MAVESPNOW_WIFI_MGMTBUFS   2
#define MAVESPNOW_TX_POWER        80  /* 80 * 0.25 dBm = 20 dBm */
#define MAVESPNOW_WIFI_PROTOCOL   WIFI_PROTOCOL_LR
#define MAVESPNOW_PHY_MODE        WIFI_PHY_MODE_LR
#define MAVESPNOW_PHY_RATE        WIFI_PHY_RATE_LORA_250K
#define MAVESPNOW_CHANNEL_MIN     1
#define MAVESPNOW_CHANNEL_MAX     13
#define MAVESPNOW_DISCOVERY_CH    1
#define MAVESPNOW_SWITCH_DELAY_MS 50

/* Set this to 1 in the aircraft build and 0 in the ground-radio build.
 * The aircraft reports the RSSI it measured for ground-to-air packets to
 * the ground as RADIO_STATUS.remrssi.  No RSSI message is injected into PX4.
 */

#define MAVESPNOW_TX_RSSI_REPORT  1
#define MAVESPNOW_RSSI_PERIOD_MS  1000
#define MAVESPNOW_RADIO_SYSID     51  /* SiK-compatible '3' */
#define MAVESPNOW_RADIO_COMPID    68  /* MAV_COMP_ID_TELEMETRY_RADIO */
#define MAVESPNOW_RADIO_FRAME_LEN (MAVLINK_MSG_ID_RADIO_STATUS_LEN + \
                                   MAVLINK_NUM_NON_PAYLOAD_BYTES + \
                                   MAVLINK_SIGNATURE_BLOCK_LEN)

/* Use the frame limit exported by the same ESP-NOW HAL as the pktradio
 * driver.  The API currently defines ESP_NOW_MAX_DATA_LEN as 250 bytes.
 */

#define MAVESPNOW_FRAME_LEN       ESP_NOW_MAX_DATA_LEN
#define MAVESPNOW_HEADER_LEN      6
#define MAVESPNOW_PAYLOAD_LEN     (MAVESPNOW_FRAME_LEN - \
                                   MAVESPNOW_HEADER_LEN)
#define MAVESPNOW_MAGIC0          'M'
#define MAVESPNOW_MAGIC1          'V'
#define MAVESPNOW_VERSION         1

#define MAVESPNOW_CTRL_MAGIC0     'E'
#define MAVESPNOW_CTRL_MAGIC1     'D'
#define MAVESPNOW_CTRL_VERSION    1
#define MAVESPNOW_CTRL_DISCOVER   1
#define MAVESPNOW_CTRL_OFFER      2
#define MAVESPNOW_CTRL_ACCEPT     3
#define MAVESPNOW_CTRL_LEN        12

#if MAVLINK_ESPNOW_ADDR_LEN != ESP_NOW_ETH_ALEN
#  error "MAVLink and ESP-NOW address sizes do not match"
#endif

#if MAVLINK_ESPNOW_KEY_LEN != ESP_NOW_KEY_LEN
#  error "MAVLink PMK and ESP-NOW key sizes do not match"
#endif

struct mavlink_espnow_dev_s
{
  struct mavlink_espnow_config_s config;
  mutex_t txlock;
  mutex_t cfglock;
  sem_t txdone;
  sem_t rxready;
  spinlock_t rxlock;
  struct work_s discovery_work;
  struct work_s switch_work;
  struct termios termios;
  FAR struct pollfd *fds[MAVESPNOW_NPOLLWAITERS];
  uint8_t rxbuf[MAVESPNOW_RXBUFSIZE];
  size_t rxhead;
  size_t rxtail;
  size_t rxcount;
  uint8_t txseq;
  uint8_t rxseq;
  uint8_t local_addr[MAVLINK_ESPNOW_ADDR_LEN];
  uint8_t expected_peer[MAVLINK_ESPNOW_ADDR_LEN];
  uint8_t pending_peer[MAVLINK_ESPNOW_ADDR_LEN];
  uint8_t selected_channel;
#if MAVESPNOW_TX_RSSI_REPORT
  mavlink_status_t radio_tx_status;
  clock_t last_rssi_tx;
  int8_t last_rssi_dbm;
#endif
  volatile esp_now_send_status_t txstatus;
  volatile bool txpending;
  volatile bool discovery_pending;
  bool have_rxseq;
  bool link_ready;
  bool logged_discovery;
#if MAVESPNOW_TX_RSSI_REPORT
  bool have_rssi;
#endif
  bool running;
};

static FAR struct mavlink_espnow_dev_s *g_mavespnow;
static const uint8_t g_mavespnow_broadcast[MAVLINK_ESPNOW_ADDR_LEN] =
{
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

static int mavespnow_err(esp_err_t err)
{
  switch (err)
    {
      case ESP_OK:
        return OK;
      case ESP_ERR_ESPNOW_ARG:
        return -EINVAL;
      case ESP_ERR_ESPNOW_NO_MEM:
      case ESP_ERR_NO_MEM:
        return -ENOMEM;
      case ESP_ERR_ESPNOW_NOT_FOUND:
        return -ENOENT;
      case ESP_ERR_ESPNOW_EXIST:
        return -EEXIST;
      case ESP_ERR_ESPNOW_FULL:
        return -ENOSPC;
      case ESP_ERR_ESPNOW_NOT_INIT:
      case ESP_ERR_ESPNOW_INTERNAL:
      case ESP_ERR_ESPNOW_IF:
      default:
        return -EIO;
    }
}

static int mavespnow_wifi_start(void)
{
  wifi_init_config_t wifi = WIFI_INIT_CONFIG_DEFAULT();
  wifi_country_t country =
  {
    .cc = "CN",
    .schan = MAVESPNOW_CHANNEL_MIN,
    .nchan = MAVESPNOW_CHANNEL_MAX,
    .max_tx_power = MAVESPNOW_TX_POWER,
    .policy = WIFI_COUNTRY_POLICY_MANUAL,
  };
  esp_err_t err;

  /* ESP-NOW still needs the vendor Wi-Fi driver and an enabled STA radio,
   * but it does not need an esp-netif or a NuttX network device.  Keep only
   * a small set of radio buffers and disable features used by IP traffic.
   */

  wifi.static_rx_buf_num =
    MAVESPNOW_WIFI_RXBUFS;
  wifi.dynamic_rx_buf_num =
    MAVESPNOW_WIFI_RXBUFS;
  wifi.static_tx_buf_num =
    MAVESPNOW_WIFI_TXBUFS;
  wifi.dynamic_tx_buf_num =
    MAVESPNOW_WIFI_TXBUFS;
  wifi.rx_mgmt_buf_num =
    MAVESPNOW_WIFI_MGMTBUFS;
  wifi.mgmt_sbuf_num = 6;
  wifi.csi_enable = 0;
  wifi.ampdu_rx_enable = 0;
  wifi.ampdu_tx_enable = 0;
  wifi.amsdu_tx_enable = 0;
  wifi.nvs_enable = 0;
  wifi.rx_ba_win = 0;
  wifi.feature_caps = 0;
  wifi.sta_disconnected_pm = false;
  /* This is capacity only; peer.encrypt=false still disables LMK encryption.
   * Keep one slot because some vendor HAL revisions reject a zero value.
   */

  wifi.espnow_max_encrypt_num = 1;

  err = esp_wifi_init(&wifi);
  if (err != ESP_OK)
    {
      wlerr("ERROR: esp_wifi_init failed: %d\n", err);
      return mavespnow_err(err);
    }

  err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (err == ESP_OK)
    {
      err = esp_wifi_set_mode(WIFI_MODE_STA);
    }

  if (err == ESP_OK)
    {
      err = esp_wifi_start();
    }

  if (err != ESP_OK)
    {
      wlerr("ERROR: Wi-Fi mode/start failed: %d\n", err);
      esp_wifi_deinit();
      return mavespnow_err(err);
    }

  err = esp_wifi_set_country(&country);
  if (err == ESP_OK)
    {
      err = esp_wifi_set_ps(WIFI_PS_NONE);
    }
  if (err == ESP_OK)
    {
      err = esp_wifi_set_max_tx_power(MAVESPNOW_TX_POWER);
    }

  if (err != ESP_OK)
    {
      wlerr("ERROR: Wi-Fi power configuration failed: %d\n", err);
      esp_wifi_stop();
      esp_wifi_deinit();
      return mavespnow_err(err);
    }

  return OK;
}

/* Estimate congestion from visible access points.  A 20 MHz 2.4 GHz signal
 * overlaps nearby numbered channels, so each AP contributes a decreasing
 * score out to four channels on either side.  This is an AP-density estimate,
 * not a true airtime survey, but is available without keeping the radio in
 * promiscuous mode or delaying normal MAVLink traffic after startup. */

static uint8_t mavespnow_select_channel(void)
{
  FAR wifi_ap_record_t *records;
  wifi_scan_config_t scan;
  uint16_t score[MAVESPNOW_CHANNEL_MAX + 1];
  uint16_t count = 0;
  uint16_t fetched;
  uint8_t selected = MAVESPNOW_CHANNEL_MIN;
  esp_err_t err;
  int channel;
  int i;

  memset(score, 0, sizeof(score));
  memset(&scan, 0, sizeof(scan));
  scan.show_hidden = true;
  err = esp_wifi_scan_start(&scan, true);
  if (err != ESP_OK)
    {
      wlwarn("WARNING: Wi-Fi scan failed (%d); using channel %u\n",
             err, selected);
      return selected;
    }

  err = esp_wifi_scan_get_ap_num(&count);
  if (err != ESP_OK || count == 0)
    {
      if (err != ESP_OK)
        {
          wlwarn("WARNING: cannot read Wi-Fi scan results (%d); "
                 "using channel %u\n", err, selected);
          esp_wifi_clear_ap_list();
        }

      return selected;
    }

  records = kmm_malloc((size_t)count * sizeof(*records));
  if (records == NULL)
    {
      wlwarn("WARNING: no memory for %u Wi-Fi scan results; "
             "using channel %u\n", count, selected);
      esp_wifi_clear_ap_list();
      return selected;
    }

  fetched = count;
  err = esp_wifi_scan_get_ap_records(&fetched, records);
  if (err != ESP_OK)
    {
      wlwarn("WARNING: cannot fetch Wi-Fi scan results (%d); "
             "using channel %u\n", err, selected);
      esp_wifi_clear_ap_list();
      kmm_free(records);
      return selected;
    }

  for (i = 0; i < fetched; i++)
    {
      int primary = records[i].primary;

      if (primary < MAVESPNOW_CHANNEL_MIN ||
          primary > MAVESPNOW_CHANNEL_MAX)
        {
          continue;
        }

      for (channel = MAVESPNOW_CHANNEL_MIN;
           channel <= MAVESPNOW_CHANNEL_MAX; channel++)
        {
          int distance = channel > primary ? channel - primary :
                                             primary - channel;
          if (distance <= 4)
            {
              score[channel] += 5 - distance;
            }
        }
    }

  kmm_free(records);
  for (channel = MAVESPNOW_CHANNEL_MIN + 1;
       channel <= MAVESPNOW_CHANNEL_MAX; channel++)
    {
      if (score[channel] < score[selected])
        {
          selected = channel;
        }
    }

  wlinfo("Wi-Fi scan found %u APs; selected ESP-NOW channel %u "
         "(occupancy score %u)\n", fetched, selected, score[selected]);
  return selected;
}

static bool mavespnow_bad_key(FAR const uint8_t *key)
{
  uint8_t value = 0;
  int i;

  for (i = 0; i < MAVLINK_ESPNOW_KEY_LEN; i++)
    {
      value |= key[i];
    }

  return value == 0;
}

static bool mavespnow_bad_addr(FAR const uint8_t *addr)
{
  uint8_t value = 0;
  uint8_t all_ff = 0xff;
  int i;

  for (i = 0; i < MAVLINK_ESPNOW_ADDR_LEN; i++)
    {
      value |= addr[i];
      all_ff &= addr[i];
    }

  return value == 0 || ((addr[0] & 1) != 0 && all_ff != 0xff);
}

static bool mavespnow_is_broadcast(FAR const uint8_t *addr)
{
  int i;

  for (i = 0; i < MAVLINK_ESPNOW_ADDR_LEN; i++)
    {
      if (addr[i] != 0xff)
        {
          return false;
        }
    }

  return true;
}

static int mavespnow_open(FAR struct file *filep)
{
  UNUSED(filep);
  return OK;
}

static int mavespnow_close(FAR struct file *filep)
{
  UNUSED(filep);
  return OK;
}

static ssize_t mavespnow_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  FAR struct mavlink_espnow_dev_s *priv = filep->f_inode->i_private;
  irqstate_t flags;
  size_t first;
  size_t copied;
  int ret;

  if (buflen == 0)
    {
      return 0;
    }

  for (;;)
    {
      flags = spin_lock_irqsave(&priv->rxlock);
      copied = priv->rxcount < buflen ? priv->rxcount : buflen;

      if (copied != 0)
        {
          first = sizeof(priv->rxbuf) - priv->rxtail;
          if (first > copied)
            {
              first = copied;
            }

          memcpy(buffer, &priv->rxbuf[priv->rxtail], first);
          memcpy(buffer + first, priv->rxbuf, copied - first);
          priv->rxtail = (priv->rxtail + copied) % sizeof(priv->rxbuf);
          priv->rxcount -= copied;
          spin_unlock_irqrestore(&priv->rxlock, flags);
          return copied;
        }

      spin_unlock_irqrestore(&priv->rxlock, flags);

      if ((filep->f_oflags & O_NONBLOCK) != 0)
        {
          return -EAGAIN;
        }

      ret = nxsem_wait(&priv->rxready);
      if (ret < 0)
        {
          return ret;
        }
    }
}

/* Send one ESP-NOW transport fragment.  The caller holds txlock. */

static int mavespnow_send_fragment(FAR struct mavlink_espnow_dev_s *priv,
                                   FAR const uint8_t *payload,
                                   size_t payload_len)
{
  uint8_t frame[MAVESPNOW_FRAME_LEN];
  int ret;

  if (payload_len > MAVESPNOW_PAYLOAD_LEN)
    {
      return -EMSGSIZE;
    }

  /* A timed-out write may still have a completion callback in flight. */

  if (priv->txpending)
    {
      ret = nxsem_tickwait_uninterruptible(
              &priv->txdone, MSEC2TICK(MAVESPNOW_TXTIMEOUT_MS));
      if (ret < 0)
        {
          return -ETIMEDOUT;
        }
    }

  frame[0] = MAVESPNOW_MAGIC0;
  frame[1] = MAVESPNOW_MAGIC1;
  frame[2] = MAVESPNOW_VERSION;
  frame[3] = priv->txseq++;
  frame[4] = payload_len & 0xff;
  frame[5] = (payload_len >> 8) & 0xff;
  memcpy(&frame[MAVESPNOW_HEADER_LEN], payload, payload_len);

  while (nxsem_trywait(&priv->txdone) == OK)
    {
    }

  priv->txpending = true;
  ret = mavespnow_err(esp_now_send(priv->config.peer_addr, frame,
                                  MAVESPNOW_HEADER_LEN + payload_len));
  if (ret < 0)
    {
      priv->txpending = false;
      return ret;
    }

  ret = nxsem_tickwait_uninterruptible(
          &priv->txdone, MSEC2TICK(MAVESPNOW_TXTIMEOUT_MS));
  if (ret < 0)
    {
      return -ETIMEDOUT;
    }

  if (priv->txstatus != ESP_NOW_SEND_SUCCESS)
    {
      /* Drop this frame without changing the peer or channel. Discovery is
       * once per boot, so a temporary outage can recover immediately. */

      return -EHOSTUNREACH;
    }

  return OK;
}

#if MAVESPNOW_TX_RSSI_REPORT
static uint8_t mavespnow_rssi_to_sik(int rssi_dbm)
{
  int scaled;

  if (rssi_dbm < -127)
    {
      rssi_dbm = -127;
    }
  else if (rssi_dbm > 6)
    {
      rssi_dbm = 6;
    }

  scaled = ((rssi_dbm + 127) * 19 + 5) / 10;
  return (uint8_t)scaled;
}

static int mavespnow_send_rssi(FAR struct mavlink_espnow_dev_s *priv)
{
  uint8_t payload[MAVESPNOW_RADIO_FRAME_LEN];
  mavlink_message_t message;
  irqstate_t flags;
  clock_t now;
  int8_t rssi_dbm;
  uint16_t payload_len;
  bool report_due;
  int ret;

  now = clock_systime_ticks();
  flags = spin_lock_irqsave(&priv->rxlock);
  rssi_dbm = priv->last_rssi_dbm;
  report_due = priv->have_rssi &&
               (priv->last_rssi_tx == 0 ||
                now - priv->last_rssi_tx >=
                MSEC2TICK(MAVESPNOW_RSSI_PERIOD_MS));
  spin_unlock_irqrestore(&priv->rxlock, flags);

  if (!report_due)
    {
      return OK;
    }

  /* This value was measured at the aircraft, so it is the remote RSSI from
   * the ground station's point of view.  Report the selected Wi-Fi channel
   * through txbuf; unknown local RSSI/noise fields use UINT8_MAX as required
   * by RADIO_STATUS.
   */

  mavlink_msg_radio_status_pack_status(MAVESPNOW_RADIO_SYSID,
                                       MAVESPNOW_RADIO_COMPID,
                                       &priv->radio_tx_status, &message,
                                       UINT8_MAX,
                                       mavespnow_rssi_to_sik(rssi_dbm),
                                       priv->config.channel,
                                       UINT8_MAX, UINT8_MAX, 0, 0);
  payload_len = mavlink_msg_to_send_buffer(payload, &message);
  ret = mavespnow_send_fragment(priv, payload, payload_len);
  if (ret == OK)
    {
      flags = spin_lock_irqsave(&priv->rxlock);
      priv->last_rssi_tx = now;
      spin_unlock_irqrestore(&priv->rxlock, flags);
    }

  return ret;
}
#endif

static ssize_t mavespnow_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen)
{
  FAR struct mavlink_espnow_dev_s *priv = filep->f_inode->i_private;
  size_t sent = 0;
  size_t chunk;
  int ret;

  if (!priv->running || !priv->link_ready)
    {
      return -ENETDOWN;
    }

  if (buflen == 0)
    {
      return 0;
    }

  ret = nxmutex_lock(&priv->txlock);
  if (ret < 0)
    {
      return ret;
    }

  while (sent < buflen)
    {
      chunk = buflen - sent;
      if (chunk > MAVESPNOW_PAYLOAD_LEN)
        {
          chunk = MAVESPNOW_PAYLOAD_LEN;
        }

      ret = mavespnow_send_fragment(priv,
                                    (FAR const uint8_t *)buffer + sent,
                                    chunk);
      if (ret < 0)
        {
          break;
        }

      sent += chunk;
    }

#if MAVESPNOW_TX_RSSI_REPORT
  /* Append the link report only after the caller's complete write, keeping
   * it outside the application MAVLink frame.  Failure of this auxiliary
   * report must not turn a successfully sent PX4 message into a short write.
   */

  if (sent == buflen)
    {
      (void)mavespnow_send_rssi(priv);
    }
#endif

  nxmutex_unlock(&priv->txlock);
  return sent != 0 ? (ssize_t)sent : (ssize_t)ret;
}

static void mavespnow_flush_rx(FAR struct mavlink_espnow_dev_s *priv)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->rxlock);
  priv->rxhead = 0;
  priv->rxtail = 0;
  priv->rxcount = 0;
  spin_unlock_irqrestore(&priv->rxlock, flags);
}

static int mavespnow_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct mavlink_espnow_dev_s *priv = filep->f_inode->i_private;
  FAR struct termios *termiosp = (FAR struct termios *)(uintptr_t)arg;
  int ret;

  switch (cmd)
    {
      case TCGETS:
        if (termiosp == NULL)
          {
            return -EINVAL;
          }

        ret = nxmutex_lock(&priv->cfglock);
        if (ret < 0)
          {
            return ret;
          }

        memcpy(termiosp, &priv->termios, sizeof(*termiosp));
        nxmutex_unlock(&priv->cfglock);
        return OK;

      case TCSETS:
      case TCSETSW:
      case TCSETSF:
        if (termiosp == NULL)
          {
            return -EINVAL;
          }

        /* Report hardware flow control as unsupported.  PX4 uses this
         * failure to fall back when FLOW_CONTROL_AUTO is selected.
         */

        if ((termiosp->c_cflag & CRTSCTS) != 0)
          {
            return -EINVAL;
          }

        ret = nxmutex_lock(&priv->cfglock);
        if (ret < 0)
          {
            return ret;
          }

        memcpy(&priv->termios, termiosp, sizeof(priv->termios));
        nxmutex_unlock(&priv->cfglock);

        if (cmd == TCSETSF)
          {
            mavespnow_flush_rx(priv);
          }

        return OK;

      case TCDRN:
        /* write() waits for every ESP-NOW completion callback. */

        return OK;

      case TCFLSH:
        if (arg == TCIFLUSH || arg == TCIOFLUSH)
          {
            mavespnow_flush_rx(priv);
          }
        else if (arg != TCOFLUSH)
          {
            return -EINVAL;
          }

        return OK;

      case TCXONC:
        /* There are no RTS/CTS or software-flow-control lines. */

        return OK;

      case FIONSPACE:
        if (arg == 0)
          {
            return -EINVAL;
          }

        /* PX4 uses this value to decide whether it may emit the next MAVLink
         * message.  write() fragments larger messages and applies its own
         * serialization, so advertise a small multi-frame window.
         */

        *(FAR int *)(uintptr_t)arg =
          MAVESPNOW_PAYLOAD_LEN * MAVESPNOW_WIFI_TXBUFS;
        return OK;

      default:
        return -ENOTTY;
    }
}

static int mavespnow_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup)
{
  FAR struct mavlink_espnow_dev_s *priv = filep->f_inode->i_private;
  irqstate_t flags;
  pollevent_t eventset = 0;
  int i;
  int ret = OK;

  flags = spin_lock_irqsave(&priv->rxlock);

  if (setup)
    {
      for (i = 0; i < MAVESPNOW_NPOLLWAITERS; i++)
        {
          if (priv->fds[i] == NULL)
            {
              priv->fds[i] = fds;
              fds->priv = &priv->fds[i];
              break;
            }
        }

      if (i == MAVESPNOW_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
        }
      else
        {
          if (priv->rxcount != 0)
            {
              eventset |= POLLIN;
            }

          if (priv->running && priv->link_ready)
            {
              eventset |= POLLOUT;
            }
        }
    }
  else if (fds->priv != NULL)
    {
      FAR struct pollfd **slot = fds->priv;
      *slot = NULL;
      fds->priv = NULL;
    }

  spin_unlock_irqrestore(&priv->rxlock, flags);

  if (eventset != 0)
    {
      poll_notify(&fds, 1, eventset);
    }

  return ret;
}

static const struct file_operations g_mavespnow_fops =
{
  .open  = mavespnow_open,
  .close = mavespnow_close,
  .read  = mavespnow_read,
  .write = mavespnow_write,
  .ioctl = mavespnow_ioctl,
  .poll  = mavespnow_poll,
};

static void mavespnow_switch_worker(FAR void *arg)
{
  FAR struct mavlink_espnow_dev_s *priv = arg;
  FAR struct pollfd *fds[MAVESPNOW_NPOLLWAITERS];
  irqstate_t flags;
  int i;
  int ret;

  ret = nxmutex_lock(&priv->txlock);
  if (ret < 0 || !priv->running || priv->link_ready)
    {
      if (ret >= 0)
        {
          nxmutex_unlock(&priv->txlock);
        }

      return;
    }

  ret = mavespnow_err(esp_wifi_set_channel(priv->selected_channel,
                                           WIFI_SECOND_CHAN_NONE));
  if (ret == OK)
    {
      priv->link_ready = true;
      flags = spin_lock_irqsave(&priv->rxlock);
      for (i = 0; i < MAVESPNOW_NPOLLWAITERS; i++)
        {
          fds[i] = priv->fds[i];
        }

      spin_unlock_irqrestore(&priv->rxlock, flags);
      poll_notify(fds, MAVESPNOW_NPOLLWAITERS, POLLOUT);
      wlinfo("ESP-NOW peer %02x:%02x:%02x:%02x:%02x:%02x; "
             "unicast channel %u\n",
             priv->config.peer_addr[0], priv->config.peer_addr[1],
             priv->config.peer_addr[2], priv->config.peer_addr[3],
             priv->config.peer_addr[4], priv->config.peer_addr[5],
             priv->selected_channel);
    }
  else
    {
      wlwarn("WARNING: cannot switch to negotiated channel: %d\n", ret);
    }

  nxmutex_unlock(&priv->txlock);
}

/* Complete discovery outside the Wi-Fi receive callback.  The OFFER is a
 * response to a new discovery request, not a retry of an earlier packet. */

static void mavespnow_discovery_worker(FAR void *arg)
{
  FAR struct mavlink_espnow_dev_s *priv = arg;
  esp_now_peer_info_t peer;
  esp_now_rate_config_t rate;
  uint8_t address[MAVLINK_ESPNOW_ADDR_LEN];
  uint8_t frame[MAVESPNOW_CTRL_LEN] =
  {
    MAVESPNOW_CTRL_MAGIC0, MAVESPNOW_CTRL_MAGIC1,
    MAVESPNOW_CTRL_VERSION, MAVESPNOW_CTRL_OFFER, 0, 0
  };
  irqstate_t flags;
  int ret;

  flags = spin_lock_irqsave(&priv->rxlock);
  memcpy(address, priv->pending_peer, sizeof(address));
  spin_unlock_irqrestore(&priv->rxlock, flags);

  ret = nxmutex_lock(&priv->txlock);
  if (ret < 0 || !priv->running || priv->link_ready ||
      priv->discovery_pending)
    {
      if (ret >= 0)
        {
          nxmutex_unlock(&priv->txlock);
        }

      return;
    }

  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, address, sizeof(peer.peer_addr));
  peer.ifidx = WIFI_IF_STA;
  peer.channel = 0; /* Follow the STA channel after negotiation. */
  peer.encrypt = false;
  if (!mavespnow_is_broadcast(priv->config.peer_addr) &&
      memcmp(priv->config.peer_addr, address,
             MAVLINK_ESPNOW_ADDR_LEN) != 0 &&
      esp_now_is_peer_exist(priv->config.peer_addr))
    {
      (void)esp_now_del_peer(priv->config.peer_addr);
    }

  if (!esp_now_is_peer_exist(address))
    {
      ret = mavespnow_err(esp_now_add_peer(&peer));
      if (ret < 0)
        {
          wlwarn("WARNING: cannot add discovered peer: %d\n", ret);
          nxmutex_unlock(&priv->txlock);
          return;
        }
    }

  memset(&rate, 0, sizeof(rate));
  rate.phymode = MAVESPNOW_PHY_MODE;
  rate.rate = MAVESPNOW_PHY_RATE;
  ret = mavespnow_err(esp_now_set_peer_rate_config(address, &rate));
  if (ret < 0)
    {
      wlwarn("WARNING: cannot configure discovered peer: %d\n", ret);
      nxmutex_unlock(&priv->txlock);
      return;
    }

  frame[4] = priv->selected_channel;
  memcpy(&frame[6], priv->local_addr, MAVLINK_ESPNOW_ADDR_LEN);
  if (!priv->logged_discovery)
    {
      wlinfo("ESP-NOW discovery from %02x:%02x:%02x:%02x:%02x:%02x; "
             "offering channel %u\n",
             address[0], address[1], address[2], address[3], address[4],
             address[5], priv->selected_channel);
      priv->logged_discovery = true;
    }

  memcpy(priv->config.peer_addr, address, MAVLINK_ESPNOW_ADDR_LEN);
  priv->discovery_pending = true;
  priv->txpending = true;
  /* Discovery stays broadcast in both directions.  The controller cannot be
   * assumed to have installed this aircraft as a peer until it receives this
   * OFFER and reads the MAC carried in it. */

  ret = mavespnow_err(esp_now_send(g_mavespnow_broadcast, frame,
                                  sizeof(frame)));
  if (ret < 0)
    {
      priv->txpending = false;
      priv->discovery_pending = false;
      wlwarn("WARNING: discovery offer failed: %d\n", ret);
    }

  nxmutex_unlock(&priv->txlock);
}

static void mavespnow_recv_cb(FAR const esp_now_recv_info_t *info,
                              FAR const uint8_t *data, int data_len)
{
  FAR struct mavlink_espnow_dev_s *priv = g_mavespnow;
  FAR const uint8_t *payload;
  FAR struct pollfd *fds[MAVESPNOW_NPOLLWAITERS];
  irqstate_t flags;
  size_t payload_len;
  size_t first;
  int i;

  if (priv == NULL || !priv->running || info == NULL ||
      info->src_addr == NULL || data == NULL)
    {
      return;
    }

  /* The device is explicitly on channel 1 in this state.  Do not require the
   * optional rx_ctrl channel field: some LR-capable HALs report it as zero. */

  if (!priv->link_ready && !priv->discovery_pending &&
      data_len == MAVESPNOW_CTRL_LEN &&
      data[0] == MAVESPNOW_CTRL_MAGIC0 &&
      data[1] == MAVESPNOW_CTRL_MAGIC1 &&
      data[2] == MAVESPNOW_CTRL_VERSION &&
      data[3] == MAVESPNOW_CTRL_DISCOVER &&
      data[4] == MAVESPNOW_DISCOVERY_CH &&
      memcmp(&data[6], info->src_addr, MAVLINK_ESPNOW_ADDR_LEN) == 0 &&
      (mavespnow_is_broadcast(priv->expected_peer) ||
       memcmp(info->src_addr, priv->expected_peer,
              MAVLINK_ESPNOW_ADDR_LEN) == 0))
    {
      flags = spin_lock_irqsave(&priv->rxlock);
      memcpy(priv->pending_peer, info->src_addr,
             MAVLINK_ESPNOW_ADDR_LEN);
      spin_unlock_irqrestore(&priv->rxlock, flags);
      if (work_available(&priv->discovery_work))
        {
          (void)work_queue(HPWORK, &priv->discovery_work,
                           mavespnow_discovery_worker, priv, 0);
        }

      return;
    }

  if (!priv->link_ready && data_len == MAVESPNOW_CTRL_LEN &&
      data[0] == MAVESPNOW_CTRL_MAGIC0 &&
      data[1] == MAVESPNOW_CTRL_MAGIC1 &&
      data[2] == MAVESPNOW_CTRL_VERSION &&
      data[3] == MAVESPNOW_CTRL_ACCEPT &&
      data[4] == priv->selected_channel &&
      memcmp(&data[6], info->src_addr, MAVLINK_ESPNOW_ADDR_LEN) == 0 &&
      memcmp(info->src_addr, priv->config.peer_addr,
             MAVLINK_ESPNOW_ADDR_LEN) == 0)
    {
      if (work_available(&priv->switch_work))
        {
          (void)work_queue(HPWORK, &priv->switch_work,
                           mavespnow_switch_worker, priv,
                           MSEC2TICK(MAVESPNOW_SWITCH_DELAY_MS));
        }

      return;
    }

  if (!priv->link_ready ||
      data_len < MAVESPNOW_HEADER_LEN ||
      memcmp(info->src_addr, priv->config.peer_addr,
             MAVLINK_ESPNOW_ADDR_LEN) != 0 ||
      data[0] != MAVESPNOW_MAGIC0 || data[1] != MAVESPNOW_MAGIC1 ||
      data[2] != MAVESPNOW_VERSION)
    {
      return;
    }

  payload_len = (size_t)data[4] | ((size_t)data[5] << 8);
  if (payload_len > MAVESPNOW_PAYLOAD_LEN ||
      payload_len != (size_t)data_len - MAVESPNOW_HEADER_LEN)
    {
      return;
    }

  payload = &data[MAVESPNOW_HEADER_LEN];
  flags = spin_lock_irqsave(&priv->rxlock);

#if MAVESPNOW_TX_RSSI_REPORT
  /* Cache only.  Do not expose this measurement to the local PX4 reader;
   * mavespnow_write() will send it to the ground endpoint instead.
   */

  if (info->rx_ctrl != NULL)
    {
      priv->last_rssi_dbm = info->rx_ctrl->rssi;
      priv->have_rssi = true;
    }
#endif

  /* Drop a complete radio fragment when the stream buffer is full.  A
   * partial fragment would corrupt the MAVLink byte stream more severely.
   */

  if (payload_len <= sizeof(priv->rxbuf) - priv->rxcount)
    {
      first = sizeof(priv->rxbuf) - priv->rxhead;
      if (first > payload_len)
        {
          first = payload_len;
        }

      memcpy(&priv->rxbuf[priv->rxhead], payload, first);
      memcpy(priv->rxbuf, payload + first, payload_len - first);
      priv->rxhead = (priv->rxhead + payload_len) % sizeof(priv->rxbuf);
      priv->rxcount += payload_len;
      priv->rxseq = data[3];
      priv->have_rxseq = true;

      for (i = 0; i < MAVESPNOW_NPOLLWAITERS; i++)
        {
          fds[i] = priv->fds[i];
        }

      spin_unlock_irqrestore(&priv->rxlock, flags);
      nxsem_post(&priv->rxready);
      poll_notify(fds, MAVESPNOW_NPOLLWAITERS, POLLIN);
    }
  else
    {
      spin_unlock_irqrestore(&priv->rxlock, flags);
    }
}

static void mavespnow_send_cb(FAR const uint8_t *mac_address,
                              esp_now_send_status_t status)
{
  FAR struct mavlink_espnow_dev_s *priv = g_mavespnow;

  if (priv != NULL && priv->running && priv->txpending &&
      priv->discovery_pending && mac_address != NULL &&
      memcmp(mac_address, g_mavespnow_broadcast,
             MAVLINK_ESPNOW_ADDR_LEN) == 0)
    {
      priv->txstatus = status;
      priv->txpending = false;
      priv->discovery_pending = false;
      nxsem_post(&priv->txdone);
    }
  else if (priv != NULL && priv->running && priv->txpending &&
      mac_address != NULL &&
      memcmp(mac_address, priv->config.peer_addr,
             MAVLINK_ESPNOW_ADDR_LEN) == 0)
    {
      priv->txstatus = status;
      priv->txpending = false;
      nxsem_post(&priv->txdone);
    }
}

int mavlink_espnow_register(
  FAR const char *devpath,
  FAR const struct mavlink_espnow_config_s *config)
{
  FAR struct mavlink_espnow_dev_s *priv;
  esp_now_peer_info_t peer;
  esp_now_rate_config_t rate;
  int ret;

  if (devpath == NULL || config == NULL)
    {
      wlerr("ERROR: NULL device path or configuration\n");
      return -EINVAL;
    }

  if (config->channel > MAVESPNOW_CHANNEL_MAX)
    {
      wlerr("ERROR: invalid ESP-NOW channel: %u\n", config->channel);
      return -EINVAL;
    }

  if (mavespnow_bad_addr(config->peer_addr))
    {
      wlerr("ERROR: invalid ESP-NOW peer address\n");
      return -EINVAL;
    }

  if (mavespnow_bad_key(config->pmk))
    {
      wlerr("ERROR: ESP-NOW PMK must not be all zero\n");
      return -EINVAL;
    }

  if (g_mavespnow != NULL)
    {
      return -EBUSY;
    }

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  memcpy(&priv->config, config, sizeof(*config));
  memcpy(priv->expected_peer, config->peer_addr,
         MAVLINK_ESPNOW_ADDR_LEN);
  nxmutex_init(&priv->txlock);
  nxmutex_init(&priv->cfglock);
  nxsem_init(&priv->txdone, 0, 0);
  nxsem_init(&priv->rxready, 0, 0);
  spin_lock_init(&priv->rxlock);
#if MAVESPNOW_TX_RSSI_REPORT
  priv->radio_tx_status.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
#endif

  /* PX4 treats every MAVLink character device as a UART and configures it
   * through termios.  Store those settings as virtual attributes; baud and
   * flow-control flags do not alter the ESP-NOW radio transport.
   */

  memset(&priv->termios, 0, sizeof(priv->termios));
  priv->termios.c_cflag = CS8 | CREAD | CLOCAL;
  priv->termios.c_speed = B57600;
  priv->termios.c_cc[VMIN] = 1;

  ret = mavespnow_wifi_start();
  if (ret < 0)
    {
      goto err_free;
    }

  if (priv->config.channel == 0)
    {
      priv->config.channel = mavespnow_select_channel();
    }

  priv->selected_channel = priv->config.channel;

  ret = mavespnow_err(
          esp_wifi_set_channel(MAVESPNOW_DISCOVERY_CH,
                               WIFI_SECOND_CHAN_NONE));
  if (ret < 0)
    {
      wlerr("ERROR: esp_wifi_set_channel failed: %d\n", ret);
      goto err_wifi;
    }

  ret = mavespnow_err(
          esp_wifi_set_protocol(WIFI_IF_STA, MAVESPNOW_WIFI_PROTOCOL));
  if (ret < 0)
    {
      wlerr("ERROR: Wi-Fi LR configuration failed: %d\n", ret);
      goto err_wifi;
    }

  ret = mavespnow_err(esp_now_init());
  if (ret < 0)
    {
      wlerr("ERROR: esp_now_init failed: %d\n", ret);
      goto err_wifi;
    }

  ret = mavespnow_err(esp_now_set_pmk(priv->config.pmk));
  if (ret < 0)
    {
      wlerr("ERROR: esp_now_set_pmk failed: %d\n", ret);
      goto err_deinit;
    }

  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, g_mavespnow_broadcast, sizeof(peer.peer_addr));
  peer.ifidx = WIFI_IF_STA;
  peer.channel = MAVESPNOW_DISCOVERY_CH;
  peer.encrypt = false;

  ret = mavespnow_err(esp_now_add_peer(&peer));
  if (ret < 0)
    {
      wlerr("ERROR: esp_now_add_peer failed: %d\n", ret);
      goto err_deinit;
    }

  memset(&rate, 0, sizeof(rate));
  rate.phymode = MAVESPNOW_PHY_MODE;
  rate.rate = MAVESPNOW_PHY_RATE;
  rate.ersu = false;
  rate.dcm = false;

  ret = mavespnow_err(
          esp_now_set_peer_rate_config(g_mavespnow_broadcast, &rate));
  if (ret < 0)
    {
      wlerr("ERROR: ESP-NOW LR 250K rate configuration failed: %d\n", ret);
      goto err_peer;
    }

  ret = mavespnow_err(esp_wifi_get_mac(WIFI_IF_STA, priv->local_addr));
  if (ret < 0)
    {
      wlerr("ERROR: cannot read station MAC: %d\n", ret);
      goto err_peer;
    }

  g_mavespnow = priv;
  ret = mavespnow_err(esp_now_register_recv_cb(mavespnow_recv_cb));
  if (ret < 0)
    {
      goto err_peer;
    }

  ret = mavespnow_err(esp_now_register_send_cb(mavespnow_send_cb));
  if (ret < 0)
    {
      goto err_recv_cb;
    }

  priv->running = true;
  ret = register_driver(devpath, &g_mavespnow_fops, 0660, priv);
  if (ret < 0)
    {
      priv->running = false;
      esp_now_unregister_send_cb();
      goto err_recv_cb;
    }

  return OK;

err_recv_cb:
  esp_now_unregister_recv_cb();
err_peer:
  priv->running = false;
  g_mavespnow = NULL;
  esp_now_del_peer(g_mavespnow_broadcast);
err_deinit:
  esp_now_deinit();
err_wifi:
  esp_wifi_stop();
  esp_wifi_deinit();
err_free:
  nxsem_destroy(&priv->rxready);
  nxsem_destroy(&priv->txdone);
  nxmutex_destroy(&priv->cfglock);
  nxmutex_destroy(&priv->txlock);
  memset(priv, 0, sizeof(*priv));
  kmm_free(priv);
  return ret;
}
