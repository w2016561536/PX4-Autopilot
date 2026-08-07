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
#include <esp_now_mavlink.h>

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
#define MAVESPNOW_TX_POWER        84  /* 80 * 0.25 dBm = 20 dBm */
#define MAVESPNOW_WIFI_PROTOCOL   WIFI_PROTOCOL_LR
#define MAVESPNOW_PHY_MODE        WIFI_PHY_MODE_LR
#define MAVESPNOW_PHY_RATE        WIFI_PHY_RATE_LORA_500K

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
  struct termios termios;
  FAR struct pollfd *fds[MAVESPNOW_NPOLLWAITERS];
  uint8_t rxbuf[MAVESPNOW_RXBUFSIZE];
  size_t rxhead;
  size_t rxtail;
  size_t rxcount;
  uint8_t txseq;
  uint8_t rxseq;
  volatile esp_now_send_status_t txstatus;
  volatile bool txpending;
  bool have_rxseq;
  bool running;
};

static FAR struct mavlink_espnow_dev_s *g_mavespnow;

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

  err = esp_wifi_set_ps(WIFI_PS_NONE);
  if (err == ESP_OK)
    {
      err = esp_wifi_set_max_tx_power(MAVESPNOW_TX_POWER);
    }

  if (err == ESP_OK)
    {
      err = esp_wifi_set_protocol(WIFI_IF_STA,
                                  MAVESPNOW_WIFI_PROTOCOL);
    }

  if (err != ESP_OK)
    {
      wlerr("ERROR: Wi-Fi power/LR configuration failed: %d\n", err);
      esp_wifi_stop();
      esp_wifi_deinit();
      return mavespnow_err(err);
    }

  return OK;
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

static ssize_t mavespnow_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen)
{
  FAR struct mavlink_espnow_dev_s *priv = filep->f_inode->i_private;
  uint8_t frame[MAVESPNOW_FRAME_LEN];
  size_t sent = 0;
  size_t chunk;
  int ret;

  if (!priv->running)
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
      /* A timed-out write may still have a radio completion callback in
       * flight.  Consume that callback before reusing the single completion
       * semaphore, otherwise it could be mistaken for the next packet.
       */

      if (priv->txpending)
        {
          ret = nxsem_tickwait_uninterruptible(
                  &priv->txdone,
                  MSEC2TICK(MAVESPNOW_TXTIMEOUT_MS));
          if (ret < 0)
            {
              ret = -ETIMEDOUT;
              break;
            }
        }

      chunk = buflen - sent;
      if (chunk > MAVESPNOW_PAYLOAD_LEN)
        {
          chunk = MAVESPNOW_PAYLOAD_LEN;
        }

      frame[0] = MAVESPNOW_MAGIC0;
      frame[1] = MAVESPNOW_MAGIC1;
      frame[2] = MAVESPNOW_VERSION;
      frame[3] = priv->txseq++;
      frame[4] = chunk & 0xff;
      frame[5] = (chunk >> 8) & 0xff;
      memcpy(&frame[MAVESPNOW_HEADER_LEN], buffer + sent, chunk);

      /* Remove a stale completion token before starting the sole in-flight
       * transfer.  txlock preserves packet order for concurrent writers.
       */

      while (nxsem_trywait(&priv->txdone) == OK)
        {
        }

      priv->txpending = true;
      ret = mavespnow_err(esp_now_send(priv->config.peer_addr, frame,
                                      MAVESPNOW_HEADER_LEN + chunk));
      if (ret < 0)
        {
          priv->txpending = false;
          break;
        }

      ret = nxsem_tickwait_uninterruptible(
              &priv->txdone,
              MSEC2TICK(MAVESPNOW_TXTIMEOUT_MS));
      if (ret < 0)
        {
          ret = -ETIMEDOUT;
          break;
        }

      if (priv->txstatus != ESP_NOW_SEND_SUCCESS)
        {
          ret = -EHOSTUNREACH;
          break;
        }

      sent += chunk;
    }

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

          if (priv->running)
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
      info->src_addr == NULL || data == NULL ||
      data_len < MAVESPNOW_HEADER_LEN ||
      (!mavespnow_is_broadcast(priv->config.peer_addr) &&
       memcmp(info->src_addr, priv->config.peer_addr,
              MAVLINK_ESPNOW_ADDR_LEN) != 0) ||
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

  if (config->channel > 14)
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
  nxmutex_init(&priv->txlock);
  nxmutex_init(&priv->cfglock);
  nxsem_init(&priv->txdone, 0, 0);
  nxsem_init(&priv->rxready, 0, 0);
  spin_lock_init(&priv->rxlock);

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

  if (priv->config.channel != 0)
    {
      ret = mavespnow_err(
              esp_wifi_set_channel(priv->config.channel,
                                   WIFI_SECOND_CHAN_NONE));
      if (ret < 0)
        {
          wlerr("ERROR: esp_wifi_set_channel failed: %d\n", ret);
          goto err_wifi;
        }
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
  memcpy(peer.peer_addr, priv->config.peer_addr, sizeof(peer.peer_addr));
  peer.ifidx = WIFI_IF_STA;
  peer.channel = priv->config.channel;
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
          esp_now_set_peer_rate_config(priv->config.peer_addr, &rate));
  if (ret < 0)
    {
      wlerr("ERROR: ESP-NOW LR 250K rate configuration failed: %d\n", ret);
      goto err_peer;
    }

  g_mavespnow = priv;
  ret = mavespnow_err(esp_now_register_recv_cb(mavespnow_recv_cb));
  if (ret < 0)
    {
      goto err_peer;
    }

  ret = mavespnow_err(esp_now_register_send_cb((esp_now_send_cb_t)mavespnow_send_cb));
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
  g_mavespnow = NULL;
  esp_now_del_peer(priv->config.peer_addr);
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
