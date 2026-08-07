// #include "espressif/esp_wlan.h"
#include "netutils/netlib.h"
#include "netutils/dhcpd.h"
#include "esp32s3_board_wlan.h"
#include "esp_wifi_types.h"
#include "esp_mac.h"
#include <px4_log.h>

extern "C" {  int wapi_main(int argc, char *argv[]); }
extern esp_err_t esp_read_mac(uint8_t *mac, esp_mac_type_t type);

extern "C" { __EXPORT int edit_ssid_main(int argc, char *argv[]); }

int edit_ssid_main(int argc, char *argv[])
{
	uint8_t mac[6];
	char ssid[32];
	char *wapi_argv[5];
	const char *ifname;
	const char *prefix;
	int ret;

	/* 参数检查 */

	if (argc < 3) {
		PX4_ERR("Usage: edit_ssid <ifname> <prefix>");
		return -EINVAL;
	}

	ifname = argv[1];
	prefix = argv[2];

	/* 1. 读取 SoftAP MAC */

	ret = esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);

	if (ret < 0) {
		PX4_ERR("ERROR: Failed to read MAC address");
		return ret;
	}

	/* 2. 拼 SSID（<= 32 bytes） */

	snprintf(ssid, sizeof(ssid),
		 "%s_%02X%02X%02X",
		 prefix,
		 mac[3], mac[4], mac[5]);

	/* 3. 组织 wapi 命令参数 */

	wapi_argv[0] = const_cast<char *>("wapi");
	wapi_argv[1] = const_cast<char *>("essid");
	wapi_argv[2] = const_cast<char *>(ifname);
	wapi_argv[3] = ssid;
	wapi_argv[4] = const_cast<char *>("1");

	/* 4. 调用 wapi_main */

	ret = wapi_main(5, wapi_argv);

	if (ret < 0) {
		PX4_ERR("ERROR: wapi essid failed: %d", ret);
	}

	return ret;
}
