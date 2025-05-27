#include <Arduino.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"

#define NUM_TARGETS      3
#define WIFI_CHANNEL     1
#define SCAN_TIMEOUT_MS 10000

static const char *TAG = "RSSI_SNIFFER";

// Define your target MAC addresses here
static const uint8_t target_macs[NUM_TARGETS][6] = {
  {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01},
  {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x02},
  {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x03}
};
static bool found[NUM_TARGETS] = { false };
static int16_t rssi_values[NUM_TARGETS] = { 0 };
static int found_count = 0;

// IEEE 802.11 MAC header for promiscuous parsing
typedef struct {
  uint16_t frame_ctrl;
  uint16_t duration_id;
  uint8_t addr1[6];
  uint8_t addr2[6];
  uint8_t addr3[6];
  uint16_t seq_ctrl;
} wifi_ieee80211_mac_hdr_t;

// Promiscuous packet callback
static void sniffer_callback(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) return;
  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
  wifi_ieee80211_mac_hdr_t *hdr = (wifi_ieee80211_mac_hdr_t *)pkt->payload;
  const uint8_t *mac = hdr->addr2; // transmitter address

  for (int i = 0; i < NUM_TARGETS; i++) {
    if (!found[i] && memcmp(mac, target_macs[i], 6) == 0) {
      found[i] = true;
      rssi_values[i] = pkt->rx_ctrl.rssi;
      found_count++;
      ESP_LOGI(TAG, "Target %d found: %02x:%02x:%02x:%02x:%02x:%02x RSSI=%d dBm",
               i+1,
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
               rssi_values[i]);
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Initializing RSSI sniffer...");

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize network interface and event loop
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // Initialize Wi-Fi in NULL mode for sniffing
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
  ESP_ERROR_CHECK(esp_wifi_start());

  // Set channel and enable promiscuous mode
  ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
  esp_wifi_set_promiscuous_rx_cb(&sniffer_callback);
  esp_wifi_set_promiscuous(true);

  Serial.printf("Sniffer started on channel %d, waiting for %d targets...\n",
                WIFI_CHANNEL, NUM_TARGETS);
}

void loop() {
  static TickType_t start = xTaskGetTickCount();

  if (found_count < NUM_TARGETS) {
    if (xTaskGetTickCount() - start > pdMS_TO_TICKS(SCAN_TIMEOUT_MS)) {
      Serial.printf("Scan timeout (%d ms). Found %d/%d targets\n",
                    SCAN_TIMEOUT_MS, found_count, NUM_TARGETS);
      // disable to save power
      esp_wifi_set_promiscuous(false);
      found_count = NUM_TARGETS; // prevent repeat
    }
    delay(100);
    return;
  }

  Serial.println("RSSI capture complete. Results:");
  for (int i = 0; i < NUM_TARGETS; i++) {
    if (found[i]) {
      Serial.printf("Target %d: %d dBm\n", i+1, rssi_values[i]);
    } else {
      Serial.printf("Target %d: not detected\n", i+1);
    }
  }

  while (true) {
    delay(1000);
  } // halt further processing
}
