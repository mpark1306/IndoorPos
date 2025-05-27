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
  {0xE8, 0x6B, 0xEA, 0xD4, 0x30, 0x34}, // Mine
  {0xEC, 0x64, 0xC9, 0x85, 0x72, 0xC0}, // Daniel ESP32 #2
  {0x0C, 0x8B, 0x95, 0x76, 0xAD, 0x20}  // Zain ESP32
};
static bool found[NUM_TARGETS];
static int16_t rssi_values[NUM_TARGETS];
static int found_count;

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
  const uint8_t *mac = hdr->addr2;

  for (int i = 0; i < NUM_TARGETS; i++) {
    if (!found[i] && memcmp(mac, target_macs[i], 6) == 0) {
      found[i] = true;
      rssi_values[i] = pkt->rx_ctrl.rssi;
      found_count++;
      ESP_LOGI(TAG, "Target %d found: %02x:%02x:%02x:%02x:%02x:%02x RSSI=%d dBm",
               i+1, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], rssi_values[i]);
      break;
    }
  }
}

void start_sniffer() {
  // reset state
  memset(found, 0, sizeof(found));
  found_count = 0;
  memset(rssi_values, 0, sizeof(rssi_values));
  // (re)enable promiscuous
  esp_wifi_set_promiscuous(true);
  // log
  Serial.printf("[Scan] Sniffer started on channel %d, targeting %d MACs\n", WIFI_CHANNEL, NUM_TARGETS);
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

  // Initialize Wi-Fi in NULL mode
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
  ESP_ERROR_CHECK(esp_wifi_start());

  // Set channel and callback
  ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
  esp_wifi_set_promiscuous_rx_cb(&sniffer_callback);

  // begin first scan
  start_sniffer();
}

void loop() {
  static TickType_t start_time = 0;
  if (start_time == 0) start_time = xTaskGetTickCount();

  // scanning phase
  if (found_count < NUM_TARGETS) {
    // check timeout
    if (xTaskGetTickCount() - start_time > pdMS_TO_TICKS(SCAN_TIMEOUT_MS)) {
      esp_wifi_set_promiscuous(false);
      Serial.printf("[Scan] Timeout %d ms: found %d/%d targets\n",
                    SCAN_TIMEOUT_MS, found_count, NUM_TARGETS);
      // if none found, restart scan
      if (found_count == 0) {
        Serial.println("[Scan] No targets found, restarting scan");
        start_time = xTaskGetTickCount();
        start_sniffer();
      } else {
        // partial results: report and then restart full scan
        Serial.println("[Scan] Partial results, restarting scan");
        for (int i = 0; i < NUM_TARGETS; i++) {
          if (found[i]) Serial.printf("  Target %d: %d dBm\n", i+1, rssi_values[i]);
          else          Serial.printf("  Target %d: not detected\n", i+1);
        }
        start_time = xTaskGetTickCount();
        start_sniffer();
      }
    }
    delay(100);
    return;
  }

  // all found: final report
  Serial.println("[Scan] All targets found! Results:");
  for (int i = 0; i < NUM_TARGETS; i++) {
    Serial.printf("  Target %d: %d dBm\n", i+1, rssi_values[i]);
  }
  // stop scanning
  esp_wifi_set_promiscuous(false);

  // halt or add further logic
  Serial.println("[Scan] Complete. Halting.");
  while (true) { delay(1000); }
}
