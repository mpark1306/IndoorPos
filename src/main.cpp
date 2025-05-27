#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_log.h>

#define NUM_TARGETS       3
#define WIFI_CHANNEL      1
#define SCAN_TIMEOUT_MS 10000
#define PING_INTERVAL_MS   200

static const char *TAG = "RSSI_MESH";

// Fixed list of all device MACs
static const uint8_t target_macs[NUM_TARGETS][6] = {
  {0xE8, 0x6B, 0xEA, 0xD4, 0x30, 0x34}, // Device A
  {0xEC, 0x64, 0xC9, 0x85, 0x72, 0xC0}, // Device B
  {0x0C, 0x8B, 0x95, 0x76, 0xAD, 0x20}  // Device C
};

// Broadcast address for ESP-NOW
static const uint8_t broadcast_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static bool found[NUM_TARGETS];
static int16_t rssi_values[NUM_TARGETS];
static int found_count;
static uint8_t self_mac[6];
static unsigned long scan_start_ms;

// IEEE 802.11 MAC header for promiscuous parsing
typedef struct {
  uint16_t frame_ctrl;
  uint16_t duration_id;
  uint8_t addr1[6];
  uint8_t addr2[6];
  uint8_t addr3[6];
  uint16_t seq_ctrl;
} wifi_ieee80211_mac_hdr_t;

// Promiscuous callback: pick up all ESP-NOW data frames
static void sniffer_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_DATA) return;
  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
  wifi_ieee80211_mac_hdr_t *hdr = (wifi_ieee80211_mac_hdr_t *)pkt->payload;
  const uint8_t *mac = hdr->addr2;

  for (int i = 0; i < NUM_TARGETS; i++) {
    if (!found[i] && memcmp(mac, target_macs[i], 6) == 0) {
      found[i] = true;
      rssi_values[i] = pkt->rx_ctrl.rssi;
      found_count++;
      ESP_LOGI(TAG, "[%02x:%02x:%02x:%02x:%02x:%02x] found, RSSI=%d dBm", 
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], rssi_values[i]);
      break;
    }
  }
}

static void start_scan() {
  found_count = 0;
  for (int i = 0; i < NUM_TARGETS; i++) {
    if (memcmp(self_mac, target_macs[i], 6) == 0) {
      found[i] = true;
      rssi_values[i] = 0;
      found_count++;
    } else {
      found[i] = false;
      rssi_values[i] = 0;
    }
  }
  scan_start_ms = millis();
  esp_wifi_set_promiscuous(true);
  Serial.printf("[Scan] Started, expecting %d more devices\n", NUM_TARGETS - found_count);
}

static void send_ping() {
  const char *msg = "PING";
  esp_err_t err = esp_now_send(broadcast_mac, (uint8_t *)msg, strlen(msg));
  if (err != ESP_OK) {
    Serial.printf("ESP-NOW send error: %s\n", esp_err_to_name(err));
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Initializing mesh RSSI sniffer...");

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  WiFi.mode(WIFI_MODE_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    while (true) { delay(1000); }
  }
  esp_wifi_set_promiscuous_rx_cb(&sniffer_cb);
  esp_now_register_send_cb(NULL);

  // Register a broadcast peer properly
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  for (size_t idx = 0; idx < sizeof(broadcast_mac); ++idx) {
    peer.peer_addr[idx] = broadcast_mac[idx];
  }
  peer.channel = WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  ESP_ERROR_CHECK(esp_now_add_peer(&peer));

  esp_read_mac(self_mac, ESP_MAC_WIFI_STA);
  Serial.printf("Self MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
                self_mac[0], self_mac[1], self_mac[2],
                self_mac[3], self_mac[4], self_mac[5]);

  start_scan();
}

void loop() {
  static unsigned long last_ping = 0;
  if (millis() - last_ping > PING_INTERVAL_MS) {
    send_ping();
    last_ping = millis();
  }

  if (found_count < NUM_TARGETS) {
    if (millis() - scan_start_ms > SCAN_TIMEOUT_MS) {
      esp_wifi_set_promiscuous(false);
      Serial.printf("[Scan] Timeout %lu ms, found %d/%d\n",
                    SCAN_TIMEOUT_MS, found_count, NUM_TARGETS);
      start_scan();
    }
  } else {
    Serial.println("[Scan] All devices detected! Final RSSI:");
    for (int i = 0; i < NUM_TARGETS; i++) {
      Serial.printf("  %02x:%02x:%02x:%02x:%02x:%02x -> %d dBm\n",
                    target_macs[i][0],target_macs[i][1],target_macs[i][2],
                    target_macs[i][3],target_macs[i][4],target_macs[i][5],
                    rssi_values[i]);
    }
    while (true) { delay(1000); }
  }
}
