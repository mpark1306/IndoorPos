#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <nvs_flash.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_log.h>

// Replace with your Wi-Fi credentials
#define WIFI_SSID  "IoT_H3/4"
#define WIFI_PASS  "98806829"

// Aggregator server details (UDP)
IPAddress aggregatorIP(192, 168, 0, 204);  // TODO: set your server IP
const uint16_t AGG_PORT = 5005;

// Channel to sniff on
static uint8_t sniffChannel = 1;

// Self MAC address buffer
static uint8_t self_mac[6];

WiFiUDP udp;

// IEEE 802.11 MAC header structure
typedef struct {
  uint16_t frame_ctrl;
  uint16_t duration_id;
  uint8_t addr1[6];
  uint8_t addr2[6];  // transmitter
  uint8_t addr3[6];
  uint16_t seq_ctrl;
} wifi_ieee80211_mac_hdr_t;

// Promiscuous callback
static void snifferCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) return;
  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*)buf;
  wifi_ieee80211_mac_hdr_t *hdr = (wifi_ieee80211_mac_hdr_t*)pkt->payload;
  const uint8_t *mac = hdr->addr2;
  // skip self
  if (memcmp(mac, self_mac, 6) == 0) return;
  // format MAC
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  int8_t rssi = pkt->rx_ctrl.rssi;
  // Serial log
  Serial.printf("[Found] %s RSSI=%d dBm\n", macStr, rssi);
  // send UDP JSON
  char json[128];
  snprintf(json, sizeof(json), "{\"anchor\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"target\":\"%s\",\"rssi\":%d}",
           self_mac[0],self_mac[1],self_mac[2],self_mac[3],self_mac[4],self_mac[5],
           macStr, rssi);
  udp.beginPacket(aggregatorIP, AGG_PORT);
  udp.write((uint8_t*)json, strlen(json));
  udp.endPacket();
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Init NVS and TCP/IP
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // Connect to Wi-Fi
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to Wi-Fi '%s'", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.println(" Connected!");

  // Determine home channel
  wifi_second_chan_t sec;
  esp_wifi_get_channel(&sniffChannel, &sec);
  Serial.printf("Sniffing on channel %d\n", sniffChannel);
  esp_wifi_set_channel(sniffChannel, WIFI_SECOND_CHAN_NONE);

  // Read self MAC
  esp_read_mac(self_mac, ESP_MAC_WIFI_STA);
  Serial.printf("Self MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
                self_mac[0],self_mac[1],self_mac[2],self_mac[3],self_mac[4],self_mac[5]);

  // Start UDP
  udp.begin(AGG_PORT);

  // Enable promiscuous sniffing
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&snifferCallback);

  Serial.println("Started sniffer and streaming to aggregator");
}

void loop() {
  delay(1000);
}