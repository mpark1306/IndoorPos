#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <nvs_flash.h>

// Wi-Fi credentials
const char* WIFI_SSID = "IoT_H3/4";
const char* WIFI_PASS = "98806829";

// Aggregator server details
IPAddress aggregatorIP(192, 168, 1, 100);
const uint16_t AGGREGATOR_PORT = 5005;

// MAC address of the tag device to track
static const uint8_t tag_mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
// (Replace with actual tag MAC)

// Sniffing channel (must match tag TX channel)
#define SNIFF_CHANNEL 1

WiFiUDP udp;
uint8_t self_mac[6];

// Packet header for JSON payload
struct Payload {
  uint8_t anchor_mac[6];
  int16_t rssi;
  unsigned long ts;
};

// Promiscuous packet callback
void snifferCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_DATA) return;
  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*)buf;
  // parse 802.11 header
  const uint8_t *frame = pkt->payload;
  const uint8_t *src = frame + 10; // addr2 offset in MAC header
  if (memcmp(src, tag_mac, 6) != 0) return;

  // fill payload
  Payload p;
  memcpy(p.anchor_mac, self_mac, 6);
  p.rssi = pkt->rx_ctrl.rssi;
  p.ts = millis();

  // send via UDP as JSON
  char json[128];
  snprintf(json, sizeof(json), "{\"anchor\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"rssi\":%d,\"ts\":%lu}",
           p.anchor_mac[0],p.anchor_mac[1],p.anchor_mac[2],
           p.anchor_mac[3],p.anchor_mac[4],p.anchor_mac[5],
           p.rssi, p.ts);

  // Begin UDP packet
  if (udp.beginPacket(aggregatorIP, AGGREGATOR_PORT) != 1) {
    Serial.println("Error: UDP beginPacket failed");
    return;
  }
  udp.write((uint8_t*)json, strlen(json));
  int res = udp.endPacket();
  if (res == 1) {
    Serial.println("Confirmation: Data sent to aggregator");
  } else {
    Serial.println("Error: UDP endPacket failed");
  }
}

void connectWiFi() {
  Serial.printf("Connecting to %s...\n", WIFI_SSID);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println("\nWi-Fi connected");
  WiFi.setAutoReconnect(true);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // init NVS (required for Wi-Fi)
  nvs_flash_init();

  // get own MAC
  esp_read_mac(self_mac, ESP_MAC_WIFI_STA);
  Serial.printf("Anchor MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
                self_mac[0],self_mac[1],self_mac[2],
                self_mac[3],self_mac[4],self_mac[5]);

  // connect to Wi-Fi
  connectWiFi();

  // init UDP
  udp.begin(0);
  // Test UDP connectivity by sending a heartbeat
  {
    const char *testMsg = "{\"type\":\"heartbeat\"}";
    if (udp.beginPacket(aggregatorIP, AGGREGATOR_PORT) == 1) {
      udp.write((uint8_t*)testMsg, strlen(testMsg));
      if (udp.endPacket() == 1) {
        Serial.println("Confirmation: UDP heartbeat sent successfully");
      } else {
        Serial.println("Error: UDP heartbeat send failed");
      }
    } else {
      Serial.println("Error: UDP heartbeat beginPacket failed");
    }
  }

  // configure promiscuous sniffing
  esp_wifi_set_promiscuous_rx_cb(&snifferCallback);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(SNIFF_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println("Started RSSI anchor sniffing");
}

void loop() {
  // nothing here; all handled in callback
  delay(1000);
}