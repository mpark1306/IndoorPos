#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <esp_wifi.h>

// Replace with your Wi-Fi credentials
#define WIFI_SSID  "IoT_H3/4"
#define WIFI_PASS  "98806829"

// Heartbeat interval in milliseconds
#define HEARTBEAT_INTERVAL 2000

// List of all ESP32 MAC addresses in the mesh
static const uint8_t mesh_macs[][6] = {
  {0xE8, 0x6B, 0xEA, 0xD4, 0x30, 0x34}, // Device A
  {0xEC, 0x64, 0xC9, 0x85, 0x72, 0xC0}, // Device B
  {0x0C, 0x8B, 0x95, 0x76, 0xAD, 0x20}  // Device C
};

// Self MAC address
static uint8_t self_mac[6];
// Number of peers in mesh
static const size_t NUM_PEERS = sizeof(mesh_macs) / sizeof(mesh_macs[0]);

// Heartbeat payload structure
typedef struct {
  uint32_t ts;
} Heartbeat;

// Callback invoked when a send completes
void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2],
           mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.printf("[Send] To %s -> %s\n", macStr,
                status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Callback invoked when data is received
void onRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2],
           mac_addr[3], mac_addr[4], mac_addr[5]);
  if (len == sizeof(Heartbeat)) {
    Heartbeat hb;
    memcpy(&hb, data, len);
    Serial.printf("[Recv] From %s @ ts=%lu\n", macStr, hb.ts);
  } else {
    Serial.printf("[Recv] %d bytes from %s\n", len, macStr);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  // Connect to Wi-Fi (optional, for logging)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to Wi-Fi '%s'", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println(" Connected!");

    // Determine and set channel for ESP-NOW to match Wi-Fi AP channel
uint8_t primary_chan = 1;
wifi_second_chan_t secondary_chan = WIFI_SECOND_CHAN_NONE;
esp_wifi_get_channel(&primary_chan, &secondary_chan);
Serial.printf("Using channel %d for ESP-NOW\n", primary_chan);
esp_wifi_set_channel(primary_chan, WIFI_SECOND_CHAN_NONE);

  // Read and log own MAC address
  esp_read_mac(self_mac, ESP_MAC_WIFI_STA);
  Serial.printf("[Self] MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                self_mac[0], self_mac[1], self_mac[2],
                self_mac[3], self_mac[4], self_mac[5]);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error: ESP-NOW init failed");
    while (true) { delay(1000); }
  }
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);

  // Add peers (exclude self)
  for (size_t i = 0; i < NUM_PEERS; ++i) {
    if (memcmp(self_mac, mesh_macs[i], 6) == 0) continue;
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, mesh_macs[i], 6);
    peer.channel = primary_chan;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
      Serial.printf("[Peer] Added %02x:%02x:%02x:%02x:%02x:%02x\n",
                    mesh_macs[i][0], mesh_macs[i][1], mesh_macs[i][2],
                    mesh_macs[i][3], mesh_macs[i][4], mesh_macs[i][5]);
    } else {
      Serial.printf("[Peer] Failed to add index %u\n", i);
    }
  }

  Serial.println("[Setup] Ready. Starting heartbeat...");
}

void loop() {
  static uint32_t lastTs = 0;
  uint32_t now = millis();
  if (now - lastTs >= HEARTBEAT_INTERVAL) {
    lastTs = now;
    Heartbeat hb = { .ts = now };
    for (size_t i = 0; i < NUM_PEERS; ++i) {
      if (memcmp(self_mac, mesh_macs[i], 6) == 0) continue;
      esp_err_t res = esp_now_send(mesh_macs[i], (uint8_t*)&hb, sizeof(hb));
      if (res != ESP_OK) {
        Serial.printf("[Error] Send to %u failed\n", i);
      }
    }
  }
  delay(100);
}
