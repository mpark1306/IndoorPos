#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <esp_now.h>
#include "mbedtls/aes.h"
#include <ArduinoJson.h>

// —— CONFIG ——

// Shared 16-byte AES key (must be identical on *all* your ESP32 mesh nodes)
static const uint8_t meshKey[16] = {
  0x10,0x20,0x30,0x40, 0x50,0x60,0x70,0x80,
  0x90,0xA0,0xB0,0xC0, 0xD0,0xE0,0xF0,0x00
};

// MACs of your three fixed stations + their known (x,y) coords
struct Station { float x, y; };
static constexpr Station stations[3] = {
  { 0.0,  0.0  },  // Station A @ (0,0)
  { 5.0,  0.0  },  // Station B @ (5,0)
  { 2.5,  4.0  }   // Station C @ (2.5,4)
};

// Minimum RSSI to consider
static constexpr int8_t RSSI_THRESHOLD = -80;

// —— HELPERS ——

// AES-ECB encrypt 6-byte MAC → 16-byte block
void encrypt_mac(const uint8_t mac[6], uint8_t out[16]) {
  uint8_t block[16] = {0};
  memcpy(block, mac, 6);
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  mbedtls_aes_setkey_enc(&ctx, meshKey, 128);
  mbedtls_aes_crypt_ecb(&ctx,
                        MBEDTLS_AES_ENCRYPT,
                        block,
                        block);
  memcpy(out, block, 16);
  mbedtls_aes_free(&ctx);
}

// AES-ECB decrypt 16-byte block → 6-byte MAC
void decrypt_mac(const uint8_t in[16], uint8_t mac[6]) {
  uint8_t block[16];
  memcpy(block, in, 16);
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  mbedtls_aes_setkey_dec(&ctx, meshKey, 128);
  mbedtls_aes_crypt_ecb(&ctx,
                        MBEDTLS_AES_DECRYPT,
                        block,
                        block);
  memcpy(mac, block, 6);
  mbedtls_aes_free(&ctx);
}

// Convert RSSI (dBm) → distance (m) via log-path-loss
float rssi_to_distance(int8_t rssi) {
  const float A = -40;    // RSSI@1m (calibrate!)
  const float n = 2.7;    // path-loss exponent (environment)
  return powf(10.0f, (A - (float)rssi) / (10.0f * n));
}

// Trilateration with exactly 3 stations
struct Pos { float x, y; };
Pos trilaterate(float d0, float d1, float d2) {
  // Solve:
  //  (x−x0)²+(y−y0)² = d0²
  //  (x−x1)²+(y−y1)² = d1²
  //  (x−x2)²+(y−y2)² = d2²
  const auto &p0 = stations[0];
  const auto &p1 = stations[1];
  const auto &p2 = stations[2];

  float A = 2*(p1.x - p0.x);
  float B = 2*(p1.y - p0.y);
  float C = d0*d0 - d1*d1 - p0.x*p0.x + p1.x*p1.x - p0.y*p0.y + p1.y*p1.y;
  float D = 2*(p2.x - p0.x);
  float E = 2*(p2.y - p0.y);
  float F = d0*d0 - d2*d2 - p0.x*p0.x + p2.x*p2.x - p0.y*p0.y + p2.y*p2.y;

  float denom = (A*E - B*D);
  if (fabs(denom) < 1e-6) return {NAN, NAN}; // degenerate

  float x = (C*E - B*F) / denom;
  float y = (A*F - C*D) / denom;
  return { x, y };
}

// —— DATA BUFFERS ——

// We store exactly one reading per station per cycle
int8_t  latestRssi[3] = {127,127,127};
bool    gotRssi[3]    = {false,false,false};

// Promiscuous callback
typedef struct {
  uint16_t        frame_ctrl;
  uint16_t        duration_id;
  uint8_t         addr1[6];
  uint8_t         addr2[6];  // source MAC
  uint8_t         addr3[6];
  uint16_t        seq_ctrl;
} __attribute__((packed)) ieee80211_hdr_t;

void IRAM_ATTR sniffer_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
  if (!(type == WIFI_PKT_MGMT || type == WIFI_PKT_DATA))
    return;

  auto *pkt = (wifi_promiscuous_pkt_t*)buf;
  auto *hdr = (ieee80211_hdr_t*)pkt->payload;
  int8_t  rssi = pkt->rx_ctrl.rssi;
  if (rssi < RSSI_THRESHOLD) return;

  // Check if the src MAC matches one of *our* 3 fixed station MACs
  for (int i = 0; i < 3; i++) {
    // TODO: replace these placeholders with your *real* station MACs
    static const uint8_t stationMacs[3][6] = {
      {0x30,0xAE,0xA4,0x11,0x22,0x33}, 
      {0x30,0xAE,0xA4,0x44,0x55,0x66},
      {0x30,0xAE,0xA4,0x77,0x88,0x99}
    };
    if (memcmp(hdr->addr2, stationMacs[i], 6) == 0) {
      latestRssi[i] = rssi;
      gotRssi[i]    = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- Encrypted-Mesh & Trilateration Demo ---");

  // WiFi in STA to sniff, but no connection
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect();

  // Start promiscuous sniffing
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&sniffer_cb);

  // Init ESP-NOW for mesh-wide encrypted broadcast
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed"); while(true) delay(1000);
  }
  // Add *all* peers (here broadcast) with encryption key
  esp_now_peer_info_t peer = {};
  memset(peer.peer_addr, 0xFF, 6);
  peer.encrypt  = true;
  memcpy(peer.lmk, meshKey, 16);
  esp_now_add_peer(&peer);
}

void loop() {
  static uint32_t last = millis();
  if (millis() - last < 1000) return;
  last = millis();

  // Only proceed if we’ve seen all 3 stations this round
  if (gotRssi[0] && gotRssi[1] && gotRssi[2]) {
    // 1) Calculate distances
    float d0 = rssi_to_distance(latestRssi[0]);
    float d1 = rssi_to_distance(latestRssi[1]);
    float d2 = rssi_to_distance(latestRssi[2]);
    Pos p = trilaterate(d0,d1,d2);

    // 2) Prepare JSON
    StaticJsonDocument<128> doc;
    doc["x"] = p.x;
    doc["y"] = p.y;
    char payload[128];
    size_t n = serializeJson(doc, payload);

    // 3) Encrypt & broadcast it
    uint8_t cipher[128];
    // (we’ll do AES-ECB on the JSON directly, zero-padded)
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, meshKey, 128);
    // pad payload to 16-byte blocks
    size_t blocks = (n + 15) / 16;
    for (size_t i = 0; i < blocks; i++) {
      uint8_t block[16] = {0};
      memcpy(block, payload + i*16, min((size_t)16, n - i*16));
      mbedtls_aes_crypt_ecb(&aes,
                            MBEDTLS_AES_ENCRYPT,
                            block,
                            cipher + i*16);
    }
    mbedtls_aes_free(&aes);

    esp_err_t res = esp_now_send(nullptr, cipher, blocks*16);
    Serial.printf("Triangulated (%.2f,%.2f), sent %u bytes → %d\n",
                  p.x, p.y, (unsigned)(blocks*16), res);
  } else {
    Serial.println("Waiting for 3 stations’ RSSI...");
  }

  // Clear for next round
  memset(gotRssi,    0, sizeof(gotRssi));
  // leave latestRssi values as-is (will be overwritten)
}
