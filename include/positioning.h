#pragma once

struct Position {
    float x;
    float y;
};

struct RSSIReading {
    uint8_t mac[6];
    int rssi;
    uint32_t timestamp;
};

// Triangulering og hjælpe-funktioner
Position triangulate(const RSSIReading readings[], size_t count);
void clear_readings();
