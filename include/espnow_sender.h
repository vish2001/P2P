#pragma once

#include <Arduino.h>
#include "peer_config.h"

#if ESP_NOW_ENABLED

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// =============================================================================
// ESP-NOW Sender for Ranging Data (Drop-in replacement for WiFi UDP)
// =============================================================================

class ESPNowSender {
public:
    ESPNowSender();
    
    // Initialize ESP-NOW connection
    bool begin();
    
    // Check if connected (always true for ESP-NOW once initialized)
    bool isConnected();
    
    // Reconnect if disconnected (no-op for ESP-NOW)
    void checkConnection();
    
    // Send ranging result to server
    // Format: "R,<from_id>,<to_id>,<distance_cm>,<rssi>,<timestamp>"
    void sendRangingResult(uint8_t from_id, uint8_t to_id, 
                           float distance_cm, float rssi, uint32_t timestamp);
    
    // Send neighbor table summary
    // Format: "N,<node_id>,<neighbor_id>,<hello_count>,<range_pct>,<dist>,<rssi>"
    void sendNeighborInfo(uint8_t node_id, uint8_t neighbor_id,
                          uint8_t hello_count, uint8_t range_pct,
                          float distance_cm, float rssi);
    
    // Send heartbeat/status
    // Format: "H,<node_id>,<frame_num>,<neighbor_count>,<uptime_ms>"
    void sendHeartbeat(uint8_t node_id, uint32_t frame_num, 
                       uint8_t neighbor_count, uint32_t uptime_ms);

private:
    bool initialized;
    uint8_t base_mac[6];
    uint32_t last_send_time;
    uint32_t send_count;
    uint32_t fail_count;
    
    void sendPacket(const char* data);
    static void onSent(const uint8_t *mac, esp_now_send_status_t status);
};

// Global instance with same name pattern as WiFi sender
extern ESPNowSender espnowSender;

#endif // ESP_NOW_ENABLED
