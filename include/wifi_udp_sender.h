#pragma once

#include <Arduino.h>
#include "peer_config.h"

#if WIFI_ENABLED

#include <WiFi.h>
#include <WiFiUdp.h>

// =============================================================================
// WiFi UDP Sender for Ranging Data
// =============================================================================

class WiFiUDPSender {
public:
    WiFiUDPSender();
    
    // Initialize WiFi connection
    bool begin();
    
    // Check if connected
    bool isConnected();
    
    // Reconnect if disconnected
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
    WiFiUDP udp;
    IPAddress serverIP;
    bool connected;
    uint32_t last_send_time;
    uint32_t last_reconnect_attempt;
    
    void sendPacket(const char* data);
};

extern WiFiUDPSender wifiSender;

#endif // WIFI_ENABLED
