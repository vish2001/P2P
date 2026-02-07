#include "espnow_sender.h"

#if ESP_NOW_ENABLED

// Global instance
ESPNowSender espnowSender;

// Broadcast MAC - all base stations in range will receive
static const uint8_t BROADCAST_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Static callback tracking
static volatile uint32_t _send_success = 0;
static volatile uint32_t _send_fail = 0;

ESPNowSender::ESPNowSender() {
    initialized = false;
    last_send_time = 0;
    send_count = 0;
    fail_count = 0;
    
    // Use broadcast MAC for multi-base-station support
    memcpy(base_mac, BROADCAST_MAC, 6);
}

bool ESPNowSender::begin() {
    Serial.println("[ESP-NOW] Initializing (broadcast mode)...");
    
    // Set WiFi mode to station
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // Print our MAC
    Serial.print("[ESP-NOW] Node MAC: ");
    Serial.println(WiFi.macAddress());
    
    // Set to channel 1 (ESP-NOW works on same channel as WiFi)
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init failed!");
        return false;
    }
    
    // Register callback
    esp_now_register_send_cb(onSent);
    
    // Add broadcast peer - this allows sending to all base stations
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel = 0;  // 0 = use current channel
    peer.encrypt = false;
    
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[ESP-NOW] Failed to add broadcast peer");
        return false;
    }
    
    // Reduce WiFi TX power for energy saving
    esp_wifi_set_max_tx_power(50); // 12.5 dBm instead of 20 dBm
    
    initialized = true;
    Serial.println("[ESP-NOW] Ready (broadcasting to all base stations)");
    return true;
}

bool ESPNowSender::isConnected() {
    return initialized;
}

void ESPNowSender::checkConnection() {
    // ESP-NOW doesn't need reconnection like WiFi
    if (!initialized) {
        begin();
    }
}

void ESPNowSender::sendRangingResult(uint8_t from_id, uint8_t to_id, 
                                      float distance_cm, float rssi, uint32_t timestamp) {
    if (!initialized) return;
    
    // Rate limiting
    if (millis() - last_send_time < ESP_NOW_MIN_INTERVAL) return;
    
    // Format same as WiFi UDP for compatibility with receiver
    char buffer[250];
    snprintf(buffer, sizeof(buffer), "R,%d,%d,%.1f,%.1f,%lu", 
             from_id, to_id, distance_cm, rssi, timestamp);
    
    sendPacket(buffer);
}

void ESPNowSender::sendNeighborInfo(uint8_t node_id, uint8_t neighbor_id,
                                     uint8_t hello_count, uint8_t range_pct,
                                     float distance_cm, float rssi) {
    if (!initialized) return;
    
    char buffer[250];
    snprintf(buffer, sizeof(buffer), "N,%d,%d,%d,%d,%.1f,%.1f",
             node_id, neighbor_id, hello_count, range_pct, distance_cm, rssi);
    
    sendPacket(buffer);
}

void ESPNowSender::sendHeartbeat(uint8_t node_id, uint32_t frame_num, 
                                  uint8_t neighbor_count, uint32_t uptime_ms) {
    if (!initialized) return;
    
    char buffer[250];
    snprintf(buffer, sizeof(buffer), "H,%d,%lu,%d,%lu",
             node_id, frame_num, neighbor_count, uptime_ms);
    
    sendPacket(buffer);
}

void ESPNowSender::sendPacket(const char* data) {
    // ESP-NOW max payload is 250 bytes
    int len = strlen(data);
    if (len > 249) len = 249;
    
    esp_err_t result = esp_now_send(base_mac, (uint8_t*)data, len + 1);
    
    if (result == ESP_OK) {
        send_count++;
        last_send_time = millis();
    } else {
        fail_count++;
        if (DEBUG_OUTPUT) {
            Serial.print("[ESP-NOW] Send failed: ");
            Serial.println(result);
        }
    }
}

void ESPNowSender::onSent(const uint8_t *mac, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        _send_success++;
    } else {
        _send_fail++;
    }
}

#endif // ESP_NOW_ENABLED
