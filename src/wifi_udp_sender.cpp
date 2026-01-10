#include "wifi_udp_sender.h"

#if WIFI_ENABLED

WiFiUDPSender wifiSender;

WiFiUDPSender::WiFiUDPSender() {
    connected = false;
    last_send_time = 0;
    last_reconnect_attempt = 0;
}

bool WiFiUDPSender::begin() {
    Serial.println("[WIFI] Connecting to WiFi...");
    Serial.print("[WIFI] SSID: ");
    Serial.println(WIFI_SSID);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    // Wait for connection (max 10 seconds)
    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED && timeout < 20) {
        delay(500);
        Serial.print(".");
        timeout++;
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
        connected = true;
        Serial.println("[WIFI] Connected!");
        Serial.print("[WIFI] IP: ");
        Serial.println(WiFi.localIP());
        
        // Parse server IP
        serverIP.fromString(UDP_SERVER_IP);
        Serial.print("[WIFI] Server: ");
        Serial.print(UDP_SERVER_IP);
        Serial.print(":");
        Serial.println(UDP_SERVER_PORT);
        
        // Start UDP
        udp.begin(UDP_SERVER_PORT + TAG_ID);  // Local port = server port + node ID
        
        return true;
    } else {
        connected = false;
        Serial.println("[WIFI] Connection failed!");
        return false;
    }
}

bool WiFiUDPSender::isConnected() {
    return connected && (WiFi.status() == WL_CONNECTED);
}

void WiFiUDPSender::checkConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        connected = false;
        
        // Try to reconnect every 5 seconds
        if (millis() - last_reconnect_attempt > 5000) {
            last_reconnect_attempt = millis();
            Serial.println("[WIFI] Reconnecting...");
            WiFi.reconnect();
        }
    } else if (!connected) {
        connected = true;
        Serial.println("[WIFI] Reconnected!");
    }
}

void WiFiUDPSender::sendPacket(const char* data) {
    if (!isConnected()) return;
    
    // Rate limiting
    if (millis() - last_send_time < UDP_SEND_INTERVAL_MS) return;
    last_send_time = millis();
    
    udp.beginPacket(serverIP, UDP_SERVER_PORT);
    udp.print(data);
    udp.endPacket();
}

void WiFiUDPSender::sendRangingResult(uint8_t from_id, uint8_t to_id, 
                                       float distance_cm, float rssi, 
                                       uint32_t timestamp) {
    if (!isConnected()) return;
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer), 
             "R,%d,%d,%.2f,%.1f,%lu",
             from_id, to_id, distance_cm, rssi, timestamp);
    
    sendPacket(buffer);
}

void WiFiUDPSender::sendNeighborInfo(uint8_t node_id, uint8_t neighbor_id,
                                      uint8_t hello_count, uint8_t range_pct,
                                      float distance_cm, float rssi) {
    if (!isConnected()) return;
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "N,%d,%d,%d,%d,%.2f,%.1f",
             node_id, neighbor_id, hello_count, range_pct, distance_cm, rssi);
    
    sendPacket(buffer);
}

void WiFiUDPSender::sendHeartbeat(uint8_t node_id, uint32_t frame_num,
                                   uint8_t neighbor_count, uint32_t uptime_ms) {
    if (!isConnected()) return;
    
    char buffer[64];
    snprintf(buffer, sizeof(buffer),
             "H,%d,%lu,%d,%lu",
             node_id, frame_num, neighbor_count, uptime_ms);
    
    sendPacket(buffer);
}

#endif // WIFI_ENABLED
