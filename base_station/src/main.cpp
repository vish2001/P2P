// =============================================================================
// UWB MESH BASE STATION - ESP-NOW Receiver
// =============================================================================
// Receives ranging data from mesh nodes via ESP-NOW broadcast.
// Forwards to PC via Serial (USB) or WiFi UDP.
// 
// Deploy multiple base stations around the area for full coverage.
// Each base station forwards what it receives to a central aggregator.
//
// DATA FORMAT (to PC):
//   All lines prefixed with timestamp from base station boot time
//   T,<timestamp_ms>,S,<station_id>,R,<from>,<to>,<distance>,<rssi>,<node_uptime>
//   T,<timestamp_ms>,S,<station_id>,N,<node>,<neighbor>,<hello>,<range_pct>,<dist>,<rssi>
//   T,<timestamp_ms>,S,<station_id>,H,<node>,<frame>,<neighbors>,<uptime>
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

// Output mode: Serial, UDP, or both
#define OUTPUT_SERIAL    1
#define OUTPUT_UDP       0

// UDP settings (if OUTPUT_UDP enabled)
#define UDP_TARGET_IP    "192.168.1.100"
#define UDP_TARGET_PORT  5000

// Station ID (set unique for each base station)
#define STATION_ID       1

// WiFi settings (only needed for UDP output)
#define WIFI_SSID        "YourNetwork"
#define WIFI_PASSWORD    "YourPassword"

// CSV logging
#define LOG_CSV_FORMAT   1  // Output in CSV format for easy parsing

// =============================================================================
// GLOBALS
// =============================================================================

#if OUTPUT_UDP
#include <WiFiUdp.h>
WiFiUDP udp;
#endif

uint32_t packets_received = 0;
uint32_t last_status_time = 0;
uint32_t boot_time_ms = 0;

// =============================================================================
// ESP-NOW CALLBACK
// =============================================================================

void onDataReceived(const uint8_t* mac, const uint8_t* data, int len) {
    packets_received++;
    
    if (len < 2) return;
    
    // Get timestamp immediately
    uint32_t timestamp_ms = millis();
    
    // Parse incoming data (text format from nodes)
    char incoming[256];
    memcpy(incoming, data, min(len, 255));
    incoming[min(len, 255)] = '\0';
    
    char output[512];
    
#if LOG_CSV_FORMAT
    // CSV format: timestamp,station_id,type,data...
    snprintf(output, sizeof(output), "%lu,%d,%s",
             timestamp_ms, STATION_ID, incoming);
#else
    // Verbose format
    snprintf(output, sizeof(output), "T,%lu,S,%d,%s",
             timestamp_ms, STATION_ID, incoming);
#endif
    
#if OUTPUT_SERIAL
    Serial.println(output);
#endif

#if OUTPUT_UDP
    udp.beginPacket(UDP_TARGET_IP, UDP_TARGET_PORT);
    udp.print(output);
    udp.endPacket();
#endif
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(921600);
    delay(1000);
    
    boot_time_ms = millis();
    
    Serial.println();
    Serial.println("# ==========================================");
    Serial.println("#   UWB MESH BASE STATION");
    Serial.print("#   Station ID: ");
    Serial.println(STATION_ID);
    Serial.println("# ==========================================");
    
    // Initialize WiFi in station mode (required for ESP-NOW)
    WiFi.mode(WIFI_STA);
    
    // Print MAC address
    Serial.print("# MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.println("#");
    
#if LOG_CSV_FORMAT
    Serial.println("# CSV Format: timestamp_ms,station_id,type,data...");
    Serial.println("# Types: R=Ranging, N=Neighbor, H=Heartbeat");
    Serial.println("#");
    Serial.println("# R format: timestamp,station,R,from_id,to_id,distance_cm,rssi,node_uptime");
    Serial.println("# N format: timestamp,station,N,node_id,neighbor_id,hello_count,range_pct,distance_cm,rssi");
    Serial.println("# H format: timestamp,station,H,node_id,frame_num,neighbor_count,uptime_ms");
    Serial.println("#");
#endif
    
#if OUTPUT_UDP
    // Connect to WiFi for UDP output
    Serial.print("# [WIFI] Connecting to ");
    Serial.println(WIFI_SSID);
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int timeout = 20;
    while (WiFi.status() != WL_CONNECTED && timeout > 0) {
        delay(500);
        Serial.print(".");
        timeout--;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.println("# [WIFI] Connected!");
        Serial.print("# [WIFI] IP: ");
        Serial.println(WiFi.localIP());
        
        udp.begin(UDP_TARGET_PORT);
    } else {
        Serial.println();
        Serial.println("# [WIFI] Connection failed - UDP disabled");
    }
#endif
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("# [ESP-NOW] Init failed!");
        return;
    }
    
    // Register receive callback
    esp_now_register_recv_cb(onDataReceived);
    
    Serial.println("# [ESP-NOW] Ready - listening for mesh data...");
    Serial.println("# ==========================================");
    Serial.println();
    
    last_status_time = millis();
}

// =============================================================================
// LOOP
// =============================================================================

void loop() {
    // Print status every 30 seconds (as comment so it doesn't interfere with CSV)
    if (millis() - last_status_time > 30000) {
        Serial.print("# [STATUS] Packets received: ");
        Serial.print(packets_received);
        Serial.print(", Uptime: ");
        Serial.print((millis() - boot_time_ms) / 1000);
        Serial.println("s");
        last_status_time = millis();
    }
    
    delay(10);
}
