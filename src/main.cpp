// =============================================================================
// PEER-TO-PEER UWB MESH SYSTEM
// =============================================================================
// ALL NODES ARE IDENTICAL PEERS. No anchors, no tags, no coordinator.
// Positions are computed centrally as RELATIVE coordinates.
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include "peer_config.h"
#include "DW3000.h"
#include "neighbor_table.h"
#include "tdma_scheduler.h"
#include "range_logger.h"

#if WIFI_ENABLED
#include "wifi_udp_sender.h"
#endif

#if ESP_NOW_ENABLED  
#include "espnow_sender.h"
#endif

// =============================================================================
// GLOBAL STATE
// =============================================================================

// TAG_ID - auto-generated from MAC
uint8_t TAG_ID = 1;

// DW3000 driver
DW3000Class uwb;

// Required by driver
int ANTENNA_DELAY = ANTENNA_DELAY_DEFAULT;
int led_status = 0;
int destination = 0;
int sender = 0;

// State machine
enum class NodeState {
    LISTENING,
    INIT_SEND_POLL,
    INIT_WAIT_RESP,
    INIT_SEND_FINAL,
    INIT_WAIT_REPORT,
    INIT_CALCULATE,
    RESP_SEND_RESP,
    RESP_WAIT_FINAL,
    RESP_SEND_REPORT,
    HELLO_SEND,
};

NodeState state = NodeState::LISTENING;
uint32_t state_entry_time = 0;
uint8_t retry_count = 0;

// Ranging session
struct RangingSession {
    uint8_t target_id;
    long long tx_poll;
    long long rx_resp;
    long long tx_final;
    uint32_t t_round_a;
    uint32_t t_reply_a;
    int clock_offset;
    bool is_initiator;
    long long rx_poll;
    long long tx_resp;
    long long rx_final;
    uint32_t t_round_b;
    uint32_t t_reply_b;
    uint32_t report_t_round_b;
    uint32_t report_t_reply_b;
} session;

uint8_t hello_seq = 0;
uint8_t lbt_retries = 0;

#define BROADCAST_ID 0xFF

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
void initializeUWB();
void resetRadio();
void hardResetDW();
void generateNodeID();
bool performLBT();
bool checkStateTimeout(uint32_t timeout_ms);
void transitionTo(NodeState new_state);
void enterListeningMode();

void handleListeningState();
void handleInitSendPoll();
void handleInitWaitResp();
void handleInitSendFinal();
void handleInitWaitReport();
void handleInitCalculate();
void handleRespSendResp();
void handleRespWaitFinal();
void handleRespSendReport();
void handleHelloSend();

void printStatus();

// =============================================================================
// AUTO-ADDRESSING FROM MAC
// =============================================================================
void generateNodeID() {
#if TAG_ID_OVERRIDE > 0
    TAG_ID = TAG_ID_OVERRIDE;
    Serial.print("[ID] Override: ");
    Serial.println(TAG_ID);
#else
    // Get full MAC address
    uint64_t mac = ESP.getEfuseMac();
    
    // Print MAC for debugging
    Serial.print("[MAC] ");
    for (int i = 0; i < 6; i++) {
        uint8_t byte = (mac >> (i * 8)) & 0xFF;
        if (byte < 0x10) Serial.print("0");
        Serial.print(byte, HEX);
        if (i < 5) Serial.print(":");
    }
    Serial.println();
    
    // FNV-1a hash for better distribution
    uint32_t hash = 2166136261;  // FNV offset basis
    for (int i = 0; i < 6; i++) {
        uint8_t byte = (mac >> (i * 8)) & 0xFF;
        hash ^= byte;
        hash *= 16777619;  // FNV prime
    }
    
    // Map to 1-254 range
    uint8_t id = (hash % 253) + 1;  // Results in 1-253
    
    TAG_ID = id;
    Serial.print("[ID] Auto-generated: ");
    Serial.println(TAG_ID);
#endif
    
    Serial.print("[SLOT] ");
    Serial.print(TAG_ID % NUM_SLOTS);
    Serial.print(" / ");
    Serial.println(NUM_SLOTS - 1);
    
    scheduler.recalculateSlot();
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);
    
    Serial.println("\n==========================================");
    Serial.println("  P2P UWB MESH - All Nodes Equal");
    Serial.println("==========================================");
    
    generateNodeID();
    
    Serial.print("Slot: ");
    Serial.print(scheduler.getMySlot());
    Serial.print("/");
    Serial.print(NUM_SLOTS - 1);
    Serial.print(" | Frame: ");
    Serial.print(FRAME_LENGTH_MS);
    Serial.println("ms");
    Serial.println("==========================================\n");
    
    randomSeed(TAG_ID * 1000 + millis());
    
#if ESP_NOW_ENABLED
    Serial.println("[COMM] ESP-NOW init...");
    espnowSender.begin();
    if (REDUCE_CPU_FREQ) {
        setCpuFrequencyMhz(CPU_FREQ_NORMAL);
    }
#elif WIFI_ENABLED
    Serial.println("[COMM] WiFi init...");
    wifiSender.begin();
#endif
    
    initializeUWB();
    
    rangeLogger.setVerbose(true);
    rangeLogger.setJsonOutput(false);
    
    enterListeningMode();
    
    Serial.println("[MESH] Ready\n");
}

// =============================================================================
// BROADCAST NEIGHBOR TABLE TO BASE STATION
// =============================================================================
void broadcastNeighborTable() {
    uint8_t count = neighborTable.getActiveCount();
    
    // Send heartbeat first
#if ESP_NOW_ENABLED
    espnowSender.sendHeartbeat(TAG_ID, scheduler.getFrameNumber(), count, millis());
#elif WIFI_ENABLED
    wifiSender.sendHeartbeat(TAG_ID, scheduler.getFrameNumber(), count, millis());
#endif
    
    // Get neighbor array and iterate through active entries
    const Neighbor* neighbors = neighborTable.getNeighborArray();
    
    for (uint8_t i = 0; i < MAX_NEIGHBORS; i++) {
        const Neighbor& n = neighbors[i];
        if (!n.active) continue;
        
        // Calculate ranging success percentage
        uint8_t range_pct = 0;
        if (n.ranging_attempts > 0) {
            range_pct = (n.ranging_successes * 100) / n.ranging_attempts;
        }
        
#if ESP_NOW_ENABLED
        espnowSender.sendNeighborInfo(
            TAG_ID,
            n.id,
            n.hello_count,
            range_pct,
            n.filtered_distance_cm,
            n.avg_rssi
        );
#elif WIFI_ENABLED
        wifiSender.sendNeighborInfo(
            TAG_ID,
            n.id,
            n.hello_count,
            range_pct,
            n.filtered_distance_cm,
            n.avg_rssi
        );
#endif
        
        delay(5);  // Small delay between packets to avoid flooding
    }
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
#if ESP_NOW_ENABLED
    static uint32_t last_check = 0;
    if (millis() - last_check > 5000) {
        last_check = millis();
        espnowSender.checkConnection();
    }
#elif WIFI_ENABLED
    static uint32_t last_check = 0;
    if (millis() - last_check > 1000) {
        last_check = millis();
        wifiSender.checkConnection();
    }
#endif
    
    scheduler.update();
    
    switch (state) {
        case NodeState::LISTENING:      handleListeningState(); break;
        case NodeState::INIT_SEND_POLL: handleInitSendPoll(); break;
        case NodeState::INIT_WAIT_RESP: handleInitWaitResp(); break;
        case NodeState::INIT_SEND_FINAL: handleInitSendFinal(); break;
        case NodeState::INIT_WAIT_REPORT: handleInitWaitReport(); break;
        case NodeState::INIT_CALCULATE: handleInitCalculate(); break;
        case NodeState::RESP_SEND_RESP: handleRespSendResp(); break;
        case NodeState::RESP_WAIT_FINAL: handleRespWaitFinal(); break;
        case NodeState::RESP_SEND_REPORT: handleRespSendReport(); break;
        case NodeState::HELLO_SEND:     handleHelloSend(); break;
        default:
            enterListeningMode();
            break;
    }
    
    // Status every 10s
    static uint32_t last_status = 0;
    if (millis() - last_status > 10000) {
        printStatus();
        last_status = millis();
    }
    
    // Mesh report every 30s
    static uint32_t last_mesh = 0;
    if (millis() - last_mesh > 30000) {
        neighborTable.printMeshStatus();
        last_mesh = millis();
    }
    
    // Send neighbor table to base station every 5s
    static uint32_t last_neighbor_broadcast = 0;
    if (millis() - last_neighbor_broadcast > 5000) {
        broadcastNeighborTable();
        last_neighbor_broadcast = millis();
    }
    
    // Flush logs
    static uint32_t last_flush = 0;
    if (millis() - last_flush > 1000) {
        rangeLogger.flushToSerial();
        last_flush = millis();
    }
}

// =============================================================================
// LISTEN-BEFORE-TALK
// =============================================================================
#if LBT_ENABLED
bool performLBT() {
    uwb.clearSystemStatus();
    uwb.standardRX();
    
    uint32_t start = millis();
    while (millis() - start < LBT_LISTEN_MS) {
        if (uwb.receivedFrameSucc() == 1) {
            uwb.clearSystemStatus();
            return false;  // Channel busy
        }
        delayMicroseconds(100);
    }
    return true;  // Channel clear
}
#else
bool performLBT() { return true; }
#endif

// =============================================================================
// LISTENING STATE
// =============================================================================
void handleListeningState() {
    // Send HELLO if due
    if (scheduler.shouldSendHello()) {
        transitionTo(NodeState::HELLO_SEND);
        return;
    }
    
    // Initiate ranging in my slot
    if (scheduler.canInitiateRanging()) {
        uint8_t target = neighborTable.getNextRangingTarget();
        if (target != 0) {
            session.target_id = target;
            session.is_initiator = true;
            retry_count = 0;
            lbt_retries = 0;
            
            Serial.print("[RANGE] -> ");
            Serial.println(target);
            
            transitionTo(NodeState::INIT_SEND_POLL);
            return;
        }
    }
    
    // Process incoming
    int rx = uwb.receivedFrameSucc();
    
    if (rx == 1) {
        uint8_t mode = uwb.read(RX_BUFFER_0_REG, 0x00) & 0x07;
        uint8_t sender_id = uwb.read(RX_BUFFER_0_REG, 0x01) & 0xFF;
        uint8_t dest_id = uwb.read(RX_BUFFER_0_REG, 0x02) & 0xFF;
        uint8_t stage = uwb.read(RX_BUFFER_0_REG, 0x03) & 0x07;
        
        // Mode=1 is DS-TWR format (used for both HELLO and ranging)
        if (mode == 1) {
            // Stage 0 = HELLO beacon
            if (stage == 0 && dest_id == BROADCAST_ID) {
                neighborTable.processHello(sender_id, 0, neighborTable.getActiveCount());
                
                // SLOT COLLISION DETECTION
                uint8_t sender_slot = sender_id % NUM_SLOTS;
                uint8_t my_slot = TAG_ID % NUM_SLOTS;
                if (sender_slot == my_slot) {
                    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                    Serial.print("!! SLOT COLLISION: Node ");
                    Serial.print(sender_id);
                    Serial.print(" has same slot ");
                    Serial.println(my_slot);
                    Serial.println("!! Consider using -DTAG_ID_OVERRIDE=X");
                    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                }
                
#if SYNC_TO_LOWER_ID
                if (sender_id < TAG_ID) {
                    uint8_t sender_slot = sender_id % NUM_SLOTS;
                    uint32_t now = millis();
                    uint32_t frame_start = now - (sender_slot * SLOT_LENGTH_MS) - (SLOT_LENGTH_MS / 2);
                    scheduler.syncFrameStart(frame_start);
                }
#endif
                
                uwb.clearSystemStatus();
                uwb.standardRX();
                return;
            }
            
            // Stage 1 = POLL (someone wants to range with us)
            if (stage == STAGE_POLL && (dest_id == TAG_ID || dest_id == BROADCAST_ID)) {
                session.target_id = sender_id;
                session.is_initiator = false;
                session.rx_poll = uwb.readRXTimestamp();
                
                uwb.clearSystemStatus();
                
                Serial.print("[POLL] <- ");
                Serial.println(sender_id);
                
                scheduler.enterRespondingMode();
                transitionTo(NodeState::RESP_SEND_RESP);
                return;
            }
        }
        
        uwb.clearSystemStatus();
        uwb.standardRX();
    } else if (rx == 2) {
        uwb.clearSystemStatus();
        uwb.standardRX();
    }
}

// =============================================================================
// INITIATOR STATES
// =============================================================================
void handleInitSendPoll() {
    // Jitter
    if (scheduler.shouldAddJitter()) {
        delay(scheduler.getJitterDelay());
    }
    
    // LBT
#if LBT_ENABLED
    if (!performLBT()) {
        lbt_retries++;
        if (lbt_retries >= LBT_MAX_RETRIES) {
            Serial.println("[LBT] Busy, abort");
            scheduler.reportCollision();
            neighborTable.recordCollision(session.target_id);
            enterListeningMode();
            return;
        }
        delay(random(LBT_BACKOFF_BASE_MS, LBT_BACKOFF_MAX_MS));
        return;
    }
#endif
    
    uwb.clearSystemStatus();
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(session.target_id);
    
    if (REDUCE_CPU_FREQ) setCpuFrequencyMhz(CPU_FREQ_HIGH);
    
    uwb.ds_sendFrame(STAGE_POLL);
    session.tx_poll = uwb.readTXTimestamp();
    
    transitionTo(NodeState::INIT_WAIT_RESP);
}

void handleInitWaitResp() {
    if (checkStateTimeout(RESPONSE_TIMEOUT_MS)) {
        retry_count++;
        if (retry_count < MAX_RANGING_RETRIES) {
            transitionTo(NodeState::INIT_SEND_POLL);
        } else {
            LOG_RANGE_FAILURE(TAG_ID, session.target_id, scheduler.getFrameNumber(), "RESP_TIMEOUT");
            neighborTable.recordRangingFailure(session.target_id);
            enterListeningMode();
        }
        return;
    }
    
    int rx = uwb.receivedFrameSucc();
    if (rx == 1) {
        uint8_t mode = uwb.read(RX_BUFFER_0_REG, 0x00) & 0x07;
        uint8_t sender_id = uwb.read(RX_BUFFER_0_REG, 0x01) & 0xFF;
        uint8_t stage = uwb.read(RX_BUFFER_0_REG, 0x03) & 0x07;
        unsigned long long rx_ts = uwb.readRXTimestamp();
        
        uwb.clearSystemStatus();
        
        if (mode == 1 && sender_id == session.target_id && stage == STAGE_RESP) {
            session.rx_resp = rx_ts;
            session.t_round_a = session.rx_resp - session.tx_poll;
            transitionTo(NodeState::INIT_SEND_FINAL);
        } else {
            uwb.standardRX();
        }
    } else if (rx == 2) {
        uwb.clearSystemStatus();
        uwb.standardRX();
    }
}

void handleInitSendFinal() {
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(session.target_id);
    
    uwb.ds_sendFrame(STAGE_FINAL);
    
    session.tx_final = uwb.readTXTimestamp();
    session.t_reply_a = session.tx_final - session.rx_resp;
    
    transitionTo(NodeState::INIT_WAIT_REPORT);
}

void handleInitWaitReport() {
    if (checkStateTimeout(RESPONSE_TIMEOUT_MS)) {
        LOG_RANGE_FAILURE(TAG_ID, session.target_id, scheduler.getFrameNumber(), "REPORT_TIMEOUT");
        neighborTable.recordRangingFailure(session.target_id);
        enterListeningMode();
        return;
    }
    
    int rx = uwb.receivedFrameSucc();
    if (rx == 1) {
        uint8_t mode = uwb.read(RX_BUFFER_0_REG, 0x00) & 0x07;
        uint8_t sender_id = uwb.read(RX_BUFFER_0_REG, 0x01) & 0xFF;
        uint8_t stage = uwb.read(RX_BUFFER_0_REG, 0x03) & 0x07;
        
        if (mode == 1 && sender_id == session.target_id && stage == STAGE_REPORT) {
            session.report_t_round_b = uwb.read(RX_BUFFER_0_REG, 0x04);
            session.report_t_reply_b = uwb.read(RX_BUFFER_0_REG, 0x08);
            session.clock_offset = uwb.getRawClockOffset();
            
            uwb.clearSystemStatus();
            transitionTo(NodeState::INIT_CALCULATE);
        } else {
            uwb.clearSystemStatus();
            uwb.standardRX();
        }
    } else if (rx == 2) {
        uwb.clearSystemStatus();
        uwb.standardRX();
    }
}

void handleInitCalculate() {
    int ranging_time = uwb.ds_processRTInfo(
        session.t_round_a, session.t_reply_a,
        session.report_t_round_b, session.report_t_reply_b,
        session.clock_offset
    );
    
    float distance_cm = uwb.convertToCM(ranging_time);
    float rssi = uwb.getSignalStrength();
    float fp_rssi = uwb.getFirstPathSignalStrength();
    
    LOG_RANGE_SUCCESS(TAG_ID, session.target_id, distance_cm, rssi, fp_rssi, 
                      scheduler.getFrameNumber());
    neighborTable.recordRangingSuccess(session.target_id, distance_cm, rssi);
    
    // Get FILTERED distance to send (raw values can be noisy/negative)
    Neighbor* n = neighborTable.getNeighbor(session.target_id);
    float filtered_dist = (n && n->filtered_distance_cm > 0) ? n->filtered_distance_cm : distance_cm;
    
#if ESP_NOW_ENABLED
    espnowSender.sendRangingResult(TAG_ID, session.target_id, filtered_dist, rssi, millis());
#elif WIFI_ENABLED
    wifiSender.sendRangingResult(TAG_ID, session.target_id, filtered_dist, rssi, millis());
#endif
    
    Serial.print("[DIST] ");
    Serial.print(TAG_ID);
    Serial.print("->");
    Serial.print(session.target_id);
    Serial.print(": ");
    Serial.print(distance_cm, 1);
    Serial.print(" cm (filtered: ");
    Serial.print(filtered_dist, 1);
    Serial.println(" cm)");
    
    enterListeningMode();
}

// =============================================================================
// RESPONDER STATES
// =============================================================================
void handleRespSendResp() {
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(session.target_id);
    
    uwb.ds_sendFrame(STAGE_RESP);
    
    session.tx_resp = uwb.readTXTimestamp();
    session.t_reply_b = session.tx_resp - session.rx_poll;
    
    transitionTo(NodeState::RESP_WAIT_FINAL);
}

void handleRespWaitFinal() {
    if (checkStateTimeout(RESPONSE_TIMEOUT_MS)) {
        scheduler.exitRespondingMode();
        enterListeningMode();
        return;
    }
    
    int rx = uwb.receivedFrameSucc();
    if (rx == 1) {
        uint8_t mode = uwb.read(RX_BUFFER_0_REG, 0x00) & 0x07;
        uint8_t sender_id = uwb.read(RX_BUFFER_0_REG, 0x01) & 0xFF;
        uint8_t stage = uwb.read(RX_BUFFER_0_REG, 0x03) & 0x07;
        unsigned long long rx_ts = uwb.readRXTimestamp();
        
        uwb.clearSystemStatus();
        
        if (mode == 1 && sender_id == session.target_id && stage == STAGE_FINAL) {
            session.rx_final = rx_ts;
            session.t_round_b = session.rx_final - session.tx_resp;
            transitionTo(NodeState::RESP_SEND_REPORT);
        } else {
            uwb.standardRX();
        }
    } else if (rx == 2) {
        uwb.clearSystemStatus();
        uwb.standardRX();
    }
}

void handleRespSendReport() {
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(session.target_id);
    
    uwb.ds_sendRTInfo(session.t_round_b, session.t_reply_b);
    
    scheduler.exitRespondingMode();
    enterListeningMode();
}

// =============================================================================
// HELLO BEACON
// =============================================================================
void handleHelloSend() {
    delay(random(0, 50));
    
    uwb.clearSystemStatus();
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(BROADCAST_ID);
    
    // Use ds_sendFrame with stage=0 for HELLO (same code path as ranging)
    uwb.ds_sendFrame(0);  // Stage 0 = HELLO
    
    scheduler.markHelloSent();
    
    if (DEBUG_OUTPUT) {
        Serial.print("[HELLO] seq=");
        Serial.println(hello_seq);
    }
    hello_seq++;
    
    enterListeningMode();
}

// =============================================================================
// HELPERS
// =============================================================================
void transitionTo(NodeState new_state) {
    state = new_state;
    state_entry_time = millis();
}

bool checkStateTimeout(uint32_t timeout_ms) {
    return (millis() - state_entry_time) > timeout_ms;
}

void enterListeningMode() {
    state = NodeState::LISTENING;
    state_entry_time = millis();
    uwb.clearSystemStatus();
    uwb.standardRX();
    
    if (REDUCE_CPU_FREQ) {
        setCpuFrequencyMhz(CPU_FREQ_NORMAL);
    }
}

// =============================================================================
// UWB INIT
// =============================================================================
void initializeUWB() {
    Serial.println("[UWB] Init...");
    
    uwb.begin();
    hardResetDW();
    delay(200);
    
    if (!uwb.checkSPI()) {
        Serial.println("[FATAL] SPI fail");
        while (1) delay(1000);
    }
    
    int timeout = 100;
    while (!uwb.checkForIDLE() && timeout-- > 0) delay(100);
    if (timeout <= 0) {
        Serial.println("[FATAL] IDLE timeout");
        while (1) delay(1000);
    }
    
    uwb.softReset();
    delay(200);
    
    if (!uwb.checkForIDLE()) {
        Serial.println("[FATAL] IDLE fail");
        while (1) delay(1000);
    }
    
    uwb.init();
    uwb.setupGPIO();
    uwb.setTXAntennaDelay(ANTENNA_DELAY_DEFAULT);
    uwb.setSenderID(TAG_ID);
    uwb.configureAsTX();
    uwb.clearSystemStatus();
    
    Serial.println("[UWB] Ready");
}

void resetRadio() {
    uwb.softReset();
    delay(100);
    uwb.init();
    uwb.setTXAntennaDelay(ANTENNA_DELAY_DEFAULT);
    uwb.setSenderID(TAG_ID);
    uwb.clearSystemStatus();
    uwb.configureAsTX();
    uwb.standardRX();
}

void hardResetDW() {
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    delay(3);
    digitalWrite(PIN_RST, HIGH);
    delay(5);
}

// =============================================================================
// STATUS
// =============================================================================
void printStatus() {
    Serial.println("\n======== MESH NODE ========");
    Serial.print("ID: ");
    Serial.print(TAG_ID);
    Serial.print(" | Slot: ");
    Serial.print(scheduler.getMySlot());
    Serial.print("/");
    Serial.print(NUM_SLOTS - 1);
    Serial.print(" | Frame: ");
    Serial.println(scheduler.getFrameNumber());
    
    Serial.print("Neighbors: ");
    Serial.print(neighborTable.getActiveCount());
    Serial.print(" | Eligible: ");
    Serial.println(neighborTable.getEligibleCount());
    
    neighborTable.printTable();
    Serial.println("===========================\n");
}
