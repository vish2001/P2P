// =============================================================================
// PEER-TO-PEER UWB RANGING SYSTEM
// =============================================================================
// Unified firmware for all nodes. Each node is an identical peer.
// No anchors, no tags, no coordinator.
//
// Key behaviors:
// - Deterministic TDMA slot: my_slot = TAG_ID % NUM_SLOTS
// - In MY slot: initiate DS-TWR to one K-neighbor
// - In OTHER slots: respond to incoming DS-TWR requests
// - HELLO beacons broadcast every ~2s for neighbor discovery
// - Frame sync: align timing to lower-ID neighbors
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include "peer_config.h"
#include "DW3000.h"
#include "neighbor_table.h"
#include "tdma_scheduler.h"
#include "range_logger.h"

// =============================================================================
// GLOBAL STATE
// =============================================================================

// DW3000 driver instance
DW3000Class uwb;

// Global variables required by DW3000 driver
int ANTENNA_DELAY = ANTENNA_DELAY_DEFAULT;
int led_status = 0;
int destination = 0;
int sender = 0;

// State machine states
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

// Current state
NodeState state = NodeState::LISTENING;

// Timing for state machine
uint32_t state_entry_time = 0;
uint8_t retry_count = 0;

// Current ranging session data
struct RangingSession {
    uint8_t target_id;
    long long tx_poll;
    long long rx_resp;
    long long tx_final;
    int t_round_a;
    int t_reply_a;
    int clock_offset;
    bool is_initiator;
    long long rx_poll;
    long long tx_resp;
    long long rx_final;
    int t_round_b;
    int t_reply_b;
} session;

// HELLO beacon sequence number
uint8_t hello_seq = 0;

// =============================================================================
// FRAME FORMAT
// =============================================================================
// We use the DW3000 driver's existing frame structure:
// Byte 0: Mode/Type (we use 0 for standard, 1 for DS-TWR)
// Byte 1: Sender ID
// Byte 2: Destination ID
// Byte 3: Stage/Sequence

#define BROADCAST_ID         0xFF

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================
void initializeUWB();
void resetRadio();
void hardResetDW();

void processReceivedFrame();
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
// SETUP
// =============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);  // Wait for serial
    
    Serial.println("\n==========================================");
    Serial.println("  PEER-TO-PEER UWB RANGING SYSTEM v2");
    Serial.println("==========================================");
    Serial.print("Node ID: ");
    Serial.println(TAG_ID);
    Serial.print("TDMA Slot: ");
    Serial.print(MY_SLOT);
    Serial.print(" / ");
    Serial.println(NUM_SLOTS - 1);
    Serial.print("Debug: ");
    Serial.println(DEBUG_OUTPUT ? "ON" : "OFF");
    Serial.println("==========================================\n");
    
    // Initialize UWB hardware
    initializeUWB();
    
    // Configure logger
    rangeLogger.setVerbose(true);
    rangeLogger.setJsonOutput(false);
    
    // Start in listening mode
    enterListeningMode();
    
    Serial.println("[INFO] Setup complete. Starting main loop.\n");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
    // Update scheduler timing
    scheduler.update();
    
    // State machine
    switch (state) {
        case NodeState::LISTENING:
            handleListeningState();
            break;
        case NodeState::INIT_SEND_POLL:
            handleInitSendPoll();
            break;
        case NodeState::INIT_WAIT_RESP:
            handleInitWaitResp();
            break;
        case NodeState::INIT_SEND_FINAL:
            handleInitSendFinal();
            break;
        case NodeState::INIT_WAIT_REPORT:
            handleInitWaitReport();
            break;
        case NodeState::INIT_CALCULATE:
            handleInitCalculate();
            break;
        case NodeState::RESP_SEND_RESP:
            handleRespSendResp();
            break;
        case NodeState::RESP_WAIT_FINAL:
            handleRespWaitFinal();
            break;
        case NodeState::RESP_SEND_REPORT:
            handleRespSendReport();
            break;
        case NodeState::HELLO_SEND:
            handleHelloSend();
            break;
        default:
            Serial.println("[ERROR] Unknown state!");
            enterListeningMode();
            break;
    }
    
    // Periodic status output (every 5 seconds)
    static uint32_t last_status = 0;
    if (millis() - last_status > 5000) {
        printStatus();
        last_status = millis();
    }
    
    // Flush logs periodically
    static uint32_t last_flush = 0;
    if (millis() - last_flush > 1000) {
        rangeLogger.flushToSerial();
        last_flush = millis();
    }
}

// =============================================================================
// LISTENING STATE
// =============================================================================
void handleListeningState() {
    // Priority 1: Send HELLO if due
    if (scheduler.shouldSendHello()) {
        Serial.println("[STATE] -> HELLO_SEND");
        transitionTo(NodeState::HELLO_SEND);
        return;
    }
    
    // Priority 2: Initiate ranging if in my slot and have neighbors
    if (scheduler.canInitiateRanging()) {
        uint8_t target = neighborTable.getNextRangingTarget();
        if (target != 0) {
            session.target_id = target;
            session.is_initiator = true;
            Serial.print("[STATE] -> INIT_SEND_POLL to node ");
            Serial.println(target);
            transitionTo(NodeState::INIT_SEND_POLL);
            return;
        }
    }
    
    // Priority 3: Check for incoming frames
    int rx_status = uwb.receivedFrameSucc();
    if (rx_status == 1) {
        processReceivedFrame();
    } else if (rx_status == 2) {
        // RX error - clear and restart
        Serial.println("[RX] Error, restarting RX");
        uwb.clearSystemStatus();
        uwb.standardRX();
    }
}

// =============================================================================
// HELLO STATE
// =============================================================================
void handleHelloSend() {
    Serial.println("[HELLO] Sending...");
    
    // Use the driver's proven TX method
    // Set destination to broadcast
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(BROADCAST_ID);
    
    // Use ds_sendFrame with stage=0 (DS-TWR uses 1,2,3,4)
    // This uses the exact same code path that works for ranging
    uwb.ds_sendFrame(0);  // Stage 0 = HELLO
    
    Serial.print("[HELLO] Sent seq=");
    Serial.println(hello_seq);
    hello_seq++;
    scheduler.markHelloSent();
    
    state = NodeState::LISTENING;
    state_entry_time = millis();
}

// =============================================================================
// FRAME PROCESSING
// =============================================================================
void processReceivedFrame() {
    // Read frame data
    uint8_t mode = uwb.read(RX_BUFFER_0_REG, 0x00) & 0x07;
    uint8_t sender_id = uwb.read(RX_BUFFER_0_REG, 0x01) & 0xFF;
    uint8_t dest_id = uwb.read(RX_BUFFER_0_REG, 0x02) & 0xFF;
    uint8_t stage = uwb.read(RX_BUFFER_0_REG, 0x03) & 0x07;
    
    Serial.print("[RX] m=");
    Serial.print(mode);
    Serial.print(" from=");
    Serial.print(sender_id);
    Serial.print(" to=");
    Serial.print(dest_id);
    Serial.print(" stg=");
    Serial.println(stage);
    
    uwb.clearSystemStatus();
    
    // Ignore invalid frames
    if (sender_id == TAG_ID || sender_id == 0) {
        uwb.standardRX();
        return;
    }
    
    // Mode=1 is DS-TWR format (used for both HELLO and ranging)
    if (mode == 1) {
        
        // Stage 0 = HELLO beacon
        if (stage == 0 && dest_id == BROADCAST_ID) {
            Serial.print("[HELLO] From ");
            Serial.println(sender_id);
            
            // Add to neighbor table
            neighborTable.processHello(sender_id, 0, 100);
            
            // Sync to lower-ID nodes
            #if SYNC_TO_LOWER_ID
            if (sender_id < TAG_ID) {
                uint8_t sender_slot = sender_id % NUM_SLOTS;
                uint32_t now = millis();
                uint32_t estimated_frame_start = now - (sender_slot * SLOT_LENGTH_MS) - (SLOT_LENGTH_MS / 2);
                scheduler.syncFrameStart(estimated_frame_start);
                Serial.print("[SYNC] To node ");
                Serial.println(sender_id);
            }
            #endif
            
            uwb.standardRX();
            return;
        }
        
        // Stage 1 = POLL (start of DS-TWR)
        if (stage == STAGE_POLL && (dest_id == TAG_ID || dest_id == BROADCAST_ID)) {
            Serial.print("[POLL] From ");
            Serial.println(sender_id);
            
            session.target_id = sender_id;
            session.is_initiator = false;
            session.rx_poll = uwb.readRXTimestamp();
            
            scheduler.enterRespondingMode();
            transitionTo(NodeState::RESP_SEND_RESP);
            return;
        }
    }
    
    // Unknown frame - ignore
    uwb.standardRX();
}

// =============================================================================
// INITIATOR STATES
// =============================================================================
void handleInitSendPoll() {
    uwb.clearSystemStatus();
    
    // Set IDs
    uwb.setSenderID(TAG_ID);
    uwb.setDestinationID(session.target_id);
    
    // Send POLL using DS-TWR frame (mode=1, stage=1)
    uwb.ds_sendFrame(STAGE_POLL);
    
    session.tx_poll = uwb.readTXTimestamp();
    
    Serial.print("[POLL] Sent to node ");
    Serial.println(session.target_id);
    
    transitionTo(NodeState::INIT_WAIT_RESP);
}

void handleInitWaitResp() {
    // Check timeout
    if (checkStateTimeout(RESPONSE_TIMEOUT_MS)) {
        Serial.println("[TIMEOUT] Waiting for RESP");
        if (++retry_count <= MAX_RANGING_RETRIES) {
            transitionTo(NodeState::INIT_SEND_POLL);
        } else {
            LOG_RANGE_FAILURE(TAG_ID, session.target_id, scheduler.getFrameNumber(), "RESP_TIMEOUT");
            neighborTable.recordRangingFailure(session.target_id);
            retry_count = 0;
            enterListeningMode();
        }
        return;
    }
    
    // Check for response
    int rx_status = uwb.receivedFrameSucc();
    if (rx_status == 1) {
        uint8_t mode = uwb.read(RX_BUFFER_0_REG, 0x00) & 0x07;
        uint8_t sender_id = uwb.read(RX_BUFFER_0_REG, 0x01) & 0xFF;
        uint8_t stage = uwb.read(RX_BUFFER_0_REG, 0x03) & 0x07;
        
        uwb.clearSystemStatus();
        
        if (mode == 1 && sender_id == session.target_id && stage == STAGE_RESP) {
            session.rx_resp = uwb.readRXTimestamp();
            session.t_round_a = session.rx_resp - session.tx_poll;
            retry_count = 0;
            
            Serial.println("[RESP] Received");
            transitionTo(NodeState::INIT_SEND_FINAL);
        } else {
            uwb.standardRX();
        }
    } else if (rx_status == 2) {
        uwb.clearSystemStatus();
        uwb.standardRX();
    }
}

void handleInitSendFinal() {
    uwb.ds_sendFrame(STAGE_FINAL);
    
    session.tx_final = uwb.readTXTimestamp();
    session.t_reply_a = session.tx_final - session.rx_resp;
    
    Serial.println("[FINAL] Sent");
    transitionTo(NodeState::INIT_WAIT_REPORT);
}

void handleInitWaitReport() {
    // Check timeout
    if (checkStateTimeout(RESPONSE_TIMEOUT_MS)) {
        Serial.println("[TIMEOUT] Waiting for REPORT");
        LOG_RANGE_FAILURE(TAG_ID, session.target_id, scheduler.getFrameNumber(), "REPORT_TIMEOUT");
        neighborTable.recordRangingFailure(session.target_id);
        enterListeningMode();
        return;
    }
    
    // Check for REPORT
    int rx_status = uwb.receivedFrameSucc();
    if (rx_status == 1) {
        uint8_t mode = uwb.read(RX_BUFFER_0_REG, 0x00) & 0x07;
        uint8_t sender_id = uwb.read(RX_BUFFER_0_REG, 0x01) & 0xFF;
        uint8_t stage = uwb.read(RX_BUFFER_0_REG, 0x03) & 0x07;
        
        uwb.clearSystemStatus();
        
        if (mode == 1 && sender_id == session.target_id && stage == STAGE_REPORT) {
            session.clock_offset = uwb.getRawClockOffset();
            Serial.println("[REPORT] Received");
            transitionTo(NodeState::INIT_CALCULATE);
        } else {
            uwb.standardRX();
        }
    } else if (rx_status == 2) {
        uwb.clearSystemStatus();
        uwb.standardRX();
    }
}

void handleInitCalculate() {
    // Read timing info from REPORT payload
    int t_round_b = uwb.read(RX_BUFFER_0_REG, 0x04);
    int t_reply_b = uwb.read(RX_BUFFER_0_REG, 0x08);
    
    // Calculate distance
    int ranging_time = uwb.ds_processRTInfo(
        session.t_round_a, session.t_reply_a,
        t_round_b, t_reply_b,
        session.clock_offset
    );
    
    float distance_cm = uwb.convertToCM(ranging_time);
    float rssi = uwb.getSignalStrength();
    float fp_rssi = uwb.getFirstPathSignalStrength();
    
    // Log and record
    LOG_RANGE_SUCCESS(TAG_ID, session.target_id, distance_cm, rssi, fp_rssi, 
                      scheduler.getFrameNumber());
    neighborTable.recordRangingSuccess(session.target_id, distance_cm, rssi);
    
    // Print result
    Serial.print("[DISTANCE] ");
    Serial.print(TAG_ID);
    Serial.print(" -> ");
    Serial.print(session.target_id);
    Serial.print(": ");
    Serial.print(distance_cm, 1);
    Serial.println(" cm");
    
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
    
    Serial.println("[RESP] Sent");
    transitionTo(NodeState::RESP_WAIT_FINAL);
}

void handleRespWaitFinal() {
    // Check timeout
    if (checkStateTimeout(RESPONSE_TIMEOUT_MS)) {
        Serial.println("[TIMEOUT] Waiting for FINAL");
        scheduler.exitRespondingMode();
        enterListeningMode();
        return;
    }
    
    // Check for FINAL
    int rx_status = uwb.receivedFrameSucc();
    if (rx_status == 1) {
        uint8_t mode = uwb.read(RX_BUFFER_0_REG, 0x00) & 0x07;
        uint8_t sender_id = uwb.read(RX_BUFFER_0_REG, 0x01) & 0xFF;
        uint8_t stage = uwb.read(RX_BUFFER_0_REG, 0x03) & 0x07;
        
        uwb.clearSystemStatus();
        
        if (mode == 1 && sender_id == session.target_id && stage == STAGE_FINAL) {
            session.rx_final = uwb.readRXTimestamp();
            session.t_round_b = session.rx_final - session.tx_resp;
            
            Serial.println("[FINAL] Received");
            transitionTo(NodeState::RESP_SEND_REPORT);
        } else {
            uwb.standardRX();
        }
    } else if (rx_status == 2) {
        uwb.clearSystemStatus();
        uwb.standardRX();
    }
}

void handleRespSendReport() {
    // Send REPORT with timing info using driver's method
    uwb.ds_sendRTInfo(session.t_round_b, session.t_reply_b);
    
    Serial.println("[REPORT] Sent");
    
    scheduler.exitRespondingMode();
    enterListeningMode();
}

// =============================================================================
// HELPER FUNCTIONS
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
}

// =============================================================================
// UWB INITIALIZATION
// =============================================================================
void initializeUWB() {
    Serial.println("[INIT] Starting UWB...");
    
    uwb.begin();
    hardResetDW();
    delay(200);
    
    if (!uwb.checkSPI()) {
        Serial.println("[FATAL] SPI failed!");
        while (1) { delay(1000); }
    }
    Serial.println("[INIT] SPI OK");
    
    int timeout = 100;
    while (!uwb.checkForIDLE() && timeout-- > 0) {
        delay(100);
    }
    if (timeout <= 0) {
        Serial.println("[FATAL] IDLE timeout!");
        while (1) { delay(1000); }
    }
    
    uwb.softReset();
    delay(200);
    
    if (!uwb.checkForIDLE()) {
        Serial.println("[FATAL] IDLE failed after reset!");
        while (1) { delay(1000); }
    }
    
    uwb.init();
    uwb.setupGPIO();
    uwb.setTXAntennaDelay(ANTENNA_DELAY_DEFAULT);
    uwb.setSenderID(TAG_ID);
    uwb.configureAsTX();
    uwb.clearSystemStatus();
    
    Serial.println("[INIT] UWB ready");
}

void resetRadio() {
    Serial.println("[RESET] Radio reset...");
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
// STATUS OUTPUT
// =============================================================================
void printStatus() {
    Serial.println("\n========== NODE STATUS ==========");
    Serial.print("Node ID: ");
    Serial.print(TAG_ID);
    Serial.print(" | Slot: ");
    Serial.print(MY_SLOT);
    Serial.print(" | Frame: ");
    Serial.println(scheduler.getFrameNumber());
    
    SlotInfo slot = scheduler.getSlotInfo();
    Serial.print("Current Slot: ");
    Serial.print(slot.current_slot);
    Serial.print(" | Is My Slot: ");
    Serial.print(slot.is_my_slot ? "YES" : "NO");
    Serial.print(" | Synced: ");
    Serial.println(scheduler.isSynced() ? "YES" : "NO");
    
    Serial.print("Active Neighbors: ");
    Serial.println(neighborTable.getActiveNeighborCount());
    
    neighborTable.printTable();
    
    Serial.println("=================================\n");
}