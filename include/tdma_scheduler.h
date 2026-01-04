#pragma once
#include <Arduino.h>
#include "peer_config.h"

// =============================================================================
// TDMA SCHEDULER
// =============================================================================
// Deterministic time-division multiple access for collision-free ranging
// 
// Key principle: my_slot = TAG_ID % NUM_SLOTS
// - In MY slot: I am allowed to INITIATE ranging
// - In OTHER slots: I may only RESPOND to ranging requests
//
// Synchronization: Nodes sync their frame timing to lower-ID neighbors
// =============================================================================

// Slot timing info
struct SlotInfo {
    uint8_t current_slot;          // Which slot we're in (0 to NUM_SLOTS-1)
    uint32_t slot_start_ms;        // millis() when current slot started
    uint32_t slot_elapsed_ms;      // How long into current slot
    uint32_t slot_remaining_ms;    // Time remaining in current slot
    bool is_my_slot;               // True if current_slot == MY_SLOT
    uint32_t frame_start_ms;       // millis() when current frame started
    uint16_t frame_number;         // Incrementing frame counter
};

class TDMAScheduler {
public:
    TDMAScheduler();
    
    // --- Core Functions ---
    void update();
    SlotInfo getSlotInfo();
    bool canInitiateRanging();
    bool shouldSendHello();
    void markHelloSent();
    
    // --- State Management ---
    void enterRespondingMode();
    void exitRespondingMode();
    bool isResponding();
    
    // --- Synchronization ---
    void syncFrameStart(uint32_t new_frame_start);
    bool isSynced();
    uint32_t getLastSyncTime();
    
    // --- Timing Helpers ---
    uint32_t timeUntilMySlot();
    uint16_t getFrameNumber();
    uint32_t getFrameStart();
    
    // --- Debug ---
    void printStatus();

private:
    uint32_t frame_start_ms;
    uint16_t frame_number;
    uint32_t last_hello_ms;
    uint32_t last_sync_ms;
    bool responding;
    bool hello_pending;
    bool synced;
    
    uint8_t calculateCurrentSlot();
    void checkFrameRollover();
};

// Global scheduler instance
extern TDMAScheduler scheduler;