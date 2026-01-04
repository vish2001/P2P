#include "tdma_scheduler.h"

// Global instance
TDMAScheduler scheduler;

// =============================================================================
// CONSTRUCTOR
// =============================================================================
TDMAScheduler::TDMAScheduler() {
    frame_start_ms = millis();
    frame_number = 0;
    last_hello_ms = 0;
    last_sync_ms = 0;
    responding = false;
    hello_pending = true;  // Send HELLO at startup
    synced = false;
}

// =============================================================================
// CORE UPDATE
// =============================================================================
void TDMAScheduler::update() {
    checkFrameRollover();
    
    // Check if HELLO is due
    uint32_t now = millis();
    if (now - last_hello_ms >= HELLO_INTERVAL_MS) {
        hello_pending = true;
    }
}

// =============================================================================
// SLOT CALCULATION
// =============================================================================
uint8_t TDMAScheduler::calculateCurrentSlot() {
    uint32_t now = millis();
    uint32_t frame_elapsed = now - frame_start_ms;
    
    // Handle wraparound
    if (frame_elapsed >= FRAME_LENGTH_MS) {
        frame_elapsed = frame_elapsed % FRAME_LENGTH_MS;
    }
    
    return (uint8_t)(frame_elapsed / SLOT_LENGTH_MS);
}

void TDMAScheduler::checkFrameRollover() {
    uint32_t now = millis();
    uint32_t frame_elapsed = now - frame_start_ms;
    
    if (frame_elapsed >= FRAME_LENGTH_MS) {
        frame_start_ms = now - (frame_elapsed % FRAME_LENGTH_MS);
        frame_number++;
        
        if (DEBUG_OUTPUT && (frame_number % 10 == 0)) {
            Serial.print("[TDMA] Frame ");
            Serial.println(frame_number);
        }
    }
}

// =============================================================================
// SLOT INFO
// =============================================================================
SlotInfo TDMAScheduler::getSlotInfo() {
    SlotInfo info;
    uint32_t now = millis();
    uint32_t frame_elapsed = now - frame_start_ms;
    
    info.frame_start_ms = frame_start_ms;
    info.frame_number = frame_number;
    info.current_slot = calculateCurrentSlot();
    info.slot_start_ms = frame_start_ms + (info.current_slot * SLOT_LENGTH_MS);
    info.slot_elapsed_ms = now - info.slot_start_ms;
    info.slot_remaining_ms = SLOT_LENGTH_MS - info.slot_elapsed_ms;
    info.is_my_slot = (info.current_slot == MY_SLOT);
    
    return info;
}

// =============================================================================
// INITIATION CHECK
// =============================================================================
bool TDMAScheduler::canInitiateRanging() {
    if (responding) {
        return false;
    }
    
    SlotInfo info = getSlotInfo();
    
    if (!info.is_my_slot) {
        return false;
    }
    
    // Need at least 30ms for DS-TWR exchange
    const uint32_t MIN_TIME_FOR_RANGING = 30;
    
    if (info.slot_remaining_ms < MIN_TIME_FOR_RANGING) {
        return false;
    }
    
    return true;
}

// =============================================================================
// HELLO TIMING
// =============================================================================
bool TDMAScheduler::shouldSendHello() {
    return hello_pending && !responding;
}

void TDMAScheduler::markHelloSent() {
    last_hello_ms = millis();
    hello_pending = false;
}

// =============================================================================
// RESPONDING STATE
// =============================================================================
void TDMAScheduler::enterRespondingMode() {
    responding = true;
}

void TDMAScheduler::exitRespondingMode() {
    responding = false;
}

bool TDMAScheduler::isResponding() {
    return responding;
}

// =============================================================================
// SYNCHRONIZATION
// =============================================================================
void TDMAScheduler::syncFrameStart(uint32_t new_frame_start) {
    uint32_t now = millis();
    
    // Don't re-sync too frequently
    if (synced && (now - last_sync_ms < SYNC_HOLDOFF_MS)) {
        return;
    }
    
    uint32_t old_frame_start = frame_start_ms;
    frame_start_ms = new_frame_start;
    last_sync_ms = now;
    synced = true;
    
    Serial.print("[SYNC] Frame adjusted by ");
    Serial.print((int32_t)(new_frame_start - old_frame_start));
    Serial.println(" ms");
}

bool TDMAScheduler::isSynced() {
    return synced;
}

uint32_t TDMAScheduler::getLastSyncTime() {
    return last_sync_ms;
}

// =============================================================================
// TIMING HELPERS
// =============================================================================
uint32_t TDMAScheduler::timeUntilMySlot() {
    SlotInfo info = getSlotInfo();
    
    if (info.is_my_slot) {
        return 0;
    }
    
    uint8_t slots_until_mine;
    if (MY_SLOT > info.current_slot) {
        slots_until_mine = MY_SLOT - info.current_slot;
    } else {
        slots_until_mine = NUM_SLOTS - info.current_slot + MY_SLOT;
    }
    
    return info.slot_remaining_ms + ((slots_until_mine - 1) * SLOT_LENGTH_MS);
}

uint16_t TDMAScheduler::getFrameNumber() {
    return frame_number;
}

uint32_t TDMAScheduler::getFrameStart() {
    return frame_start_ms;
}

// =============================================================================
// DEBUG
// =============================================================================
void TDMAScheduler::printStatus() {
    SlotInfo info = getSlotInfo();
    
    Serial.println("\n=== TDMA STATUS ===");
    Serial.print("Frame: ");
    Serial.print(info.frame_number);
    Serial.print(" | Synced: ");
    Serial.println(synced ? "YES" : "NO");
    Serial.print("Current Slot: ");
    Serial.print(info.current_slot);
    Serial.print(" | My Slot: ");
    Serial.print(MY_SLOT);
    Serial.print(" | Is Mine: ");
    Serial.println(info.is_my_slot ? "YES" : "NO");
    Serial.print("Slot Remaining: ");
    Serial.print(info.slot_remaining_ms);
    Serial.println(" ms");
    Serial.println("===================\n");
}