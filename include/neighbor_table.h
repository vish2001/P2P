#pragma once
#include <Arduino.h>
#include "peer_config.h"

// =============================================================================
// NEIGHBOR TABLE MANAGEMENT
// =============================================================================
// Tracks discovered peers via HELLO beacons and ranging success
// =============================================================================

// Neighbor entry structure
struct Neighbor {
    uint8_t id;                          // Neighbor's TAG_ID
    uint32_t last_hello_time;            // millis() of last HELLO received
    uint16_t hello_count;                // Total HELLOs received from this neighbor
    uint16_t ranging_attempts;           // Total ranging attempts
    uint16_t ranging_successes;          // Successful ranges
    float last_distance_cm;              // Most recent distance measurement
    float distance_history[FILTER_SIZE]; // Rolling window for median filter
    uint8_t history_index;               // Current index in history
    float filtered_distance_cm;          // Median-filtered distance
    float last_rssi;                     // Signal strength (dBm)
    bool active;                         // Is this slot in use?
};

// K-neighbor selection result
struct SelectedNeighbor {
    uint8_t id;
    float priority_score;
};

class NeighborTable {
public:
    NeighborTable();
    
    // --- HELLO Processing ---
    // Called when a HELLO beacon is received
    void processHello(uint8_t neighbor_id, uint8_t seq_num, uint8_t battery_level);
    
    // --- Ranging Results ---
    // Called after successful ranging
    void recordRangingSuccess(uint8_t neighbor_id, float distance_cm, float rssi);
    
    // Called after failed ranging attempt
    void recordRangingFailure(uint8_t neighbor_id);
    
    // --- Neighbor Selection ---
    // Get K best neighbors for ranging (sorted by priority)
    uint8_t selectKNeighbors(SelectedNeighbor* out_neighbors, uint8_t max_k);
    
    // Get next neighbor to range with (round-robin through K-selected)
    uint8_t getNextRangingTarget();
    
    // --- Maintenance ---
    // Remove stale neighbors (no HELLO for NEIGHBOR_TIMEOUT_MS)
    void pruneStaleNeighbors();
    
    // --- Queries ---
    Neighbor* getNeighbor(uint8_t neighbor_id);
    uint8_t getActiveNeighborCount();
    float getFilteredDistance(uint8_t neighbor_id);
    float getRangingSuccessRate(uint8_t neighbor_id);
    
    // --- Debug ---
    void printTable();
    void printSelectedNeighbors();

private:
    Neighbor neighbors[MAX_NEIGHBORS];
    SelectedNeighbor selected[K_NEIGHBORS];
    uint8_t selected_count;
    uint8_t current_ranging_index;  // Round-robin index into selected[]
    
    // Find or create neighbor entry
    Neighbor* findOrCreate(uint8_t neighbor_id);
    
    // Calculate priority score for neighbor selection
    float calculatePriority(const Neighbor& n);
    
    // Update median-filtered distance
    void updateFilteredDistance(Neighbor& n, float new_distance);
    
    // Check if distance is valid
    bool isValidDistance(float distance);
    
    // Calculate median of array
    float calculateMedian(float* arr, int size);
};

// Global neighbor table instance
extern NeighborTable neighborTable;