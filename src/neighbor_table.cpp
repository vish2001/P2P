#include "neighbor_table.h"

// Global instance
NeighborTable neighborTable;

// =============================================================================
// CONSTRUCTOR
// =============================================================================
NeighborTable::NeighborTable() {
    // Initialize all neighbor slots as inactive
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        neighbors[i].active = false;
        neighbors[i].id = 0;
        neighbors[i].last_hello_time = 0;
        neighbors[i].hello_count = 0;
        neighbors[i].ranging_attempts = 0;
        neighbors[i].ranging_successes = 0;
        neighbors[i].last_distance_cm = 0;
        neighbors[i].filtered_distance_cm = 0;
        neighbors[i].history_index = 0;
        neighbors[i].last_rssi = -100;
        
        for (int j = 0; j < FILTER_SIZE; j++) {
            neighbors[i].distance_history[j] = 0;
        }
    }
    
    selected_count = 0;
    current_ranging_index = 0;
}

// =============================================================================
// HELLO PROCESSING
// =============================================================================
void NeighborTable::processHello(uint8_t neighbor_id, uint8_t seq_num, uint8_t battery_level) {
    // Don't track ourselves
    if (neighbor_id == TAG_ID) return;
    
    Neighbor* n = findOrCreate(neighbor_id);
    if (n == nullptr) {
        Serial.println("[NEIGHBOR] Table full, cannot add new neighbor");
        return;
    }
    
    n->last_hello_time = millis();
    n->hello_count++;
    
    // Optional: could track seq_num for gap detection
    // Optional: could store battery_level for energy-aware selection
    
    if (DEBUG_OUTPUT) {
        Serial.print("[HELLO] From node ");
        Serial.print(neighbor_id);
        Serial.print(" (count: ");
        Serial.print(n->hello_count);
        Serial.println(")");
    }
}

// =============================================================================
// RANGING RESULTS
// =============================================================================
void NeighborTable::recordRangingSuccess(uint8_t neighbor_id, float distance_cm, float rssi) {
    Neighbor* n = getNeighbor(neighbor_id);
    if (n == nullptr) {
        // Neighbor not in table (shouldn't happen if we only range with known neighbors)
        Serial.print("[WARN] Ranging success for unknown neighbor ");
        Serial.println(neighbor_id);
        return;
    }
    
    n->ranging_attempts++;
    n->ranging_successes++;
    n->last_rssi = rssi;
    
    // Outlier rejection: reject large sudden jumps
    if (n->filtered_distance_cm > 0 && 
        abs(distance_cm - n->filtered_distance_cm) > OUTLIER_THRESHOLD) {
        Serial.print("[FILTER] Outlier rejected: ");
        Serial.print(distance_cm);
        Serial.print(" cm (expected ~");
        Serial.print(n->filtered_distance_cm);
        Serial.println(" cm)");
        return;
    }
    
    n->last_distance_cm = distance_cm;
    updateFilteredDistance(*n, distance_cm);
}

void NeighborTable::recordRangingFailure(uint8_t neighbor_id) {
    Neighbor* n = getNeighbor(neighbor_id);
    if (n == nullptr) return;
    
    n->ranging_attempts++;
    // Don't increment successes
}

// =============================================================================
// NEIGHBOR SELECTION
// =============================================================================
uint8_t NeighborTable::selectKNeighbors(SelectedNeighbor* out_neighbors, uint8_t max_k) {
    // First, prune stale neighbors
    pruneStaleNeighbors();
    
    // Collect all eligible neighbors with their priority scores
    SelectedNeighbor candidates[MAX_NEIGHBORS];
    uint8_t candidate_count = 0;
    
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].active) continue;
        
        // Must have received minimum HELLOs to be eligible
        if (neighbors[i].hello_count < MIN_HELLO_COUNT) continue;
        
        candidates[candidate_count].id = neighbors[i].id;
        candidates[candidate_count].priority_score = calculatePriority(neighbors[i]);
        candidate_count++;
    }
    
    // Sort by priority (descending) - simple bubble sort for small N
    for (int i = 0; i < candidate_count - 1; i++) {
        for (int j = i + 1; j < candidate_count; j++) {
            if (candidates[j].priority_score > candidates[i].priority_score) {
                SelectedNeighbor temp = candidates[i];
                candidates[i] = candidates[j];
                candidates[j] = temp;
            }
        }
    }
    
    // Take top K
    uint8_t result_count = min((uint8_t)candidate_count, max_k);
    for (int i = 0; i < result_count; i++) {
        out_neighbors[i] = candidates[i];
    }
    
    // Update internal selected list
    selected_count = result_count;
    for (int i = 0; i < result_count; i++) {
        selected[i] = candidates[i];
    }
    
    return result_count;
}

uint8_t NeighborTable::getNextRangingTarget() {
    // Simple approach: find ALL eligible neighbors, cycle through them
    uint8_t eligible[MAX_NEIGHBORS];
    uint8_t eligible_count = 0;
    
    // Prune stale neighbors first
    pruneStaleNeighbors();
    
    // Collect all neighbors with enough HELLOs
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].active) continue;
        if (neighbors[i].hello_count < MIN_HELLO_COUNT) continue;
        eligible[eligible_count++] = neighbors[i].id;
    }
    
    if (eligible_count == 0) {
        return 0;  // No neighbors
    }
    
    // Simple round-robin using static index
    static uint8_t rr_index = 0;
    rr_index = rr_index % eligible_count;
    
    uint8_t target = eligible[rr_index];
    rr_index = (rr_index + 1) % eligible_count;
    
    // Debug output
    Serial.print("[RR] ");
    Serial.print(eligible_count);
    Serial.print(" neighbors, picking ");
    Serial.println(target);
    
    return target;
}

// =============================================================================
// PRIORITY CALCULATION
// =============================================================================
float NeighborTable::calculatePriority(const Neighbor& n) {
    // Priority formula:
    // 1. Recency of HELLO (higher = more recent = better)
    // 2. Ranging success rate (higher = better)
    // 3. Shorter distance (optional, lower = better for connectivity)
    
    float score = 0.0;
    
    // Recency: exponential decay, recent HELLOs score higher
    uint32_t age_ms = millis() - n.last_hello_time;
    float recency_score = exp(-((float)age_ms / 5000.0)); // Half-life ~5s
    score += recency_score * 50.0; // Weight: 50
    
    // Success rate: higher is better
    float success_rate = getRangingSuccessRate(n.id);
    score += success_rate * 30.0; // Weight: 30
    
    // Distance: prefer closer neighbors (if we have data)
    if (n.filtered_distance_cm > 0) {
        // Inverse relationship: closer = higher score
        float distance_score = 1.0 / (1.0 + n.filtered_distance_cm / 500.0);
        score += distance_score * 20.0; // Weight: 20
    } else {
        // No distance data yet, give neutral score
        score += 10.0;
    }
    
    return score;
}

// =============================================================================
// MAINTENANCE
// =============================================================================
void NeighborTable::pruneStaleNeighbors() {
    uint32_t now = millis();
    
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].active) continue;
        
        if (now - neighbors[i].last_hello_time > NEIGHBOR_TIMEOUT_MS) {
            Serial.print("[NEIGHBOR] Pruning stale neighbor ");
            Serial.println(neighbors[i].id);
            
            neighbors[i].active = false;
            neighbors[i].id = 0;
            neighbors[i].hello_count = 0;
            neighbors[i].ranging_attempts = 0;
            neighbors[i].ranging_successes = 0;
        }
    }
}

// =============================================================================
// QUERIES
// =============================================================================
Neighbor* NeighborTable::getNeighbor(uint8_t neighbor_id) {
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].active && neighbors[i].id == neighbor_id) {
            return &neighbors[i];
        }
    }
    return nullptr;
}

uint8_t NeighborTable::getActiveNeighborCount() {
    uint8_t count = 0;
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].active) count++;
    }
    return count;
}

float NeighborTable::getFilteredDistance(uint8_t neighbor_id) {
    Neighbor* n = getNeighbor(neighbor_id);
    if (n == nullptr) return -1;
    return n->filtered_distance_cm;
}

float NeighborTable::getRangingSuccessRate(uint8_t neighbor_id) {
    Neighbor* n = getNeighbor(neighbor_id);
    if (n == nullptr) return 0;
    if (n->ranging_attempts == 0) return 0.5; // Default 50% if no data
    return (float)n->ranging_successes / (float)n->ranging_attempts;
}

// =============================================================================
// PRIVATE HELPERS
// =============================================================================
Neighbor* NeighborTable::findOrCreate(uint8_t neighbor_id) {
    // First, look for existing entry
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (neighbors[i].active && neighbors[i].id == neighbor_id) {
            return &neighbors[i];
        }
    }
    
    // Not found, create new entry in first empty slot
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].active) {
            neighbors[i].active = true;
            neighbors[i].id = neighbor_id;
            neighbors[i].last_hello_time = millis();
            neighbors[i].hello_count = 0;
            neighbors[i].ranging_attempts = 0;
            neighbors[i].ranging_successes = 0;
            neighbors[i].last_distance_cm = 0;
            neighbors[i].filtered_distance_cm = 0;
            neighbors[i].history_index = 0;
            neighbors[i].last_rssi = -100;
            
            for (int j = 0; j < FILTER_SIZE; j++) {
                neighbors[i].distance_history[j] = 0;
            }
            
            Serial.print("[NEIGHBOR] New neighbor discovered: ");
            Serial.println(neighbor_id);
            
            return &neighbors[i];
        }
    }
    
    return nullptr; // Table full
}

void NeighborTable::updateFilteredDistance(Neighbor& n, float new_distance) {
    if (!isValidDistance(new_distance)) return;
    
    // Add to rolling window
    n.distance_history[n.history_index] = new_distance;
    n.history_index = (n.history_index + 1) % FILTER_SIZE;
    
    // Calculate median of valid entries
    float valid[FILTER_SIZE];
    int valid_count = 0;
    
    for (int i = 0; i < FILTER_SIZE; i++) {
        if (isValidDistance(n.distance_history[i])) {
            valid[valid_count++] = n.distance_history[i];
        }
    }
    
    if (valid_count > 0) {
        n.filtered_distance_cm = calculateMedian(valid, valid_count);
    }
}

bool NeighborTable::isValidDistance(float distance) {
    return (distance >= MIN_DISTANCE_CM && distance <= MAX_DISTANCE_CM);
}

float NeighborTable::calculateMedian(float* arr, int size) {
    if (size == 0) return 0;
    if (size == 1) return arr[0];
    
    // Copy array for sorting
    float temp[FILTER_SIZE];
    for (int i = 0; i < size; i++) {
        temp[i] = arr[i];
    }
    
    // Simple bubble sort
    for (int i = 0; i < size - 1; i++) {
        for (int j = i + 1; j < size; j++) {
            if (temp[j] < temp[i]) {
                float t = temp[i];
                temp[i] = temp[j];
                temp[j] = t;
            }
        }
    }
    
    // Return median
    if (size % 2 == 0) {
        return (temp[size / 2 - 1] + temp[size / 2]) / 2.0;
    } else {
        return temp[size / 2];
    }
}

// =============================================================================
// DEBUG OUTPUT
// =============================================================================
void NeighborTable::printTable() {
    Serial.println("\n=== NEIGHBOR TABLE ===");
    Serial.println("ID\tHello\tRange%\tDist(cm)\tRSSI\tAge(ms)");
    
    uint32_t now = millis();
    
    for (int i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].active) continue;
        
        Serial.print(neighbors[i].id);
        Serial.print("\t");
        Serial.print(neighbors[i].hello_count);
        Serial.print("\t");
        Serial.print((int)(getRangingSuccessRate(neighbors[i].id) * 100));
        Serial.print("%\t");
        Serial.print(neighbors[i].filtered_distance_cm, 1);
        Serial.print("\t\t");
        Serial.print(neighbors[i].last_rssi, 1);
        Serial.print("\t");
        Serial.println(now - neighbors[i].last_hello_time);
    }
    Serial.println("======================\n");
}

void NeighborTable::printSelectedNeighbors() {
    Serial.print("[K-SELECT] ");
    for (int i = 0; i < selected_count; i++) {
        Serial.print(selected[i].id);
        Serial.print("(");
        Serial.print(selected[i].priority_score, 1);
        Serial.print(") ");
    }
    Serial.println();
}