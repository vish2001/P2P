# P2P UWB Mesh System

## Philosophy

**ALL NODES ARE IDENTICAL PEERS.**

- No anchors
- No tags  
- No coordinator
- No fixed positions

Every node auto-generates its ID from MAC address, discovers neighbors, and ranges with them. The central station computes **relative positions** from pairwise distances using MDS.

## Quick Start

```bash
# Flash any number of nodes (they auto-configure)
pio run -e node -t upload

# Run the visualizer
cd test
python mesh_localizer.py
```

That's it. No anchor configuration, no ID assignment, no special setup.

## How It Works

### Firmware (ESP32)

1. **Auto-ID**: Each node hashes its MAC address to generate a unique ID (1-254)
2. **TDMA**: Nodes get deterministic time slots: `slot = ID % 53` (prime to minimize collisions)
3. **Discovery**: Nodes broadcast HELLO beacons every 3 seconds
4. **Ranging**: In its slot, each node ranges with one neighbor (round-robin through all)
5. **Reporting**: Distance measurements sent to central station via ESP-NOW/UDP

### Python Localizer

1. **Collect**: Receives pairwise distances from all nodes
2. **MDS**: Computes 2D positions that best match the measured distances
3. **Procrustes**: Aligns each frame to the previous to prevent flips/jumps
4. **Display**: Shows the mesh topology with nodes and links

## Key Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| NUM_SLOTS | 53 | Prime number minimizes ID collisions |
| SLOT_LENGTH | 25ms | Time for complete DS-TWR exchange |
| FRAME_LENGTH | 1325ms | Full cycle through all slots |
| HELLO_INTERVAL | 3s | Neighbor discovery beacons |
| NEIGHBOR_TIMEOUT | 15s | Remove silent neighbors |

## Coordinate System

The output coordinates are **RELATIVE**, not absolute:

- The mesh can rotate, translate, or flip between runs
- **Distances between nodes are accurate**
- **Shape is preserved**
- There is no "correct" orientation

If you need absolute positioning, you must:
1. Post-process to align to known reference points, OR
2. Use a different approach (this system is designed for relative tracking)

## Scaling

Tested configurations:

| Nodes | Collision Rate | Update Rate |
|-------|---------------|-------------|
| 10 | ~2% | 0.75 Hz/node |
| 25 | ~4% | 0.75 Hz/node |
| 50 | ~6% | 0.75 Hz/node |

The system handles sparse connectivity (clustered environments) through:
- Round-robin ranging with ALL neighbors (not just closest)
- Priority boost for stale links
- Priority boost for long-range links (potential bridges)

## Troubleshooting

**"No nodes showing"**
- Check serial output on nodes for HELLO beacons
- Verify UDP packets reaching the PC (Wireshark)

**"Positions jumping around"**
- Normal if nodes are moving
- Enable Procrustes alignment (default on)
- Check for NLOS/multipath issues

**"Map folded/flipped"**
- Add more node connections (denser mesh)
- This is a fundamental limitation of MDS with sparse data

**"Slot collision detected"**
- Rare with 53 slots
- Two nodes with IDs differing by 53 share a slot
- System handles it via jitter, but ranging may be slower

## Files

```
P2P-mesh/
├── include/
│   ├── peer_config.h      # All configuration
│   ├── neighbor_table.h   # Mesh neighbor management
│   └── tdma_scheduler.h   # Time slot scheduling
├── src/
│   ├── main.cpp           # Main firmware
│   ├── neighbor_table.cpp # Connectivity logic
│   └── tdma_scheduler.cpp # TDMA implementation
├── test/
│   └── mesh_localizer.py  # Python visualization
└── platformio.ini         # Build configuration
```

## Protocol

```
HELLO Beacon (broadcast every 3s):
[0xAA][SenderID][0xFF][Seq][NeighborCount][Slot][0][0]

DS-TWR Exchange (~20ms):
Initiator          Responder
    |---- POLL ------->|
    |<---- RESP -------|
    |---- FINAL ------>|
    |<--- REPORT ------|

UDP to Central (per range):
R,<from>,<to>,<distance_cm>,<rssi>,<timestamp>
```
