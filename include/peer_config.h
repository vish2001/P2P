#pragma once
#include <Arduino.h>

// =============================================================================
// PEER-TO-PEER UWB LOCALIZATION SYSTEM - CONFIGURATION
// =============================================================================
// All nodes are identical peers. No anchors, no tags, no coordinator.
// Deterministic TDMA slot ownership: my_slot = tag_id % NUM_SLOTS
// =============================================================================

// --- Hardware Pin Mapping (ESP32 + DW3000) ---
#define PIN_RST   27
#define PIN_CS     5
#define PIN_IRQ   34
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define LED_PIN    2

// =============================================================================
// NODE IDENTITY
// =============================================================================
// Each physical device must have a unique TAG_ID (0-255)
// Can be overridden via build flags: -DTAG_ID=X
#ifndef TAG_ID
#define TAG_ID  1
#endif

// =============================================================================
// TDMA TIMING PARAMETERS
// =============================================================================
#define FRAME_LENGTH_MS      1000    // Total frame duration (ms)
#define NUM_SLOTS            8       // Number of TDMA slots per frame
#define SLOT_LENGTH_MS       (FRAME_LENGTH_MS / NUM_SLOTS)  // 125ms per slot

// Slot ownership: deterministic, no negotiation
#define MY_SLOT              (TAG_ID % NUM_SLOTS)

// =============================================================================
// HELLO BEACON PARAMETERS
// =============================================================================
#define HELLO_INTERVAL_MS    2000    // Broadcast HELLO every 2 seconds
#define HELLO_FRAME_TYPE     0xAA    // Magic byte to identify HELLO frames
#define NEIGHBOR_TIMEOUT_MS  10000   // Remove neighbor if no HELLO for 10s

// =============================================================================
// FRAME SYNCHRONIZATION
// =============================================================================
#define SYNC_TO_LOWER_ID     1       // Sync frame timing to lower-ID nodes
#define SYNC_HOLDOFF_MS      5000    // Don't re-sync within this window

// =============================================================================
// NEIGHBOR MANAGEMENT
// =============================================================================
#define MAX_NEIGHBORS        16      // Maximum neighbors to track
#define K_NEIGHBORS          4       // Number of neighbors to range with
#define MIN_HELLO_COUNT      2       // Minimum HELLOs before considering for ranging

// =============================================================================
// DS-TWR PROTOCOL PARAMETERS
// =============================================================================
#define DS_TWR_FRAME_TYPE    0x01    // Magic byte for DS-TWR frames
#define RESPONSE_TIMEOUT_MS  15      // Timeout waiting for response
#define MAX_RANGING_RETRIES  2       // Max retries per ranging attempt

// DS-TWR stages (stored in frame)
#define STAGE_POLL           1
#define STAGE_RESP           2
#define STAGE_FINAL          3
#define STAGE_REPORT         4
#define STAGE_ERROR          7

// =============================================================================
// LISTEN-BEFORE-TALK (Collision Avoidance)
// =============================================================================
#define LBT_LISTEN_MS        3       // Listen for activity before transmitting
#define LBT_BACKOFF_MS       10      // Backoff if channel busy

// =============================================================================
// DISTANCE FILTERING
// =============================================================================
#define FILTER_SIZE          5       // Median filter window (last N ranges)
#define MIN_DISTANCE_CM      0.0     // Minimum valid distance
#define MAX_DISTANCE_CM      5000.0  // Maximum valid distance (50m)
#define OUTLIER_THRESHOLD    100.0   // Reject jumps > 1m

// =============================================================================
// LOGGING & DEBUG
// =============================================================================
#define LOG_BUFFER_SIZE      32      // Number of log entries to buffer
#define SERIAL_BAUD          921600  // Serial output baud rate

// Debug output - SET TO 1 FOR DEBUGGING
#ifndef DEBUG_OUTPUT
#define DEBUG_OUTPUT         1       // Enable verbose debug prints
#endif

// =============================================================================
// ANTENNA CALIBRATION
// =============================================================================
#define ANTENNA_DELAY_DEFAULT 16350  // Default antenna delay

// =============================================================================
// DW3000 RADIO CONSTANTS
// =============================================================================
#define LEN_RX_CAL_CONF 4
#define LEN_TX_FCTRL_CONF 6
#define LEN_AON_DIG_CFG_CONF 3

#define PMSC_STATE_IDLE 0x3
#define FCS_LEN 2

#define STDRD_SYS_CONFIG 0x188
#define DTUNE0_CONFIG 0x0F

#define SYS_STATUS_FRAME_RX_SUCC 0x2000
#define SYS_STATUS_RX_ERR 0x4279000
#define SYS_STATUS_FRAME_TX_SUCC 0x80

#define PREAMBLE_32 4
#define PREAMBLE_64 8
#define PREAMBLE_128 5
#define PREAMBLE_256 9
#define PREAMBLE_512 11
#define PREAMBLE_1024 2
#define PREAMBLE_2048 10
#define PREAMBLE_4096 3
#define PREAMBLE_1536 6

#define CHANNEL_5 0x0
#define CHANNEL_9 0x1

#define PAC4 0x03
#define PAC8 0x00
#define PAC16 0x01
#define PAC32 0x02

#define DATARATE_6_8MB 0x1
#define DATARATE_850KB 0x0

#define PHR_MODE_STANDARD 0x0
#define PHR_MODE_LONG 0x1

#define PHR_RATE_6_8MB 0x1
#define PHR_RATE_850KB 0x0

// Masks
#define SPIRDY_MASK 0x80
#define RCINIT_MASK 0x100
#define BIAS_CTRL_BIAS_MASK 0x1F

// Registers
#define GEN_CFG_AES_LOW_REG 0x00
#define GEN_CFG_AES_HIGH_REG 0x01
#define STS_CFG_REG 0x2
#define RX_TUNE_REG 0x3
#define EXT_SYNC_REG 0x4
#define GPIO_CTRL_REG 0x5
#define DRX_REG 0x6
#define RF_CONF_REG 0x7
#define RF_CAL_REG 0x8
#define FS_CTRL_REG 0x9
#define AON_REG 0xA
#define OTP_IF_REG 0xB
#define CIA_REG1 0xC
#define CIA_REG2 0xD
#define CIA_REG3 0xE
#define DIG_DIAG_REG 0xF
#define PMSC_REG 0x11
#define RX_BUFFER_0_REG 0x12
#define RX_BUFFER_1_REG 0x13
#define TX_BUFFER_REG 0x14
#define ACC_MEM_REG 0x15
#define SCRATCH_RAM_REG 0x16
#define AES_RAM_REG 0x17
#define SET_1_2_REG 0x18
#define INDIRECT_PTR_A_REG 0x1D
#define INDIRECT_PTR_B_REG 0x1E
#define IN_PTR_CFG_REG 0x1F

#define TRANSMIT_DELAY 0x3B9ACA00
#define TRANSMIT_DIFF 0x1FF

#define NS_UNIT 4.0064102564102564
#define PS_UNIT 15.6500400641025641
#define SPEED_OF_LIGHT 0.029979245800

#define CLOCK_OFFSET_CHAN_5_CONSTANT -0.5731e-3f
#define CLOCK_OFFSET_CHAN_9_CONSTANT -0.1252e-3f

#define NO_OFFSET 0x0

// =============================================================================
// GLOBAL STATE VARIABLES (externed, defined in main)
// =============================================================================
extern int ANTENNA_DELAY;
extern int led_status;
extern int destination;
extern int sender;