#ifndef MG_ZS_BATTERY_H
#define MG_ZS_BATTERY_H
#include <Arduino.h>
#include "../include.h"

#define BATTERY_SELECTED
#define MAX_PACK_VOLTAGE_DV 4040  // 404.0V maximum pack voltage
#define MIN_PACK_VOLTAGE_DV 3100  // 310.0V minimum pack voltage
#define MAX_CELL_DEVIATION_MV 150 // Maximum allowed deviation between cells
#define MAX_CELL_VOLTAGE_MV 4250  // Battery is put into emergency stop if one cell goes over this value
#define MIN_CELL_VOLTAGE_MV 2700  // Battery is put into emergency stop if one cell goes below this value

// System constants
#define INTERVAL_10_MS 10
#define INTERVAL_100_MS 100
#define INTERVAL_1_S 1000
#define INTERVAL_10_S 10000
#define INTERVAL_10_MS_DELAYED 20   // Threshold for CAN overrun detection (in ms)
#define BOOTUP_TIME 5000            // Wait 5 seconds on startup before checking CAN timing

// Battery physical specifications
#define NUMBER_OF_CELLS 96  // MG ZS EV battery has 96 cells

void setup_battery(void);
void update_values_battery(void);
void handle_incoming_can_frame_battery(CAN_frame rx_frame);
void transmit_can_battery(void);

// External function that should be declared in a common header
void transmit_can_frame(CAN_frame* tx_frame, int interface);

#endif
