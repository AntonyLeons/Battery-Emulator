#include "../include.h"
#ifdef MG_ZS_BATTERY
#include "../datalayer/datalayer.h"
#include "../datalayer/datalayer_extended.h"
#include "../devboard/utils/events.h"
#include "MG-ZS-BATTERY.h"

/* TODO: 
- Get contactor closing working
- Figure out which CAN messages need to be sent towards the battery to keep it alive
- Map all values from battery CAN messages
- Most important ones 
*/

/* Do not change code below unless you are sure what you are doing */
static unsigned long previousMillis10 = 0;   // will store last time a 10ms CAN Message was send
static unsigned long previousMillis100 = 0;  // will store last time a 100ms CAN Message was send

// Basic battery state variables
static uint16_t SOC_BMS = 7500;              // Battery SOC in 0.01% (7500 = 75.00%)
static uint16_t SOC_Display = 7500;          // Display SOC in 0.01% (7500 = 75.00%)
static uint16_t batterySOH = 9800;           // Battery State of Health in 0.01% (9800 = 98.00%)
static uint16_t batteryVoltage = 3850;       // Battery voltage in decivolts (3850 = 385.0V)
static int16_t batteryCurrent = 0;           // Battery current in deciamps (150 = 15.0A)
static uint16_t CellVoltMax_mV = 3800;       // Maximum cell voltage in mV
static uint16_t CellVoltMin_mV = 3700;       // Minimum cell voltage in mV
static uint8_t CellVmaxNo = 0;               // Cell number with maximum voltage
static uint8_t CellVminNo = 0;               // Cell number with minimum voltage
static int16_t temperatureMax = 250;         // Maximum temperature in decicelsius (250 = 25.0°C)
static int16_t temperatureMin = 220;         // Minimum temperature in decicelsius (220 = 22.0°C)
static uint16_t allowedDischargePower = 50000; // Max discharge power in watts (50000 = 50kW)
static uint16_t allowedChargePower = 50000;    // Max charge power in watts (50000 = 50kW)
static int16_t leadAcidBatteryVoltage = 120;   // 12V battery voltage in decivolts (120 = 12.0V)
static bool contactorStatus = false;           // Contactor status (false = open, true = closed)
static uint16_t cellVoltages[108] = {0};       // Array to store cell voltages
static uint8_t num_cells = 96;                 // Number of cells in the battery pack
static bool isolation_status = true;           // Battery isolation status
static uint8_t counter_10ms = 0;               // Counter for 10ms messages
static uint8_t counter_100ms = 0;              // Counter for 100ms messages

// Main CAN frames needed for MG ZS EV battery emulation
// Updated IDs based on CAN logs
CAN_frame MG_ZS_0FB = {.FD = false,
                      .ext_ID = false,
                      .DLC = 8,
                      .ID = 0x0FB,
                      .data = {0x0D, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x05, 0xFF}};

// UDS diagnostic response frame
CAN_frame MG_ZS_UDS_RESPONSE = {.FD = false,
                              .ext_ID = false,
                              .DLC = 8, 
                              .ID = 0x789,
                              .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

// Battery status frame
CAN_frame MG_ZS_19C = {.FD = false,
                      .ext_ID = false,
                      .DLC = 8,
                      .ID = 0x19C,
                      .data = {0x06, 0xA0, 0x26, 0xA0, 0x7F, 0xFF, 0x00, 0x7F}};

// SOC and temperature frame
CAN_frame MG_ZS_1E5 = {.FD = false,
                      .ext_ID = false,
                      .DLC = 8,
                      .ID = 0x1E5,
                      .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

// Battery voltage and current frame
CAN_frame MG_ZS_232 = {.FD = false,
                      .ext_ID = false,
                      .DLC = 8,
                      .ID = 0x232,
                      .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

// Status information frame
CAN_frame MG_ZS_0C1 = {.FD = false,
                      .ext_ID = false,
                      .DLC = 8,
                      .ID = 0x0C1,
                      .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

// Power limits and control frame
CAN_frame MG_ZS_0C5 = {.FD = false,
                      .ext_ID = false,
                      .DLC = 8,
                      .ID = 0x0C5,
                      .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

// Cell voltage information frame
CAN_frame MG_ZS_1C7 = {.FD = false,
                      .ext_ID = false,
                      .DLC = 8,
                      .ID = 0x1C7,
                      .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

// Additional status frame
CAN_frame MG_ZS_0AC = {.FD = false,
                      .ext_ID = false,
                      .DLC = 8,
                      .ID = 0x0AC,
                      .data = {0x06, 0xA0, 0x00, 0xA0, 0x00, 0x00, 0x00, 0x00}};

// Additional status frame
CAN_frame MG_ZS_0AF = {.FD = false,
                      .ext_ID = false,
                      .DLC = 8,
                      .ID = 0x0AF,
                      .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

// Handle UDS diagnostic requests according to MG ZS EV protocol
void handle_uds_request(CAN_frame rx_frame) {
  // Check service type in the second byte (B1)
  if (rx_frame.data.u8[1] == 0x22) { // Read Data By Identifier
    // The parameter ID is in bytes B2 and B3
    uint8_t pid_high = rx_frame.data.u8[2];
    uint8_t pid_low = rx_frame.data.u8[3];
    
    // Clear the response frame first
    memset(MG_ZS_UDS_RESPONSE.data.u8, 0, 8);
    
    // Set standard positive response header
    MG_ZS_UDS_RESPONSE.data.u8[0] = 0x04;  // Default length (may be updated)
    MG_ZS_UDS_RESPONSE.data.u8[1] = 0x62;  // 0x62 is positive response to 0x22
    MG_ZS_UDS_RESPONSE.data.u8[2] = pid_high;
    MG_ZS_UDS_RESPONSE.data.u8[3] = pid_low;
    
    // Handle the specific PID
    if (pid_high == 0xB0) {
      switch (pid_low) {
        case 0x41:  // Battery Bus Voltage
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x04;
          MG_ZS_UDS_RESPONSE.data.u8[4] = 0x06;
          MG_ZS_UDS_RESPONSE.data.u8[5] = 0x8F;
          // Fill remaining bytes with 0xAA (unused)
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0xAA;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0xAA;
          break;
          
        case 0x42:  // Battery Voltage
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
          // Convert batteryVoltage (in decivolts) to the format expected by UDS
          uint16_t voltage = batteryVoltage;  // in decivolts (385.0V = 3850)
          MG_ZS_UDS_RESPONSE.data.u8[4] = (voltage >> 8) & 0xFF;  // High byte
          MG_ZS_UDS_RESPONSE.data.u8[5] = voltage & 0xFF;        // Low byte
          // Fill remaining bytes with 0xAA (unused)
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0xAA;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0xAA;
          break;
          
        case 0x43:  // Battery Current
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
          // Convert batteryCurrent (in deciamps) to the format expected by UDS
          // Example in the table shows 9C 53 which would be a specific current value
          // We'll use the current value from our model
          int16_t current = batteryCurrent * 10;  // Convert to deciamps (for consistency)
          MG_ZS_UDS_RESPONSE.data.u8[4] = (current >> 8) & 0xFF;  // High byte
          MG_ZS_UDS_RESPONSE.data.u8[5] = current & 0xFF;        // Low byte
          // Fill remaining bytes with 0xAA (unused)
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0xAA;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0xAA;
          break;
          
        case 0x45:  // Battery Resistance
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
          // Use a fixed resistance value (3F FD from the example)
          MG_ZS_UDS_RESPONSE.data.u8[4] = 0x3F;
          MG_ZS_UDS_RESPONSE.data.u8[5] = 0xFD;
          // Fill remaining bytes with 0xAA (unused)
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0xAA;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0xAA;
          break;
          
        case 0x46:  // Battery SoC
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
          // Convert SOC_BMS (0.01%) to the format expected by UDS
          // Example in table shows 02 85 which is probably a percentage value
          uint16_t soc = SOC_BMS / 25;  // Convert from 0.01% to the UDS format
          MG_ZS_UDS_RESPONSE.data.u8[4] = (soc >> 8) & 0xFF;  // High byte
          MG_ZS_UDS_RESPONSE.data.u8[5] = soc & 0xFF;        // Low byte
          // Fill remaining bytes with 0xAA (unused)
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0xAA;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0xAA;
          break;
          
        case 0x47:  // BMS Error
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
          // No errors in our emulator
          MG_ZS_UDS_RESPONSE.data.u8[4] = 0x00;
          MG_ZS_UDS_RESPONSE.data.u8[5] = 0x00;
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
          break;
          
        case 0x48:  // BMS Status
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x04;
          // Status code 03 from the example
          MG_ZS_UDS_RESPONSE.data.u8[4] = 0x03;
          // Fill remaining bytes with 0xAA (unused)
          MG_ZS_UDS_RESPONSE.data.u8[5] = 0xAA;
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0xAA;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0xAA;
          break;
          
        case 0x49:  // System Main Relay B
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
          // Always 1 according to the note
          MG_ZS_UDS_RESPONSE.data.u8[4] = 0x01;
          MG_ZS_UDS_RESPONSE.data.u8[5] = 0x00;
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
          break;
          
        case 0x4A:  // System Main Relay G
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
          // Always 1 according to the note
          MG_ZS_UDS_RESPONSE.data.u8[4] = 0x01;
          MG_ZS_UDS_RESPONSE.data.u8[5] = 0x00;
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
          break;
          
        case 0x52:  // System Main Relay P
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
          // Always 1 according to the note
          MG_ZS_UDS_RESPONSE.data.u8[4] = 0x01;
          MG_ZS_UDS_RESPONSE.data.u8[5] = 0x00;
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
          break;
          
        case 0x56:  // Battery Temp
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x04;
          // Use our temperature value, converted to the format expected by UDS
          MG_ZS_UDS_RESPONSE.data.u8[4] = temperatureMax & 0xFF;
          MG_ZS_UDS_RESPONSE.data.u8[5] = 0x00;
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
          break;
          
        case 0x58:  // Max Cell Voltage
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x06;
          // Use our max cell voltage
          MG_ZS_UDS_RESPONSE.data.u8[4] = (CellVoltMax_mV >> 8) & 0xFF;
          MG_ZS_UDS_RESPONSE.data.u8[5] = CellVoltMax_mV & 0xFF;
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0xFF;  // Separator or indicator
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0xAA;  // Unused
          break;
          
        case 0x59:  // Min Cell Voltage
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x06;
          // Use our min cell voltage
          MG_ZS_UDS_RESPONSE.data.u8[4] = (CellVoltMin_mV >> 8) & 0xFF;
          MG_ZS_UDS_RESPONSE.data.u8[5] = CellVoltMin_mV & 0xFF;
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0xFF;  // Separator or indicator
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;  // Unused
          break;
          
        case 0x5C:  // Battery Coolant Temp
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x04;
          // Use a default coolant temperature (0x6D from example)
          MG_ZS_UDS_RESPONSE.data.u8[4] = 0x6D;  // About 25°C in their format
          // Fill remaining bytes with 0xAA (unused)
          MG_ZS_UDS_RESPONSE.data.u8[5] = 0xAA;
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0xAA;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0xAA;
          break;
          
        case 0x61:  // Battery SoH
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
          // Provide battery SOH - example shows 27 10 = ~98%
          MG_ZS_UDS_RESPONSE.data.u8[4] = (batterySOH >> 8) & 0xFF;
          MG_ZS_UDS_RESPONSE.data.u8[5] = batterySOH & 0xFF;
          // Fill remaining bytes with 0xAA (unused)
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0xAA;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0xAA;
          break;
          
        case 0x6D:  // BMS Time
          // This is a multi-frame response, but we'll simulate with what fits
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x10;  // First frame of multi-frame response
          MG_ZS_UDS_RESPONSE.data.u8[1] = 0x09;  // Additional length
          MG_ZS_UDS_RESPONSE.data.u8[2] = 0x62;  // Service response
          MG_ZS_UDS_RESPONSE.data.u8[3] = 0xB0;
          MG_ZS_UDS_RESPONSE.data.u8[4] = 0x6D;
          
          // Use current date/time for the response
          // Format: YY MM DD HH MM SS
          MG_ZS_UDS_RESPONSE.data.u8[5] = 0x15;  // Year (e.g., 2021 = 0x15)
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0x04;  // Month (e.g., April = 0x04)
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0x12;  // Day (e.g., 18th = 0x12)
          break;
          
        default:
          // Unknown PID - send negative response
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x03;
          MG_ZS_UDS_RESPONSE.data.u8[1] = 0x7F;  // Negative response
          MG_ZS_UDS_RESPONSE.data.u8[2] = 0x22;  // Service ID
          MG_ZS_UDS_RESPONSE.data.u8[3] = 0x11;  // Error code: service not supported
          break;
      }
    } 
    // Handle DCDC requests
    else if (rx_frame.ID == 0x785 && pid_high == 0xB0) {
      // We're going to emulate DCDC responses
      switch (pid_low) {
        case 0x22:  // DCDC LV Current
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
          MG_ZS_UDS_RESPONSE.data.u8[4] = 0x00;  // High byte
          MG_ZS_UDS_RESPONSE.data.u8[5] = 0x0C;  // Low byte
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0xAA;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0xAA;
          break;
          
        case 0x21:  // DCDC LV Voltage
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
          MG_ZS_UDS_RESPONSE.data.u8[4] = 0x00;  // High byte
          MG_ZS_UDS_RESPONSE.data.u8[5] = 0x76;  // Low byte
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0xAA;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0xAA;
          break;
          
        case 0x25:  // DCDC Power Load
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
          MG_ZS_UDS_RESPONSE.data.u8[4] = 0x00;  // High byte
          MG_ZS_UDS_RESPONSE.data.u8[5] = 0x1B;  // Low byte
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0xAA;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0xAA;
          break;
          
        case 0x26:  // DCDC Temperature
          MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
          MG_ZS_UDS_RESPONSE.data.u8[4] = 0x00;  // High byte
          MG_ZS_UDS_RESPONSE.data.u8[5] = 0x4A;  // Low byte
          MG_ZS_UDS_RESPONSE.data.u8[6] = 0xAA;
          MG_ZS_UDS_RESPONSE.data.u8[7] = 0xAA;
          break;
      }
    }
    // Handle VCU requests
    else if (rx_frame.ID == 0x7E3) {
      // We're simulating VCU responses here
      switch (pid_high) {
        case 0xBA:
          if (pid_low == 0x00) {  // Vehicle Speed
            MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
            MG_ZS_UDS_RESPONSE.data.u8[4] = 0x59;  // High byte
            MG_ZS_UDS_RESPONSE.data.u8[5] = 0x23;  // Low byte
            MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;  // Additional bytes
            MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
          }
          break;
          
        case 0xB7:
          switch (pid_low) {
            case 0x1B:  // Charger Connected
              MG_ZS_UDS_RESPONSE.data.u8[0] = 0x04;
              MG_ZS_UDS_RESPONSE.data.u8[4] = 0x00;  // 0 = not connected
              MG_ZS_UDS_RESPONSE.data.u8[5] = 0x00;
              MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
              MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
              break;
              
            case 0x12:  // Max Charge Rate
              MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
              MG_ZS_UDS_RESPONSE.data.u8[4] = 0x01;  // High byte
              MG_ZS_UDS_RESPONSE.data.u8[5] = 0x9C;  // Low byte
              MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
              MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
              break;
              
            case 0x02:  // BMS Running State
              MG_ZS_UDS_RESPONSE.data.u8[0] = 0x04;
              MG_ZS_UDS_RESPONSE.data.u8[4] = 0x03;  // From example
              MG_ZS_UDS_RESPONSE.data.u8[5] = 0x00;
              MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
              MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
              break;
              
            case 0x03:  // HV Contactors
              MG_ZS_UDS_RESPONSE.data.u8[0] = 0x04;
              MG_ZS_UDS_RESPONSE.data.u8[4] = contactorStatus ? 0x01 : 0x00;
              MG_ZS_UDS_RESPONSE.data.u8[5] = 0x00;
              MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
              MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
              break;
              
            case 0x05:  // Battery Voltage (via VCU)
              MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
              // Different format than the BMS reports
              uint16_t vcuVoltage = batteryVoltage / 10;  // Adjust to match example format
              MG_ZS_UDS_RESPONSE.data.u8[4] = (vcuVoltage >> 8) & 0xFF;
              MG_ZS_UDS_RESPONSE.data.u8[5] = vcuVoltage & 0xFF;
              MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
              MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
              break;
          }
          break;
          
        case 0xB3:
          if (pid_low == 0x09) {  // Motor Coolant
            MG_ZS_UDS_RESPONSE.data.u8[0] = 0x04;
            MG_ZS_UDS_RESPONSE.data.u8[4] = 0x4A;  // ~35°C in their format
            MG_ZS_UDS_RESPONSE.data.u8[5] = 0x00;
            MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
            MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
          }
          break;
          
        case 0xB4:
          switch (pid_low) {
            case 0x05:  // Motor Temp
              MG_ZS_UDS_RESPONSE.data.u8[0] = 0x04;
              MG_ZS_UDS_RESPONSE.data.u8[4] = 0x4A;  // Same as coolant
              MG_ZS_UDS_RESPONSE.data.u8[5] = 0x00;
              MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
              MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
              break;
              
            case 0x01:  // Motor Torque
              MG_ZS_UDS_RESPONSE.data.u8[0] = 0x04;
              MG_ZS_UDS_RESPONSE.data.u8[4] = 0x7F;  // From example
              MG_ZS_UDS_RESPONSE.data.u8[5] = 0xFF;
              MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
              MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
              break;
              
            case 0x02:  // Motor Speed
              MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
              MG_ZS_UDS_RESPONSE.data.u8[4] = 0x7F;  // From example
              MG_ZS_UDS_RESPONSE.data.u8[5] = 0xFF;
              MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
              MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
              break;
          }
          break;
          
        case 0x01:
          if (pid_low == 0x12) {  // 12V Supply Voltage
            MG_ZS_UDS_RESPONSE.data.u8[0] = 0x05;
            MG_ZS_UDS_RESPONSE.data.u8[4] = 0x8E;  // From example
            MG_ZS_UDS_RESPONSE.data.u8[5] = 0x00;
            MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
            MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
          }
          break;
          
        case 0xB1:
          if (pid_low == 0x8C) {  // Ignition State
            MG_ZS_UDS_RESPONSE.data.u8[0] = 0x04;
            MG_ZS_UDS_RESPONSE.data.u8[4] = 0x02;  // From example
            MG_ZS_UDS_RESPONSE.data.u8[5] = 0x00;
            MG_ZS_UDS_RESPONSE.data.u8[6] = 0x00;
            MG_ZS_UDS_RESPONSE.data.u8[7] = 0x00;
          }
          break;
      }
    }
    // Update the ID to match the expected response address
    if (rx_frame.ID == 0x781) {
      MG_ZS_UDS_RESPONSE.ID = 0x789;  // BMS response ID
    } else if (rx_frame.ID == 0x785) {
      MG_ZS_UDS_RESPONSE.ID = 0x78D;  // DCDC response ID
    } else if (rx_frame.ID == 0x7E3) {
      MG_ZS_UDS_RESPONSE.ID = 0x7EB;  // VCU response ID
    }
    
    // Send the response
    transmit_can_frame(&MG_ZS_UDS_RESPONSE, can_config.battery);
    
#ifdef DEBUG_LOG
    logging.print("MG-ZS UDS request: 0x");
    logging.print(rx_frame.ID, HEX);
    logging.print(" PID ");
    logging.print(pid_high, HEX);
    logging.print(" ");
    logging.println(pid_low, HEX);
#endif
  }
  else if (rx_frame.data.u8[1] == 0x2E) {
    // Write data by identifier - we could implement this later
    // For now, just send a positive response mirroring the request
    MG_ZS_UDS_RESPONSE.data.u8[0] = 0x03;
    MG_ZS_UDS_RESPONSE.data.u8[1] = 0x6E;  // Positive response to 0x2E
    MG_ZS_UDS_RESPONSE.data.u8[2] = rx_frame.data.u8[2];
    MG_ZS_UDS_RESPONSE.data.u8[3] = rx_frame.data.u8[3];
    // Rest of the bytes stay 0
    
    // Update the ID to match the expected response address
    if (rx_frame.ID == 0x781) {
      MG_ZS_UDS_RESPONSE.ID = 0x789;
    } else if (rx_frame.ID == 0x785) {
      MG_ZS_UDS_RESPONSE.ID = 0x78D;
    } else if (rx_frame.ID == 0x7E3) {
      MG_ZS_UDS_RESPONSE.ID = 0x7EB;
    }
    
    // Send the response
    transmit_can_frame(&MG_ZS_UDS_RESPONSE, can_config.battery);
  }
  else if (rx_frame.data.u8[1] == 0x10) {
    // Diagnostic session control - respond with positive acknowledgment
    MG_ZS_UDS_RESPONSE.data.u8[0] = 0x03;
    MG_ZS_UDS_RESPONSE.data.u8[1] = 0x50;  // Positive response to 0x10
    MG_ZS_UDS_RESPONSE.data.u8[2] = rx_frame.data.u8[2]; // Session type
    MG_ZS_UDS_RESPONSE.data.u8[3] = 0x00;
    // Rest of the bytes stay 0
    
    // Update the ID to match the expected response address
    if (rx_frame.ID == 0x781) {
      MG_ZS_UDS_RESPONSE.ID = 0x789;
    } else if (rx_frame.ID == 0x785) {
      MG_ZS_UDS_RESPONSE.ID = 0x78D;
    } else if (rx_frame.ID == 0x7E3) {
      MG_ZS_UDS_RESPONSE.ID = 0x7EB;
    }
    
    // Send the response
    transmit_can_frame(&MG_ZS_UDS_RESPONSE, can_config.battery);
  }
  else {
    // Unsupported UDS service - send negative response
    MG_ZS_UDS_RESPONSE.data.u8[0] = 0x03;
    MG_ZS_UDS_RESPONSE.data.u8[1] = 0x7F;  // Negative response
    MG_ZS_UDS_RESPONSE.data.u8[2] = rx_frame.data.u8[1];  // Service ID that was requested
    MG_ZS_UDS_RESPONSE.data.u8[3] = 0x11;  // Error code: service not supported
    // Rest of the bytes stay 0
    
    // Update the ID to match the expected response address
    if (rx_frame.ID == 0x781) {
      MG_ZS_UDS_RESPONSE.ID = 0x789;
    } else if (rx_frame.ID == 0x785) {
      MG_ZS_UDS_RESPONSE.ID = 0x78D;
    } else if (rx_frame.ID == 0x7E3) {
      MG_ZS_UDS_RESPONSE.ID = 0x7EB;
    }
    
    // Send the response
    transmit_can_frame(&MG_ZS_UDS_RESPONSE, can_config.battery);
  }
}

void handle_incoming_can_frame_battery(CAN_frame rx_frame) {
  datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
  
  // Declare variables outside the switch statement to avoid jump errors
  uint8_t socValue = 0;
  uint8_t powerIndicator = 0;
  uint8_t remainingData = 0;
  
  switch (rx_frame.ID) {
    // Handle UDS diagnostic requests (0x781 is UDS request ID)
    case 0x781:
      handle_uds_request(rx_frame);
      break;
      
    case 0x0AF:  // Battery state data frame
      // Based on log analysis: bytes 2, 5, 6, 7 contain important data
      
      // Byte 5 appears to be SOC value
      // From the logs, we can see it goes from 0x81 (129 decimal) which is 81% SOC
      socValue = rx_frame.data.u8[5];
      if (socValue <= 100) {
        SOC_BMS = socValue * 100; // Convert to 0.01% format (e.g., 81 -> 8100)
      }
      
      // Bytes 2 and 6 appear to be related to remaining capacity or power
      powerIndicator = rx_frame.data.u8[2];
      remainingData = rx_frame.data.u8[6];
      
      // Decode power values - values from log show correlation between byte 2 and available power
      // Higher values in byte 2 appear when SOC is higher
      allowedDischargePower = 40000 + (powerIndicator * 200);
      allowedChargePower = 30000 - ((100 - socValue) * 200);
      
      // Update battery voltage based on SOC (simple estimate)
      // SOC 0% = ~310V, SOC 100% = ~404V
      batteryVoltage = MIN_PACK_VOLTAGE_DV + ((MAX_PACK_VOLTAGE_DV - MIN_PACK_VOLTAGE_DV) * SOC_BMS / 10000);

      // Update battery status
      if (remainingData > 0xF0) {
        batteryCurrent = 0; // Idle state
      } else if (socValue < 100) {
        batteryCurrent = 10; // Charging at low current
      } else {
        batteryCurrent = 0; // Fully charged
      }
      
#ifdef DEBUG_LOG
      logging.printf("MG SOC: %d.%02d%%, Power: %dkW/%dkW\n", 
                    SOC_BMS/100, SOC_BMS%100, 
                    allowedDischargePower/1000, allowedChargePower/1000);
#endif
      break;
      
    case 0x171:  // BMS status request
      // Vehicle is requesting BMS status information
      // Set appropriate response in the next transmission cycle
#ifdef DEBUG_LOG
      logging.println("MGZS: BMS status request received");
#endif
      break;
      
    case 0x172:  // BMS command frame - contactor control
      // Check if this is a contactor close/open request
      if ((rx_frame.data.u8[0] & 0x01) == 0x01) {  // Bit 0 = contactor request
        bool requestStatus = (rx_frame.data.u8[1] & 0x01);
        
        // Only allow closing if isolation is good and SOC within safe limits
        if (requestStatus && isolation_status && (SOC_BMS > 100) && (SOC_BMS < 9900)) {
          contactorStatus = true;
          datalayer.battery.status.real_bms_status = BMS_ACTIVE;
#ifdef DEBUG_LOG
          logging.println("MGZS: Contactor close command accepted");
#endif
        } else {
          contactorStatus = false;
          datalayer.battery.status.real_bms_status = BMS_STANDBY;
#ifdef DEBUG_LOG
          logging.println("MGZS: Contactor open command executed or close rejected");
          if (!isolation_status) {
            logging.println("MGZS: Contactor close rejected due to isolation fault");
          }
          if (SOC_BMS <= 100) {
            logging.println("MGZS: Contactor close rejected due to low SOC");
          }
          if (SOC_BMS >= 9900) {
            logging.println("MGZS: Contactor close rejected due to high SOC");
          }
#endif
        }
      }
      break;
      
    case 0x173:  // System safety check
      // Update isolation status based on vehicle request
      // Typically we just keep isolation status as good unless there's an issue
      break;
      
    case 0x293:  // SOC request
      // Vehicle is requesting SOC information
      // Will be handled in the transmit function
      break;
      
    case 0x295:  // Battery configuration
      // Process battery configuration messages
      break;
      
    case 0x297:  // Extended battery status
      // Handle extended status messages
      break;
      
    case 0x334:  // Voltage/Current request
      // Vehicle may request real-time voltage/current
      // Will be handled in the transmit function
      break;
      
    case 0x391:  // Vehicle charge request
      // Handle charge control messages
      if ((rx_frame.data.u8[0] & 0x10) == 0x10) {  // Charging requested
        // Set up charging mode parameters
        batteryCurrent = 50;  // 5.0A charging current (example)
        
        // Adjust power limits during charging
        allowedChargePower = 40000;     // 40kW max charge
        allowedDischargePower = 1000;   // Limit discharge during charging
#ifdef DEBUG_LOG
        logging.println("MGZS: Charging mode activated");
#endif
      } else {
        // Normal driving mode
        allowedChargePower = 50000;     // 50kW max charge
        allowedDischargePower = 50000;  // 50kW max discharge
        batteryCurrent = 0;             // Reset current when not charging
      }
      break;
      
    case 0x3BC:  // Power limits request
      // Vehicle may request updated power limits
      // Will be handled in the transmit function
      break;
      
    case 0x3C0:  // Thermal management
      // Handle thermal management requests
      if (rx_frame.data.u8[0] & 0x01) {  // Cooling request
        // Acknowledge cooling request
#ifdef DEBUG_LOG
        logging.println("MGZS: Battery cooling requested");
#endif
      } else if (rx_frame.data.u8[0] & 0x02) { // Heating request
        // Acknowledge heating request
#ifdef DEBUG_LOG
        logging.println("MGZS: Battery heating requested");
#endif
      }
      break;
      
    case 0x620:  // Wake-up message
      // Vehicle may send this to wake up the battery
#ifdef DEBUG_LOG
      logging.println("MGZS: Battery wake-up message received");
#endif
      break;
      
    default:
      // Unknown message - ignore
      break;
  }
  
  // Calculate cell statistics and update min/max values
  // In a real implementation, this would be based on actual data
  
  // Simulate battery behavior (discharge when current is negative, charge when positive)
  static unsigned long lastUpdate = millis();
  unsigned long now = millis();
  unsigned long elapsed = now - lastUpdate;
  lastUpdate = now;
  
  // Only update SOC if enough time has passed (at least 100ms)
  if (elapsed > 100) {
    float hoursFraction = elapsed / 3600000.0; // Convert ms to hours
    
    // Calculate energy change (in Wh) based on current
    float energyChange = (batteryCurrent / 10.0) * (batteryVoltage / 10.0) * hoursFraction;
    
    // For a 44.5kWh battery, 1Wh = ~0.00225% SOC change
    float socChangePercent = energyChange * 100.0 / 44500.0;
    uint16_t socChange = (uint16_t)(socChangePercent * 100); // Convert to 0.01% units
    
    // Apply change with bounds checking
    if (batteryCurrent < 0) { // Discharging
      if (SOC_BMS > socChange) {
        SOC_BMS -= socChange;
      } else {
        SOC_BMS = 0;
      }
    } else if (batteryCurrent > 0) { // Charging
      if (SOC_BMS < (10000 - socChange)) {
        SOC_BMS += socChange;
      } else {
        SOC_BMS = 10000;
      }
    }
    
    // Update cell voltages based on SOC (simple model)
    // SOC 0% = ~3.1V per cell, SOC 100% = ~4.0V per cell
    uint16_t baseVoltage = 3100; // 3.1V in mV
    uint16_t voltageRange = 900;  // 0.9V range in mV
    
    uint16_t avgCellVoltage = baseVoltage + (voltageRange * SOC_BMS / 10000);
    
    // Update cell voltages with some variation
    for (uint8_t i = 0; i < num_cells; i++) {
      // Add some random variation between cells (±20mV)
      int16_t variation = -20 + (i % 41); // -20 to +20 mV variation based on cell index
      cellVoltages[i] = avgCellVoltage + variation;
    }
    
    // Find min/max cell voltages
    CellVoltMax_mV = cellVoltages[0];
    CellVoltMin_mV = cellVoltages[0];
    CellVmaxNo = 0;
    CellVminNo = 0;
    
    for (uint8_t i = 1; i < num_cells; i++) {
      if (cellVoltages[i] > CellVoltMax_mV) {
        CellVoltMax_mV = cellVoltages[i];
        CellVmaxNo = i;
      }
      if (cellVoltages[i] < CellVoltMin_mV) {
        CellVoltMin_mV = cellVoltages[i];
        CellVminNo = i;
      }
    }
    
    // Calculate pack voltage based on cell voltages
    batteryVoltage = ((uint32_t)avgCellVoltage * num_cells) / 100; // Convert to decivolts
    
    // Update the display SOC to follow the real SOC (with some lag)
    if (SOC_Display < SOC_BMS) {
      SOC_Display += 1 + (SOC_BMS - SOC_Display) / 100; // Faster catch-up for larger differences
    } else if (SOC_Display > SOC_BMS) {
      SOC_Display -= 1 + (SOC_Display - SOC_BMS) / 100; // Faster catch-up for larger differences
    }
  }
}

void update_values_battery() {
  if (datalayer.system.settings.equipment_stop_active == true) {
    // If emergency stop is active, don't wake up the battery
    return;
  }

  // Map all values from MG ZS EV battery to the datalayer structure
  datalayer.battery.status.real_soc = SOC_BMS;  // Use internal BMS SOC
  datalayer.battery.status.voltage_dV = batteryVoltage;  // Total pack voltage in decivolts
  datalayer.battery.status.current_dA = batteryCurrent * 10;  // Convert from amps to deciamps

  // Map power limits
  datalayer.battery.status.max_discharge_power_W = allowedDischargePower;
  datalayer.battery.status.max_charge_power_W = allowedChargePower;

  // Map temperature readings
  datalayer.battery.status.temperature_min_dC = temperatureMin * 10; // Convert to decicelsius
  datalayer.battery.status.temperature_max_dC = temperatureMax * 10; // Convert to decicelsius

  // Calculate cell statistics for BMS safety checks
  if (num_cells > 0) {
    uint16_t min_cell_v = 9999;
    uint16_t max_cell_v = 0;
    for (uint8_t i = 0; i < num_cells; i++) {
      if (cellVoltages[i] < min_cell_v && cellVoltages[i] > 0) {
        min_cell_v = cellVoltages[i];
      }
      if (cellVoltages[i] > max_cell_v) {
        max_cell_v = cellVoltages[i];
      }
    }
    
    datalayer.battery.status.cell_min_voltage_mV = min_cell_v;
    datalayer.battery.status.cell_max_voltage_mV = max_cell_v;
    
    // Check if voltage spread is within safe limits
    uint16_t voltage_spread = max_cell_v - min_cell_v;
    if (voltage_spread > MAX_CELL_DEVIATION_MV) {
      set_event(EVENT_CELL_DEVIATION_HIGH, voltage_spread);
    } else {
      clear_event(EVENT_CELL_DEVIATION_HIGH);
    }
  }

  // Battery design parameters
  datalayer.battery.info.number_of_cells = num_cells;
  datalayer.battery.info.max_design_voltage_dV = MAX_PACK_VOLTAGE_DV;
  datalayer.battery.info.min_design_voltage_dV = MIN_PACK_VOLTAGE_DV;
  datalayer.battery.info.max_cell_voltage_mV = MAX_CELL_VOLTAGE_MV;
  datalayer.battery.info.min_cell_voltage_mV = MIN_CELL_VOLTAGE_MV;
  datalayer.battery.info.max_cell_voltage_deviation_mV = MAX_CELL_DEVIATION_MV;

  // Safety checks
  if (!isolation_status) {
    set_event(EVENT_BATTERY_ISOLATION, 0);
  } else {
    clear_event(EVENT_BATTERY_ISOLATION);
  }

  // Allow contactor closing if the battery is detected on CAN
  if (SOC_BMS > 0) {
    datalayer.system.status.battery_allows_contactor_closing = true;
    datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
  }
}

void transmit_can_battery() {
  unsigned long currentMillis = millis();
  
  //Send 10ms message
  if (currentMillis - previousMillis10 >= INTERVAL_10_MS) {
    // Check if sending of CAN messages has been delayed too much.
    if ((currentMillis - previousMillis10 >= INTERVAL_10_MS_DELAYED) && (currentMillis > BOOTUP_TIME)) {
      set_event(EVENT_CAN_OVERRUN, (currentMillis - previousMillis10));
    } else {
      clear_event(EVENT_CAN_OVERRUN);
    }
    previousMillis10 = currentMillis;
    
    counter_10ms = (counter_10ms + 1) % 16;  // 4-bit counter (0-15)
    
    // Update main keep-alive frame
    MG_ZS_0FB.data.u8[0] = counter_10ms & 0x0F;
    
    // Transmit 10ms frame
    transmit_can_frame(&MG_ZS_0FB, can_config.battery);
  }
  
  // Send 100ms CAN Messages
  if (currentMillis - previousMillis100 >= INTERVAL_100_MS) {
    previousMillis100 = currentMillis;
    
    counter_100ms = (counter_100ms + 1) % 16;  // 4-bit counter (0-15)
    
    // Battery status frame - 0x19C
    MG_ZS_19C.data.u8[0] = counter_100ms & 0x0F;
    MG_ZS_19C.data.u8[1] = isolation_status ? 0x01 : 0x00;  // Bit 0 = isolation status
    MG_ZS_19C.data.u8[2] = contactorStatus ? 0x01 : 0x00;   // Bit 0 = contactor status
    MG_ZS_19C.data.u8[7] = 0x01;  // Default ready state
    transmit_can_frame(&MG_ZS_19C, can_config.battery);
    
    // SOC frame - 0x1E5
    MG_ZS_1E5.data.u8[0] = counter_100ms & 0x0F;
    MG_ZS_1E5.data.u8[1] = (SOC_BMS >> 8) & 0xFF;  // High byte of SOC
    MG_ZS_1E5.data.u8[2] = SOC_BMS & 0xFF;         // Low byte of SOC
    MG_ZS_1E5.data.u8[3] = (batterySOH >> 8) & 0xFF;  // High byte of SOH
    MG_ZS_1E5.data.u8[4] = batterySOH & 0xFF;         // Low byte of SOH
    transmit_can_frame(&MG_ZS_1E5, can_config.battery);
    
    // Voltage and current frame - 0x232
    MG_ZS_232.data.u8[0] = counter_100ms & 0x0F;
    // Pack voltage (decivolts)
    MG_ZS_232.data.u8[1] = (batteryVoltage >> 8) & 0xFF;  // High byte
    MG_ZS_232.data.u8[2] = batteryVoltage & 0xFF;         // Low byte
    // Current (deciamps) - Note: may need to convert signed to unsigned for CAN
    uint16_t currentUnsigned = batteryCurrent < 0 ? 0x8000 | (abs(batteryCurrent) & 0x7FFF) : batteryCurrent & 0x7FFF;
    MG_ZS_232.data.u8[3] = (currentUnsigned >> 8) & 0xFF;  // High byte
    MG_ZS_232.data.u8[4] = currentUnsigned & 0xFF;         // Low byte
    MG_ZS_232.data.u8[5] = leadAcidBatteryVoltage & 0xFF;  // 12V battery voltage
    transmit_can_frame(&MG_ZS_232, can_config.battery);
    
    // Temperature frame - 0x0C1
    MG_ZS_0C1.data.u8[0] = counter_100ms & 0x0F;
    MG_ZS_0C1.data.u8[1] = temperatureMax & 0xFF;   // Max temperature
    MG_ZS_0C1.data.u8[2] = temperatureMin & 0xFF;   // Min temperature
    transmit_can_frame(&MG_ZS_0C1, can_config.battery);
    
    // Power limits frame - 0x0C5
    MG_ZS_0C5.data.u8[0] = counter_100ms & 0x0F;
    // Allowed charge power (in 100W units)
    uint16_t chargePower = allowedChargePower / 100;
    MG_ZS_0C5.data.u8[1] = (chargePower >> 8) & 0xFF;  // High byte
    MG_ZS_0C5.data.u8[2] = chargePower & 0xFF;         // Low byte
    // Allowed discharge power (in 100W units)
    uint16_t dischargePower = allowedDischargePower / 100;
    MG_ZS_0C5.data.u8[3] = (dischargePower >> 8) & 0xFF;  // High byte
    MG_ZS_0C5.data.u8[4] = dischargePower & 0xFF;         // Low byte
    transmit_can_frame(&MG_ZS_0C5, can_config.battery);
    
    // Cell voltage frame - 0x1C7
    // Cycle through cell groups (4 cells per message)
    static uint8_t cell_group = 0;
    
    MG_ZS_1C7.data.u8[0] = (counter_100ms & 0x0F) | ((cell_group & 0x0F) << 4);
    
    // Calculate the current cell group index
    uint8_t startCell = cell_group * 4;
    if (startCell < num_cells) {
      // First cell in group
      uint16_t cell1V = cellVoltages[startCell];
      MG_ZS_1C7.data.u8[1] = (cell1V >> 8) & 0xFF;
      MG_ZS_1C7.data.u8[2] = cell1V & 0xFF;
      
      // Second cell in group (if available)
      if (startCell + 1 < num_cells) {
        uint16_t cell2V = cellVoltages[startCell + 1];
        MG_ZS_1C7.data.u8[3] = (cell2V >> 8) & 0xFF;
        MG_ZS_1C7.data.u8[4] = cell2V & 0xFF;
      } else {
        MG_ZS_1C7.data.u8[3] = 0;
        MG_ZS_1C7.data.u8[4] = 0;
      }
      
      // Third cell in group (if available)
      if (startCell + 2 < num_cells) {
        uint16_t cell3V = cellVoltages[startCell + 2];
        MG_ZS_1C7.data.u8[5] = (cell3V >> 8) & 0xFF;
        MG_ZS_1C7.data.u8[6] = cell3V & 0xFF;
      } else {
        MG_ZS_1C7.data.u8[5] = 0;
        MG_ZS_1C7.data.u8[6] = 0;
      }
      
      // Fourth cell's data would go in next frame
      
      // Also include max/min data every 6th cycle
      if (cell_group % 6 == 0) {
        // Send max/min info in the seventh byte
        MG_ZS_1C7.data.u8[7] = CellVmaxNo;
      } else if (cell_group % 6 == 3) {
        // Send min cell info in the seventh byte
        MG_ZS_1C7.data.u8[7] = CellVminNo;
      } else {
        MG_ZS_1C7.data.u8[7] = 0;
      }
    } else {
      // Reset if we're past the number of cells
      cell_group = 0;
      
      // Send max/min summary when cycling back to start
      MG_ZS_1C7.data.u8[1] = (CellVoltMax_mV >> 8) & 0xFF;  // Max cell voltage high byte
      MG_ZS_1C7.data.u8[2] = CellVoltMax_mV & 0xFF;         // Max cell voltage low byte
      MG_ZS_1C7.data.u8[3] = CellVmaxNo;                    // Max cell number
      MG_ZS_1C7.data.u8[4] = (CellVoltMin_mV >> 8) & 0xFF;  // Min cell voltage high byte
      MG_ZS_1C7.data.u8[5] = CellVoltMin_mV & 0xFF;         // Min cell voltage low byte
      MG_ZS_1C7.data.u8[6] = CellVminNo;                    // Min cell number
      MG_ZS_1C7.data.u8[7] = 0xFF;                          // Indicator for summary frame
    }
    
    transmit_can_frame(&MG_ZS_1C7, can_config.battery);
    
    // Increment the cell group for the next time
    cell_group = (cell_group + 1) % ((num_cells + 3) / 4);  // Ceiling division by 4
  }
}

void setup_battery(void) {  // Performs one time setup at startup
  strncpy(datalayer.system.info.battery_protocol, "MG ZS battery", 63);
  datalayer.system.info.battery_protocol[63] = '\0';

  // Set battery specs
  datalayer.battery.info.max_design_voltage_dV = MAX_PACK_VOLTAGE_DV;
  datalayer.battery.info.min_design_voltage_dV = MIN_PACK_VOLTAGE_DV;
  datalayer.battery.info.max_cell_voltage_mV = MAX_CELL_VOLTAGE_MV;
  datalayer.battery.info.min_cell_voltage_mV = MIN_CELL_VOLTAGE_MV;
  datalayer.battery.info.max_cell_voltage_deviation_mV = MAX_CELL_DEVIATION_MV;
  datalayer.battery.info.number_of_cells = num_cells;
  datalayer.battery.info.total_capacity_Wh = 44500; // 44.5 kWh battery
  
  // Initialize cell voltages to a reasonable default
  for (uint8_t i = 0; i < num_cells; i++) {
    cellVoltages[i] = 3750; // 3.75V nominal voltage
  }
  
  // Initialize battery parameters
  datalayer.battery.status.real_soc = SOC_BMS;
  datalayer.battery.status.reported_soc = SOC_BMS;
  datalayer.battery.status.voltage_dV = batteryVoltage;
  datalayer.battery.status.current_dA = batteryCurrent;
  datalayer.battery.status.max_charge_power_W = allowedChargePower;
  datalayer.battery.status.max_discharge_power_W = allowedDischargePower;
  datalayer.battery.status.temperature_min_dC = temperatureMin;
  datalayer.battery.status.temperature_max_dC = temperatureMax;
  datalayer.battery.status.real_bms_status = BMS_STANDBY;
  datalayer.battery.status.bms_status = STANDBY; // Use the bms_status_enum type
  datalayer.system.status.battery_allows_contactor_closing = isolation_status;
  
#ifdef DEBUG_LOG
  logging.println("MG-ZS-BATTERY: Initialized battery emulator");
  logging.print("Battery capacity: ");
  logging.print(datalayer.battery.info.total_capacity_Wh / 1000.0, 1);
  logging.println(" kWh");
  logging.print("Number of cells: ");
  logging.println(num_cells);
  logging.print("Initial SOC: ");
  logging.print(SOC_BMS / 100.0, 2);
  logging.println("%");
#endif
}

#endif
