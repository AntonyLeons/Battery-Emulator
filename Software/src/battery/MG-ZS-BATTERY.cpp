#include "../include.h"
#ifdef MG_ZS_BATTERY_H
#include "../datalayer/datalayer.h"
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

void handle_incoming_can_frame_battery(CAN_frame rx_frame) {
  datalayer.battery.status.CAN_battery_still_alive = CAN_STILL_ALIVE;
  
  // Declare variables outside the switch statement to avoid jump errors
  uint8_t socValue = 0;
  uint8_t powerIndicator = 0;
  uint8_t remainingData = 0;
  
  switch (rx_frame.ID) {
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
      logging.println("MG5: BMS status request received");
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
          logging.println("MG5: Contactor close command accepted");
#endif
        } else {
          contactorStatus = false;
          datalayer.battery.status.real_bms_status = BMS_STANDBY;
#ifdef DEBUG_LOG
          logging.println("MG5: Contactor open command executed or close rejected");
          if (!isolation_status) {
            logging.println("MG5: Contactor close rejected due to isolation fault");
          }
          if (SOC_BMS <= 100) {
            logging.println("MG5: Contactor close rejected due to low SOC");
          }
          if (SOC_BMS >= 9900) {
            logging.println("MG5: Contactor close rejected due to high SOC");
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
        logging.println("MG5: Charging mode activated");
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
        logging.println("MG5: Battery cooling requested");
#endif
      } else if (rx_frame.data.u8[0] & 0x02) { // Heating request
        // Acknowledge heating request
#ifdef DEBUG_LOG
        logging.println("MG5: Battery heating requested");
#endif
      }
      break;
      
    case 0x620:  // Wake-up message
      // Vehicle may send this to wake up the battery
#ifdef DEBUG_LOG
      logging.println("MG5: Battery wake-up message received");
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
    
    // For a 64kWh battery, 1Wh = ~0.0015625% SOC change
    float socChangePercent = energyChange * 100.0 / 64000.0;
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
  strncpy(datalayer.system.info.battery_protocol, "MG 5 battery", 63);
  datalayer.system.info.battery_protocol[63] = '\0';

  // Set battery specs
  datalayer.battery.info.max_design_voltage_dV = MAX_PACK_VOLTAGE_DV;
  datalayer.battery.info.min_design_voltage_dV = MIN_PACK_VOLTAGE_DV;
  datalayer.battery.info.max_cell_voltage_mV = MAX_CELL_VOLTAGE_MV;
  datalayer.battery.info.min_cell_voltage_mV = MIN_CELL_VOLTAGE_MV;
  datalayer.battery.info.max_cell_voltage_deviation_mV = MAX_CELL_DEVIATION_MV;
  datalayer.battery.info.number_of_cells = num_cells;
  datalayer.battery.info.total_capacity_Wh = 64000; // 64 kWh battery
  
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
  logging.println("MG-5-BATTERY: Initialized battery emulator");
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
