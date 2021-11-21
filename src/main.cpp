#include <Arduino.h>
#include <Wire.h>
#include <MCP23017.h>

#define MCP23017_I2C_ADDRESS 0x20 // I2C address of the MCP23017 IC
#define INTA_PIN 2                // Row interrupts

enum DetectionState
{
  WaitingForPress,
  PressDetected,
  WaitingForRelease
};

volatile auto currentState = DetectionState::WaitingForPress;
auto activeRow = 0;
auto activeColumn = 0;

MCP23017 _mcp(MCP23017_I2C_ADDRESS);

/**
 * @brief Get the bit position for a single low bit in a byte.
 * 
 * @param value The byte to check, with the active bit set to low and inactive bits set high
 * @return int The position of the single low bit in the byte
 */
int getBitPosition(uint8_t value)
{
  // The entire key detection logic uses low for active so invert the bits
  // to ensure this magic works properly.
  value = ~value;
  // value is in {1, 2, 4, 8, 16, 32, 64, 128}, returned values are {0, 1, ..., 7}
  return (((value & 0xAA) != 0) |
          (((value & 0xCC) != 0) << 1) |
          (((value & 0xF0) != 0) << 2));
}

/**
 * @brief Interrupt handler for when the row changed interrupt fires.
 * 
 */
void rowChanged()
{
  if (currentState == WaitingForPress)
  {
    currentState = DetectionState::PressDetected;
  }
}

/**
 * @brief Initializes the MCP23017 to detect interrupts on row changes.
 * 
 * @param setPullups True if the pullup resistors should get configured. This only needs to happen once when the chip
 * is first initialized at board startup.
 */
void InitForRowDetection(bool setPullups)
{
  _mcp.writeRegister(MCP23017Register::IODIR_A, 0x00, 0xFF); // Columns as output, rows as input
  _mcp.writeRegister(MCP23017Register::GPIO_A, 0x00, 0xFF);  // Reset columns to 0s and rows to 1s

  if (setPullups)
  {
    _mcp.writeRegister(MCP23017Register::GPPU_A, 0xFF, 0x00); // Columns have pull-up resistors on, rows have pull-up resistors off
  }

  _mcp.writeRegister(MCP23017Register::INTCON_A, 0x00, 0xFF); // Turn off interrupts for columns, on for rows
  _mcp.writeRegister(MCP23017Register::DEFVAL_A, 0x00, 0xFF); // Default value of 0 for columns, 1 for rows

  // Turn on interrupts
  _mcp.interruptMode(MCP23017InterruptMode::Or);         // Interrupt on one line
  _mcp.writeRegister(MCP23017Register::GPINTEN_B, 0xFF); // Turn on the interrupts for the rows

  _mcp.clearInterrupts(); // Clear all interrupts which could come from initialization
}

void setup()
{
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin();

  while (!Serial)
    ; // Wait for the serial port to connect

  // This gives enough time to start up a connected logic analyzer
  Serial.println("Waiting 5 seconds to begin.");
  delay(5000);

  _mcp.init();

  // Set all the registers for proper interrupt detection on rows
  InitForRowDetection(true);

  // Register for interrupts on the Arduino side
  pinMode(INTA_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTA_PIN), rowChanged, CHANGE);
}

/**
 * @brief Determines which button is currently pressed when a row changed interrupt fires.
 * 
 */
void CheckForButton()
{
  uint8_t rowStates;
  uint8_t columnStates;

  // Read the row interrupts to find out which one caused the interrupt.
  rowStates = _mcp.readRegister(MCP23017Register::INTCAP_B);

  // Once the row is known reconfigure a bunch of registers to read the active column
  _mcp.writeRegister(MCP23017Register::IODIR_A, 0xFF, 0x00);   // Switch columns to input, rows to output
  _mcp.writeRegister(MCP23017Register::INTCON_A, 0xFF, 0x00);  // Turn on interrupts for columns, off for rows
  _mcp.writeRegister(MCP23017Register::DEFVAL_A, 0xFF, 0x00);  // Default value of 1 for columns, 0 for rows
  _mcp.writeRegister(MCP23017Register::GPINTEN_A, 0xFF, 0x00); // Temporarily enable column interrupts even though they aren't used, disable row interrupts

  // Write 0s to the rows then read the columns to find out what button is pressed.
  _mcp.writePort(MCP23017Port::B, 0x00); //  This step is missing from the application note.
  columnStates = _mcp.readPort(MCP23017Port::A);

  activeRow = getBitPosition(rowStates);
  activeColumn = getBitPosition(columnStates);

  Serial.print("Detected press at row: ");
  Serial.print(activeRow);
  Serial.print(" column: ");
  Serial.println(activeColumn);

  // Flip all the registers back to the default configuration to look for
  // when the row clears.
  currentState = WaitingForRelease;
  InitForRowDetection(false);
}

/**
 * @brief Determines when a button was released by watching for the row port to reset
 * to all 1s.
 * 
 */
void CheckForRelease()
{
  uint8_t rowState;

  // Try clearing the interrupts by reading the current state for the rows
  rowState = _mcp.readPort(MCP23017Port::B);

  // If all the inputs for the row are back to 1s then the button was released
  if (rowState == 0xFF)
  {
    Serial.print("Detected release at row: ");
    Serial.print(activeRow);
    Serial.print(" column: ");
    Serial.println(activeColumn);
    currentState = WaitingForPress;
  }
}

void loop()
{
  // Fininte state machine for button detection
  switch (currentState)
  {
  case WaitingForPress:
    // Nothing to do here, interrupts will handle it
    break;
  case PressDetected:
    CheckForButton();
    break;
  case WaitingForRelease:
    CheckForRelease();
    break;
  }
}
