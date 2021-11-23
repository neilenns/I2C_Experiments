#include <Arduino.h>
#include <Wire.h>
#include <MCP23017.h>

const uint8_t ROW_I2C_ADDRESS = 0x20;            // I2C address of the MCP23017 IC
const uint8_t COLUMN_I2C_ADDRESS = 0x21;         // I2C address of the MCP23017 IC
const uint8_t INTA_PIN = 2;                      // Row interrupts
const uint16_t ROW_PULLUPS = 0b1111111111111100; // Pullups for the unconnected row pins. The high byte is PORTB, low byte PORTA.

enum DetectionState
{
  WaitingForPress,
  PressDetected,
  WaitingForRelease
};

volatile auto currentState = DetectionState::WaitingForPress;
auto activeRow = 0;
auto activeColumn = 0;

MCP23017 rows(ROW_I2C_ADDRESS);
MCP23017 columns(COLUMN_I2C_ADDRESS);

void write16AsBits(uint16_t value)
{
  for (int i = 0; i < 8; i++)
  {
    bool b = value & 0x8000;
    Serial.print(b);
    value = value << 1;
  }

  Serial.print(" ");
  for (int i = 0; i < 8; i++)
  {
    bool b = value & 0x8000;
    Serial.print(b);
    value = value << 1;
  }
}

void write8AsBits(uint8_t value)
{
  for (int i = 0; i < 8; i++)
  {
    bool b = value & 0x80;
    Serial.print(b);
    value = value << 1;
  }
}

/**
 * @brief Get the bit position for a single low bit in a byte.
 * 
 * @param value The byte to check, with the active bit set to low and inactive bits set high
 * @return int The position of the single low bit in the byte
 */
int getBitPosition(uint16_t value)
{
  // The entire key detection logic uses low for active so invert the bits
  // to ensure this magic works properly.
  value = ~value;
  // value is a power of two, returned values are {0, 1, ..., 15}
  // This is effectively calculating log2(n) since it's guaranteed there will only ever
  // be one bit set in the value. It's a variation of the method shown
  // at http://graphics.stanford.edu/~seander/bithacks.html#IntegerLog.
  return (((value & 0xAAAAAAAA) != 0) |
          (((value & 0xCCCCCCCC) != 0) << 1) |
          (((value & 0xF0F0F0F0) != 0) << 2) |
          (((value & 0xFF00FF00) != 0) << 3));
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
  columns.writeRegister(MCP23017Register::IODIR_A, 0x00, 0x00); // Columns as output
  columns.writeRegister(MCP23017Register::GPIO_A, 0x00, 0x00);  // Reset columns to 0s
  rows.writeRegister(MCP23017Register::IODIR_A, 0xFF, 0xFF);    // Rows as input
  rows.writeRegister(MCP23017Register::GPIO_A, 0xFF, 0xFF);     // Reset rows to 1s

  if (setPullups)
  {
    columns.writeRegister(MCP23017Register::GPPU_A, 0xFF, 0xFF);                                     // Columns have pull-up resistors on
    rows.writeRegister(MCP23017Register::GPPU_A, (uint8_t)ROW_PULLUPS, (uint8_t)(ROW_PULLUPS >> 8)); // Rows have pull-up resistors off for all
  }

  columns.writeRegister(MCP23017Register::INTCON_A, 0x00, 0x00); // Turn interrupts off for columns
  columns.writeRegister(MCP23017Register::DEFVAL_A, 0x00, 0x00); // Default value of 0 for columns
  rows.writeRegister(MCP23017Register::INTCON_A, 0xFF, 0xFF);    // Turn interrupts on for rows
  rows.writeRegister(MCP23017Register::DEFVAL_A, 0xFF, 0xFF);    // Default value of 1 for rows

  // Turn on interrupts
  rows.interruptMode(MCP23017InterruptMode::Or);               // Interrupt on one line
  rows.writeRegister(MCP23017Register::GPINTEN_A, 0xFF, 0xFF); // Turn on the interrupts for the rows

  rows.clearInterrupts(); // Clear all interrupts which could come from initialization
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

  columns.init();
  rows.init();

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
  uint16_t rowStates;
  uint16_t columnStates;

  // Read the current state of all 16 row pins. PORTA will be the low byte,
  // PORTB will be the high byte.
  rowStates = rows.read();

  // Once the row is known reconfigure a bunch of registers to read the active column
  columns.writeRegister(MCP23017Register::IODIR_A, 0xFF, 0xFF);   // Switch columns to input
  rows.writeRegister(MCP23017Register::IODIR_A, 0x00, 0x00);      // Switch rows to output
  columns.writeRegister(MCP23017Register::INTCON_A, 0xFF, 0xFF);  // Turn interrupts on for columns
  rows.writeRegister(MCP23017Register::INTCON_A, 0x00, 0x00);     // Turn interrupts off for rows
  columns.writeRegister(MCP23017Register::DEFVAL_A, 0xFF, 0xFF);  // Default value of 1 for columns
  rows.writeRegister(MCP23017Register::DEFVAL_A, 0x00, 0x00);     // Default value of 0 for rows
  columns.writeRegister(MCP23017Register::GPINTEN_A, 0xFF, 0xFF); // Temporarily enable column interrupts even though they aren't used
  rows.writeRegister(MCP23017Register::GPINTEN_A, 0x00, 0x00);    // Temporarily disable row interrupts

  // Write 0s to the rows then read the columns to find out what button is pressed.
  // This step is missing from the application note.
  rows.write(0x0000);

  // Read the current state of all 16 column pins. PORTA will be the low byte,
  // PORTB will be the high byte.
  columnStates = columns.read();

  activeRow = getBitPosition(rowStates);
  activeColumn = getBitPosition(columnStates);

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
  uint16_t rowState;

  // Try clearing the interrupts by reading the current state for the rows
  rowState = rows.read();

  // If all the inputs for the row are back to 1s then the button was released
  if (rowState == 0xFFFF)
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
