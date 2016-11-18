// Application includes
#include "cyclop_plus.h"

// Library includes
#include <avr/pgmspace.h>
#include <string.h>
#include <EEPROM.h>
#ifdef SSD1306_OLED_DRIVER
#include <Adafruit_SSD1306.h>
#endif
#ifdef SH1106_OLED_DRIVER
#include "libraries/adafruit_sh1106/Adafruit_SH1106.h"
#endif
#include <Adafruit_GFX.h>

//******************************************************************************
//* File scope function declarations

uint16_t autoScan(uint16_t frequency);
uint16_t averageAnalogRead(uint8_t pin);
void     batteryMeter(void);
uint8_t  bestChannelMatch(uint16_t frequency);
void     drawAutoScanScreen(void);
void     drawBattery(uint8_t xPos, uint8_t yPos, uint8_t value);
void     drawChannelScreen(uint8_t channel, uint16_t rssi);
void     drawEmptyScreen(void);
void     drawScannerScreen(void);
uint8_t  getClickType(uint8_t buttonPin);
uint16_t graphicScanner(uint16_t frequency);
char    *longNameOfChannel(uint8_t channel, char *name);
uint8_t  nextChannel(uint8_t channel);
uint8_t  previousChannel(uint8_t channel);
bool     readEeprom(void);
char    *shortNameOfChannel(uint8_t channel, char *name);
void     setRTC6715Frequency(uint16_t frequency);
void     updateScannerScreen(uint8_t position, uint8_t value);
void     writeEeprom(void);

//******************************************************************************
//* Positions in the frequency table for the 40 channels
//* Direct access via array operations does not work since data is stored in
//* flash, not in RAM. Use getPosition to retrieve data

const uint8_t positions[] PROGMEM = {
  19, 32, 18, 17, 33, 16,  7, 34,
  8, 24,  6,  9, 25,  5, 35, 10,
  26,  4, 11, 27, 3, 36, 12, 28,
  2, 13, 29, 37,  1, 14, 30,  0,
  15, 31, 38, 20, 21, 39, 22, 23
};

uint16_t getPosition(uint8_t channel) {
  return pgm_read_byte_near(positions + channel);
}

//******************************************************************************
//* Frequencies for the 40 channels
//* Direct access via array operations does not work since data is stored in
//* flash, not in RAM. Use getFrequency to retrieve data

const uint16_t channelFrequencies[] PROGMEM = {
  5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // Band A - Boscam A
  5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // Band B - Boscam B
  5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // Band E - DJI
  5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, // Band F - FatShark \ Immersion
  5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917  // Band C - Raceband
};

uint16_t getFrequency( uint8_t channel ) {
  return pgm_read_word_near(channelFrequencies + getPosition(channel));
}

//******************************************************************************
//* Other file scope variables
#ifdef SSD1306_OLED_DRIVER
Adafruit_SSD1306 display(4);
#endif
#ifdef SH1106_OLED_DRIVER
Adafruit_SH1106 display(4);
#endif
uint8_t lastClick = NO_CLICK;
uint8_t currentChannel = 0;
uint8_t lastChannel = 0;
uint16_t currentRssi = 0;
uint8_t ledState = LED_ON;
unsigned long saveScreenTimer;
unsigned long displayUpdateTimer = 0;
unsigned long eepromSaveTimer = 0;
unsigned long alarmTimer = 0;
uint8_t alarmSoundOn = 0;
uint16_t alarmOnPeriod = 0;
uint16_t alarmOffPeriod = 0;

//******************************************************************************
//* function: setup
//******************************************************************************
void setup()
{
  // initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  // initialize button pin
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, INPUT_PULLUP);

  // initialize alarm
  pinMode(ALARM_PIN, OUTPUT);

  // SPI pins for RX control
  pinMode (SLAVE_SELECT_PIN, OUTPUT);
  pinMode (SPI_DATA_PIN, OUTPUT);
  pinMode (SPI_CLOCK_PIN, OUTPUT);

  // Read current channel and options data from EEPROM
  if (!readEeprom()) {
    currentChannel = CHANNEL_MIN;
  }

  // Start receiver
  setRTC6715Frequency(getFrequency(currentChannel));

  // Initialize the display
#ifdef SSD1306_OLED_DRIVER
  display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADR);
#endif
#ifdef SH1106_OLED_DRIVER
  display.begin(SH1106_SWITCHCAPVCC, OLED_I2C_ADR);
#endif
  display.clearDisplay();
  display.display();

  // Wait at least the delay time before entering screen save mode
  saveScreenTimer = millis() + SAVE_SCREEN_DELAY_MS;

  return;
}

//******************************************************************************
//* function: loop
//******************************************************************************
void loop()
{
  switch (lastClick = getClickType(BUTTON_PIN))
  {
    case NO_CLICK: // do nothing
      break;

    case LONG_LONG_CLICK: // auto search
      drawAutoScanScreen();
      currentChannel = bestChannelMatch(autoScan(getFrequency(currentChannel)));
      drawChannelScreen(currentChannel, 0);
      displayUpdateTimer = millis() +  RSSI_STABILITY_DELAY_MS;
      break;

    case LONG_CLICK:      // graphical band scanner
      currentChannel = bestChannelMatch(graphicScanner(getFrequency(currentChannel)));
      drawChannelScreen(currentChannel, 0);
      displayUpdateTimer = millis() +  RSSI_STABILITY_DELAY_MS;
      break;

    case SINGLE_CLICK: // up the frequency
      currentChannel = nextChannel(currentChannel);
      setRTC6715Frequency(getFrequency(currentChannel));
      drawChannelScreen(currentChannel, 0);
      break;

    case DOUBLE_CLICK:  // down the frequency
      currentChannel = previousChannel(currentChannel);
      setRTC6715Frequency(getFrequency(currentChannel));
      drawChannelScreen(currentChannel, 0);
      break;
  }

  // Check if the display needs updating
  if (millis() > displayUpdateTimer)
      drawEmptyScreen();

      currentRssi = averageAnalogRead(RSSI_PIN);
      drawChannelScreen(currentChannel, currentRssi);
      displayUpdateTimer = millis() + 1000;

  // Check if EEPROM needs a save. Reduce EEPROM writes by not saving to often
  if ((currentChannel != lastChannel) && (millis() > eepromSaveTimer))
  {
    writeEeprom();
    lastChannel = currentChannel;
    eepromSaveTimer = millis() + 10000;
  }

  // Toggle alarm on or off
  if (alarmOnPeriod) {
    if (millis() > alarmTimer) {
      alarmSoundOn = !alarmSoundOn;
      if (alarmSoundOn) {
        analogWrite(ALARM_PIN, 32);
        alarmTimer = millis() + alarmOnPeriod;
      }
      else {
        analogWrite(ALARM_PIN, 0);
        alarmTimer = millis() + alarmOffPeriod;
      }
    }
  }
  else
    analogWrite(ALARM_PIN, 0);
}

//******************************************************************************
//* function: writeEeprom
//******************************************************************************
void writeEeprom(void) {
  EEPROM.write(EEPROM_CHANNEL, currentChannel);
  EEPROM.write(EEPROM_CHECK, 238);
}

//******************************************************************************
//* function: readEeprom
//******************************************************************************
bool readEeprom(void) {
  if (EEPROM.read(EEPROM_CHECK) != 238)
    return false;
  currentChannel = EEPROM.read(EEPROM_CHANNEL);
  return true;
}

//******************************************************************************
//* function: get_click_type
//*         : Polls the specified pin and returns the type of click that was
//*         : performed NO_CLICK, SINGLE_CLICK, DOUBLE_CLICK, LONG_CLICK
//*         : or LONG_LONG_CLICK
//******************************************************************************
uint8_t getClickType(uint8_t buttonPin) {
  uint16_t timer = 0;
  uint8_t click_type = NO_CLICK;

  // check if the key has been pressed
  if (digitalRead(buttonPin) == !BUTTON_PRESSED)
    return ( NO_CLICK );

  while (digitalRead(buttonPin) == BUTTON_PRESSED) {
    timer++;
    delay(5);
  }

  if (timer < 40)                  // 120 * 5 ms = 0.6s
    click_type = SINGLE_CLICK;
  if (timer >= 40 && timer < 120)  // 300 * 5 ms = 1.5s
    click_type = LONG_CLICK;
  if (timer >= 120)
    click_type = LONG_LONG_CLICK;

  // Check if there is a second click
  timer = 0;
  while ((digitalRead(buttonPin) == !BUTTON_PRESSED) && (timer++ < 40)) {
    delay(5);
  }

  if (timer >= 20)                  // 40 * 5 ms = 0.2s
    return click_type;

  if (digitalRead(buttonPin) == BUTTON_PRESSED ) {
    click_type = DOUBLE_CLICK;
    while (digitalRead(buttonPin) == BUTTON_PRESSED) ;
  }

  return (click_type);
}

//******************************************************************************
//* function: nextChannel
//******************************************************************************
uint8_t nextChannel(uint8_t channel)
{
  if (channel > (CHANNEL_MAX - 1))
    return CHANNEL_MIN;
  else
    return channel + 1;
}

//******************************************************************************
//* function: previousChannel
//******************************************************************************
uint8_t previousChannel(uint8_t channel)
{
  if ((channel > CHANNEL_MAX) || (channel == CHANNEL_MIN))
    return CHANNEL_MAX;
  else
    return channel - 1;
}

//******************************************************************************
//* function: bestChannelMatch
//*         : finds the best matching standard channel for a given frequency
//******************************************************************************
uint8_t bestChannelMatch(uint16_t frequency)
{
  int16_t comp;
  int16_t bestComp = 300;
  uint8_t bestChannel = CHANNEL_MIN;
  uint8_t i;

  for (i = CHANNEL_MIN; i <= CHANNEL_MAX; i++) {
    comp = abs((int16_t)getFrequency(i) - (int16_t)frequency);
    if (comp < bestComp)
    {
      bestComp = comp;
      bestChannel = i;
    }
  }
  return bestChannel;
}

//******************************************************************************
//* function: graphicScanner
//*         : scans the 5.8 GHz band in 3 MHz increments and draws a graphical
//*         : representation. when the button is pressed the currently
//*         : scanned frequency is returned.
//******************************************************************************
uint16_t graphicScanner(uint16_t frequency) {
  uint8_t i;
  uint16_t scanRssi;
  uint16_t bestRssi = 0;
  uint16_t scanFrequency = frequency;
  uint16_t bestFrequency = frequency;
  uint8_t clickType;
  uint8_t rssiDisplayValue;

  // Draw screen frame etc
  drawScannerScreen();

  while (digitalRead(BUTTON_PIN) != BUTTON_PRESSED) {
    scanFrequency += 3;
    if (scanFrequency > FREQUENCY_MAX)
      scanFrequency = FREQUENCY_MIN;
    setRTC6715Frequency(scanFrequency);
    delay(RSSI_STABILITY_DELAY_MS);
    scanRssi = averageAnalogRead(RSSI_PIN);
    rssiDisplayValue = (scanRssi - 140) / 10;    // Roughly 2 - 46
    updateScannerScreen(100 - ((FREQUENCY_MAX - scanFrequency) / 3), rssiDisplayValue);
  }

  return (scanFrequency);
}

//******************************************************************************
//* function: autoScan
//******************************************************************************
uint16_t autoScan(uint16_t frequency) {
  uint8_t i;
  uint16_t scanRssi = 0;
  uint16_t bestRssi = 0;
  uint16_t scanFrequency;
  uint16_t bestFrequency;

  // Skip 10 MHz forward to avoid detecting the current channel
  scanFrequency = frequency + 10;
  if (!(scanFrequency % 2))
    scanFrequency++;        // RTC6715 can only generate odd frequencies

  // Coarse tuning
  bestFrequency = scanFrequency;
  for (i = 0; i < 60 && (scanRssi < RSSI_TRESHOLD); i++) {
    if (scanFrequency <= (FREQUENCY_MAX - 5))
      scanFrequency += 5;
    else
      scanFrequency = FREQUENCY_MIN;
    setRTC6715Frequency(scanFrequency);
    delay(RSSI_STABILITY_DELAY_MS);
    scanRssi = averageAnalogRead(RSSI_PIN);
    if (digitalRead(BUTTON_PIN) == BUTTON_PRESSED)
      return frequency;
    if (bestRssi < scanRssi) {
      bestRssi = scanRssi;
      bestFrequency = scanFrequency;
    }
  }
  // Fine tuning
  scanFrequency = bestFrequency - 20;
  bestRssi = 0;
  for (i = 0; i < 20; i++, scanFrequency += 2) {
    setRTC6715Frequency(scanFrequency);
    delay(RSSI_STABILITY_DELAY_MS);
    scanRssi = averageAnalogRead(RSSI_PIN);
    if (digitalRead(BUTTON_PIN) == BUTTON_PRESSED)
      return frequency;
    if (bestRssi < scanRssi) {
      bestRssi = scanRssi;
      bestFrequency = scanFrequency;
    }
  }
  // Return the best frequency
  setRTC6715Frequency(bestFrequency);
  return (bestFrequency);
}

//******************************************************************************
//* function: averageAnalogRead
//*         : returns an averaged value between (in theory) 0 and 1024
//*         : this function is called often, so it is speed optimized
//******************************************************************************
uint16_t averageAnalogRead( uint8_t pin)
{
  uint16_t rssi = 0;
  uint8_t i = 32;

  for ( ; i ; i--) {
    rssi += analogRead(pin);
  }
  return (rssi >> 5);
}

//******************************************************************************
//* function: shortNameOfChannel
//******************************************************************************
char *shortNameOfChannel(uint8_t channel, char *name)
{
  uint8_t channelIndex = getPosition(channel);
  if (channelIndex < 8)
    name[0] = 'A';
  else if (channelIndex < 16)
    name[0] = 'B';
  else if (channelIndex < 24)
    name[0] = 'E';
  else if (channelIndex < 32)
    name[0] = 'F';
  else
    name[0] = 'R';
  name[1] = (channelIndex % 8) + '0' + 1;
  name[2] = 0;
  return name;
}

//******************************************************************************
//* function: longNameOfChannel
//******************************************************************************
char *longNameOfChannel(uint8_t channel, char *name)
{
  uint8_t len;
  uint8_t channelIndex = getPosition(channel);

  if (channelIndex < 8)
    strcpy(name, "BandA ");
  else if (channelIndex < 16)
    strcpy(name, "BandB ");
  else if (channelIndex < 24)
    strcpy(name, "BandE ");
  else if (channelIndex < 32)
    strcpy(name, "IRC ");
  else
    strcpy(name, "Race ");

  len = strlen(name);
  name[len] = (channelIndex % 8) + '0' + 1;
  name[len + 1] = 0;
  return name;
}

//******************************************************************************
//* function: calcFrequencyData
//*         : calculates the frequency value for the syntheziser register B of
//*         : the RTC6751 circuit that is used within the RX5808/RX5880 modules.
//*         : this value is inteded to be loaded to register at adress 1 via SPI
//*         :
//*  Formula: frequency = ( N*32 + A )*2 + 479
//******************************************************************************
uint16_t calcFrequencyData(uint16_t frequency)
{
  uint16_t N;
  uint8_t A;
  frequency = (frequency - 479) / 2;
  N = frequency / 32;
  A = frequency % 32;
  return (N << 7) |  A;
}

//******************************************************************************
//* function: setRTC6715Frequency
//*         : for a given frequency the register setting for synth register B of
//*         : the RTC6715 circuit is calculated and bitbanged via the SPI bus
//*         : please note that the synth register A is assumed to have default
//*         : values.
//*
//* SPI data: 4  bits  Register Address  LSB first
//*         : 1  bit   Read or Write     0=Read 1=Write
//*         : 13 bits  N-Register Data   LSB first
//*         : 7  bits  A-Register        LSB first
//******************************************************************************
void setRTC6715Frequency(uint16_t frequency)
{
  uint16_t sRegB;
  uint8_t i;

  sRegB = calcFrequencyData(frequency);

  // Bit bang the syntheziser register

  // Clock
  spiEnableHigh();
  delayMicroseconds(1);
  spiEnableLow();

  // Address (0x1)
  spi_1();
  spi_0();
  spi_0();
  spi_0();

  // Read/Write (Write)
  spi_1();

  // Data (16 LSB bits)
  for (i = 16; i; i--, sRegB >>= 1 ) {
    (sRegB & 0x1) ? spi_1() : spi_0();
  }
  // Data zero padding
  spi_0();
  spi_0();
  spi_0();
  spi_0();

  // Clock
  spiEnableHigh();
  delayMicroseconds(1);

  digitalWrite(SLAVE_SELECT_PIN, LOW);
  digitalWrite(SPI_CLOCK_PIN, LOW);
  digitalWrite(SPI_DATA_PIN, LOW);
}

//******************************************************************************
//* function: spi_1
//******************************************************************************
void spi_1()
{
  digitalWrite(SPI_CLOCK_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(SPI_DATA_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(SPI_CLOCK_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(SPI_CLOCK_PIN, LOW);
  delayMicroseconds(1);
}

//******************************************************************************
//* function: spi_0
//******************************************************************************
void spi_0()
{
  digitalWrite(SPI_CLOCK_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(SPI_DATA_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(SPI_CLOCK_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(SPI_CLOCK_PIN, LOW);
  delayMicroseconds(1);
}

//******************************************************************************
//* function: spiEnableLow
//******************************************************************************
void spiEnableLow()
{
  delayMicroseconds(1);
  digitalWrite(SLAVE_SELECT_PIN, LOW);
  delayMicroseconds(1);
}

//******************************************************************************
//* function: spiEnableHigh
//******************************************************************************
void spiEnableHigh()
{
  delayMicroseconds(1);
  digitalWrite(SLAVE_SELECT_PIN, HIGH);
  delayMicroseconds(1);
}

//******************************************************************************
//* function: batteryMeter
//*         : Measured voltage values
//*         : 3s LiPo
//*         : max = 4.2v * 3 = 12.6v = 643
//*         : min = 3.6v * 3 = 10.8v = 551
//*         : 2s LiPo
//*         : max = 4.2v * 2 = 8.4v = 429
//*         : min = 3.6v * 2 = 7.2v = 367
//******************************************************************************
void batteryMeter(void)
{
  uint16_t voltage;
  uint8_t value;
  uint16_t minV;
  uint16_t maxV;

  minV = 551;
  maxV = 643;

  voltage = averageAnalogRead(VOLTAGE_METER_PIN);

  if (voltage >= maxV)
    value = 99;
  else if (voltage <= minV)
    value = 0;
  else
    value = (uint8_t)((voltage - minV) / (float)(maxV - minV) * 100.0);

  // Set alarm periods
  if (value < 5)
  {
    alarmOnPeriod = ALARM_MAX_ON;
    alarmOffPeriod = ALARM_MAX_OFF;
  }
  else if (value < 15)
  {
    alarmOnPeriod = ALARM_MED_ON;
    alarmOffPeriod = ALARM_MED_OFF;
  }
  else if (value < 25)
  {
    alarmOnPeriod = ALARM_MIN_ON;
    alarmOffPeriod = ALARM_MIN_OFF;
  }
  else
  {
    alarmOnPeriod = 0;
    alarmOffPeriod = 0;
  }
  drawBattery(58, 32, value);
}

//******************************************************************************
//* function: drawChannelScreen
//*         : draws the standard screen with channel information
//******************************************************************************
void drawChannelScreen(uint8_t channel, uint16_t rssi) {
  char buffer[22];
  uint8_t i;

  // normalize rssi
  if (rssi <= 175)
    rssi = 0;
  else if (rssi > 530)
    rssi = 99;
  else {
    rssi -= 175;
    rssi = rssi * 99;
    rssi = rssi / (530 - 175);
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(10, 0);
  display.setTextSize(3);
  display.print(getFrequency(channel));
  display.setCursor(75, 7);
  display.setTextSize(2);
  display.print(F(" MHz"));
  display.drawLine(0, 24, 127, 24, WHITE);
  display.setCursor(0, 27);
  display.setTextSize(1);
  display.print(F("  Channel    RSSI"));
  display.setCursor(0, 39);
  display.setTextSize(2);
  display.print(F(" "));
  display.print(F(" "));
  display.print(shortNameOfChannel(channel, buffer));
  display.print(F("   "));
  if (rssi < 10)
    display.print(F(" "));
  display.print(rssi);
  display.setCursor(0, 57);
  display.setTextSize(1);
  longNameOfChannel(channel, buffer);
  i = (21 - strlen(buffer)) / 2;
  for (; i; i--) {
    display.print(F(" "));
  }
  display.print(buffer);
  batteryMeter();
  display.display();
}

//******************************************************************************
//* function: drawAutoScanScreen
//******************************************************************************
void drawAutoScanScreen( void ) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(10, 0);
  display.setTextSize(3);
  display.print(F("SCAN"));
  display.setCursor(75, 7);
  display.setTextSize(2);
  display.print(F(" MHz"));
  display.drawLine(0, 24, 127, 24, WHITE);
  display.setCursor(0, 27);
  display.setTextSize(1);
  display.print(F("  Channel    RSSI"));
  batteryMeter();
  display.display();
}

//******************************************************************************
//* function: drawScannerScreen
//******************************************************************************
void drawScannerScreen( void ) {
  display.clearDisplay();
  display.drawLine(0, 55, 127, 55, WHITE);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 57);
  display.print(F("5.65     5.8     5.95"));
  updateScannerScreen(0, 0);
}

//******************************************************************************
//* function: updateScannerScreen
//*         : position = 0 to 99
//*         : value = 0 to 53
//*         : must be fast since there are frequent updates
//******************************************************************************
void updateScannerScreen(uint8_t position, uint8_t value ) {
  // uint8_t i;
  static uint8_t last_position = 14;
  static uint8_t last_value = 0;

  // The scan graph only uses the 100 positions in the middle of the screen
  position = position + 14;

  // Errase the scan line from the last pass
  display.drawFastVLine( last_position, 0, 54 - last_value, BLACK );

  // Draw the current scan line
  display.drawFastVLine( position, 0, 54, WHITE );

  // Save position and value for the next pass
  last_position = position;
  if (value > 53)
    last_value = 53;
  else
    last_value = value;
  display.display();
}

//******************************************************************************
//* function: drawBattery
//*         : value = 0 to 100
//******************************************************************************
void drawBattery(uint8_t xPos, uint8_t yPos, uint8_t value ) {
  display.drawRect(3 + xPos,  0 + yPos, 4, 2, WHITE);
  display.drawRect(0 + xPos, 2 + yPos, 10, 20, WHITE);
  display.drawRect(2 + xPos,  4 + yPos, 6, 16, BLACK);
  if (value > 85)
    display.drawRect(3 + xPos,  5 + yPos, 4, 2, WHITE);
  if (value > 65)
    display.drawRect(3 + xPos,  8 + yPos, 4, 2, WHITE);
  if (value > 45)
    display.drawRect(3 + xPos,  11 + yPos, 4, 2, WHITE);
  if (value > 25)
    display.drawRect(3 + xPos, 14 + yPos, 4, 2, WHITE);
  if (value > 5)
    display.drawRect(3 + xPos, 17 + yPos, 4, 2, WHITE);
}

//******************************************************************************
//* function: drawEmptyScreen
//******************************************************************************
void drawEmptyScreen( void)
{
  display.clearDisplay();
  display.display();
}

