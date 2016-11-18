#ifndef cyclop_plus_h
#define cyclop_plus_h

// The OLED I2C address may be 0x2C or 0x3C. 0x3C is almost always used
//#define OLED_I2C_ADR      0x2C
#define OLED_I2C_ADR      0x3C

// SSD1306 and SH1106 OLED displays are supported. Select one.
#define SSD1306_OLED_DRIVER 
//#define SH1106_OLED_DRIVER

// This definition is used by the ADAFRUIT library
#define OLED_128x64_ADAFRUIT_SCREENS
#define SSD1306_128_64

// Delay after key click before screen save (in milli seconds)
#define SAVE_SCREEN_DELAY_MS      10000

// Alarm timing constants (in milli seconds)
#define ALARM_MAX_ON      50
#define ALARM_MAX_OFF     200
#define ALARM_MED_ON      100
#define ALARM_MED_OFF     1000
#define ALARM_MIN_ON      200
#define ALARM_MIN_OFF     3000

// Digital pin definitions
#define SPI_CLOCK_PIN     2
#define SLAVE_SELECT_PIN  3
#define SPI_DATA_PIN      4
#define BUTTON_PIN        5
#define ALARM_PIN         6
#define LED_PIN           13

// Analog pin definitions
#define VOLTAGE_METER_PIN A1
#define RSSI_PIN          A6

// Minimum delay between setting a channel and trusting the RSSI values
#define RSSI_STABILITY_DELAY_MS 25

// RSSI threshold for accepting a channel
#define RSSI_TRESHOLD     250

// Channels in use 
#define CHANNEL_MIN       0
#define CHANNEL_MAX       39

// Max and Min frequencies
#define FREQUENCY_MIN     5645
#define FREQUENCY_MAX     5945

//EEPROM addresses
#define EEPROM_CHANNEL    0
#define EEPROM_CHECK      1

// click types
#define NO_CLICK          0
#define SINGLE_CLICK      1
#define DOUBLE_CLICK      2
#define LONG_CLICK        3
#define LONG_LONG_CLICK   4

// Button pins go low or high on button clicks
#define BUTTON_PRESSED    LOW

// LED state defines
#define LED_OFF           LOW
#define LED_ON            HIGH

#endif // cyclop_plus_h
