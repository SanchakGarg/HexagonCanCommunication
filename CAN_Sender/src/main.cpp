/*
 * Adafruit MCP2515 FeatherWing CAN Sender Example
 */

#include <Adafruit_MCP2515.h>
#include <Arduino.h>
#include <Wire.h>

#ifdef ESP8266
   #define CS_PIN    2
#elif defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3)
   #define CS_PIN    14
#elif defined(TEENSYDUINO)
   #define CS_PIN    8
#elif defined(ARDUINO_STM32_FEATHER)
   #define CS_PIN    PC5
#elif defined(ARDUINO_NRF52832_FEATHER)  /* BSP 0.6.5 and higher! */
   #define CS_PIN    27
#elif defined(ARDUINO_MAX32620FTHR) || defined(ARDUINO_MAX32630FTHR)
   #define CS_PIN    P3_2
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040)
   #define CS_PIN    7
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_CAN)
   #define CS_PIN    PIN_CAN_CS
#elif defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_RASPBERRY_PI_PICO_W) // PiCowbell CAN Bus
   #define CS_PIN    17
#else
    // Anything else, defaults!
   #define CS_PIN    5
#endif

// Set CAN bus baud rate
#define CAN_BAUDRATE (1000000)

#define CMD_LED_CONTROL 0x20
#define SUB_CMD_ON 0x01
#define SUB_CMD_OFF 0x00

Adafruit_MCP2515 mcp(CS_PIN);

// Function prototype
void sendLedCommand(uint8_t subCmd);

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);

  Serial.println("MCP2515 Sender test!");

  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while(1) delay(10);
  }
  Serial.println("MCP2515 chip found");
}

void sendLedCommand(uint8_t subCmd) {
  Serial.print("Sending LED command... ");
  
  mcp.beginPacket(CMD_LED_CONTROL);
  mcp.write(subCmd);
  mcp.endPacket();
  
  Serial.println("done");
}

void loop() {
  Serial.println("Enter 'on' to turn on the LED, or 'off' to turn it off:");
  while (!Serial.available()) {
    delay(100);
  }
  
  String input = Serial.readStringUntil('\n');
  input.trim();
  
  if (input == "on") {
    sendLedCommand(SUB_CMD_ON);
  } 
  else if (input == "off") {
    sendLedCommand(SUB_CMD_OFF);
  } 
  else {
    Serial.println("Invalid command");
  }
  
}