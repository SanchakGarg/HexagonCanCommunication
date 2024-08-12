/*
 * Adafruit MCP2515 FeatherWing CAN Receiver Example
 */
#define FASTLED_RP2040_CLOCKLESS_M0_FALLBACK 0
#include <Adafruit_MCP2515.h>
#include <Wire.h>
#ifdef ESP8266
#define CS_PIN 2
#elif defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3)
#define CS_PIN 14
#elif defined(TEENSYDUINO)
#define CS_PIN 8
#elif defined(ARDUINO_STM32_FEATHER)
#define CS_PIN PC5
#elif defined(ARDUINO_NRF52832_FEATHER) /* BSP 0.6.5 and higher! */
#define CS_PIN 27
#elif defined(ARDUINO_MAX32620FTHR) || defined(ARDUINO_MAX32630FTHR)
#define CS_PIN P3_2
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040)
#define CS_PIN 7
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_CAN)
#define CS_PIN PIN_CAN_CS
#elif defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_RASPBERRY_PI_PICO_W) // PiCowbell CAN Bus
#define CS_PIN 17
#else
// Anything else, defaults!
#define CS_PIN 5
#endif

// Set CAN bus baud rate
#define CAN_BAUDRATE (1000000)

#include <FastLED.h>

#define CMD_LED_CONTROL 0x20
#define SUB_CMD_ON 0x01
#define SUB_CMD_OFF 0x00

#define LED_PIN 2
#define NUM_LEDS 60

#define FIXED_COLOR CRGB::Red
CRGB leds[NUM_LEDS];

Adafruit_MCP2515 mcp(CS_PIN);

void setLedColor(CRGB color)
{
   for (int i = 0; i < NUM_LEDS; i++)
   {
      leds[i] = color;
   }
   FastLED.show();
   Serial.println("RED");
}

void setup()
{
   FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
   Serial.begin(115200);
   delay(2500);
   while (!Serial)
      delay(10);

   Serial.println("MCP2515 Receiver test!");

   if (!mcp.begin(CAN_BAUDRATE))
   {
      Serial.println("Error initializing MCP2515.");
      while (1)
         delay(10);
   }
   Serial.println("MCP2515 chip found");

   FastLED.setBrightness(250); // Set the brightness (0-255)
}

void loop()
{

   int packetSize = mcp.parsePacket();

   if (packetSize)
   {
      if (mcp.packetId() == CMD_LED_CONTROL && packetSize == 1)
      {
         uint8_t subCmd = mcp.read();

         if (subCmd == SUB_CMD_ON)
         {
            setLedColor(FIXED_COLOR);
            Serial.println("LED turned on with fixed color");
         }
         else if (subCmd == SUB_CMD_OFF)
         {
            setLedColor(CRGB::Black);
            Serial.println("LED turned off");
         }
      }
      else
      {
         // Handle other packet types if needed
         Serial.print("Received packet with id 0x");
         Serial.println(mcp.packetId(), HEX);
      }
   }
}