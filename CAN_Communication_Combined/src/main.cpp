/*
 * Adafruit MCP2515 FeatherWing CAN Sender/Receiver Example
 */
#define FASTLED_RP2040_CLOCKLESS_M0_FALLBACK 0
#define m3
#include <vector>
#include <Adafruit_MCP2515.h>
#include <Wire.h>
#include <FastLED.h>
#include <FreeRTOS.h>   
#include <task.h>

#define NUM_LEDS 72
#define pin 2

CRGB leds[NUM_LEDS];

#ifdef m1
#define canID 0x12
#elif defined(m2)
#define canID 0x13
#elif defined(m3)
#define canID 0x14
#endif

#ifdef m1
#define SendCanID 0x13
#elif defined(m2)
#define SendCanID 0x14
#elif defined(m3)
#define SendCanID 0x12
#endif

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
#define CAN_BAUDRATE (250000)
int tapCount = 0;
#define TP 28
int r = 0;

Adafruit_MCP2515 mcp(CS_PIN);
std::vector<CRGB> colors = {CRGB::Blue,CRGB::Green,CRGB::Purple,CRGB::DarkOliveGreen,CRGB::Amethyst,CRGB::Chocolate,CRGB::Coral,CRGB::DarkOrchid,CRGB::DarkKhaki,CRGB::DarkSalmon};

void animation(void *parameter) {
    while (1) {
      for(auto color:colors){
         for (int i = 0; i < NUM_LEDS; i++) {
            leds[i] = color;
            FastLED.show();
             vTaskDelay(1   / portTICK_PERIOD_MS);
        }
      
      }
      }

}

void lightUpStrip(void *parameter) {
    CRGB color = *(CRGB *)parameter;
    int initTime;
    for (int i = 0; i < NUM_LEDS; i++) {
        initTime = millis();

        leds[i] = color;
        leds[i-1] = CRGB::Black;
        FastLED.show();
      //   delay(0); // Slight delay to visualize the transition


        vTaskDelay(20  / portTICK_PERIOD_MS);
    }


    vTaskDelete(NULL); // Delete the task once it's done
}

void setup() {

    // Initialize FreeRTOS tasks
    xTaskCreate(animation, "-1", 2048, NULL, 1, NULL);
    FastLED.addLeds<NEOPIXEL, pin>(leds, NUM_LEDS);
    pinMode(TP, INPUT);
    Serial.begin(115200);
    // Serial.println("MCP2515 Sender/Receiver test!");

    if (!mcp.begin(CAN_BAUDRATE)) {
        // Serial.println("Error initializing MCP2515.");
        while (1)
            delay(10);
    }
    // Serial.println("MCP2515 chip found");
    FastLED.setBrightness(50);
}

void loop() {
    Serial.println(rp2040.getFreeHeap());
    Serial.println(tapCount);
    // Transmit when TP pin changes state
    if (digitalRead(TP) == r) {
        // No state change
    } else {
        r = digitalRead(TP);

        if (r == 1) {
            // Serial.println("sending 1...");
            mcp.beginPacket(SendCanID);
            mcp.write(1);
            mcp.endPacket();
        }
    }

    // Receive and process packets
    int packetSize = mcp.parsePacket();
    if (packetSize) {
        // Check if the received packet's ID matches our canID
        if (mcp.packetId() == canID) {
            // Only process non-RTR packets
            while (mcp.available()) {
                int receivedData = mcp.read();
                if (receivedData == 1) {
                    // Serial.println("blink!");
                    CRGB color = CRGB::Red;
                    String taskName = String(tapCount);
                    if(rp2040.getFreeHeap() > 100000) xTaskCreate(lightUpStrip, taskName.c_str(), 2048, &color, 1, NULL);
                    tapCount++;
                }
            }
        }
    }

    // Run the FreeRTOS scheduler
    // vTaskDelay(1 / portTICK_PERIOD_MS); // Yield to other tasks
}
