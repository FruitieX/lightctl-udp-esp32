#include "FastLED.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

boolean connected = false;
WiFiUDP Udp;

// WiFi credentials
#define SSID "**********"
#define PASSWORD "**********"

// lightctl server
#define IP "192.168.1.101"
#define PORT 1234

// LED strip
#define CONFIG 2

#if CONFIG == 0
#define ID "Fönsterbräde"
#define NUM_STRIPS 1
#define DATA_PINS_ {13}
#define NUM_LEDS_ {89}
#define TOTAL_LEDS 89

#define MAX_MA 1600 // Power limiter, consider what your power supply & wiring can handle. Remember that the ESP32+WiFi also draws power. (100-200 mA?)

#elif CONFIG == 1
#define ID "Skumppaflaska"
#define NUM_STRIPS 1
#define DATA_PINS_ {13}
#define NUM_LEDS_ {60}
#define TOTAL_LEDS 60

#define MAX_MA 750

#elif CONFIG == 2
#define ID "WC väggskåp"

#define NUM_STRIPS 2
#define DATA_PINS_ {12, 13}
#define NUM_LEDS_ {60, 73}
#define TOTAL_LEDS 133

#define MAX_MA 1500

#endif

// Enable debug mode
//#define DEBUG

constexpr uint16_t DATA_PINS[] = DATA_PINS_;
constexpr uint16_t NUM_LEDS[] = NUM_LEDS_;

CRGB leds[NUM_STRIPS][TOTAL_LEDS];

uint8_t cur_payload[TOTAL_LEDS * 3 + 1]; // last uint8_t specifies amount of dither steps (0 disables dithering)

void checkWifi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.begin(SSID, PASSWORD);

    unsigned long timeout = millis() + 60000;
    
    // Cool animation while connecting to wifi :-)
    uint16_t index = 0;
    CHSV loading_color = CHSV(0, 255, 255);
    while(WiFi.status() != WL_CONNECTED) {
      delay(20);
  
      loading_color.hue++;

      for (uint8_t strip = 0; strip < NUM_STRIPS; strip++) {
        leds[strip][index % (NUM_LEDS[strip])] = loading_color;
        
        for (uint16_t i = 0; i < NUM_LEDS[strip]; i++) {
          leds[strip][i].fadeToBlackBy(10);
        }
      }
      
      index++;
  
      FastLED.show();

      if (millis() > timeout) break;
    }

    WiFi.onEvent(WiFiEvent);
  }
}

void sendKeepalive() {
  String payload = "{\"id\": \"" + String(ID) + "\", \"numLights\": " + TOTAL_LEDS + "}";

  Udp.beginPacket(IP, PORT);
  Udp.print(payload);
  Udp.endPacket();
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
  switch(event) {
  case SYSTEM_EVENT_STA_GOT_IP:
    //When connected set 
    //Serial.print("WiFi connected! IP address: ");
    //Serial.println(WiFi.localIP());  
    //initializes the UDP state
    //This initializes the transfer buffer
    Udp.begin(WiFi.localIP(), PORT);
    connected = true;
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    connected = false;
    break;
  }
}

void setup() { 
  // for loop won't do here because FastLED wants compile time constants
  FastLED.addLeds<WS2812Controller800Khz, DATA_PINS[0], GRB>(leds[0], NUM_LEDS[0]);
  #if NUM_STRIPS > 1
  FastLED.addLeds<WS2812Controller800Khz, DATA_PINS[1], GRB>(leds[1], NUM_LEDS[1]);
  #endif
  #if NUM_STRIPS > 2
  FastLED.addLeds<WS2812Controller800Khz, DATA_PINS[2], GRB>(leds[2], NUM_LEDS[2]);
  #endif
  #if NUM_STRIPS > 3
  FastLED.addLeds<WS2812Controller800Khz, DATA_PINS[3], GRB>(leds[3], NUM_LEDS[3]);
  #endif
  // etc... feel free to add more if needed
  
  FastLED.setMaxPowerInVoltsAndMilliamps(5, MAX_MA);
  FastLED.setDither(0);
  FastLED.show(); // leds shine white by default, turn them off after esp32 has booted
  
  checkWifi();
}

uint8_t val_lookup[256] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,2,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,7,7,7,8,8,9,9,9,10,10,10,11,11,12,12,13,13,14,14,15,15,16,16,17,17,18,18,19,19,20,20,21,21,22,23,23,24,25,25,26,26,27,28,28,29,30,30,31,32,33,33,34,35,36,36,37,38,39,39,40,41,42,43,43,44,45,46,47,48,49,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,81,82,83,84,85,86,87,89,90,91,92,93,95,96,97,98,100,101,102,103,105,106,107,108,110,111,112,114,115,116,118,119,121,122,123,125,126,127,129,130,132,133,135,136,138,139,141,142,144,145,147,148,150,151,153,154,156,157,159,160,162,164,165,167,169,170,172,173,175,177,178,180,182,183,185,187,189,190,192,194,196,197,199,201,203,204,206,208,210,212,213,215,217,219,221,223,225,226,228,230,232,234,236,238,240,242,244,246,248,250,252,254};
uint8_t dither_lookup[256] = {0,1,3,8,15,24,35,48,63,80,99,120,143,168,195,224,0,32,67,104,143,184,227,16,63,112,163,216,15,72,131,192,0,64,131,200,15,88,163,240,63,144,227,56,143,232,67,160,0,96,195,40,143,248,99,208,63,176,35,152,15,136,3,128,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t dither_value = 0;

void set_led (uint16_t offs, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t strip = 0;

  // Find out which strip to control and led count before said strip
  uint16_t cnt_before = 0;
  for (uint8_t i = 0; i < NUM_STRIPS; i++) {
    strip = i;

    // Break once sum is about to become too large (= led is in current strip)
    if (cnt_before + NUM_LEDS[strip] > offs) {
      break;
    }

    // Otherwise add current strip led count to cnt_before
    cnt_before += NUM_LEDS[strip];
  }
  
  leds[strip][offs - cnt_before].setRGB(r, g, b);
}

unsigned long lastKeepalive = 0;
void loop() {
  checkWifi();

  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Udp.read(cur_payload, TOTAL_LEDS * 3 + 1);
  }

  if (!lastKeepalive || millis() - lastKeepalive > 5000) {
    lastKeepalive = millis();
    sendKeepalive();
  }

  uint8_t dither_steps = cur_payload[TOTAL_LEDS * 3];
  if (dither_steps != 0) {
    dither_value = dither_value + 256 / dither_steps;
  } else {
    dither_value = 0;
  }

  for (uint16_t i = 0; i < TOTAL_LEDS; i++) {
    uint8_t r = cur_payload[i * 3 + 0];
    uint8_t g = cur_payload[i * 3 + 1];
    uint8_t b = cur_payload[i * 3 + 2];

    r = dither_value < dither_lookup[r] ? val_lookup[r] + 1: val_lookup[r];
    g = dither_value < dither_lookup[g] ? val_lookup[g] + 1: val_lookup[g];
    b = dither_value < dither_lookup[b] ? val_lookup[b] + 1: val_lookup[b];

    set_led(i, r, g, b);
  }
  
  FastLED.show();
}
