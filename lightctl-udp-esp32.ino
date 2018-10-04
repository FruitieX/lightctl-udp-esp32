#include "FastLED.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

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
constexpr uint16_t DATA_PINS[] = {13};
constexpr uint16_t NUM_LEDS[] = {89};
#define TOTAL_LEDS 89

#define MAX_MA 1600 // Power limiter, consider what your power supply & wiring can handle. Remember that the ESP32+WiFi also draws power. (100-200 mA?)

#elif CONFIG == 1
#define ID "Skumppaflaska"
#define NUM_STRIPS 1
constexpr uint16_t DATA_PINS[] = {13};
constexpr uint16_t NUM_LEDS[] = {60};
#define TOTAL_LEDS 60

#define MAX_MA 1000

#elif CONFIG == 2
#define ID "WC väggskåp"

#define NUM_STRIPS 2
constexpr uint16_t DATA_PINS[] = {12, 13};
constexpr uint16_t NUM_LEDS[] = {60, 73};
#define TOTAL_LEDS 133

#define MAX_MA 1500

#elif CONFIG == 3
#define ID "Block"

#define NUM_STRIPS 1
constexpr uint16_t DATA_PINS[] = {13};
constexpr uint16_t NUM_LEDS[] = {16};
#define TOTAL_LEDS 16

#define MAX_MA 500

#endif

// Enable debug mode
//#define DEBUG

uint8_t gamma_lookup_r[256];
uint8_t dither_lookup_r[256];
uint8_t gamma_lookup_g[256];
uint8_t dither_lookup_g[256];
uint8_t gamma_lookup_b[256];
uint8_t dither_lookup_b[256];

// actual gamma value is / 100
uint8_t GAMMA_RED = 220;
uint8_t GAMMA_GREEN = 250;
uint8_t GAMMA_BLUE = 180;
uint8_t CONTRAST_RED = 255;
uint8_t CONTRAST_GREEN = 255;
uint8_t CONTRAST_BLUE = 255;

// at some point we don't need to dither anymore
uint8_t dither_cutoff = 64;

void calcGammaChannel(uint8_t gamma_lookup[], uint8_t dither_lookup[], uint8_t gamma, uint8_t contrast) {
  uint8_t i = 0;

  do {
    double x = (double) i / 255.0;
    double val = (double) contrast / 255.0 * pow(x, (double) gamma / 100.0);

    gamma_lookup[i] = floor(val * 255.0);

    double trash;
    dither_lookup[i] = i >= dither_cutoff ? 0 : floor(modf(val * 255.0, &trash) * 255.0);
  } while (i++ != 255);

  // zero always means off
  gamma_lookup[i] = 0;
  dither_lookup[i] = 0;

  // set any following zero values to 1; this way we get slightly
  // less wrong color at extremely low brightness
  for (i = 1;; i++) {
    if (dither_lookup[i] == 0) {
      dither_lookup[i] = 1;
    } else {
      break;
    }
  }

  #ifdef DEBUG
  Serial.println("gamma_lookup:");
  for (i = 0; i < 60; i++) {
    Serial.print(String(gamma_lookup[i]) + ",\t");
  }
  Serial.println("...");

  Serial.println("dither_lookup:");
  for (i = 0; i < 60; i++) {
    Serial.print(String(dither_lookup[i]) + ",\t");
  }
  Serial.println("...");
  #endif
}

void recalcGamma() {
  #ifdef DEBUG
  Serial.println("recalcGamma()" + String(GAMMA_RED) + ", " + String(GAMMA_GREEN) + ", " + String(GAMMA_BLUE));
  #endif
  calcGammaChannel(gamma_lookup_r, dither_lookup_r, GAMMA_RED, CONTRAST_RED);
  calcGammaChannel(gamma_lookup_g, dither_lookup_g, GAMMA_GREEN, CONTRAST_GREEN);
  calcGammaChannel(gamma_lookup_b, dither_lookup_b, GAMMA_BLUE, CONTRAST_BLUE);
}

CRGB leds[NUM_STRIPS][TOTAL_LEDS];

#define PACKET_LENGTH TOTAL_LEDS * 3 + 7
uint8_t cur_payload[PACKET_LENGTH]; // last uint8_t specifies amount of dither steps (0 disables dithering)

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
  #ifdef DEBUG
  Serial.begin(115200);
  #endif

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

  recalcGamma();
  checkWifi();

  ArduinoOTA.setHostname(ID);
  ArduinoOTA.begin();
}

uint8_t dither_value = 0;

void set_led (uint16_t offs, uint8_t r, uint8_t g, uint8_t b) {
  uint8_t strip = 0;

  // Find out which strip to control and led count prior to that strip
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

uint16_t dither_table[12] = {
  0b000000000000, //0%
  //0b100000000000, //8% // skip over this to reduce extreme flicker
  0b100000100000, //16%
  0b100010001000, //25%
  0b100100100100, //33%
  0b110100100100, //42%
  0b101010101010, //50%
  0b110101010101, //58%
  0b110110110110, //66%
  0b111011101110, //75%
  0b111110111110, //83%
  0b111111111110, //92%
  0b111111111111, //100%
};

boolean should_dither(uint8_t val, uint8_t tresh) {
  // rounding up division: https://stackoverflow.com/a/2745086
  uint8_t index = (tresh * 12 + 256 - 1) / 256;
  index = index > 11 ? 11 : index;

  uint16_t table_entry = dither_table[index];
  //val = val % 12;
  return (table_entry >> val) & 1;
}

unsigned long lastKeepalive = 0;
void loop() {
  checkWifi();

  ArduinoOTA.handle();

  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Udp.read(cur_payload, PACKET_LENGTH);

    boolean gamma_changed =
      cur_payload[TOTAL_LEDS * 3 + 1] != GAMMA_RED ||
      cur_payload[TOTAL_LEDS * 3 + 2] != GAMMA_GREEN ||
      cur_payload[TOTAL_LEDS * 3 + 3] != GAMMA_BLUE ||
      cur_payload[TOTAL_LEDS * 3 + 4] != CONTRAST_RED ||
      cur_payload[TOTAL_LEDS * 3 + 5] != CONTRAST_GREEN ||
      cur_payload[TOTAL_LEDS * 3 + 6] != CONTRAST_BLUE;

    GAMMA_RED = cur_payload[TOTAL_LEDS * 3 + 1];
    GAMMA_GREEN = cur_payload[TOTAL_LEDS * 3 + 2];
    GAMMA_BLUE = cur_payload[TOTAL_LEDS * 3 + 3];
    CONTRAST_RED = cur_payload[TOTAL_LEDS * 3 + 4];
    CONTRAST_GREEN = cur_payload[TOTAL_LEDS * 3 + 5];
    CONTRAST_BLUE = cur_payload[TOTAL_LEDS * 3 + 6];

    if (gamma_changed) {
      recalcGamma();
    }
  }

  if (!lastKeepalive || millis() - lastKeepalive > 5000) {
    lastKeepalive = millis();
    sendKeepalive();
  }

  dither_value = (dither_value + 1) % 12;
  if (!cur_payload[TOTAL_LEDS * 3 + 0]) {
    // disable dithering if requested
    dither_value = 0;
  }

  for (uint16_t i = 0; i < TOTAL_LEDS; i++) {
    // Convert HSV to RGB
    CHSV hsv(cur_payload[i * 3 + 0], cur_payload[i * 3 + 1], cur_payload[i * 3 + 2]);
    CRGB rgb;
    hsv2rgb_rainbow(hsv, rgb);

    // Convert HSV with maxed out brightness to RGB
    CHSV hsv_max(cur_payload[i * 3 + 0], cur_payload[i * 3 + 1], 255);
    CRGB rgb_max;
    hsv2rgb_rainbow(hsv_max, rgb_max);

    uint8_t r_adjusted = should_dither(dither_value, dither_lookup_r[rgb.red]) ? gamma_lookup_r[rgb.red] + 1: gamma_lookup_r[rgb.red];
    uint8_t g_adjusted = should_dither(dither_value, dither_lookup_g[rgb.green]) ? gamma_lookup_g[rgb.green] + 1: gamma_lookup_g[rgb.green];
    uint8_t b_adjusted = should_dither(dither_value, dither_lookup_b[rgb.blue]) ? gamma_lookup_b[rgb.blue] + 1: gamma_lookup_b[rgb.blue];

    set_led(i, r_adjusted, g_adjusted, b_adjusted);
  }

  FastLED.show();
}
