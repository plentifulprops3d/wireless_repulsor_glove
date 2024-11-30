#include <WiFi.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <DFRobotDFPlayerMini.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <FastLED.h>
#include <Wire.h>

#define BUTTON_PIN 1
#define NEOPIXEL_PIN 10
#define NUM_PIXELS 8
#define I2C_SDA_PIN 5
#define I2C_SCL_PIN 6
#define DFPLAYER_RX 7
#define DFPLAYER_TX 8

uint8_t receiverAddress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t soundBoardMAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Sound effect board MAC

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
DFRobotDFPlayerMini dfPlayer;
CRGB leds[NUM_PIXELS];

unsigned long buttonPressStartTime = 0;
const unsigned long longPressThreshold = 1000;
const unsigned long veryLongPressThreshold = 5000;
const unsigned long extraLongPressThreshold = 8000;

enum Mode {
  ONBOARD_SOUND,
  WIRELESS_SOUND
};

Mode currentMode = ONBOARD_SOUND;

enum FadeState {
  OFF,
  FADING_ON,
  ON,
  FADING_OFF,
  ALWAYS_ON,
  BLAST
};

FadeState fadeState = OFF;
unsigned long fadeStartTime = 0;
const unsigned long fadeDuration = 500;

const int filterSize = 20;
float angleBuffer[filterSize] = {0};
int bufferIndex = 0;

const float movementThreshold = 0.8; // Increased sensitivity
const unsigned long angleCheckInterval = 50;
unsigned long lastAngleCheckTime = 0;

unsigned long blastStartTime = 0;
const unsigned long blastDuration = 1500; // 1.5 seconds
const unsigned long blastCooldown = 1000; // Cooldown between blasts

bool blastTriggered = false;
unsigned long lastBlastTime = 0;

void handleButtonPress();
void handleAccelerometer();
void handleNeoPixels();
void sendWirelessData(uint8_t data, const uint8_t *address);
float calculateMovingAverage(float newValue);

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  WiFi.mode(WIFI_STA);
  Serial.println("WiFi initialized");
  Serial.print("Controller MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("ESP-NOW initialized");

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add receiver peer");
    return;
  }
  Serial.println("Receiver peer added");

  memcpy(peerInfo.peer_addr, soundBoardMAC, 6);

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add soundboard peer");
    return;
  }
  Serial.println("Soundboard peer added");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  if (!accel.begin()) {
    Serial.println("No ADXL345 detected");
    while (1);
  }
  Serial.println("ADXL345 initialized");

  FastLED.addLeds<NEOPIXEL, NEOPIXEL_PIN>(leds, NUM_PIXELS);
  FastLED.clear();
  FastLED.show();
  Serial.println("FastLED initialized");

  Serial1.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
  if (!dfPlayer.begin(Serial1)) {
    Serial.println("DFPlayer Mini not detected");
    while (1);
  }
  dfPlayer.setTimeOut(500);
  dfPlayer.volume(30);
  Serial.println("DFPlayer Mini initialized");
}

void loop() {
  handleButtonPress();
  handleAccelerometer();
  handleNeoPixels();
}

void handleButtonPress() {
  static int lastButtonState = HIGH;
  int buttonState = digitalRead(BUTTON_PIN);
  unsigned long currentTime = millis();

  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPressStartTime = currentTime;
  } else if (buttonState == HIGH && lastButtonState == LOW) {
    unsigned long pressDuration = currentTime - buttonPressStartTime;
    if (pressDuration >= extraLongPressThreshold) {
      currentMode = (currentMode == ONBOARD_SOUND) ? WIRELESS_SOUND : ONBOARD_SOUND;
      Serial.print("Mode switched to: ");
      Serial.println(currentMode == ONBOARD_SOUND ? "ONBOARD_SOUND" : "WIRELESS_SOUND");
    } else if (pressDuration >= veryLongPressThreshold) {
      if (fadeState == ALWAYS_ON) {
        fadeState = OFF;
        Serial.println("Exiting ALWAYS_ON mode");
      } else {
        fadeState = ALWAYS_ON;
        Serial.println("Entering ALWAYS_ON mode");
      }
    } else if (pressDuration >= longPressThreshold) {
      uint8_t data = 102;
      sendWirelessData(data, receiverAddress);
      Serial.println("Long press detected, data sent");
    } else {
      uint8_t data = 101;
      sendWirelessData(data, receiverAddress);
      Serial.println("Short press detected, data sent");
    }
    delay(200);
  }

  lastButtonState = buttonState;
}

void handleAccelerometer() {
  unsigned long currentTime = millis();

  if (currentTime - lastAngleCheckTime < angleCheckInterval) {
    return;
  }
  lastAngleCheckTime = currentTime;

  sensors_event_t event;
  accel.getEvent(&event);

  float angle = atan2(event.acceleration.y, event.acceleration.z) * 180 / PI;
  float filteredAngle = calculateMovingAverage(angle);

  static float lastFilteredAngle = 0;

  if (abs(filteredAngle - lastFilteredAngle) < movementThreshold) {
    return;
  }

  float accelerationChange = event.acceleration.x;
  float yAngle = atan2(event.acceleration.x, event.acceleration.z) * 180 / PI;
  float zAngle = atan2(event.acceleration.x, event.acceleration.y) * 180 / PI;

  if (!blastTriggered && accelerationChange < -0.9 && abs(yAngle) < 45 && abs(zAngle) < 30) {
    if (fadeState == ON) {
      if (currentTime - lastBlastTime > blastCooldown) {
        fadeState = BLAST;
        blastTriggered = true;
        blastStartTime = millis();
        if (currentMode == WIRELESS_SOUND) {
          sendWirelessData(13, soundBoardMAC);
        } else {
          dfPlayer.play(3);
        }
        Serial.println("BLAST activated");
        lastBlastTime = currentTime;
      }
    }
  }

  lastFilteredAngle = filteredAngle;

  if (filteredAngle >= 35 && fadeState == OFF) {
    if (currentMode == WIRELESS_SOUND) {
      sendWirelessData(12, soundBoardMAC);
    } else {
      dfPlayer.play(2);
    }
    fadeState = FADING_ON;
    fadeStartTime = millis();
    Serial.println("Starting to fade on");
  } else if (filteredAngle < 35 && fadeState == ON) {
    if (currentMode == WIRELESS_SOUND) {
      sendWirelessData(11, soundBoardMAC);
    } else {
      dfPlayer.play(1);
    }
    fadeState = FADING_OFF;
    fadeStartTime = millis();
    Serial.println("Starting to fade off");
  }
}

void handleNeoPixels() {
  unsigned long currentTime = millis();

  switch (fadeState) {
    case OFF:
      FastLED.clear();
      FastLED.show();
      break;

    case FADING_ON:
      if (currentTime - fadeStartTime < fadeDuration) {
        int brightness = map(currentTime - fadeStartTime, 0, fadeDuration, 0, 255);
        for (int i = 0; i < NUM_PIXELS; i++) {
          leds[i] = CRGB(brightness, brightness, brightness);
        }
        FastLED.show();
      } else {
        for (int i = 0; i < NUM_PIXELS; i++) {
          leds[i] = CRGB(255, 255, 255);
        }
        FastLED.show();
        fadeState = ON;
        Serial.println("Fade on complete");
      }
      break;

    case ON:
      for (int i = 0; i < NUM_PIXELS; i++) {
        leds[i] = CRGB(255, 255, 255);
      }
      FastLED.show();
      break;

    case FADING_OFF:
      if (currentTime - fadeStartTime < fadeDuration) {
        int brightness = map(currentTime - fadeStartTime, 0, fadeDuration, 255, 0);
        for (int i = 0; i < NUM_PIXELS; i++) {
          leds[i] = CRGB(brightness, brightness, brightness);
        }
        FastLED.show();
      } else {
        FastLED.clear();
        FastLED.show();
        fadeState = OFF;
        Serial.println("Fade off complete");
      }
      break;

    case ALWAYS_ON:
      for (int i = 0; i < NUM_PIXELS; i++) {
        leds[i] = CRGB(255, 255, 255);
      }
      FastLED.show();
      break;

    case BLAST:
      if (currentTime - blastStartTime < blastDuration) {
        for (int i = 0; i < NUM_PIXELS; i++) {
          int flicker = random(100, 150); // Adjust brightness for darker yellow
          if (random(3) == 0) {
            leds[i] = CRGB(flicker, flicker / 2, 0); // Darker yellow with occasional orange flicker
          } else {
            leds[i] = CRGB(flicker, flicker, flicker); // White with flicker
          }
        }
        FastLED.show();
        delay(50); // Adjust flicker speed
      } else {
        FastLED.clear();
        FastLED.show();
        blastTriggered = false;
        fadeState = ON;
        Serial.println("BLAST complete");
      }
      break;
  }
}


void sendWirelessData(uint8_t data, const uint8_t *address) {
  esp_now_send(address, &data, sizeof(data));
}

float calculateMovingAverage(float newValue) {
  static float sum = 0;
  static int count = 0;
  static float readings[filterSize];

  sum -= readings[bufferIndex];
  readings[bufferIndex] = newValue;
  sum += readings[bufferIndex];
  bufferIndex = (bufferIndex + 1) % filterSize;
  count = min(count + 1, filterSize);

  return sum / count;
}



