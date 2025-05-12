#include <Adafruit_NeoPixel.h>
#include <DFRobotDFPlayerMini.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// LED Configuration
#define LED_PIN_1     D9   // GPIO pin for first strip
#define LED_PIN_2     D10  // GPIO pin for second strip
#define LED_COUNT     8   // LEDs per strip
#define BRIGHTNESS    128  // Default brightness (0-255)

Adafruit_NeoPixel strip1(LED_COUNT, LED_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(LED_COUNT, LED_PIN_2, NEO_GRB + NEO_KHZ800);

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DFPlayer Mini UART (Serial1 on Nano 33 BLE Sense)
#define PIN_MP3_RX D0  // DFPlayer TX → Arduino D0 (RX)
#define PIN_MP3_TX D1  // DFPlayer RX → Arduino D1 (TX)
#define BUSY_PIN A6    // Optional (LOW = playing, HIGH = paused/stopped)
#define BUFFER_SIZE 20
DFRobotDFPlayerMini player;

// System states
enum Mode {
    IDLE_ANIMATION,
    MOTION_DISPLAY,
    AUDIO_VISUALIZER,
    MODE_COUNT
  };
  
  void init_IMU(void);
  void runIdleAnimation(uint32_t currentMillis);
  void runMotionDisplay(uint32_t currentMillis, float currentAngle);
  void runAudioVisualizer(uint32_t currentMillis, uint8_t currentVol);
  void updateDisplay(uint32_t currentMillis, uint16_t currentTrack, bool isConnected);
  void processcommand(void);
  void clearAll();
  uint32_t Wheel(byte WheelPos);

  
bool isPlaying = false;  // Track playback state
Mode currentMode = IDLE_ANIMATION;
bool ledsOn = true;
unsigned long lastModeChange = 0;
unsigned long lastLEDUpdate = 0;
unsigned long lastAudioUpdate = 0;
unsigned long lastDisplayUpdate = 0;
const uint16_t LED_UPDATE_INTERVAL = 20;  // 20ms = 50Hz refresh
const uint16_t AUDIO_UPDATE_INTERVAL = 10; // 10ms audio refresh
const uint16_t DISPLAY_UPDATE_INTERVAL = 200; // 200ms display refresh
bool updateTrackNext = true;  // Alternates between track/volume updates
uint16_t currentTrack = 1;    // Track number (1-65535)
uint8_t currentVolume = 20;   // Volume (0-30)

// Track names - customize these
const char* trackNames[] = {
  "Rickroll", "Rumbling", "Nyannyan",
};

void clearAll() {
    strip1.clear();
    strip2.clear();
    strip1.show();
    strip2.show();
  }
  
  // Rainbow color generator
  uint32_t Wheel(byte WheelPos) {
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85) {
      return strip1.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if(WheelPos < 170) {
      WheelPos -= 85;
      return strip1.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return strip1.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  
  void runAudioVisualizer(uint32_t currentMillis, uint8_t currentVol) {
    if (currentMillis - lastAudioUpdate >= AUDIO_UPDATE_INTERVAL) {
      lastAudioUpdate = currentMillis;
      
      // Map volume to LED activation (0-30 → 0-LED_COUNT)
      uint8_t ledsToLight = map(currentVol, 0, 30, 0, LED_COUNT);
      
      // Clear all LEDs first
      strip1.clear();
      strip2.clear();
      
      // Light up LEDs progressively (left to right)
      for (int i = 0; i < ledsToLight; i++) {
        strip1.setPixelColor(i, strip1.Color(0, 0, 255));  // Blue
        strip2.setPixelColor(i, strip2.Color(0, 0, 255));  // Mirror on second strip
      }
      
      strip1.show();
      strip2.show();
    }
  }
  
  void runIdleAnimation(uint32_t currentMillis) {
    static uint32_t animationStartTime = 0;
    if (animationStartTime == 0) animationStartTime = currentMillis;
    
    uint32_t elapsed = currentMillis - animationStartTime;
    
    // Mirror rainbow effect on both strips
    for(int i=0; i<LED_COUNT; i++) {
      uint32_t color = Wheel(((elapsed * 256 / 2000) + (i * 256 / LED_COUNT)) % 256);
      strip1.setPixelColor(i, color);
      strip2.setPixelColor(LED_COUNT-1-i, color); // Reverse direction on second strip
    }
    strip1.show();
    strip2.show();
  }
  void runMotionDisplay(uint32_t currentMillis, float currentAngle) {
    // Get robot angle from IMU (replace with your actual sensor reading)
    // Convert angle to absolute 0-90° range
    float absAngle = abs(currentAngle);
  
    if(absAngle>40) {
      absAngle = 40;
    }
    
    // Map angle to color (0°=green, 90°=red)
    uint8_t r = map(absAngle, 0, 40, 0, 255);
    uint8_t g = map(absAngle, 0, 40, 255, 0);
    uint8_t b = 0; // No blue component
    
    // Apply to all LEDs in both strips
    for(int i=0; i<LED_COUNT; i++) {
      strip1.setPixelColor(i, strip1.Color(r, g, b));
      strip2.setPixelColor(i, strip2.Color(r, g, b)); 
    }
    strip1.show();
    strip2.show();
  }

void updateDisplay(uint32_t currentMillis, uint16_t currentTrack, bool isConnected) {
  display.clearDisplay();
  
  // Battery percentage
  uint8_t vol = analogRead(A0);
  Serial.println(vol);
  // Determine number of blocks to display (0-3)
  uint8_t batteryBlocks;
  if (vol <= 1)       batteryBlocks = 0;  // Empty
  else if (vol <= 5) batteryBlocks = 1;  // 1 block
  else if (vol <= 10) batteryBlocks = 2;  // 2 blocks
  else                    batteryBlocks = 3;  // 3 blocks (full)

  // Display battery blocks (simpler visual)
  display.drawRect(104, 0, 16, 8, SSD1306_WHITE);  // Outer battery shape
  display.fillRect(120, 3, 2, 3, SSD1306_WHITE);   // Battery tip
  
  // Fill blocks based on level (each block = 4px wide, 1px spacing)
  for (uint8_t i = 0; i < batteryBlocks; i++) {
    display.fillRect(105 + (i * 5), 1, 4, 6, SSD1306_WHITE);
  }
  
  if (isConnected) {
    // Bluetooth status (top left)
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Connected");

    // Track info (center, large text)
    if (currentTrack == 0) {
      display.setTextSize(2);
      display.setCursor(0, 30);
      display.print("No Song");
    } else {
      display.setTextSize(2);
      display.setCursor(0, 30);
      display.print(trackNames[currentTrack - 1]);
    }
    
    // Play/pause status using BUSY pin
    display.setTextSize(1);
    display.setCursor(0, 50);
    if (isPlaying) {
      display.print("Now Playing");
    } else {
      display.print("Paused");
    }
  } else {
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Disconnected");
  }
  
  display.display();
}