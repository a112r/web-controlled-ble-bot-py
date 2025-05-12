#include <Adafruit_NeoPixel.h>

#define LED_PIN_1     D9   // GPIO pin for first strip
#define LED_PIN_2     D10  // GPIO pin for second strip
#define LED_COUNT     8   // LEDs per strip
#define BRIGHTNESS    32  // Default brightness (0-255)

Adafruit_NeoPixel strip1(LED_COUNT, LED_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(LED_COUNT, LED_PIN_2, NEO_GRB + NEO_KHZ800);

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
  void processcommand(void);
  void clearAll();
  uint32_t Wheel(byte WheelPos);

  Mode currentMode = IDLE_ANIMATION;
bool ledsOn = true;
unsigned long lastModeChange = 0;
unsigned long lastLEDUpdate = 0;
unsigned long lastAudioUpdate = 0;
unsigned long lastDisplayUpdate = 0;
const uint16_t LED_UPDATE_INTERVAL = 100;  // 20ms = 50Hz refresh
const uint16_t AUDIO_UPDATE_INTERVAL = 10; // 10ms audio refresh

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