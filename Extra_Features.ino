#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>
#include <String.h>
#include "Balbot_RGB.h"
#include "KalmanFilter.h"

KalmanFilter kalmanX(0.002, 0.000001, 0.05);
KalmanFilter kalmanY(0.002, 0.000001, 0.05);

float kalPitch = 0;
float gyro_x, gyro_y, gyro_z;  // nRF52840 has FPU
float accel_x, accel_y, accel_z;
float accPitch = 0;

BLEService customService("8080646C-5489-4764-A74F-E12E60FEDCF7");
BLECharacteristic customCharacteristic(
    "2FAE7161-682F-4F28-BCAA-ECE5CFA91EFE", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);
void processcommand();

void setup() {
  Serial.begin(9600);   // USB Serial (for commands)
  Serial1.begin(9600);  // Hardware Serial for DFPlayer

  // Initialize OLED display (non-blocking)
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      // Keep trying without blocking
      static unsigned long lastRetry = 0;
      if(millis() - lastRetry > 1000) {
        Serial.println("Retrying display...");
        lastRetry = millis();
      }
    }
  }
  display.clearDisplay();
  display.display();

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set the device name and local name
  BLE.setLocalName("SelfBalancingBot");
  BLE.setDeviceName("SelfBalancingBot");

  // Add the characteristic to the service
  customService.addCharacteristic(customCharacteristic);

  // Add the service
  BLE.addService(customService);

  // Set an initial value for the characteristic
  customCharacteristic.writeValue("Waiting for data");

  // Start advertising the service
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");

  Serial.println("Initializing DFPlayer...");
  
  if (!player.begin(Serial1)) {
    Serial.println("DFPlayer Mini failed to connect!");
    while (true);  // Halt if DFPlayer fails
  }

  Serial.println("DFPlayer Mini Ready!");
  Serial.println("Commands: PLAY, PAUSE, NEXT, PREV, VOL+, VOL-");
  
  player.volume(20);  // Default volume (0-30)
  isPlaying = false;  // Start in paused state

  // Initialize LED strips
  init_IMU();
  strip1.begin();
  strip2.begin();
  strip1.setBrightness(BRIGHTNESS);
  strip2.setBrightness(BRIGHTNESS);
  strip1.show();
  strip2.show();

  Serial.println("System Ready");
}

void loop() {
  uint32_t currentMillis = millis();
  static unsigned long lastUpdate = 0;
  BLEDevice central = BLE.central();

  bool isConnected = central; // Store connection status

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate connection
  }
  
  // Keep running while connected
  while (central.connected()) {
    currentMillis = millis();

    IMU.readAcceleration(accel_x, accel_y, accel_z);
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
    accPitch = -(atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0) / M_PI;
    kalPitch = kalmanY.update(accPitch, gyro_x);
    Serial.println(kalPitch);

    processcommand();

    // Update display at specified interval (even while connected)
    if (currentMillis - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
      lastDisplayUpdate = currentMillis;
      
      // Get current state (only when needed)
      if (updateTrackNext) {
        currentTrack = player.readCurrentFileNumber();
      } else {
        currentVolume = player.readVolume();
      }
      updateTrackNext = !updateTrackNext;
      
      updateDisplay(currentMillis, currentVolume, true);
      displayRGBLED(currentTrack);
    }
  }
  // Update display when not connected
  if (currentMillis - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = currentMillis;
    if (updateTrackNext) {
      currentTrack = player.readCurrentFileNumber();
    } else {
      currentVolume = player.readVolume();
    }
    updateTrackNext = !updateTrackNext;
    updateDisplay(currentMillis, currentVolume, false); // Pass false for disconnected
  }
}

void displayRGBLED(uint8_t currentVol){
      uint32_t currentMillis = millis();
      if (!ledsOn) {
      if (currentMillis - lastLEDUpdate >= LED_UPDATE_INTERVAL) {
        lastLEDUpdate = currentMillis;
        strip1.clear();
        strip2.clear();
        strip1.show();
        strip2.show();
      }
    }
    
    // Only update LEDs at the specified interval
    if (currentMillis - lastLEDUpdate >= LED_UPDATE_INTERVAL) {
      lastLEDUpdate = currentMillis;
      
      switch (currentMode) {
        case IDLE_ANIMATION:
          runIdleAnimation(currentMillis);
          break;
        case MOTION_DISPLAY:
          runMotionDisplay(currentMillis, kalPitch);
          break;
        case AUDIO_VISUALIZER:
          runAudioVisualizer(currentMillis, currentVol);
          break;
      }
    }
    return;
}

void processcommand(){
      if (customCharacteristic.written()) {
            // Get the length of the received data
             int length = customCharacteristic.valueLength();
     
             // Read the received data
             const unsigned char* receivedData = customCharacteristic.value();
     
             // Create a properly terminated string
             char receivedString[length + 1]; // +1 for null terminator
             memcpy(receivedString, receivedData, length);
             receivedString[length] = '\0'; // Null-terminate the string
     
             // Print the received data to the Serial Monitor
             Serial.print("Received data: ");
             Serial.println(receivedString);
     
             // Process received string
             if(strcmp(receivedString, "LEFT") == 0){

             } 
             else if (strcmp(receivedString, "RIGHT") == 0){

             }
             else if (strcmp(receivedString, "BACKWARD") == 0){

             }
             else if (strcmp(receivedString, "FORWARD") == 0){

             }
             else if (strcmp(receivedString, "STOP") == 0){

             }
             else if (strcmp(receivedString,"RGBONOFF") == 0){
              ledsOn = !ledsOn;
             }
             else if (strcmp(receivedString, "RGBIDLE") == 0){
              currentMode = (Mode)((currentMode + 1) % MODE_COUNT);
             }

             else if (strcmp(receivedString, "PLAYBACK") == 0){
              if (isPlaying) {
                player.pause();  // Pause if currently playing
                Serial.println("⏸ Paused");
              } else {
                player.start();  // Play if paused
                Serial.println("▶️ Playing");
              }
              isPlaying = !isPlaying;  // Toggle state
             }

             else if (strcmp(receivedString, "SKIP") == 0){
              player.next();  // Next track
              Serial.println("⏭ Next Track");
             }
             else if (strcmp(receivedString, "VOLUP") == 0){
              player.volumeUp();  // Increase volume
              Serial.println("🔊 Volume +");
             }
             else if (strcmp(receivedString, "VOLDOWN") == 0){
              player.volumeDown();  // Decrease volume
              Serial.println("🔉 Volume -");
             }
             
     
    } // end of if (customCharacteristic.written())

}

void init_IMU(void){

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
  }
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Angular speed in degrees/second");
  Serial.println("X\tY\tZ");

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");

}