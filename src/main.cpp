#include <vector>
#include <FS.h>
#include <Arduino.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include "fauxmoESP.h"


// =--------------------------------------------------------------------------------= Constants =--=

#define SERIAL_BAUDRATE               115200
#define LED_PIN                       2
#define RELAY_PIN                     12
#define BUTTON_PIN                    14
#define DEBOUNCE_MS                   20
#define HOLD_TIME_MS                  3000
#define DEFAULT_DEVICE_NAME           "Unknown Light"
#define DEVICE_NAME_LENGTH            64
#define SETUP_AP_NAME                 "Fauxmo Setup"
#define SETUP_AP_PASSWORD             "setupfauxmo"


// =----------------------------------------------------------------------------------= Globals =--=

Ticker ticker;
fauxmoESP fauxmo;

// Button Press / Long Press
int buttonValue = 0; // value read from button
int buttonLastValue = 0; // buffered value of the button's previous state
long buttonDownTime; // time the button was pressed down
long buttonUpTime; // time the button was released
bool ignoreUp = false; // whether to ignore the button release because the click+hold was triggered
bool hasBoot = false; // Handle a bug where a short press is triggered on boot

// Default Config Values
char device_name[DEVICE_NAME_LENGTH] = DEFAULT_DEVICE_NAME;

// Save data flag for setup config
bool shouldSaveConfig = false;


// =-------------------------------------------------------------------------= Helper Functions =--=

void tick() {
  // Toggle state
  int state = digitalRead(LED_PIN);  // get the current state of GPIO1 pin
  digitalWrite(LED_PIN, !state);     // set pin to the opposite state
}

void setLightState(bool state) {
  Serial.printf("Setting light state to %s\n", state ? "ON" : "OFF");
  digitalWrite(LED_PIN, !state);
  digitalWrite(RELAY_PIN, state);
}

bool getLightState() {
  return !digitalRead(LED_PIN);
}


// =----------------------------------------------------------------------------= Configuration =--=

// gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode...");
  Serial.println(WiFi.softAPIP());

  // if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());

  // entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

//callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setupWifi(bool reset = false) {
  // The extra parameters to be configured (can be either global or just in the setup). After
  // connecting, parameter.getValue() will get you the configured value:
  // id/name, placeholder/prompt, default, length
  WiFiManagerParameter custom_device_name("name", "device name", device_name, DEVICE_NAME_LENGTH);
  WiFiManager wifiManager;

  // Set callback for when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  // Set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // Add all your parameters here
  wifiManager.addParameter(&custom_device_name);

  // Force a reset to trigger captive AP wifi
  if (reset) {
    wifiManager.resetSettings();
  }

  // Fetches ssid and pass and tries to connect. If it does not connect it starts an access point
  // with the specified name and goes into a blocking loop awaiting configuration.
  if (!wifiManager.autoConnect(SETUP_AP_NAME, SETUP_AP_PASSWORD)) {
    Serial.println("Failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    delay(3000);
    ESP.reset();
    delay(5000);
  }

  Serial.println("Connected to WiFi");

  // Read updated parameters
  strcpy(device_name, custom_device_name.getValue());

  // Maybe save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("Saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["device_name"] = device_name;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("Failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();

    // Reboot after setup to ensure FauxmoESP is reconfigured.
    ESP.restart();
  }
}

void setupFileSystem() {
  // Clean FS, for testing
  // SPIFFS.format();

  // Read configuration from FS json
  Serial.println("Mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("Mounted file system");
    if (SPIFFS.exists("/config.json")) {
      // file exists, reading and loading
      Serial.println("Reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("Opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nParsed json");

          strcpy(device_name, json["device_name"]);
        } else {
          Serial.println("Failed to load json config");
        }
      }
    }
  } else {
    Serial.println("Failed to mount FS");
  }
  // end read
}

void setupButton() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(BUTTON_PIN, HIGH);
}

void setupOutput() {
  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Relay
  pinMode(RELAY_PIN, OUTPUT);
}

void setupFauxmo() {
  // Exit if device name is default.
  if (strcmp(device_name, DEFAULT_DEVICE_NAME) == 0) return;

  // Add Device
  fauxmo.addDevice(device_name);

  // Handle state switch
  fauxmo.onMessage([](unsigned char device_id, const char * name, bool state) {
    Serial.printf("[MAIN] Device #%d (%s) state: %s\n", device_id, name, state ? "ON" : "OFF");
    setLightState(state);
  });
}


// =---------------------------------------------------------------------------------= Handlers =--=

void handleButton() {
  // Read the state of the button
  buttonValue = digitalRead(BUTTON_PIN);

  // Test for button pressed and store the down time
  if (buttonValue == LOW && buttonLastValue == HIGH && (millis() - buttonUpTime) > long(DEBOUNCE_MS)) {
    buttonDownTime = millis();
  }

  // Test for button release and store the up time
  if (buttonValue == HIGH && buttonLastValue == LOW && (millis() - buttonDownTime) > long(DEBOUNCE_MS)) {
    if (ignoreUp == false) {
      if (hasBoot) {
        setLightState(!getLightState());
      } else {
        hasBoot = true;
      }
    } else {
      ignoreUp = false;
    }
    buttonUpTime = millis();
  }

  // Test for button held down for longer than the hold time
  if (buttonValue == LOW && (millis() - buttonDownTime) > long(HOLD_TIME_MS)) {
    ignoreUp = true;
    buttonDownTime = millis();
    setupWifi(true);
  }

  buttonLastValue = buttonValue;
}


// =-------------------------------------------------------------------------= Init and Runtime =--=

void setup() {
  // Init serial port and clean garbage
  Serial.begin(SERIAL_BAUDRATE);
  Serial.println();
  Serial.println();

  // Init Subsystems
  setupFileSystem();
  setupWifi();
  setupButton();
  setupOutput();
  setupFauxmo();
}

void loop() {
  fauxmo.handle();
  handleButton();
}
