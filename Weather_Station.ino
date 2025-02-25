/*
Weather Station - (C) C. Schubert 2024
======================================
  A small Weather station based on the
  Arduino Nano ESP32 and BMP280. 
  It comes with wifi connection aswell
  as MQTT data sharing capabilities.
  Hardware Version: 1.2
  Software Version: 2.8.1
Hardware Connections
====================
  PUSH BUTTON: (NOTE: required if 'deep sleep' is enabled and 'wake up from button' is enabled)
    Connection Type: PULL_DOWN;
    I/O -> A1

  DISPLAY WITH SH1106 CONTROLLER CHIP, 128x64px IN SIZE: (NOTE: required if 'headless mode' is disabled, optional if enabled)
    Connection Type: I2C
    SDA -> A4
    SCK -> A5

  BMP280 TEMPERATURE, PRESSURE AND ALTITUDE SENSOR: (NOTE: always required)
    Connection Type: I2C
    SDA -> A6
    SCK -> A7

Additional Files
================
  configuration.h:
    Contains Configuration Variables.
    May be adjusted to ones liking.

  constants.h:
    Contains necessary constant definitions.
    LEAVE THEM ALONE!!!

  init.h:
    Containt necessary definitions for sketch initialization.
    LEAVE THEM ALONE!
*/

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include "constants.h"
#include "init.h"
#include "configuration.h"

/**
Interrupt Handling Routine (Timer Interrupt)
*/
void IRAM_ATTR onHeartbeat() {
  onHeartbeatInterrupt = true;
}

/**
drawSplashScreen():
===================
Draws the splash screen
==========================================================
- Draws the display splash screen, containing version information
*/
void drawSplashScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_spleen8x16_mf);
  u8g2.drawStr(36, 15, "Weather");
  u8g2.drawStr(36, 32, "Station");
  u8g2.setFont(u8g2_font_spleen6x12_mf);
  u8g2.drawStr(7, 49, "Hardware Rev: 1.2");
  u8g2.drawStr(7, 63, "Software Ver: 2.8.1");
  u8g2.updateDisplay();
  delay(1500);
}

/**
checkAndEnableDisplayConnection():
==================================
Checks, if Display is available, if not activates headless mode
===============================================================
- Checks, if connection to display is possible (eg. if display is connected and ready to talk),
  if not activates headless mode
- It not in headless mode: enable Display Communication
- If not in headless mode: enable printing UTF8 characters
*/
void checkAndEnableDisplayConnection() {
  Serial.println("Checking for Display on PINs A4 -> SDA, A5 -> SCL");
  Wire.begin(A4, A5);
  Wire.beginTransmission(0x3C);
  if (Wire.endTransmission() != 0) {
    if (ENABLE_HEADLESS_MODE) {
      Serial.println("No Display found! Enabling Headless Mode!");
      headlessMode = true;
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_BLUE, LOW);
      delay(1000);
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_BLUE, HIGH);
      delay(150);
      return;
    } else {
      Serial.println("ERROR: NO DISPLAY FOUND! PLEASE CONNECT PINs A4 -> SDA, A5 -> SCL AND RESTART DEVICE!");
      while (1) {
        digitalWrite(LED_RED, LOW);  //Inverted, due to Active LOW RGB LED!!!
        delay(500);
        digitalWrite(LED_RED, HIGH);  //Inverted, due to Active LOW RGB LED!!!
        delay(500);
      }
    }
  }
  Serial.println("Success!");
  u8g2.begin();
  u8g2.enableUTF8Print();
}

/**
checkAndEnableBME280SensorConnection():
=================================
Verifies, that BME280 sensor is available and configures it
===========================================================
- Checks, if connection to sensor is possible (eg. if sensor is connected and ready to talk), warns user
- Configures Sensor parameters
*/
void checkAndEnableBME280SensorConnection() {
  if (SENSOR_TYPE == BME_280) {
    Serial.println("Checking for BME280 Sensor on PINs A6 -> SDA, A7 -> SCL");
    Wire1.begin(A6, A7);
    if (!bme.begin(0x76, &Wire1)) {  //Try to talk to Sensor over I2C
      Serial.println("ERROR: NO BME280 SENSOR FOUND! PLEASE CONNECT PINs A6 -> SDA, A7 -> SCL AND RESTART DEVICE!");
      if (!headlessMode) {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_spleen5x8_mf);
        u8g2.drawStr(1, 9, "ERROR: COULD NOT FIND");
        u8g2.drawStr(1, 19, "BME280 SENSOR! PLEASE");
        u8g2.drawStr(1, 29, "VERIFY CONNECTIONS:");
        u8g2.drawStr(1, 39, "A6 -> SDA");
        u8g2.drawStr(1, 49, "A7 -> SCL");
        u8g2.drawStr(1, 59, "AND RESTART DEVICE!");
        u8g2.updateDisplay();
      }

      while (1) {
        digitalWrite(LED_RED, LOW);  //Inverted, due to Active LOW RGB LED!!!
        delay(500);
        digitalWrite(LED_RED, HIGH);  //Inverted, due to Active LOW RGB LED!!!
        delay(500);
      }
    }
    Serial.println("Success!");
  }
}

/**
checkAndEnableBMP280SensorConnection():
=================================
Verifies, that BMP280 sensor is available and configures it
===========================================================
- Checks, if connection to sensor is possible (eg. if sensor is connected and ready to talk), warns user
- Configures Sensor parameters
*/
void checkAndEnableBMP280SensorConnection() {
  Serial.println("Checking for BMP280 Sensor on PINs A6 -> SDA, A7 -> SCL");
  Wire1.begin(A6, A7);
  if (!bmp.begin(0x76)) {  //Try to talk to Sensor over I2C
    Serial.println("ERROR: NO BMP280 SENSOR FOUND! PLEASE CONNECT PINs A6 -> SDA, A7 -> SCL AND RESTART DEVICE!");
    if (!headlessMode) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_spleen5x8_mf);
      u8g2.drawStr(1, 9, "ERROR: COULD NOT FIND");
      u8g2.drawStr(1, 19, "BMP280 SENSOR! PLEASE");
      u8g2.drawStr(1, 29, "VERIFY CONNECTIONS:");
      u8g2.drawStr(1, 39, "A6 -> SDA");
      u8g2.drawStr(1, 49, "A7 -> SCL");
      u8g2.drawStr(1, 59, "AND RESTART DEVICE!");
      u8g2.updateDisplay();
    }

    while (1) {
      digitalWrite(LED_RED, LOW);  //Inverted, due to Active LOW RGB LED!!!
      delay(500);
      digitalWrite(LED_RED, HIGH);  //Inverted, due to Active LOW RGB LED!!!
      delay(500);
    }
  }
  Serial.println("Success!");
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

/**
checkAndEnableWifiConnection():
===============================
Verifies, all requirements for wifi connection 
are met and enables wifi connection functionality
=================================================
- Checks if Network Features are enabled
- If enabled, checks if required configuration data is set and if not, warns user
- Tries to connect to Wifi Accesspoint
*/
void checkAndEnableWifiConnection() {
  Serial.println("Checking Wifi Credentials");
  if (ENABLE_WIFI_CONNECTIVITY) {
    wifiStatus = WL_IDLE_STATUS;
    if ((WIFI_SSID == "" || WIFI_PASSWORD == "") && deviceStatus != BOOTING_AFTER_SLEEP) {
      wifiStatus = WL_NO_SHIELD;
      Serial.println("WARNING: WIFI CREDENTIALS INVALID!");
      Serial.println("Network Features won't be available!");
      if (!headlessMode) {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_spleen5x8_mf);
        u8g2.drawStr(1, 9, "WARNING: WIFI CREDENTIALS");
        u8g2.drawStr(1, 19, "INVALID!");
        u8g2.drawStr(1, 39, "Features requiring");
        u8g2.drawStr(1, 49, "network connectivity");
        u8g2.drawStr(1, 59, "won't be available!");
        u8g2.updateDisplay();
        delay(2500);
      } else {
        for (int i = 0; i < 20; i++) {
          digitalWrite(LED_RED, LOW);    //Inverted, due to Active LOW RGB LED!!!
          digitalWrite(LED_GREEN, LOW);  //Inverted, due to Active LOW RGB LED!!!
          delay(100);
          digitalWrite(LED_RED, HIGH);    //Inverted, due to Active LOW RGB LED!!!
          digitalWrite(LED_GREEN, HIGH);  //Inverted, due to Active LOW RGB LED!!!
          delay(100);
        }
      }

      return;
    }

    if (WIFI_HOSTNAME == "" && deviceStatus != BOOTING_AFTER_SLEEP) {
      Serial.println("WARNING: HOSTNAME NOT FOUND!");
      Serial.println(" -> Device will use random Hostname for Wifi Connection!");
      if (!headlessMode) {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_spleen5x8_mf);
        u8g2.drawStr(1, 9, "WARNING: NO HOSTNAME SET! ");
        u8g2.drawStr(1, 29, "Using Random Hostname");
        u8g2.drawStr(1, 39, "for Wifi Connection!");
        u8g2.updateDisplay();
        delay(2500);
      }

      for (int i = 0; i < 20; i++) {
        digitalWrite(LED_RED, LOW);    //Inverted, due to Active LOW RGB LED!!!
        digitalWrite(LED_GREEN, LOW);  //Inverted, due to Active LOW RGB LED!!!
        delay(100);
        digitalWrite(LED_RED, HIGH);    //Inverted, due to Active LOW RGB LED!!!
        digitalWrite(LED_GREEN, HIGH);  //Inverted, due to Active LOW RGB LED!!!
        delay(100);
      }
    } else {
      WiFi.setHostname(WIFI_HOSTNAME);
    }
    if (USE_STATIC_NETWORK_CONFIG) {
      Serial.println("Configuring Static Network with Client IP " + String(IP_ADDRESS) + ", Subnet " + String(SUBNET) + " and Gateway " + String(GATEWAY));
      WiFi.config(IP_ADDRESS, GATEWAY, SUBNET);
    }
    Serial.println("Success!");
    Serial.println("Trying to connect to Network '" + String(WIFI_SSID) + "' with Hostname '" + String(WiFi.getHostname()) + "'");
    wifiStatus = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

/**
checkAndEnableMqttConnection():
===============================
Verifies, all requirements for MQTT data sharing 
are met and enables MQTT data sharing functionality
============================================
- Checks if MQTT Data Sharing is enabled
- If enabled, checks if required configuration data is set and if not, warns user
- Tries to connect to MQTT Broker (although this will most likely fail because a 
  Wifi connection most likely isn't established yet)
*/
void checkAndEnableMqttConnection() {

  if (ENABLE_MQTT_DATA_SHARING) {
    Serial.println("Checking MQTT Configuration");
    mqttStatus = MQTT_DISCONNECTED;
    if ((wifiStatus == WL_DISABLED || wifiStatus == WL_NO_SHIELD) && deviceStatus != BOOTING_AFTER_SLEEP) {
      Serial.println("WARNING: MQTT UNAVAILABLE!");
      Serial.println(" -> Entering Standalone Mode. MQTT disabled because Wifi is unavaibable!");
      mqttStatus = MQTT_UNAVAILABLE;
      if (!headlessMode) {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_spleen5x8_mf);
        u8g2.drawStr(1, 9, "WARNING: MQTT UNAVAILABLE");
        u8g2.drawStr(1, 29, "MQTT is unavailable");
        u8g2.drawStr(1, 39, "because Wifi connection");
        u8g2.drawStr(1, 49, "is unavailable!");
        u8g2.updateDisplay();
        delay(2500);
      } else {
        for (int i = 0; i < 20; i++) {
          digitalWrite(LED_RED, LOW);   //Inverted, due to Active LOW RGB LED!!!
          digitalWrite(LED_BLUE, LOW);  //Inverted, due to Active LOW RGB LED!!!
          delay(100);
          digitalWrite(LED_RED, HIGH);   //Inverted, due to Active LOW RGB LED!!!
          digitalWrite(LED_BLUE, HIGH);  //Inverted, due to Active LOW RGB LED!!!
          delay(100);
        }
      }
      return;
    }
    if (MQTT_BROKER_URL == "" || MQTT_BROKER_PORT == 0 || (MQTT_AUTHENTICATION_REQUIRED && (MQTT_USERNAME == "" || MQTT_PASSWORD == ""))) {
      mqttStatus = MQTT_UNAVAILABLE;
      Serial.println("WARNING: MQTT CONFIGURATION DATA INVALID!");
      Serial.println(" -> Data Sharing won't be available!");

      if (!headlessMode) {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_spleen5x8_mf);
        u8g2.drawStr(1, 9, "WARNING: MQTT CONFIGURA-");
        u8g2.drawStr(1, 19, "TION DATA INVALID!");
        u8g2.drawStr(1, 39, "Data sharing features");
        u8g2.drawStr(1, 49, "won't be available!");
        u8g2.updateDisplay();
        delay(2500);
      } else {
        for (int i = 0; i < 20; i++) {
          digitalWrite(LED_RED, LOW);   //Inverted, due to Active LOW RGB LED!!!
          digitalWrite(LED_BLUE, LOW);  //Inverted, due to Active LOW RGB LED!!!
          delay(100);
          digitalWrite(LED_RED, HIGH);   //Inverted, due to Active LOW RGB LED!!!
          digitalWrite(LED_BLUE, HIGH);  //Inverted, due to Active LOW RGB LED!!!
          delay(100);
        }
      }
      return;
    }
    Serial.println("Success!");

    if (MQTT_AUTHENTICATION_REQUIRED) {
      mqttClient.setUsernamePassword(MQTT_USERNAME, MQTT_PASSWORD);
      Serial.println("Trying to connect to MQTT Broker '" + String(MQTT_BROKER_URL) + "' using port " + String(MQTT_BROKER_PORT) + ". Using Authentication Credentials.");
    } else {
      Serial.println("Trying to connect to MQTT Broker '" + String(MQTT_BROKER_URL) + "' using port " + String(MQTT_BROKER_PORT) + "");
    }

    mqttClient.connect(MQTT_BROKER_URL, MQTT_BROKER_PORT);
    if (SHOW_ALTITUDE_OVER_SEALEVEL) {
      Serial.println("Using MQTT Broker data from '" + String(AIR_PRESSURE_TOPIC_NAME) + "' to calibrate airpressure at sealevel");
      mqttClient.onMessage(updateAirPressureOnSealevelData);
      mqttClient.subscribe(AIR_PRESSURE_TOPIC_NAME);
    }
  }
}

/**
checkAndEnableDeepSleep():
==========================
Verifies, all requirements for deep sleep
are met and enables deep sleep functionality
============================================
- Checks if Deep Sleep is enabled
- If enabled, checks if atleast one wake up source is activated and if not, warns user
- Enables deep sleep wake up from the different sources
*/
void checkAndEnableDeepSleep() {
  if (ENABLE_DEEP_SLEEP) {
    if (DEEP_SLEEP_WAKE_UP_FROM_BUTTON || DEEP_SLEEP_WAKE_UP_FROM_TIMER) {
      updateDeepSleepStatus(DS_ENABLED);
      Serial.println("Deep Sleep activated!");
      Serial.println("Deep Sleep Timer set to " + String(DEEP_SLEEP_TIMER) + "seconds");
      if (DEEP_SLEEP_WAKE_UP_FROM_BUTTON) {
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_2, 1);
        Serial.println("Enabled 'Wake up by Pushbutton'");
      }

      if (DEEP_SLEEP_WAKE_UP_FROM_TIMER) {
        esp_sleep_enable_timer_wakeup(DEEP_SLEEP_WAKE_UP_TIMER * 1000000);
        Serial.println("Enabled 'Wake up by Timer'");
      }
      updateDeepSleepStatus(DS_ACTIVE);
    } else {
      updateDeepSleepStatus(DS_UNAVAILABLE);
      if (!headlessMode) {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_spleen5x8_mf);
        u8g2.drawStr(1, 9, "WARNING: DEEP SLEEP");
        u8g2.drawStr(1, 19, "CONFIGURATION INVALID!");
        u8g2.drawStr(1, 39, "Deep Sleep is enabled but");
        u8g2.drawStr(1, 49, "no trigger is configured,");
        u8g2.drawStr(1, 59, "thus it won't activate!");
        u8g2.updateDisplay();
        delay(2500);
      } else {
        for (int i = 0; i < 20; i++) {
          digitalWrite(LED_BLUE, LOW);   //Inverted, due to Active LOW RGB LED!!!
          digitalWrite(LED_GREEN, LOW);  //Inverted, due to Active LOW RGB LED!!!
          delay(100);
          digitalWrite(LED_BLUE, HIGH);   //Inverted, due to Active LOW RGB LED!!!
          digitalWrite(LED_GREEN, HIGH);  //Inverted, due to Active LOW RGB LED!!!
          delay(100);
        }
      }
    }
  }
}

/**
updateDeviceStatus():
===================
Updates devices status and sends it to configured MQTT Brokertz6 56 b
==============================================
- Updates MQTT Var Status
- Redraws Statusbar on change using drawStatusbar()
- Sends Data to configured MQTT Broker
*/

void updateDeviceStatus(device_status status, bool updateStatusOnMqttBroker = false) {
  int oldStatus = deviceStatus;
  if (status != oldStatus) {
    Serial.println("Device status change: " + String(oldStatus) + " -> " + String(status));
    deviceStatus = status;
    if (updateStatusOnMqttBroker && mqttStatus == MQTT_CONNECTED) {
      Serial.println("Sending Device Status");
      mqttStatus = MQTT_SENDING;
      if (!headlessMode) {
        drawStatusbar();
      } else {
        digitalWrite(LED_GREEN, LOW);
      }
      mqttClient.beginMessage(String(WIFI_HOSTNAME) + "/Device/Id");
      mqttClient.print(DEVICE_ID);
      mqttClient.endMessage();
      mqttClient.beginMessage(String(WIFI_HOSTNAME) + "/Device/Status");
      mqttClient.print(deviceStatus);
      mqttClient.endMessage();
      mqttClient.beginMessage(String(WIFI_HOSTNAME) + "/Device/SensorType");
      mqttClient.print(String(SENSOR_TYPE));
      mqttClient.endMessage();

      mqttStatus = MQTT_CONNECTED;
      if (!headlessMode) {
        drawStatusbar();
      } else {
        digitalWrite(LED_GREEN, HIGH);
      }
    }
  }
}

/**
updateMqttStatus():
===================
Updates MQTT Status and handles re-connection
=============================================
- Updates MQTT Var
- Redraws Statusbar on change using drawStatusbar()
- If connection is lost, tries to re-connect once per function call
  as long as connection isn't re-established
*/
void updateMqttStatus() {
  if (mqttStatus != MQTT_UNAVAILABLE) {
    int oldMqttStatus = mqttStatus;
    mqttStatus = mqttClient.connected();
    if (mqttStatus != oldMqttStatus) {
      updateDeviceStatus(AWAKE, true);
      Serial.println("MQTT status change: " + String(oldMqttStatus) + " -> " + String(mqttStatus));
      if (!headlessMode) {
        drawStatusbar();
      } else {
        if ((oldMqttStatus == MQTT_DISCONNECTED) && mqttStatus == MQTT_CONNECTED) {  //now connected
          digitalWrite(LED_RED, LOW);
          digitalWrite(LED_BLUE, LOW);
          delay(500);
          digitalWrite(LED_RED, HIGH);
          digitalWrite(LED_BLUE, HIGH);
          digitalWrite(LED_GREEN, LOW);
          delay(500);
          digitalWrite(LED_GREEN, HIGH);
        } else if (oldMqttStatus == MQTT_CONNECTED && mqttStatus == MQTT_DISCONNECTED) {  //now disconnected
          digitalWrite(LED_RED, LOW);
          digitalWrite(LED_BLUE, LOW);
          delay(500);
          digitalWrite(LED_BLUE, HIGH);
          delay(500);
          digitalWrite(LED_RED, HIGH);
        }
      }
    }
    if (mqttStatus == MQTT_DISCONNECTED) {
      if (mqttConnectionRetryCycleCount > 0 && mqttConnectionRetryCycleCount % MQTT_CONNECTION_RETRY_TIMEOUT == 0) {
        mqttClient.connect(MQTT_BROKER_URL, MQTT_BROKER_PORT);
        mqttConnectionRetryCycleCount = 0;
      } else {
        mqttConnectionRetryCycleCount++;
      }


      if (SHOW_ALTITUDE_OVER_SEALEVEL) {
        Serial.println("Using MQTT Broker data from '" + String(AIR_PRESSURE_TOPIC_NAME) + "' to calibrate airpressure at sealevel");
        mqttClient.onMessage(updateAirPressureOnSealevelData);
        Serial.println(AIR_PRESSURE_TOPIC_NAME);
        mqttClient.subscribe(AIR_PRESSURE_TOPIC_NAME);
      }
    }
  }
}

/**
updateWifiStatus():
===================
Updates Wifi Status and handles re-connection
=============================================
- Updates WiFi Var
- Redraws Statusbar on change using drawStatusbar()
- If connection is lost, tries to re-connect once per function call
  as long as connection isn't re-established
*/
void updateWifiStatus() {
  if (wifiStatus != WL_NO_SHIELD) {
    int oldWifiStatus = wifiStatus;

    wifiStatus = WiFi.status();
    if (wifiStatus != oldWifiStatus) {
      updateDeviceStatus(AWAKE, true);
      Serial.println("Wifi status change: " + String(oldWifiStatus) + " -> " + String(wifiStatus));
      if (!headlessMode) {
        drawStatusbar();
      } else {
        if (oldWifiStatus != WL_CONNECTED && wifiStatus == WL_CONNECTED) {  //now connected
          digitalWrite(LED_RED, LOW);
          digitalWrite(LED_GREEN, LOW);
          delay(500);
          digitalWrite(LED_RED, HIGH);
          delay(500);
          digitalWrite(LED_GREEN, HIGH);
        } else if (oldWifiStatus == WL_CONNECTED && wifiStatus != WL_CONNECTED) {  //now disconnected
          digitalWrite(LED_RED, LOW);
          digitalWrite(LED_GREEN, LOW);
          delay(500);
          digitalWrite(LED_GREEN, HIGH);
          delay(500);
          digitalWrite(LED_RED, HIGH);
        }
      }
    }
    if (wifiStatus != WL_CONNECTED) {
      if (wifiConnectionRetryCycleCount > 0 && wifiConnectionRetryCycleCount % WIFI_CONNECTION_RETRY_TIMEOUT == 0) {
        Serial.println("Trying to connect to Network '" + String(WIFI_SSID) + "' with Hostname '" + String(WiFi.getHostname()) + "'");
        wifiStatus = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        Serial.println("wifiStatus: " + String(wifiStatus));
        wifiConnectionRetryCycleCount = 0;

      } else {
        wifiConnectionRetryCycleCount++;
      }
    }
  }
}

/**
updateDeepSleepStatus():
===================
Updates Wifi Status and handles re-connection
=============================================
- Updates WiFi Var
- Redraws Statusbar on change using drawStatusbar()
- If connection is lost, tries to re-connect once per function call
  as long as connection isn't re-established
*/

void updateDeepSleepStatus(ds_status dsStatus) {
  int oldDsStatus = deepSleepStatus;
  if (dsStatus != oldDsStatus) {
    deepSleepStatus = dsStatus;
    Serial.println("Deep Sleep Status Change: " + String(oldDsStatus) + " -> " + String(dsStatus));
  }
}

/**
fetchAirPressureAtSealevel():
===================
Fetches current airpressure at sealevel from mqtt broker
==============================================
- Checks if mqtt connections is established
- fetches mqtt data
*/
void updateAirPressureOnSealevelData(int messageSize) {
  mqttStatus = MQTT_RECEIVING;
  if (!headlessMode) {
    drawStatusbar();
  } else {
    delay(50);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, LOW);
  }
  // use the Stream interface to print the contents
  char buffer[messageSize + 1];
  if (mqttClient.messageTopic() == AIR_PRESSURE_TOPIC_NAME) {
    for (int i = 0; i < messageSize; i++) {
      buffer[i] = mqttClient.read();
    }
    buffer[messageSize] = '\0';
    airPressureAtSealevel = atof(buffer);
    Serial.println("Received airpressure data from MQTT Broker: " + String(airPressureAtSealevel));
  }
  mqttStatus = MQTT_CONNECTED;
  if (!headlessMode) {
    drawStatusbar();
  } else {
    delay(25);  //For better visibility of the "data Sending" led
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    delay(50);
  }
}

/**
gatherSensorBME280Data():
=========================
Gathers Data form BME280 Sensor
===============================
- Gathers Sensor Data (yep, that's really all this function does lol)
*/
Weather gatherBME280SensorData() {
  Serial.println("gathering Sensor Data");
  Weather w;
  w.temperature = bme.readTemperature();
  w.airPressure = bme.readPressure() / 100;
  w.humidity = bme.readHumidity();
  Serial.println("Temp: " + String(w.temperature) + ", AirPress: " + String(w.airPressure) + ", Hum: " + String(w.humidity));
  return w;
}

/**
gatherSensorBMP280Data():
=========================
Gathers Data from BMP280 Sensor
===============================
- Gathers Sensor Data (yep, that's really all this function does lol)
*/
Weather gatherBMP280SensorData() {
  Serial.println("gathering Sensor Data");
  Weather w;
  w.temperature = bmp.readTemperature();
  w.airPressure = bmp.readPressure() / 100;
  if (SHOW_ALTITUDE_OVER_SEALEVEL) {
    w.altitude = bmp.readAltitude(airPressureAtSealevel);
    Serial.println("Temp: " + String(w.temperature) + ", AirPress: " + String(w.airPressure) + ", Alt: " + String(w.altitude) + " (using Reference Value: " + String(airPressureAtSealevel) + ")");
  } else {
    Serial.println("Temp: " + String(w.temperature) + ", AirPress: " + String(w.airPressure));
  }

  return w;
}

/**
sendSensorDataOverMqtt():
=========================
Gathers Sensor Data and sends it to configured MQTT Broker
==========================================================
- Gathers Sensor Data using gatherSensorData()
- Sends Data to configured MQTT Broker
*/
void sendBME280SensorDataOverMqtt(Weather weatherData) {
  Serial.println("Sending MQTT Data");
  mqttStatus = MQTT_SENDING;
  if (!headlessMode) {
    drawStatusbar();
  } else {
    delay(50);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, LOW);
  }
  mqttClient.beginMessage(String(WIFI_HOSTNAME) + "/WeatherData/Temperature");
  mqttClient.print(weatherData.temperature);
  mqttClient.endMessage();
  mqttClient.beginMessage(String(WIFI_HOSTNAME) + "/WeatherData/AirPressure");
  mqttClient.print(weatherData.airPressure);
  mqttClient.endMessage();
  mqttClient.beginMessage(String(WIFI_HOSTNAME) + "/WeatherData/Humidity");
  mqttClient.print(weatherData.humidity);
  mqttClient.endMessage();

  mqttStatus = MQTT_CONNECTED;
  if (!headlessMode) {
    drawStatusbar();
  } else {
    delay(25);  //For better visibility of the "data Sending" led
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    delay(50);
  }
}

/**
sendSensorDataOverMqtt():
=========================
Gathers Sensor Data and sends it to configured MQTT Broker
==========================================================
- Gathers Sensor Data using gatherSensorData()
- Sends Data to configured MQTT Broker
*/
void sendBMP280SensorDataOverMqtt(Weather weatherData) {
  Serial.println("Sending MQTT Data");
  mqttStatus = MQTT_SENDING;
  if (!headlessMode) {
    drawStatusbar();
  } else {
    delay(50);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, LOW);
  }
  mqttClient.beginMessage(String(WIFI_HOSTNAME) + "/WeatherData/Temperature");
  mqttClient.print(weatherData.temperature);
  mqttClient.endMessage();
  mqttClient.beginMessage(String(WIFI_HOSTNAME) + "/WeatherData/AirPressure");
  mqttClient.print(weatherData.airPressure);
  mqttClient.endMessage();
  if (SHOW_ALTITUDE_OVER_SEALEVEL) {
    mqttClient.beginMessage(String(WIFI_HOSTNAME) + "/WeatherData/Altitude");
    mqttClient.print(weatherData.altitude);
    mqttClient.endMessage();
  }

  mqttStatus = MQTT_CONNECTED;
  if (!headlessMode) {
    drawStatusbar();
  } else {
    delay(25);  //For better visibility of the "data Sending" led
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    delay(50);
  }
}

/**
activateDeepSleep():
====================
Puts the station into deep sleep
================================
- Deactivates Heartbeat Timer Interrupt
- Gathers and sends sensor data to MQTT Broker one last time
- Clears displas and Serial Buffer
- Starts processor deep sleep
*/
void activateDeepSleep() {
  timerDetachInterrupt(heartbeat);
  Serial.println("Going to sleep...! Waking up again in " + String(DEEP_SLEEP_WAKE_UP_TIMER) + " seconds");
  if (mqttStatus == MQTT_CONNECTED) {
    if (SENSOR_TYPE == BME_280) {
      sendBME280SensorDataOverMqtt(gatherBME280SensorData());
    } else if (SENSOR_TYPE == BMP_280) {
      sendBMP280SensorDataOverMqtt(gatherBMP280SensorData());
    }
  }
  updateDeviceStatus(SLEEPING, true);

  if (!headlessMode) {
    u8g2.clearDisplay();
    u8g2.setFont(u8g2_font_spleen6x12_mf);
    u8g2.setCursor(15, 9);
    u8g2.print("Going to Sleep now!");
    if (DEEP_SLEEP_WAKE_UP_FROM_TIMER) {
      u8g2.setCursor(15, 34);
      u8g2.print("Waking up in " + String(DEEP_SLEEP_WAKE_UP_TIMER) + "s");
    } else {
      u8g2.ty += 15;
    }
    if (DEEP_SLEEP_WAKE_UP_FROM_BUTTON) {
      u8g2.setCursor(15, u8g2.ty + 15);
      u8g2.print("Waking up on");
      u8g2.setCursor(15, u8g2.ty + 10);
      u8g2.print("button press");
    }
    u8g2.sendBuffer();
    delay(3500);
    u8g2.clearDisplay();
  } else {
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_GREEN, LOW);
    delay(1000);
    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(LED_GREEN, HIGH);
  }
  Serial.flush();

  esp_deep_sleep_start();
}

/**
drawStatusbar():
================
Draws the Statusbar onto the display
==========================================================
- Draws the Statusbar onto the display
*/
void drawStatusbar() {
  u8g2.drawVLine(0, 0, 10);
  u8g2.setFont(u8g2_font_spleen5x8_mf);
  int strWidth = 3 + u8g2.drawStr(3, 8, deviceIdString) + 1;
  u8g2.drawVLine(strWidth, 0, 10);
  u8g2.drawVLine(78, 0, 10);
  if (deepSleepStatus == DS_UNAVAILABLE) {
    u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
    u8g2.drawGlyph(87, 9, 0x0047);
  } else if (deepSleepStatus == DS_ENABLED || deepSleepStatus == DS_ACTIVE) {
    if (DEEP_SLEEP_WAKE_UP_FROM_TIMER) {
      u8g2.setFont(u8g2_font_open_iconic_app_1x_t);
      u8g2.drawGlyph(81, 9, 0x0048);
    }
    if (DEEP_SLEEP_WAKE_UP_FROM_BUTTON) {
      u8g2.setFont(u8g2_font_open_iconic_other_1x_t);
      u8g2.drawGlyph(91, 9, 0x0047);
    }
  }

  u8g2.drawVLine(101, 0, 10);
  switch (mqttStatus) {
    case MQTT_DISABLED:
      break;
    case MQTT_UNAVAILABLE:
      if (wifiStatus != WL_DISABLED && wifiStatus != WL_NO_SHIELD) {
        u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
        u8g2.drawGlyph(105, 9, 0x0047);
      }
      break;
    case MQTT_DISCONNECTED:
      u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
      u8g2.drawGlyph(104, 9, 0x0045);
      break;
    case MQTT_CONNECTED:
      u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
      u8g2.drawGlyph(104, 9, 0x004f);
      break;
    case MQTT_SENDING:
      u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
      u8g2.drawGlyph(104, 9, 0x0043);
      break;
    case MQTT_RECEIVING:
      u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
      u8g2.drawGlyph(104, 9, 0x004c);
      break;
  }
  u8g2.drawVLine(114, 0, 10);
  switch (wifiStatus) {
    case WL_DISABLED:
      break;
    case WL_NO_SHIELD:
      u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
      u8g2.drawGlyph(118, 9, 0x0047);
      break;
    case WL_CONNECTED:
      u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
      u8g2.drawGlyph(117, 9, 0x0051);
      break;
    default:
      u8g2.setFont(u8g2_font_open_iconic_www_1x_t);
      u8g2.drawGlyph(117, 9, 0x004a);
      break;
  }
  u8g2.drawVLine(127, 0, 10);
  u8g2.drawHLine(0, 10, DISPLAY_WIDTH);
  u8g2.updateDisplay();
}

/**
getAndDrawBME280Data():
=================
Gathers Sensor Data and draws it onto the display
==========================================================
- Gathers Sensor Data using gatherSensorData()
- Draws the data onto the display
*/
void getAndDrawBME280Data() {
  Weather weatherData = gatherBME280SensorData();
  u8g2.setFont(u8g2_font_spleen6x12_mf);
  u8g2.setCursor(1, 27);
  u8g2.print("Temp.:    " + ((weatherData.temperature != -999 && !isnan(weatherData.temperature)) ? (String(weatherData.temperature)) : ("--.-")) + " °C");

  u8g2.setCursor(1, 44);
  u8g2.print("Pressure: " + ((weatherData.airPressure != -999 && !isnan(weatherData.airPressure)) ? (String(weatherData.airPressure)) : ("--.-")) + " hPa");

  u8g2.setCursor(1, 61);
  u8g2.print("Humidity: " + ((weatherData.humidity != -999 && !isnan(weatherData.humidity)) ? (String(weatherData.humidity)) : ("--.-")) + " %");
  u8g2.updateDisplay();
}

/**
getAndDrawBMP280Data():
=================
Gathers Sensor Data and draws it onto the display
==========================================================
- Gathers Sensor Data using gatherSensorData()
- Draws the data onto the display
*/
void getAndDrawBMP280Data() {
  Weather weatherData = gatherBMP280SensorData();
  u8g2.setFont(u8g2_font_spleen6x12_mf);
  u8g2.setCursor(1, 27);
  u8g2.print("Temp.:    " + ((weatherData.temperature != -999 && !isnan(weatherData.temperature)) ? (String(weatherData.temperature)) : ("--.-")) + " °C");

  u8g2.setCursor(1, 44);
  u8g2.print("Pressure: " + ((weatherData.airPressure != -999 && !isnan(weatherData.airPressure)) ? (String(weatherData.airPressure)) : ("--.-")) + " hPa");
  if (SHOW_ALTITUDE_OVER_SEALEVEL) {
    u8g2.setCursor(1, 61);
    u8g2.print("Altitude: " + ((weatherData.altitude != -999 && !isnan(weatherData.altitude)) ? (String(weatherData.altitude)) : ("--.-")) + " m^NN");
  }
  u8g2.updateDisplay();
}

/**
Setup Loop
*/
void setup() {
  Serial.begin(115200);
  delay(2500);  //warten, damit der Serielle Monitor auch funktioniert :D
  Serial.println("Device Booting...");
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  sprintf(deviceIdString, "Id: #%d", DEVICE_ID);
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  Serial.println("Wakeup reason: " + String(wakeup_reason));
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0 || wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    updateDeviceStatus(BOOTING_AFTER_SLEEP, true);
  }

  checkAndEnableDisplayConnection();

  if (!headlessMode && deviceStatus != BOOTING_AFTER_SLEEP) {
    drawSplashScreen();
  }
  if (SENSOR_TYPE == BME_280) {
    checkAndEnableBME280SensorConnection();
  } else if (SENSOR_TYPE == BMP_280) {
    checkAndEnableBMP280SensorConnection();
  }

  checkAndEnableWifiConnection();
  checkAndEnableMqttConnection();
  checkAndEnableDeepSleep();
  if (!headlessMode) {
    u8g2.clearDisplay();
    drawStatusbar();
    if (SENSOR_TYPE == BME_280) {
      getAndDrawBME280Data();
    } else if (SENSOR_TYPE == BMP_280) {
      getAndDrawBMP280Data();
    }
  }

  //HeartBeat Timer
  heartbeat = timerBegin(0, 80, true);
  timerAttachInterrupt(heartbeat, &onHeartbeat, true);
  timerAlarmWrite(heartbeat, DEVICE_HEARTBEAT_CYCLE_TIME * 1000000, true);
  timerAlarmEnable(heartbeat);
  updateDeviceStatus(AWAKE, true);
}

/**
Main Loop
*/
void loop() {
  if (mqttStatus == MQTT_CONNECTED) { mqttClient.poll(); }
  if (onHeartbeatInterrupt) {
    digitalWrite(LED_BUILTIN, HIGH);
    updateDeviceStatus(ACTIVE);
    //Serial.println("Heart beating... DeepSleepCounter: " + String(deepSleepCounter) + ", mqttSendCounter: " + String(mqttSendCounter));
    onHeartbeatInterrupt = false;

    if (deepSleepStatus == DS_ACTIVE) { deepSleepCounter++; }

    updateMqttStatus();
    updateWifiStatus();
    if (!headlessMode) {
      if (SENSOR_TYPE == BME_280) {
        getAndDrawBME280Data();
      } else if (SENSOR_TYPE == BMP_280) {
        getAndDrawBMP280Data();
      }
    }
    updateDeviceStatus(AWAKE);
    digitalWrite(LED_BUILTIN, LOW);
    sensorReadoutTimeout++;
  }

  if (sensorReadoutTimeout == SENSOR_READOUT_TIMEOUT) {
    if (SENSOR_TYPE == BME_280) {
      Weather tempData = gatherBME280SensorData();
      weatherDataAccumulator.temperature += tempData.temperature;
      weatherDataAccumulator.humidity += tempData.humidity;
      weatherDataAccumulator.airPressure += tempData.airPressure;
    } else if (SENSOR_TYPE == BMP_280) {
      Weather tempData = gatherBMP280SensorData();
      weatherDataAccumulator.temperature += tempData.temperature;
      weatherDataAccumulator.altitude += tempData.altitude;
      weatherDataAccumulator.airPressure += tempData.airPressure;
    }
    sensorReadoutTimeout = 0;
    mqttSendCounter++;
  }
  if (mqttSendCounter == MQTT_SEND_COUNTER) {
    updateDeviceStatus(ACTIVE);
    mqttSendCounter = 0;
    if (mqttStatus == MQTT_CONNECTED) {
      if (SENSOR_TYPE == BME_280) {
        weatherDataAccumulator.temperature = weatherDataAccumulator.temperature / 12;
        weatherDataAccumulator.humidity = weatherDataAccumulator.humidity / 12;
        weatherDataAccumulator.airPressure = weatherDataAccumulator.airPressure / 12;
        sendBME280SensorDataOverMqtt(weatherDataAccumulator);
      } else if (SENSOR_TYPE == BMP_280) {
        weatherDataAccumulator.temperature = weatherDataAccumulator.temperature / 12;
        weatherDataAccumulator.altitude = weatherDataAccumulator.altitude / 12;
        weatherDataAccumulator.airPressure = weatherDataAccumulator.airPressure / 12;
        sendBMP280SensorDataOverMqtt(weatherDataAccumulator);
      }
    }
    updateDeviceStatus(AWAKE);
  }

  if (deepSleepStatus == DS_ACTIVE && deepSleepCounter == DEEP_SLEEP_TIMER) {
    updateDeviceStatus(ACTIVE);
    activateDeepSleep();
  }
}
