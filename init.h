/*
Weather Station / init.h
======================================
  This is the initialization file for the
  weather station.
  Relevant definitions for sketch initialization
  are defined here instead of in the main file to 
  reduce clutter
  LEAVE THEM ALONE!!!

Requirements
============
  Required Hardware Revision: >= 1.2
  Required Software Version: >= 2.8.1
*/

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
Adafruit_BME280 bme;
Adafruit_BMP280 bmp(&Wire1);
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

hw_timer_t *heartbeat = NULL;
int deepSleepCounter, mqttSendCounter, wifiConnectionRetryCycleCount, mqttConnectionRetryCycleCount, sensorReadoutTimeout = 0;
bool headlessMode = false;

bool onHeartbeatInterrupt = false;  //true, damit am Anfang einmalig Daten gesendet werden
int wifiStatus = WL_DISABLED;
int mqttStatus = MQTT_DISABLED;
int deepSleepStatus = DS_DISABLED;
float airPressureAtSealevel = -999;
device_status deviceStatus = BOOTING;
Weather weatherDataAccumulator;
char deviceIdString[50];
