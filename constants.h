/*
Weather Station / constatns.h
======================================
  This is the constants file for the
  weather station.
  Relevant constants are defined here
  instead of in the main file to 
  reduce clutter
  LEAVE THEM ALONE!!!

Requirements
============
  Required Hardware Revision: >= 1.2
  Required Software Version: >= 2.8.1
*/



//DISPLAY CONFIGURATION:
const int DISPLAY_WIDTH = 128;
const int DISPLAY_HEIGHT = 64;
const int CONTENT_ZONE_UPPER_LIMIT = 16;

//SUPPORTED SENSOR TYPES
typedef enum {
  BME_280 = 0,
  BMP_280 = 1,
} sensor_type;

//WIFI STATUS VALUES:
typedef enum {
  WL_DISABLED = 256,
} wl_status_t_ext;


//MQTT STATUS VALUES:
typedef enum {
  MQTT_DISABLED = 256,
  MQTT_UNAVAILABLE = 255,
  MQTT_DISCONNECTED = 0,
  MQTT_CONNECTED = 1,
  MQTT_SENDING = 2,
  MQTT_RECEIVING = 3
} mqtt_status;

//DEVICE STATUS VALUES:
typedef enum {
  UNKNOWN = 0,
  SLEEPING = 1,
  BOOTING = 2,
  BOOTING_AFTER_SLEEP = 3,
  AWAKE = 4,
  ACTIVE = 5,
}
device_status;

typedef enum {
  DS_DISABLED = 256,
  DS_UNAVAILABLE = 255,
  DS_ENABLED = 0,
  DS_ACTIVE = 1
} ds_status;

struct Weather {
  double temperature;
  double airPressure;
  double altitude;
  double humidity;
};

#define BUTTON_PIN_BITMASK 0x200000000  // 2^33 in hex