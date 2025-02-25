/*
Weather Station / station_config.h
======================================
  This is the configuration file for the
  weather station. 
  Configuration options for provisioning
  the weather station can be chosen here.

Requirements
============
  Required Hardware Revision: >= 1.2
  Required Software Version: >= 2.8.0
*/

const int DEVICE_ID = 2;
const int DEVICE_HEARTBEAT_CYCLE_TIME = 1;        //Base Cycle Time in seconds for all Timed Operations (NORMALLY, A CHANGE ISN'T NEEDED! CHANGE AT YOUR OWN RISK!)
const sensor_type SENSOR_TYPE = BMP_280;          //Configures Sensor Type for Measurements. Can either be 'BME_280' or 'BMP_280'

const int SHOW_ALTITUDE_OVER_SEALEVEL = true;     //Only available if Wifi and MQTT are enabled and the MQTT Broker publishes AirPressure Data oder NN
const char AIR_PRESSURE_TOPIC_NAME[] = "broker/weatherData/airPressureAtSealevel"; //The MQTT Topic, the device can fetch the airpressure at sealavel from. (Note: Message Retention should be enabled on publishing side)

const int SENSOR_READOUT_TIMEOUT = 5;             //time in seconds, after which new measurements are collected (relative to DEVICE_HEARTBEAT_CYCLE_TIME);

const bool ENABLE_WIFI_CONNECTIVITY = true;       //enable Wifi Connectivity (Note: Enabling this is required for using "MQTT data sharing" feature)
const char *WIFI_SSID = "WIFI_SSID";
const char *WIFI_PASSWORD = "SECURE_PASSWORD";
const char *WIFI_HOSTNAME = "WeatherStation";     //EXAMPLE: WEATHER_STATION_#1 (must be unique on the network. Otherwise data sharing might be break on broker side)

const int WIFI_CONNECTION_RETRY_TIMEOUT = 2;      //time in seconds (multiplied by DEVICE_HEARTBEAT_CYCLE_TIME) after which a wifi reconnect is tried (might be needed for slow networks)

const bool ENABLE_MQTT_DATA_SHARING = true;       //enable MQTT data sharing
const char *MQTT_BROKER_URL = "0.0.0.0";          //"192.168.188.156";  //EXAMPLE: 192.168.137.1
const int MQTT_BROKER_PORT = 1883;                //EXAMPLE: 1883
const bool MQTT_AUTHENTICATION_REQUIRED = false;  //EXAMPLE: true
const char *MQTT_USERNAME = "";                   //EXAMPLE: example
const char *MQTT_PASSWORD = "";                   //EXAMPLE: example1
const int MQTT_CONNECTION_RETRY_TIMEOUT = 2;      //time in seconds (multiplied by DEVICE_HEARTBEAT_CYCLE_TIME) after which a mqtt reconnect is tried (might be needed for slow mqtt brokers)
const int MQTT_SEND_COUNTER = 12;                 //counter, after which the accumulated data will be sent! NOTE: The Device will publish data one last time before going to sleep aswell as sending a 'DEVICE_STATUS' update

const bool ENABLE_HEADLESS_MODE = true;           //Boot into headless mode if no display is connected?

const bool ENABLE_DEEP_SLEEP = true;               //Enable Deep Sleep Option, where the station requires only a few mA.
const int DEEP_SLEEP_TIMER = 60;                   //time in seconds (multiplied by DEVICE_HEARTBEAT_CYCLE_TIME) until the station goes to sleep
const bool DEEP_SLEEP_WAKE_UP_FROM_BUTTON = true;  //Enable waking up sthe tation by pressing the "wake up" button?
const bool DEEP_SLEEP_WAKE_UP_FROM_TIMER = true;   //Enable the station to wake up automatically to send data! NOTE: While you CAN enable this feature even if Wifi and MQTT are disabled, it doesn't really make any sense!
const int DEEP_SLEEP_WAKE_UP_TIMER = 10;           //time in seconds after which the station wakes up to send data after going to sleep