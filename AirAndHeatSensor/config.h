/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME    "hannesjoh"
#define IO_KEY         "97f18d86c39043398617bd2cbf2cea15"

/******************************* WIFI **************************************/

// the AdafruitIO_WiFi client will work with the following boards:
//   - HUZZAH ESP8266 Breakout -> https://www.adafruit.com/products/2471
//   - Feather HUZZAH ESP8266 -> https://www.adafruit.com/products/2821
//   - Feather HUZZAH ESP32 -> https://www.adafruit.com/product/3405
//   - Feather M0 WiFi -> https://www.adafruit.com/products/3010
//   - Feather WICED -> https://www.adafruit.com/products/3056

#define WIFI_SSID       "iot-research"
#define WIFI_PASS       "yNUtn6uwGa7PFrLdv3hgqDRpf89MsTHb"

// comment out the following two lines if you are using fona or ethernet
#include "AdafruitIO_WiFi.h"
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* mqttServer = "m20.cloudmqtt.com";
const int mqttPort = 10595;
const char* mqttUser = "wrdjboxc";
const char* mqttPassword = "zvXfFFo6dqtt";
