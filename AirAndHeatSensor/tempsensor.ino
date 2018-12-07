// Adafruit IO Temperature & Humidity Example
// Tutorial Link: https://learn.adafruit.com/adafruit-io-basics-temperature-and-humidity
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Written by Todd Treece for Adafruit Industries
// Copyright (c) 2016-2017 Adafruit Industries
// Licensed under the MIT license.
//
// All text above must be included in any redistribution.

/************************** Configuration ***********************************/

// edit the config.h tab and enter your Adafruit IO credentials
// and any additional configuration needed for WiFi, cellular,
// or ethernet clients.
#include "config.h"

/************************ Example Starts Here *******************************/
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "Adafruit_CCS811.h"

// pin connected to DH22 data line
#define DATA_PIN 2

// create DHT22 instance
DHT_Unified dht(DATA_PIN, DHT22);

// set up the 'temperature' and 'humidity' feeds
AdafruitIO_Feed *temperature = io.feed("temperature");
AdafruitIO_Feed *humidity = io.feed("humidity");

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_CCS811 ccs;

void callback(char* topic, byte* payload, unsigned int length) {
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
 
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }

  Serial.println();
  Serial.println("-----------------------"); 
}

void setup() {
  // start the serial connection
  Serial.begin(115200);

	WiFi.begin(WIFI_SSID, WIFI_PASS);
	
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print("Connecting to WiFi..");
	}
	Serial.println("Connected to the WiFi network");
 
  // wait for serial monitor to open
  while(! Serial);

  // initialize dht22
  dht.begin();

  // connect to io.adafruit.com
  Serial.print("Connecting to Adafruit IO");
  io.connect();

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());

	client.setServer(mqttServer, mqttPort);
	client.setCallback(callback);

	while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 
    if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
	}
	client.publish("esp/test", "Hello from ESP8266");
	client.subscribe("esp/test");

	if(!ccs.begin()){
    Serial.println("Failed to start air sensor! Please check your wiring.");
    while(1);
  }

  //calibrate temperature sensor
  while(!ccs.available());
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);
}

void loop() {
	client.loop();
  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();

  sensors_event_t event;
  dht.temperature().getEvent(&event);

  float celsius = event.temperature;

  Serial.print("celsius: ");
  Serial.print(celsius);
  Serial.println("C");

  // save celsius to Adafruit IO
  temperature->save(celsius);

  dht.humidity().getEvent(&event);

  Serial.print("humidity: ");
  Serial.print(event.relative_humidity);
  Serial.println("%");

  // save humidity to Adafruit IO
  humidity->save(event.relative_humidity);

	char temp [5];
	dtostrf(celsius, 4, 1, temp);
	client.publish("esp/test", temp);

	int rawLevel = analogRead(A0);

	// the 10kΩ/47kΩ voltage divider reduces the voltage, so the ADC Pin can handle it
	// According to Wolfram Alpha, this results in the following values:
	// 10kΩ/(47kΩ+10kΩ)*  5v = 0.8772v
	// 10kΩ/(47kΩ+10kΩ)*3.7v = 0.649v
	// 10kΩ/(47kΩ+10kΩ)*3.1v = 0.544
	// * i asumed 3.1v as minimum voltage => see LiPO discharge diagrams
	// the actual minimum i've seen was 467, which would be 2.7V immediately before automatic cutoff
	// a measurement on the LiPo Pins directly resulted in >3.0V, so thats good to know, but no danger to the battery.

	// convert battery level to percent
	int level = map(rawLevel, 500, 701, 0, 100);

	// i'd like to report back the real voltage, so apply some math to get it back
	// 1. convert the ADC level to a float
	// 2. divide by (R2[1] / R1 + R2)
	// [1] the dot is a trick to handle it as float
	float realVoltage = (float)rawLevel / 1000 / (10000. / (47000 + 10000));
	
	// build a nice string to send to influxdb or whatever you like
	char dataLine[64];
	// sprintf has no support for floats, but will be added later, so we need a String() for now
	sprintf(dataLine, "voltage percent=%d,adc=%d,real=%s,charging=%d\n",
			level < 150 ? level : 100, // cap level to 100%, just for graphing, i don't want to see your lab, when the battery actually gets to that level
			rawLevel,
			String(realVoltage, 3).c_str(),
			rawLevel > 800 ? 1 : 0 // USB is connected if the reading is ~870, as the voltage will be 5V, so we assume it's charging
	);
	
	Serial.print(dataLine);
	client.publish("esp/test", dataLine);

	if(ccs.available()){
    float temp = ccs.calculateTemperature();
    if(!ccs.readData()){
      Serial.print("CO2: ");
      Serial.print(ccs.geteCO2());
      Serial.print("ppm, TVOC: ");
      Serial.print(ccs.getTVOC());
      Serial.print("ppb   Temp:");
      Serial.println(temp);
    }
    else{
      Serial.println("ERROR!");
      while(1);
    }
  }

  // wait 5 seconds (5000 milliseconds == 5 seconds)
  // delay(5000);
  // put in 60 sec deepsleep
  ESP.deepSleep(60e6);
}
// CO2: 434ppm, TVOC: 5ppb   Temp:25.00