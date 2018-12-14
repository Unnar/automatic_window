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
AdafruitIO_Feed *battery = io.feed("battery");
AdafruitIO_Feed *IOco2 = io.feed("co2");
AdafruitIO_Feed *IOtvoc = io.feed("tvoc");

Adafruit_CCS811 ccs;

// Required for LIGHT_SLEEP_T delay mode
// extern "C" {
// #include "user_interface.h"
// }

void setup() {
    // start the serial connection
    Serial.begin(115200);

    // wait for serial monitor to open
    while(! Serial);

    // initialize dht22
    dht.begin();

    if(!ccs.begin()) {
        Serial.println("Failed to start air sensor! Please check your wiring.");
        while(1);
    }

    //calibrate temperature sensor
    while(!ccs.available());
    float temp = ccs.calculateTemperature();
    ccs.setTempOffset(temp - 25.0);
    connectWifi();
}

void connectWifi() {
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
}

void loop() {
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

    dht.humidity().getEvent(&event);

    Serial.print("humidity: ");
    Serial.print(event.relative_humidity);
    Serial.println("%");

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

    float calcTemp = 0;
    int co2 = 0, tvoc = 0;
    if(ccs.available()) {
        if(!ccs.readData()) {
            Serial.print("CO2: ");
            co2 = ccs.geteCO2();
            Serial.print(co2);
            Serial.print("ppm, TVOC: ");
            tvoc = ccs.getTVOC();
            Serial.print(tvoc);
            Serial.print("ppb   Temp:");
            calcTemp = ccs.calculateTemperature(); 
            Serial.println(calcTemp);
        }
        else {
            Serial.println("ERROR!");
            while(1);
        }
    }

    // save data to Adafruit IO
    humidity->save(event.relative_humidity);
    temperature->save(celsius);
    battery->save(level < 150 ? level : 100);
    IOco2->save(co2);
    IOtvoc->save(tvoc);

    // build a nice string to send to influxdb or whatever you like
    char dataLine[150];
    // sprintf has no support for floats, but will be added later, so we need a String() for now
    sprintf(dataLine, "voltage percent=%d,adc=%d,real=%s,charging=%d \ncelsius:%.1f,humidity:%.1f,CO2=%dppm,TVOC:%dppb,CalcTemp:%.1f\n",
            level < 150 ? level : 100, // cap level to 100%, just for graphing, i don't want to see your lab, when the battery actually gets to that level
            rawLevel,
            String(realVoltage, 3).c_str(),
            rawLevel > 800 ? 1 : 0, // USB is connected if the reading is ~870, as the voltage will be 5V, so we assume it's charging
            celsius,
            event.relative_humidity,
            co2,
            tvoc,
            calcTemp
           );


    Serial.print(dataLine);

    // WiFi.setSleepMode(WIFI_LIGHT_SLEEP);
    // wait 10 seconds (10000 milliseconds == 5 seconds)
    delay(10000);
    Serial.println("-----------------------"); 
}
