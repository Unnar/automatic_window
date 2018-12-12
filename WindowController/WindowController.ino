#include <PID.h>
#include "config.h"

AdafruitIO_Feed *temperature = io.feed("setTemp");
WiFiClient espClient;
PubSubClient client(espClient);

//Stepper motor
#define dir 1
#define stp 3
#define MS2 4
#define EN 5
#define MS1 15

//Door sensor
#define CLS 13

//Feather buttons
#define BTN_A 0
#define BTN_B 16
#define BTN_C 2

//Whether window is in manual or automatic
#define MANUAL 0
#define AUTO 1

const int STEP_MIN = 0;
const int STEP_MAX = 600;

int x;

int windowMode;

const int NUM_MEDIAN = 7;

double tKp = 1.0, tKi = 0, tKd = 1.0,
       tOutputMin = 0, tOutputMax = 100;

PID tempPID(23, 1.0, 0, 1.0, 0, 100, 60000);
PID co2PID(400, 1.0, 0, 1.0, 0, 100, 60000);

double tempOutput;
double co2Output;

double medianFilter[NUM_MEDIAN];
int ind;

int sort_asc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  double a = *((double *)cmp1);
  double b = *((double *)cmp2);
  return a < b ? -1 : (a > b ? 1 : 0);
}

void callback(char* topic, byte* payload, unsigned int length){
    if(strcmp(topic, "esp/temperature") == 0){
        char value[length];
        for(int i = 0; i < length; i++){
            value[i] = (char)payload[i];
        }
        double temp = atof(value);
        tempOutput = tempPID.compute(temp);
    } else if(strcmp(topic, "esp/co2") == 0){ 
        char value[length];
        for(int i = 0; i < length; i++){
            value[i] = (char)payload[i];
        }
        // Apply median filter to co2 value so 
        // random spikes do not affect it badly
        double co2 = atof(value);
        medianFilter[ind] = co2;
        double tmp[NUM_MEDIAN];
        for(int i = 0; i < NUM_MEDIAN; i++){
            tmp[i] = medianFilter[i];
        }
        qsort(tmp, NUM_MEDIAN, sizeof(tmp[0]), sort_asc);
        ind = (ind+1)%NUM_MEDIAN;
        co2Output = co2PID.compute(tmp[NUM_MEDIAN/2]);
    }
}

void handleMessage(AdafruitIO_Data *data){
	int value = data->toInt();
	tempPID.setSetpoint(value);
}

void setup() {
    
    pinMode(stp, OUTPUT);
    pinMode(dir, OUTPUT);
    pinMode(MS1, OUTPUT);
    pinMode(MS2, OUTPUT);
    pinMode(EN, OUTPUT);
    
    pinMode(CLS, INPUT_PULLUP);

    pinMode(BTN_A, INPUT_PULLUP);
    pinMode(BTN_B, INPUT_PULLUP);
    pinMode(BTN_C, INPUT_PULLUP);
    
    resetEDPins();

    close(); //Make sure window is closed at the start

    connectWifi();

    tempOutput = 0;
    co2Output = 0;

    client.subscribe("esp/temperature");
    client.subscribe("esp/co2"); 

    client.setCallback(callback);

    ind = 0;
    for(int i = 0; i < NUM_MEDIAN; i++){
        medianFilter[i] = 400;
    }

	temperature->onMessage(handleMessage);

    windowMode = AUTO;
}

void connectWifi() {
    WiFi.mode(WIFI_STA);
	WiFi.begin(WIFI_SSID, WIFI_PASS);
	
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print("Connecting to WiFi..");
	}
	Serial.println("Connected to the WiFi network");

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

	while (!client.connected()) {
        Serial.println("Connecting to MQTT...");
 
        if (client.connect("ESP8266Client", mqttUser, mqttPassword)) {
            Serial.println("connected");
        } else {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
	}
}

void loop() {
	client.loop();
    io.run();
    if(!digitalRead(BTN_A)) {
        windowMode = AUTO;
        tempPID.setMode(AUTO);       
        co2PID.setMode(AUTO);       
    }
    if(!digitalRead(BTN_B)) {
        if(windowMode == AUTO){
            windowMode = MANUAL;
            tempPID.setMode(MANUAL);
            co2PID.setMode(MANUAL);
        }
        digitalWrite(dir, LOW);
        takeStep();
    }
    if(!digitalRead(BTN_C)) {
        if(windowMode == AUTO){
            windowMode = MANUAL;
            tempPID.setMode(MANUAL);
            co2PID.setMode(MANUAL);
        }
        digitalWrite(dir, HIGH);
        takeStep();
    }
    if(windowMode == AUTO){
        int output = (int)max(tempOutput, co2Output);
        if(output == 0) {
            close(); 
        }
        else {
            stepToTarget(6*output);
        }
    }
}


void stepToTarget(int target){
    // Target out of range for window arm
    if(target < STEP_MIN || target > STEP_MAX)
    {
        char fstring[40];
        sprintf(fstring, "Target must be in range [%d, %d]", STEP_MIN, STEP_MAX);
        Serial.println(fstring);
        return;
    }
    if(x < target)  digitalWrite(dir, LOW);
    else            digitalWrite(dir, HIGH);
    for(int i = 0; i < abs(x-target); i++){
        takeStep();
    }
    // Window arm should now be at target
    x = target;
}

//Reverse until door sensor kicks in
void close()
{
    Serial.println("Closing window.");
    digitalWrite(dir, HIGH); //Pull direction pin high to move in "reverse"
    while(digitalRead(CLS))
    {
        takeStep();
    }
    delay(500);
    //Door sensor kicks in before window is fully closed to keep going a little further
    for(int i = 0; i < 40; i++){
        takeStep();
    }
    x = 0; //Door is now closed so x resets to 0
}

void takeStep(){
    digitalWrite(stp,HIGH); //Trigger one step
    delay(5);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(5);
}

//Reset Easy Driver pins to default states
void resetEDPins()
{
    digitalWrite(stp, LOW);
    digitalWrite(dir, LOW);
    digitalWrite(MS1, LOW);
    digitalWrite(MS2, LOW);
    digitalWrite(EN, HIGH);
}
