#include "PID.h"
#include "config.h"

AdafruitIO_Feed *setTemp = io.feed("setTemp");
AdafruitIO_Feed *temperature = io.feed("temperature");
AdafruitIO_Feed *co2 = io.feed("co2");

//Stepper motor
#define dir 4
#define stp 5
#define MS1 15
#define MS2 12
#define EN 14

//Door sensor
#define CLS 13

//Feather buttons
#define BTN_A 0
#define BTN_B 16
#define BTN_C 2

//Whether window is in manual or automatic
#define MANUAL 0
#define AUTO 1

//Stepper motor min and max
const int STEP_MIN = 0;
const int STEP_MAX = 600;

// Position of the stepper motor
int pos;

// State of the window
int windowMode;

// Number of elements we use for median filtering
const int NUM_MEDIAN = 7;
double medianFilter[NUM_MEDIAN];
int ind;

/*double tKp = 1.0, tKi = 0, tKd = 1.0,
       tOutputMin = 0, tOutputMax = 100;
*/

// PID controller for temperature
PID tempPID(23, 120.0, 0.0, 12.0, 0, 600, 10000);
// PID controller for co2
PID co2PID(450, 20.0, 0, 3.0, 0, 600, 10000);

// Store the values from the PID controllers
double tempOutput;
double co2Output;

// Comparison function for qsort
int sort_asc(const void *cmp1, const void *cmp2) {
    // Need to cast the void * to double *
    double a = *((double *)cmp1);
    double b = *((double *)cmp2);
    return a < b ? -1 : (a > b ? 1 : 0);
}


// Callback functions for the Adafruit IO feeds
void setTempMessage(AdafruitIO_Data *data) {
    Serial.println("Handling Adafruit IO setTemperature message");
  	int value = data->toInt();
    Serial.print("Received ");
    Serial.println(value);
  	tempPID.setTarget(value);
}

void temperatureMessage(AdafruitIO_Data *data) {
    Serial.println("Handling Adafruit IO temperature message");
    double value = data->toDouble();
    Serial.print("Received ");
    Serial.println(value);
    if(windowMode == MANUAL) return;
    tempOutput = tempPID.compute(value);
}

void co2Message(AdafruitIO_Data *data) {
    Serial.println("Handling Adafruit IO co2 message");
    int value = data->toInt();
    Serial.print("Received ");
    Serial.println(value);
    // Apply median filter to co2 value so 
    // random spikes do not affect it badly
    medianFilter[ind] = value;
    double tmp[NUM_MEDIAN];
    for(int i = 0; i < NUM_MEDIAN; i++){
        tmp[i] = medianFilter[i];
    }
    qsort(tmp, NUM_MEDIAN, sizeof(tmp[0]), sort_asc);
    ind = (ind+1)%NUM_MEDIAN;
    if(windowMode == MANUAL) return;
    co2Output = co2PID.compute(tmp[NUM_MEDIAN/2]);
}

void setup() {
    Serial.begin(115200);

    while(!Serial);
    
    pinMode(stp, OUTPUT);
    pinMode(dir, OUTPUT);
    pinMode(MS1, OUTPUT);
    pinMode(MS2, OUTPUT);
    pinMode(EN, OUTPUT);
    
    pinMode(CLS, INPUT_PULLUP);

    pinMode(BTN_A, INPUT_PULLUP);
    pinMode(BTN_B, INPUT_PULLUP);
    pinMode(BTN_C, INPUT_PULLUP);
    

    connectWifi();

    tempOutput = 0;
    co2Output = 0;
    
    ind = 0;
    for(int i = 0; i < NUM_MEDIAN; i++){
        medianFilter[i] = 450;
    }

    // Setting the callback functions for the Adafruit IO feeds
	  setTemp->onMessage(setTempMessage);
    temperature->onMessage(temperatureMessage);
    co2->onMessage(co2Message);

    windowMode = AUTO;
    
    pos = 1; 
    closeWindow(); //Make sure window is closed at the start
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

int cnt = 0;

void loop() {
    io.run();
    // Print info every 100 loops
    if(cnt == 100) {
        char buffr[100];
        sprintf(buffr, "tempOutput is: %.1f\nco2Output is: %.1f\nWindow should be at: %d",
                tempOutput, co2Output, pos);
        Serial.println(buffr);
        cnt = 0;
    }
    // If user presses A sets the window mode to automatic
    if(!digitalRead(BTN_A)) {
        windowMode = AUTO;
        tempPID.setMode(AUTO);       
        co2PID.setMode(AUTO);       
    }
    // Manually open the window
    if(!digitalRead(BTN_B)) {
        if(windowMode == AUTO){
            windowMode = MANUAL;
            tempPID.setMode(MANUAL);
            co2PID.setMode(MANUAL);
        }
        digitalWrite(dir, LOW);
        while(!digitalRead(BTN_B) && pos < STEP_MAX) {
            takeStep();
            pos++;
        }
    }
    // Manually close the window
    if(!digitalRead(BTN_C)) {
        if(windowMode == AUTO){
            windowMode = MANUAL;
            tempPID.setMode(MANUAL);
            co2PID.setMode(MANUAL);
        }
        int lastPos = pos;
        digitalWrite(dir, HIGH);
        while(!digitalRead(BTN_C) && pos > STEP_MIN) {
            takeStep();
            pos--; 
        }
        // If button was pressed for < ~50ms then close the window
        if(pos-lastPos <= 5){
            closeWindow();
        }
    }
    // Automatic control for the window
    if(windowMode == AUTO) {
        int output = (int)max(tempOutput, co2Output);
        output = tempOutput;
        if(output == 0) {
            closeWindow(); 
        }
        else {
            if(pos != output) {
                Serial.print("Setting window to ");
                Serial.println(output); 
            }
            stepToTarget(output);
        }
    }
    cnt++;
}


void stepToTarget(int target){
    // Target out of range for window arm
    if(target < STEP_MIN || target > STEP_MAX) {
        char fstring[40];
        sprintf(fstring, "Target must be in range [%d, %d] target was %d", STEP_MIN, STEP_MAX, target);
        Serial.println(fstring);
        return;
    }
    if(pos < target)  digitalWrite(dir, LOW);
    else            digitalWrite(dir, HIGH);
    for(int i = 0; i < abs(pos-target); i++) {
        takeStep();
    }
    // Window arm should now be at target
    pos = target;
}

//Reverse until door sensor kicks in
void closeWindow() {
    if(pos == 0) return;
    Serial.println("Closing window.");
    digitalWrite(dir, HIGH); //Pull direction pin high to move in "reverse"
    while(digitalRead(CLS)) {
        takeStep();
    }
    delay(500);
    //Door sensor kicks in before window is fully closed to keep going a little further
    for(int i = 0; i < 40; i++) {
        takeStep();
    }
    pos = 0; //Door is now closed so pos resets to 0
}

void takeStep() {
    digitalWrite(stp,HIGH); //Trigger one step
    delay(5);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(5);
}

//Reset Easy Driver pins to default states
void resetEDPins() {
    digitalWrite(stp, LOW);
    digitalWrite(dir, LOW);
    digitalWrite(MS1, LOW);
    digitalWrite(MS2, LOW);
    digitalWrite(EN, HIGH);
}
