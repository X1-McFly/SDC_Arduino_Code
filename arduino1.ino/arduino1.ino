/*
 * Bearing Ball Sorting Machine
 * By Marcus Branton, Martin McCorkle, and Ben Anderson
 * 
 * 
 * Uses an Arduino Mega 2560 to control servos, solenoids, and sensors 
 * to sort different types of bearing balls (Steel, Brass, Nylon) 
 * based on color detection.
 *
 * Begin program by connecting to the Arduino Serial Monitor and sending the command "START" to begin sorting.
 * Send "RESET" to stop the sorting process.
 */

 // relay pins: 22, 23, 24
 // servo pins: 8, 9, 10, 11
 // color sensor pins: 39 (S0), 40 (S1), 41 (S2), 42 (S3), 2 (PWM) (OUTPUT)
 // limit switch pins: 36, 37, 38

//uncomment include macros
#include <Wire.h>
#include <Servo.h>

// #define DEBUG // uncomment to enable Serial debugging

#define ARCHIMEDES_SCREW_1_RELAY_NUM 2 //pin 22 Digital
#define SOLENOID_1_RELAY_NUM 0 //pin 23 Digital
#define SOLENOID_2_RELAY_NUM 1 //pin 24 Digital

// Color sensor pins
const int color_sens_pins[] = {39, 40, 41, 42}; //Digital
const int num_color_sens_pins = sizeof(color_sens_pins) / sizeof(color_sens_pins[0]); // gets the amount of pins
#define SENSOR_OUT 2 //pin 2 PWM

// Servo motors
Servo steelServo;
Servo brassServo;
Servo nylonServo;
Servo stopGateServo;
Servo hopperGateServo;

// Solenoid delay time
const int SOLENOID_DELAY = 500; // make shorter if possible

// Limit switch for detection (limit switches are for drawers)
const int limit_sw_pins[] = {36, 37, 38};
const int num_limit_sw = sizeof(limit_sw_pins) / sizeof(limit_sw_pins[0]);

// Color detection variables
int detectedColor = 0;
int frequency = 0;

// Relay control pins
const int relayPins[] = {22, 23, 24}; 
const int NUM_RELAYS = sizeof(relayPins) / sizeof(relayPins[0]);

// states
bool isPaused = false;

// define functions
int init_color_sens();
int init_limit_switches();
int init_relay_switches();
int init_servos();
int readColor();
void processSorting(Servo& servo, int angle);
void checkSortingConditions();
void checkDrawers();
void pauseSorting();
void unpauseSorting();
int setChutes(int angle);
int checkSerial(bool blocking); // if blocking is true 'checkSerial()' will block the program from continuing until a command is received

void setup() {
    Serial.begin(9600);

    // checkSerial(true); // Check for serial commands and block until one is received

    init_color_sens();
    init_limit_switches();
    init_relay_switches();
    init_servos();
}

// recheck main loop
void loop() {

    openSensorGate();
    Serial.println(readColor());
    // delay(500);
    closeSensorGate();


    // checkSerial(false); // Check for serial commands without blocking
    // checkDrawers();

    // if (!isPaused) { 
    //     detectedColor = readColor();
    //     delay(10);
    //     checkSortingConditions();

    //     switch (detectedColor) { 
    //         case 1: // Background detected, no action
    //             delay(30); 
    //             checkSortingConditions();
    //             break;

    //         case 2: // Steel ball detected
    //             processSorting(steelServo, 70);
    //             break;

    //         case 3: // Brass ball detected
    //             processSorting(brassServo, 70);
    //             break;

    //         case 4: // Nylon ball detected (not sorted by servo)
    //             processSorting(steelServo, 0); // Keep all servos closed
    //             break;
    //     }
    // } else {
    //     checkSortingConditions();
    // }
}

int init_color_sens() {
    // Initialize color sensor pins
    for (int i = 0; i < num_color_sens_pins; i++) {
        if (color_sens_pins[i] < 0) { 
            
            #ifdef DEBUG
            Serial.print("Error: Invalid color sensor pin index ");
            Serial.println(i);
            #endif

            return -1;
        }
        pinMode(color_sens_pins[i], OUTPUT);
    }
    pinMode(SENSOR_OUT, INPUT);

    // Set color sensor frequency scaling
    digitalWrite(color_sens_pins[0], HIGH); // pin 39
    digitalWrite(color_sens_pins[1], LOW); // pin 40

    #ifdef DEBUG
    Serial.println("Color sensor initialized successfully.");
    #endif

    return 0;
}

// recheck function
int init_limit_switches() {
    // Initialize limit switch
    for (int i = 0; i < num_limit_sw; i++) {
        if (limit_sw_pins[i] < 0) {

            #ifdef DEBUG
            Serial.print("Error: Invalid limit switch pin index ");
            Serial.println(i);
            #endif

            return -1;
        }
        pinMode(limit_sw_pins[i], INPUT);
    }
    
    #ifdef DEBUG
    Serial.println("Limit switches initialized successfully.");
    #endif

    return 0;
}

int init_relay_switches() {
    // Initialize relay pins
    for (int i = 0; i < NUM_RELAYS; i++) {
        if (relayPins[i] < 0) {

            #ifdef DEBUG
            Serial.print("Error: Invalid relay pin index ");
            Serial.println(i);
            #endif

            return -1;
        }
        pinMode(relayPins[i], OUTPUT);
        digitalWrite(relayPins[i], LOW);
    }

    // Set initial state of relays
    digitalWrite(relayPins[0], LOW); //pin 22 Entrance solenoid
    digitalWrite(relayPins[1], HIGH); //pin 23 Exit solenoid
    digitalWrite(relayPins[2], HIGH); //pin 24 Archimedes Screw

    #ifdef DEBUG
    Serial.println("Relay switches initialized successfully.");
    #endif

    return 0;
}

//recheck function
int init_servos() {
    // Attach and reset servos
    brassServo.attach(8); // brass trap door
    brassServo.write(90);
    #ifdef DEBUG
    Serial.println("Brass servo initialized successfully.");
    #endif
    
    steelServo.attach(9); // steel trap door
    steelServo.write(90);
    #ifdef DEBUG
    Serial.println("Steel servo initialized successfully.");
    #endif
    
    stopGateServo.attach(10); // mini servos altogether
    stopGateServo.write(90);
    #ifdef DEBUG
    Serial.println("stopGate servo initialized successfully.");
    #endif

    hopperGateServo.attach(11);
   // hopperGateServo.write(0); //0 - open , 180 - closed
    #ifdef DEBUG
    Serial.println("hopperGateServo servo initialized successfully.");
    #endif

    return 0;  
}

//recheck this function
int readColor() {
    int red = 0, green = 0, blue = 0;

    // Read Red component
    digitalWrite(color_sens_pins[2], LOW); // pin 41
    digitalWrite(color_sens_pins[3], LOW); // pin 42
    frequency = pulseIn(SENSOR_OUT, LOW);
    if (frequency == 0) {
        #ifdef DEBUG
        Serial.println("ERROR: No signal from color sensor while reading RED.");
        #endif
        return 0; // Error state
    }
    red = frequency;
    #ifdef DEBUG
    Serial.print("R= "); Serial.print(red); Serial.print("  ");
    #endif
    delay(50);

    // Read Green component
    digitalWrite(color_sens_pins[2], HIGH);
    digitalWrite(color_sens_pins[3], HIGH);
    frequency = pulseIn(SENSOR_OUT, LOW);
    if (frequency == 0) {
        #ifdef DEBUG
        Serial.println("ERROR: No signal from color sensor while reading GREEN.");
        #endif
        return 0; // Error state
    }
    green = frequency;
    #ifdef DEBUG
    Serial.print("G= "); Serial.print(green); Serial.print("  ");
    #endif
    delay(50);

    // Read Blue component
    digitalWrite(color_sens_pins[2], LOW);
    digitalWrite(color_sens_pins[3], HIGH);
    frequency = pulseIn(SENSOR_OUT, LOW);
    if (frequency == 0) {
        #ifdef DEBUG
        Serial.println("ERROR: No signal from color sensor while reading BLUE.");
        #endif
        return 0; // Error state
    }
    blue = frequency;
    #ifdef DEBUG
    Serial.print("B= "); Serial.println(blue);
    #endif
    delay(50);

    // Check if the color readings are within a valid range
    if ((red < 0 || green < 0 || blue < 0) || 
        (red > 255 || green > 255 || blue > 255)) { 
        #ifdef DEBUG
        Serial.println("WARNING: Color values out of expected range.");
        #endif
        // return 0; // Error state
    }

    // Determine ball type based on color thresholds (placeholders)
    if (red < 45 && red > 32 && green < 65 && green > 55) return 1; // Background
    if (green < 9200 && green > 7500 && blue < 9200 && blue > 7500) return 2; // Steel
    if (red < 12000 && red > 11500 && green < 13500 && green > 11000) return 3; // Brass
    if (red < 5500 && red > 4500 && green < 5500 && green > 4500) return 4; // Nylon

    #ifdef DEBUG
    Serial.println("INFO: Unknown color detected.");
    #endif
    return 0; // Unknown color
}

void processSorting(Servo& servo, int angle) {
    digitalWrite(relayPins[0], HIGH); // Close entrance
    delay(10);
    checkSortingConditions();

    servo.write(angle); // Move servo to sorting position
    digitalWrite(relayPins[1], LOW); // Open exit
    checkSortingConditions();

    delay(SOLENOID_DELAY);
    digitalWrite(relayPins[1], HIGH); // Close exit
    delay(SOLENOID_DELAY);
    digitalWrite(relayPins[0], LOW); // Reopen entrance
}

// this function need to be completed
void checkSortingConditions() {
    // Placeholder function to handle conditions like pause or drawer removal
}

// recheck this function
void checkDrawers() {
    for (int i = 0; i < num_limit_sw; i++) {
        if (digitalRead(limit_sw_pins[i]) == HIGH) { // Check if HIGH is the right condition
            Serial.println("Drawer removed! Pausing...");
            pauseSorting();
            delay(250);
            return;
        }
        else {
            unpauseSorting();
        }
    }
}

// this function need to be completed
void pauseSorting() {
    isPaused = true;
    //digitalWrite(ARCHIMEDES_SCREW_1_RELAY_NUM, LOW); // macro needs to be defined
    //digitalWrite(ARCHIMEDES_SCREW_2_RELAY_NUM, LOW); // macro needs to be defined
    //digitalWrite(SOLENOID_1_RELAY_NUM, LOW); // macro needs to be defined
    //digitalWrite(SOLENOID_2_RELAY_NUM, LOW); // macro needs to be defined
    digitalWrite(relayPins[1], HIGH); //Exit Solenoid
    //digitalWrite(relayPins[0], LOW);
    digitalWrite(relayPins[2], LOW); //Archimedes Screw
    setChutes(90); // Close chute servos
}

// this function need to be completed
void unpauseSorting() {
    isPaused = false;
    //digitalWrite(relayPins[ARCHIMEDES_SCREW_1_RELAY_NUM], HIGH);
    //digitalWrite(relayPins[ARCHIMEDES_SCREW_2_RELAY_NUM], HIGH);
    //digitalWrite(relayPins[SOLENOID_1_RELAY_NUM], HIGH);
    //digitalWrite(relayPins[SOLENOID_2_RELAY_NUM], HIGH);
    digitalWrite(relayPins[2], HIGH); //Archimedes Screw
    setChutes(0); // Open chute servos
}

// this function need to be completed
int setChutes(int angle) {
    stopGateServo.write(angle);
    return 0;
    //return 1;
}

void checkSerial(bool blocking) {
    do {
        if (Serial.available()) {
            String command = Serial.readStringUntil('\n');

            command.trim(); // Remove trailing \r or whitespace

            if (command == "START") { 
                isPaused = false;
                stopGateServo.write(0);
                Serial.println("Sorting started!");
            } else if (command == "RESET") {
                isPaused = true;
                stopGateServo.write(180);
                Serial.println("Sorting reset.");
            } else {
                Serial.println("Not a valid command. Please use START or RESET.");
            }
        }
    } while (blocking && Serial.available() == 0);
}

void openSensorGate() {
    digitalWrite(13,LOW);
    digitalWrite(7, LOW);
    // delay(100);
    digitalWrite(7,HIGH);
    delay(300);
    digitalWrite(7,LOW);
  }
  
  void closeSensorGate() {
    delay(500);
    digitalWrite(13, HIGH);
    delay(500);
  }
