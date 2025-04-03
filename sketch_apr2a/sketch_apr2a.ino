/*
 * Bearing Ball Sorting Machine
 * By Marcus Branton, Martin McCorkle, and Ben Anderson
 * 
 * Last Updated April 2, 2025 by Martin McCorkle
 * 
 * Uses an Arduino Mega 2560 to control servos, solenoids, and sensors 
 * to sort different types of bearing balls (Steel, Brass, Nylon) 
 * based on color detection.
 */


//uncomment include macros
#include <Wire.h>
#include <Servo.h>

#define DEBUG // uncomment to enable Serial debugging

#define ARCHIMEDES_SCREW_1 0 // need to be properly defined
#define ARCHIMEDES_SCREW_2 0 // need to be properly defined
#define SOLENOID_1 0 // need to be properly defined
#define SOLENOID_2 0 // need to be properly defined

// Color sensor pins
const int color_sens_pins[] = {39, 40, 41, 42};
const int num_color_sens_pins = sizeof(color_sens_pins) / sizeof(color_sens_pins[0]); // gets the amount of pins
#define SENSOR_OUT 2

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
const int relayPins[] = {22, 23, 24, 25, 26, 27, 28, 29}; 
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

void setup() {
    Serial.begin(9600);

    init_color_sens();
    init_limit_switches();
    init_relay_switches();
    init_servos();
}

// recheck main loop
void loop() {

    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n'); // Read input until newline

        if (command == "START") { 
            isPaused = false;  // Set the flag to start the system
            stopGateServo.write(0); // Move servo to 0 degrees
            Serial.println("Sorting started!");
        } else if (command == "RESET") {
            isPaused = true; // Stop the system
            stopGateServo.write(180); // Move servo to 180 degrees
            Serial.println("Sorting reset.");
        }
    }
    
    checkDrawers();
    if (!isPaused) { 
        detectedColor = readColor();
        delay(10);
        checkSortingConditions();

        switch (detectedColor) { 
            case 1: // Background detected, no action
                delay(30); 
                checkSortingConditions();
                break;

            case 2: // Steel ball detected
                processSorting(steelServo, 70);
                break;

            case 3: // Brass ball detected
                processSorting(brassServo, 70);
                break;

            case 4: // Nylon ball detected (not sorted by servo)
                processSorting(steelServo, 0); // Keep all servos closed
                break;
        }
    } else {
        checkSortingConditions();
    }
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
    digitalWrite(color_sens_pins[0], HIGH);
    digitalWrite(color_sens_pins[1], LOW);

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
    digitalWrite(relayPins[0], HIGH); 
    digitalWrite(relayPins[2], HIGH);
    digitalWrite(relayPins[3], HIGH);

    #ifdef DEBUG
    Serial.println("Relay switches initialized successfully.");
    #endif

    return 0;
}

//recheck function
int init_servos() {
    // Attach and reset servos
    brassServo.attach(8);
    brassServo.write(0);
    #ifdef DEBUG
    Serial.println("Brass servo initialized successfully.");
    #endif
    
    steelServo.attach(9);
    steelServo.write(0);
    #ifdef DEBUG
    Serial.println("Steel servo initialized successfully.");
    #endif
    
    stopGateServo.attach(10);
    stopGateServo.write(0);
    #ifdef DEBUG
    Serial.println("stopGate servo initialized successfully.");
    #endif

    hopperGateServo.attach(11);
    hopperGateServo.write(0);
    #ifdef DEBUG
    Serial.println("hopperGateServo servo initialized successfully.");
    #endif

    return 0;  
}

//recheck this function
int readColor() {
    int red = 0, green = 0, blue = 0;

    // Read Red component
    digitalWrite(color_sens_pins[2], LOW);
    digitalWrite(color_sens_pins[3], LOW);
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
        return 0; // Error state
    }

    // Determine ball type based on color thresholds (placeholders)
    if (red < 45 && red > 32 && green < 65 && green > 55) return 1; // Background
    if (green < 55 && green > 43 && blue < 47 && blue > 35) return 2; // Steel
    if (red < 53 && red > 40 && green < 53 && green > 40) return 3; // Brass
    if (red < 38 && red > 24 && green < 44 && green > 30) return 4; // Nylon

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
    digitalWrite(ARCHIMEDES_SCREW_1, LOW); // macro needs to be defined
    digitalWrite(ARCHIMEDES_SCREW_2, LOW); // macro needs to be defined
    digitalWrite(SOLENOID_1, LOW); // macro needs to be defined
    digitalWrite(SOLENOID_2, LOW); // macro needs to be defined
    setChutes(90); // Close chute servos
}

// this function need to be completed
void unpauseSorting() {
    isPaused = false;
    digitalWrite(ARCHIMEDES_SCREW_1, HIGH); // macro needs to be defined
    digitalWrite(ARCHIMEDES_SCREW_2, HIGH); // macro needs to be defined
    digitalWrite(SOLENOID_1, HIGH); // macro needs to be defined
    digitalWrite(SOLENOID_2, HIGH); // macro needs to be defined
    setChutes(0); // Close chute servos
}

// this function need to be completed
int setChutes(int angle) {

    return 1;
}
