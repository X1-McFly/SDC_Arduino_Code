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

 // Arduino 1 pinout:
 // relay pins: 22, 23, 24
 // servo pins: 8, 9, 10, 11
 // color sensor pins: 39 (S0), 40 (S1), 41 (S2), 42 (S3), 2 (PWM) (OUTPUT)
 // limit switch pins: 36, 37, 38

 // Arduino 2 pinout:
 // !! ADD ARDUINO 2 PINOUTS HERE !!

//uncomment include macros
#include <Wire.h>
#include <Servo.h>

// !!IMPORTANT!! -> uncomment the appropriate board type and comment out the other when programming.
#define ARDUINO1
// #define ARDUINO2

#define DEBUG // uncomment to enable Serial debugging

// pinouts for Arduino 1 | !! Make sure this is correct !!
#ifdef ARDUINO1

    #define ARCHIMEDES_SCREW_1_RELAY_NUM 2 //pin 22 Digital
    #define SOLENOID_1_PIN 13
    #define SOLENOID_2_PIN 7

    // Color sensor pins
    const int color_sens_pins[] = {39, 40, 41, 42}; //Digital
    const int num_color_sens_pins = sizeof(color_sens_pins) / sizeof(color_sens_pins[0]); // gets the amount of pins
    #define SENSOR_OUT 2 //pin 2 PWM

    // Limit switch for detection (limit switches are for drawers)
    const int limit_sw_pins[] = {36, 37, 35};
    const int num_limit_sw = sizeof(limit_sw_pins) / sizeof(limit_sw_pins[0]);

    // Relay control pins
    const int relayPins[] = {22, 23, 24}; 
    const int NUM_RELAYS = sizeof(relayPins) / sizeof(relayPins[0]);

#endif

// pinouts for Arduino 2 | !! Make sure this is correct !!
#ifdef ARDUINO2
    // #define ARCHIMEDES_SCREW_1_RELAY_NUM 2 //pin 22 Digital
    // #define SOLENOID_1_RELAY_NUM 0 //pin 23 Digital
    // #define SOLENOID_2_RELAY_NUM 1 //pin 24 Digital

    // // Color sensor pins
    // const int color_sens_pins[] = {39, 40, 41, 42}; //Digital
    // const int num_color_sens_pins = sizeof(color_sens_pins) / sizeof(color_sens_pins[0]); // gets the amount of pins
    // #define SENSOR_OUT 2 //pin 2 PWM

    // // Limit switch for detection (limit switches are for drawers)
    // const int limit_sw_pins[] = {36, 37, 38};
    // const int num_limit_sw = sizeof(limit_sw_pins) / sizeof(limit_sw_pins[0]);

    // // Relay control pins
    // const int relayPins[] = {22, 23, 24}; 
    // const int NUM_RELAYS = sizeof(relayPins) / sizeof(relayPins[0]);
#endif

#define MOTOR_PIN1 6
#define MOTOR_PIN2 4
#define MOTOR_PIN3 3

// Servo motors
Servo steelServo;
Servo brassServo;
Servo nylonServo;
Servo stopGateServo;
Servo hopperGateServo;

// Color detection variables
int detectedColor = 0;
int frequency = 0; // is this needed? It's not used anywhere...

// states
bool isPaused = false;

// define functions
int init_color_sens();
int init_limit_switches();
int init_relay_switches();
int init_servos();
int init_solenoids();
int readColor();
void processSorting(Servo& servo, int angle);
// void checkDrawers();
void pauseSorting();
void unpauseSorting();
// int setChutes(int angle);
void blinkLED();

void setup() {
    Serial.begin(115200);
    
    delay(1000);

    #ifdef DEBUG
    Serial.println("Initializing...");
    #endif

    pinMode(MOTOR_PIN1, OUTPUT);
    pinMode(MOTOR_PIN2, OUTPUT);
    pinMode(MOTOR_PIN3, OUTPUT);

    isPaused = false;

    init_color_sens();
    init_solenoids();
    init_limit_switches();
    init_relay_switches();
    init_servos();

}

// recheck main loop
void loop() {
    

    // checkDrawers();

    if (!isPaused) { 
        steelServo.write(180);
        runMotor();
        openSensorGate();
        detectedColor = readColor();
        Serial.println(detectedColor);
        closeSensorGate();
        delay(10);

        

        switch (detectedColor) { 
            case 0: // Unknown color detected, no action
                delay(30); 
                // processSorting(steelServo, 90);
                // processSorting(brassServo, 90);
                break;
            case 1: // Steel ball detected
                processSorting(steelServo, 90);
                break;

            case 2: // Brass ball detected
                delay(400);
                processSorting(brassServo, 90);
                break;

            case 3: // Nylon ball detected (not sorted by servo)
                resetServos();
                break;
        }
    } else {      
        // checkDrawers();
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
    digitalWrite(color_sens_pins[0], HIGH); // pin 39
    digitalWrite(color_sens_pins[1], HIGH); // pin 40

    #ifdef DEBUG
    Serial.println("Color sensor initialized successfully.");
    #endif

    return 0;
}

int init_limit_switches() {
    // Initialize limit switch
    // for (int i = 0; i < num_limit_sw; i++) {
    //     if (limit_sw_pins[i] < 0) {

    //         #ifdef DEBUG
    //         Serial.print("Error: Invalid limit switch pin index ");
    //         Serial.println(i);
    //         #endif

    //         return -1;
    //     }
    //     pinMode(limit_sw_pins[i], INPUT);
    // }
    for (int i = 0; i < num_limit_sw; i++) {
        pinMode(limit_sw_pins[i], INPUT_PULLUP); // Set limit switch pins to INPUT_PULLUP
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

int init_servos() {
    // Attach and reset servos
    brassServo.attach(8); // brass trap door
    brassServo.write(0);
    #ifdef DEBUG
    Serial.println("Brass servo initialized successfully.");
    #endif
    
    steelServo.attach(9); // steel trap door
    steelServo.write(0);
    #ifdef DEBUG
    Serial.println("Steel servo initialized successfully.");
    #endif
    
    stopGateServo.attach(10); // mini servos altogether
    stopGateServo.write(90);
    #ifdef DEBUG
    Serial.println("StopGate servo initialized successfully.");
    #endif

    hopperGateServo.attach(11);
    #ifdef DEBUG
    Serial.println("HopperGate servo initialized successfully.");
    #endif

    return 0;  
}

int init_solenoids() {
    pinMode(13, OUTPUT);
    pinMode(7, OUTPUT);
}

int readColor() {
    int redFrequency = 0;
    int greenFrequency = 0;
    int blueFrequency = 0;

    delay(600); // Wait for the sensor to stabilize
  
    // Setting redFrequency (R) filteredFrequency photodiodes to be read
    digitalWrite(color_sens_pins[2], LOW); // pin 41
    digitalWrite(color_sens_pins[3], LOW); // pin 42
 
    // Reading the output frequency
    redFrequency = pulseIn(SENSOR_OUT, LOW);
 
    // Printing the redFrequency (R) value
    #ifdef DEBUG
    Serial.print("R = ");
    Serial.print(redFrequency);
    #endif
    delay(100);
 
    // Setting greenFrequency(G) filteredFrequency photodiodes to be read
    digitalWrite(color_sens_pins[2], HIGH);
    digitalWrite(color_sens_pins[3], HIGH);
 
    // Reading the output frequency
    greenFrequency = pulseIn(SENSOR_OUT, LOW);
 
    // Printing the greenFrequency(G) value  
    #ifdef DEBUG
    Serial.print(" G = ");
    Serial.print(greenFrequency);
    #endif
    delay(100);
 
    // Setting blueFrequency (B) filteredFrequency photodiodes to be read
    digitalWrite(color_sens_pins[2], LOW);
    digitalWrite(color_sens_pins[3], HIGH);
 
    // Reading the output frequency
    blueFrequency = pulseIn(SENSOR_OUT, LOW);
 
    // Printing the blueFrequency (B) value
    #ifdef DEBUG
    Serial.print(" B = ");
    Serial.println(blueFrequency);
    #endif
    delay(100);

    // Check if the color readings are within a valid range
    if ((redFrequency < 0 || greenFrequency < 0 || blueFrequency < 0) || 
        (redFrequency > 255 || greenFrequency > 255 || blueFrequency > 255)) { 
        #ifdef DEBUG
        Serial.println("WARNING: Color values out of expected range.");
        #endif
        // return 0; // Error state
    }

    // Determine ball type based on color thresholds (placeholders)
    if (redFrequency < 1500 && redFrequency > 900 && blueFrequency < 1500 && blueFrequency > 900) return 1; // Steel
    if (redFrequency < 2200 && redFrequency > 1500 && greenFrequency< 2200 && greenFrequency> 1500) return 2; // Brass
    if (redFrequency < 1000 && redFrequency > 400 && greenFrequency< 1000 && greenFrequency> 400) return 3; // Nylon

    #ifdef DEBUG
    Serial.println("INFO: Unknown color detected.");
    #endif
    return 0; // Unknown color
}

void processSorting(Servo& servo, int angle) {
    // delay(100);
    servo.write(angle); // Move servo to sorting position
    delay(800);

    #ifdef DEBUG
    Serial.println("Sorting process completed.");
    #endif
}

// void checkDrawers() {
//     for (int i = 0; i < num_limit_sw; i++) {
//         if (digitalRead(limit_sw_pins[i]) == HIGH) { // Check if LOW is the right condition
//             #ifdef DEBUG
//             Serial.println("Drawer removed! Pausing...");
//             #endif
//             pauseSorting();
//             delay(250);
//             return;
//         } else {
//           unpauseSorting();
//         }
//     }
// }

void pauseSorting() {
    isPaused = true;
    digitalWrite(relayPins[1], HIGH); // Exit Solenoid
    digitalWrite(relayPins[2], LOW); // Archimedes Screw
    // setChutes(90); // Close chute servos

    #ifdef DEBUG
    Serial.println("Sorting paused.");
    #endif
}

void unpauseSorting() {
    isPaused = false;
    digitalWrite(relayPins[2], HIGH); // Archimedes Screw
    // setChutes(0); // Open chute servos

    #ifdef DEBUG
    Serial.println("Sorting resumed.");
    #endif
}

// int setChutes(int angle) {
//     stopGateServo.write(angle);

//     #ifdef DEBUG
//     Serial.print("Chutes set to angle: ");
//     Serial.println(angle);
//     #endif

//     return 0;
// }

void openSensorGate() {
    digitalWrite(13,LOW);
    digitalWrite(7, LOW);
    // delay(100);
    digitalWrite(7,HIGH);
    delay(300);
    digitalWrite(7,LOW);
}

void closeSensorGate() {
    digitalWrite(13, HIGH);
    delay(400);
}

void resetServos() {
    steelServo.write(0); // Reset steel servo to neutral position
    brassServo.write(0); // Reset brass servo to neutral position
    delay(100);

    #ifdef DEBUG
    Serial.println("Servos reset to neutral position.");
    #endif
}

int runMotor() {
    digitalWrite(MOTOR_PIN1, HIGH);
    digitalWrite(MOTOR_PIN2, HIGH);
    digitalWrite(MOTOR_PIN3, HIGH);
}