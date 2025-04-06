#include <Wire.h>
#include <Servo.h>

// !!IMPORTANT!! -> uncomment the appropriate board type and comment out the other when programming.
#define ARDUINO1
// #define ARDUINO2

#define DEBUG // uncomment to enable Serial debugging

// pinouts for Arduino 1 | !! Make sure this is correct !!
#ifdef ARDUINO1

    #define ARCHIMEDES_SCREW_1_RELAY_NUM 2 //pin 22 Digital
    #define SOLENOID_1_RELAY_NUM 0 //pin 23 Digital
    #define SOLENOID_2_RELAY_NUM 1 //pin 24 Digital

    // Color sensor pins
    const int color_sens_pins[] = {39, 40, 41, 42}; //Digital
    const int num_color_sens_pins = sizeof(color_sens_pins) / sizeof(color_sens_pins[0]); // gets the amount of pins
    #define SENSOR_OUT 2 //pin 2 PWM

    // Limit switch for detection (limit switches are for drawers)
    const int limit_sw_pins[] = {36, 37, 38};
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

// Solenoid delay time
#define SOLENOID_DELAY 500 // make shorter if possible

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
bool isPaused = true;

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
// int checkSerial(bool blocking); // if blocking is true 'checkSerial()' will block the program from continuing until a command is received
void blinkLED();

void setup() {
    Serial.begin(9600);
    
    #ifdef DEBUG
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(7, OUTPUT);
    delay(500);
    Serial.println("Initializing...");
    #endif

    init_color_sens();
    init_limit_switches();
    init_relay_switches();
    // init_servos();
}

void loop() {
    delay(50);
    digitalWrite(13, LOW);
    digitalWrite(7, LOW);
    // delay(50);
    // digitalWrite(13, HIGH);
    // digitalWrite(7, LOW);
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
    Serial.println("StopGate servo initialized successfully.");
    #endif

    hopperGateServo.attach(11);
    #ifdef DEBUG
    Serial.println("HopperGate servo initialized successfully.");
    #endif

    return 0;  
}

int readColor() {
    int redFrequency = 0;
    int greenFrequency = 0;
    int blueFrequency = 0;
  
    // Setting RED (R) filtered photodiodes to be read
    digitalWrite(color_sens_pins[2], LOW); // pin 41
    digitalWrite(color_sens_pins[3], LOW); // pin 42
 
    // Reading the output frequency
    redFrequency = pulseIn(SENSOR_OUT, LOW);
 
    // Printing the RED (R) value
    #ifdef DEBUG
    Serial.print("R = ");
    Serial.print(redFrequency);
    #endif
    delay(100);
 
    // Setting GREEN (G) filtered photodiodes to be read
    digitalWrite(color_sens_pins[2], HIGH);
    digitalWrite(color_sens_pins[3], HIGH);
 
    // Reading the output frequency
    greenFrequency = pulseIn(SENSOR_OUT, LOW);
 
    // Printing the GREEN (G) value  
    #ifdef DEBUG
    Serial.print(" G = ");
    Serial.print(greenFrequency);
    #endif
    delay(100);
 
    // Setting BLUE (B) filtered photodiodes to be read
    digitalWrite(color_sens_pins[2], LOW);
    digitalWrite(color_sens_pins[3], HIGH);
 
    // Reading the output frequency
    blueFrequency = pulseIn(SENSOR_OUT, LOW);
 
    // Printing the BLUE (B) value
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
        return 0; // Error state
    }

    // Determine ball type based on color thresholds (placeholders)
    if (redFrequency < 45 && redFrequency > 32 && greenFrequency < 65 && greenFrequency > 55) return 1; // Background
    if (greenFrequency < 55 && greenFrequency > 43 && blueFrequency < 47 && blueFrequency > 35) return 2; // Steel
    if (redFrequency < 53 && redFrequency > 40 && greenFrequency < 53 && greenFrequency > 40) return 3; // Brass
    if (redFrequency < 38 && redFrequency > 24 && greenFrequency < 44 && greenFrequency > 30) return 4; // Nylon

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

    #ifdef DEBUG
    Serial.println("Sorting process completed.");
    #endif
}

void checkSortingConditions() {
    #ifdef DEBUG
    Serial.println("Checking sorting conditions...");
    #endif
}

void checkDrawers() {
    for (int i = 0; i < num_limit_sw; i++) {
        if (digitalRead(limit_sw_pins[i]) == LOW) { // Check if LOW is the right condition
            #ifdef DEBUG
            Serial.println("Drawer removed! Pausing...");
            #endif
            pauseSorting();
            delay(250);
            return;
        } else {
            // unpauseSorting();
        }
    }
}

void pauseSorting() {
    isPaused = true;
    digitalWrite(relayPins[1], HIGH); // Exit Solenoid
    digitalWrite(relayPins[2], LOW); // Archimedes Screw
    setChutes(90); // Close chute servos

    #ifdef DEBUG
    Serial.println("Sorting paused.");
    #endif
}

void unpauseSorting() {
    isPaused = false;
    digitalWrite(relayPins[2], HIGH); // Archimedes Screw
    setChutes(0); // Open chute servos

    #ifdef DEBUG
    Serial.println("Sorting resumed.");
    #endif
}

int setChutes(int angle) {
    stopGateServo.write(angle);

    #ifdef DEBUG
    Serial.print("Chutes set to angle: ");
    Serial.println(angle);
    #endif

    return 0;
}

void blinkLED() {
    #ifdef DEBUG
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    #endif
}