#include <Arduino.h>
#include "HX711.h"
#include <NoDelay.h>
#include <HCSR04.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal_I2C.h>
#include "AccelStepper.h"

#define DEBUG // Uncomment to disable debugging printing
#ifdef DEBUG
#define debug_print(x) Serial.print(x)
#define debug_println(x) Serial.println(x)
#else
#define debug_print(x)   // Empty statement
#define debug_println(x) // Empty statement
#endif

//-------------address-------------------
/*
0x27: LCD
0x40: Servo driver
0x70: nano
*/

//----------USER INTERFACE-------------------
unsigned int reqPilinutsLevel = 17;
int reqTubigLevel = 200;
unsigned int reqSugarLevel = 17; // required sugar level

//---------------------------------

//-----------I2C Communication-----------------
char startNano = 's';

//-----------TIME MILLIS -----------------
unsigned long currentMillis;
unsigned long previousMillis = 0; // set up timer for whole machine
const unsigned long interval = 20;
unsigned long bookmarkTime; // capture starting time
long timeleft;              // time left as it counts down

unsigned long previousMachineMillis = 0;
unsigned long previousInductionMedMillis = 0;
unsigned long previousInductionLowMillis = 0;
unsigned long previousStirringMillis = 0;
//------------------------

//--------------automation-------------
int state = -1; // stage of automation //original position and number is -1
//-------------------------------

//----------MACHINE PROCESS STATUS------------------
// this determines the current action of the machine
enum Machine_process
{
    stop_process,     // This Halts all operations
    presetup_process, // This starts the sensing of pili, sugar, and water
    dispensing_process,
    cooking_process,
    cooling_process,
    packaging_process,
};

Machine_process machine_statusnow = stop_process;

// case switch global
int dispensing_switch = 1;
bool dispensing_flag = true;

//-------------------------------

//--------------HX711-------------------
HX711 scale1;
HX711 scale2;
HX711 scales[2] = {scale1, scale2};
const uint8_t dataPin[2] = {22, 24};
const uint8_t clockPin = 23;
// TODO you need to adjust to your calibrate numbers
float calib[2] = {320, 559};
uint32_t count = 0;
//-------------------------------------------------
//-------FUNCTIONS INITIALIZE -------------
unsigned int UltraS1();
unsigned int UltraS2();
void waterLevel();
int machine_buttons();
// int dispensing_specificprocess(int dispensing_switch);
void water_dispensing();
void sugarHopper_servo();
void loadCell_servos();
void panRotation();
//-------------

//--------- FLAGS-----------------------
// make these false at stop
bool waterPump_flag = false;
bool sugarHopper_flag = false;
bool loadCell_servos_flag = false;
bool panRotation_flag = false;

int stepFlagInduction_Med = 0; // On induction cooker to Medium temp
int stepFlagInduction_Low = 0; // Adjust to Low Temp, then Off Induction cooker

int stepFlagStirring = 0;   // stirring arm sequence
int toggleFlagStirring = 0; // stirring arm toggle sequence. 1 means On, 0 means off
//---------------------

//-----------INDUCTION COOKER---------------
const int IndCooker_ONOFF = 7;
int IndCooker_FUNC = 8;
int IndCooker_negaTEMP = 9;
int IndCooker_posiTEMP = 10;
//--------------------------------

//---------Stirrer Arm---------------
const int stirrer_up = 30;   // 1 min and 11 secs pataas
const int stirrer_down = 31; // 1min and 5 secs pababa
const int stirrer_cc = 34;
const int stirrer_cw = 35;
const int stirrer_speed = 36;
// add EnA in pwm pin if using L298N motor driver

//-----------------------------

//-----------NO DELAY FUNCTIONS---------------------------
int get_Loadcells(); // Must declare function before noDelay, function can not take arguments
// noDelay loadCellvalues(3000, get_Loadcells); // Creats a noDelay varible set to 1000ms, will call ledBlink function
//----------------------------------

//--------------ULTRASONIC-----------------------------
byte triggerPin = 45;
byte echoCount = 2;
byte *echoPins = new byte[echoCount]{46, 48};
double *distances;

//----------------------------------

//-------------------WATER LEVEL-----------------------
enum WaterLevelState
{
    IDLE,
    POWER_ON,
    READ_SENSOR,
    POWER_OFF
};
// Define the pins
const int WATER_LEVEL_POW = 40; // Power pin for water level sensor
const int WATER_LEVEL_S = A0;   // Analog pin for water level sensor
// Define variables
WaterLevelState Waterstate = IDLE;
unsigned long lastStateTime = 0;
int water_level = 0;
//-----------------WATER DISPENSING--------------------
const int WaterPump = 26;
//-------------------------------

//------------------LCD----------------------
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address and size
//---------------------------------------

//------------------- PB LOGIC ----------------
const int START_PB = 29;
const int STOP_PB = 28;
unsigned long debounceDelay_PB = 50; // Debounce delay in milliseconds
bool machineOnStatus = false;
bool buttonStartPressed = false;
bool buttonStopPressed = false;
unsigned long prevTime_StartPB = 0;
unsigned long prevTime_StopPB = 0;
//--------------------------------------------------

//------------------LIGHT INDICATORS-----------------
const int IL_NoSugar = 2;
const int IL_NoNuts = 3;
const int IL_NoWater = 4;
const int IL_Stopped = 5;
const int IL_Running = 6;
//---------------------------------------

//---------------SERVO MOTORS---------------
#define SERVOMIN 650  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 2350 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
const byte sugarHopper = 6;
const byte nutsHopper = 1;
const byte arm1LDcell = 3;
const byte arm2LDcell = 7;

// intial position
float val_sugarHopper = 1500; // 2450
float val_nutsHopper = 1500;  // 1700
float val_arm1LDcell = 1600;  // 2600
float val_arm2LDcell = 1500;  // 1500

// both filtered and unfiltered must be the same value
// 1500 means bukas
float val_sugarHopperFiltered = 1500; // 1000 means over open ; 1500 means open ; 2000 means close
float val_nutsHopperFiltered = 1500;  // 1500 means close ; 1600 means open?
float val_arm1LDcellFiltered = 1600;  // 1600 means initial position ; 2600 means dispensing?
float val_arm2LDcellFiltered = 1500;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

float filter(float prevValue, float currentValue, int filter)
{
    float lengthFiltered = (prevValue + (currentValue * filter)) / (filter + 1);
    return lengthFiltered;
}

//----------------------------------------------

//--------------STEPPER MOTORS-------------------
const int stpprPANstep = 16;
const int stpprPANdir = 17;
#define motorInterfaceType 1
const int targetPosition = 1000;
const int defaultPosition = 0;
AccelStepper kawaliStepper(motorInterfaceType, stpprPANstep, stpprPANdir);
//-----------------------------

void setup()
{
#ifdef DEBUG
    Serial.begin(9600); // Adjust baud rate as needed
    debug_println("Debug mode enabled");
    debug_println(__FILE__);
    debug_println("LIBRARY VERSION: ");
    debug_println(HX711_LIB_VERSION);
    debug_println();
#endif

    for (int i = 0; i < 2; i++)
    {
        scales[i].begin(dataPin[i], clockPin);
        scales[i].set_scale(calib[i]);
        // reset the scale to zero = 0
        scales[i].tare();
    }
    //-----------------------------------
    delay(10);
    //-----------ULTRASONIC-------------------
    HCSR04.begin(triggerPin, echoPins, echoCount);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(13, OUTPUT); // TEST ONLY, REMOVE AFTER
    //-------------------------------------

    //------------Induction Cooker--------------
    // these are relays that act as pulses in capacitive touch
    pinMode(IndCooker_ONOFF, OUTPUT);
    pinMode(IndCooker_FUNC, OUTPUT);
    pinMode(IndCooker_negaTEMP, OUTPUT);
    pinMode(IndCooker_posiTEMP, OUTPUT);
    //---------------------------------------

    //--------------I2C Communication----------
    Wire.begin(); // Start I2C communication as master

    //--------------LCD----------------------
    lcd.init();          // Initialize the LCD
    lcd.backlight();     // Turn on backlight
    lcd.clear();         // Clear the LCD display
    lcd.setCursor(0, 0); // Set cursor position
    lcd.print("Setting up Machine...");
    lcd.setCursor(0, 1);
    lcd.print("Please wait..."); // Print the counter value
    delay(1000);
    //-----------------------------------------------

    //-----------stirrer arm------------------
    // 1st L298N- 12 Volts
    pinMode(stirrer_up, OUTPUT);   // in1
    pinMode(stirrer_down, OUTPUT); // in2

    // 2nd L298N - 24 Volts
    pinMode(stirrer_cc, OUTPUT);    // in1
    pinMode(stirrer_cw, OUTPUT);    // in2
    pinMode(stirrer_speed, OUTPUT); // enA
    // put pinMode of enA & enB if using L298N

    digitalWrite(stirrer_up, LOW);
    digitalWrite(stirrer_down, LOW);
    digitalWrite(stirrer_cc, LOW);
    digitalWrite(stirrer_cw, LOW);
    //-----------------------------------

    //-----------Water Level------------
    pinMode(WATER_LEVEL_POW, OUTPUT);
    digitalWrite(WATER_LEVEL_POW, LOW);
    //-----------WATER DISPENSING---------------
    pinMode(WaterPump, OUTPUT);
    //---------------

    //---------- PB LOGIC --------------------------
    pinMode(STOP_PB, INPUT_PULLUP);
    pinMode(START_PB, INPUT_PULLUP);
    //-----------------------------------

    //------------LIGHT INDICATORS ------------------
    pinMode(IL_NoSugar, OUTPUT);
    pinMode(IL_NoNuts, OUTPUT);
    pinMode(IL_NoWater, OUTPUT);
    pinMode(IL_Running, OUTPUT);
    pinMode(IL_Stopped, OUTPUT);
    //-----------------------------------------------

    //------------------SERVO MOTORS-----------
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    //--------------------------------

    //-------------STEPPER MOTORS--------
    kawaliStepper.setMaxSpeed(1000);
    kawaliStepper.setAcceleration(50);
    kawaliStepper.setSpeed(200);
    kawaliStepper.move(0); // kawaliStepper.moveTo(200) i changed this since no initial position indicator in actual
    //---------------------------

    // intitial positions
}

void loop()
{
    currentMillis = millis();
    if (currentMillis - previousMillis >= 10)
    {
        // start event
        previousMillis = currentMillis;
        // if (int prevstate = !state)
        // {
        //     // debug_println(state);
        // }

        int buttonSetState = digitalRead(START_PB);
        int buttonResetState = digitalRead(STOP_PB);

        // start button logic
        if (buttonSetState == LOW && buttonStartPressed == 0)
        {
            state = 1;
            buttonStartPressed = 1;
        }

        else if (buttonSetState == HIGH && buttonStartPressed == 1)
        {
            delay(250);
            buttonStartPressed = 0;
        }

        // stop button logic
        if (buttonResetState == LOW && buttonStopPressed == 0)
        {
            // set all to resting or initial position
            state = -1;
            buttonStopPressed = 1;
        }

        else if (buttonResetState == HIGH && buttonStopPressed == 1)
        {
            delay(250);
            buttonStopPressed = 0;
        }

        if (state == -1 && machineOnStatus == false)
        {
            // stop process is ran once.
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Machine Status:");
            lcd.setCursor(0, 1);
            lcd.print("Stopped");
            digitalWrite(IL_NoSugar, LOW);
            digitalWrite(IL_NoWater, LOW);
            digitalWrite(IL_NoNuts, LOW);
            digitalWrite(IL_Running, LOW);
            digitalWrite(IL_Stopped, HIGH);
            bookmarkTime = 0; // reset timers to 0
            timeleft = 0;
            stepFlagInduction_Low = 0; // reset to initial position
            stepFlagInduction_Med = 0;
            toggleFlagStirring = 0;

            state = 0;
            debug_println("nag-stop");
            previousMachineMillis = currentMillis;
        }

        else if (state == 0 && machineOnStatus == true)
        {
            digitalWrite(WATER_LEVEL_POW, HIGH);
            state = 1;
            debug_println(state + "yeah!");
            previousMachineMillis = currentMillis;
        }
        else if (state == 1)
        {
            // idle or presetup process
            static int runOnce = 0;
            if (runOnce == 0)
            {
                lcd.clear();
                lcd.setCursor(0, 0); // Set cursor position
                lcd.print("Please Fill the");
                lcd.setCursor(0, 1);
                lcd.print("Ingredients");
                runOnce = 1;
            }
            digitalWrite(IL_Running, HIGH); // REvise this code
            digitalWrite(WATER_LEVEL_POW, HIGH);
            digitalWrite(IL_Stopped, LOW); // REvise this code
            water_level = analogRead(WATER_LEVEL_S);
            distances = HCSR04.measureDistanceCm();
            // debug_print(distances[0]);
            // debug_print(distances[1]);
            debug_println(water_level);
            if (distances[1] <= 200 && distances[0] <= 200 && water_level > 200)
            {
                digitalWrite(LED_BUILTIN, HIGH);
                digitalWrite(13, HIGH); // TEST ONLY, REMOVE AFTER
                state = 2;              // move to dispensing process
            }
            else
            {
                digitalWrite(LED_BUILTIN, LOW);
                digitalWrite(13, LOW); // TEST ONLY, REMOVE AFTER
            }

            if (distances[0] >= reqSugarLevel)
            {
                digitalWrite(IL_NoSugar, HIGH);
                debug_println("WALANG LAMAN ASUKAL");
            }
            else
            {
                digitalWrite(IL_NoSugar, LOW);
            }

            if (distances[1] >= reqPilinutsLevel)
            {
                digitalWrite(IL_NoNuts, HIGH);
                debug_println("WALANG LAMAN NUTS");
            }
            else
            {
                digitalWrite(IL_NoNuts, LOW);
            }

            if (water_level <= 200)
            {
                digitalWrite(IL_NoWater, HIGH);
            }
            else
            {
                digitalWrite(IL_NoWater, LOW);
                debug_println("WALANG LAMAN WATER");
            }
            previousMachineMillis = currentMillis;
        }

        else if (state == 2 && currentMillis - previousMachineMillis > 1000)
        {
            // turn off water lvl sens, on Waterpump
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Water Pump: ");
            lcd.setCursor(0, 1);
            lcd.print("   Open");

            digitalWrite(WATER_LEVEL_POW, LOW);
            digitalWrite(WaterPump, HIGH);
            debug_println("WAterPump is open");
            previousMachineMillis = currentMillis;
            state = 3;
        }

        else if (state == 3 && currentMillis - previousMachineMillis > 4500)
        {
            // turn off Waterpump
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Water Pump: ");
            lcd.setCursor(0, 1);
            lcd.print("    Closed");

            digitalWrite(WaterPump, LOW);
            debug_println("WAterPump is closed");
            previousMachineMillis = currentMillis;
            state = 4;
        }

        else if (state == 4 && currentMillis - previousMachineMillis > 1500)
        {
            // open sugar hopper servos
            // update: run at 1500 milis
            static int runOnce = 0;
            if (runOnce == 0)
            {
                lcd.clear();
                lcd.setCursor(0, 0); // Set cursor position
                lcd.print("Sugar Hopper");
                lcd.setCursor(10, 1);
                lcd.print("grams");
                runOnce = 1;
            }
            val_sugarHopper = 1500; // means open
            previousMachineMillis = currentMillis;
            int value = get_Loadcells();
            debug_println("opening sugar hopper, value: ");
            if (value > 500)
            {
            }
            lcd.setCursor(0, 1);
            lcd.print(String(value));
            state = 5;
        }

        else if (state == 5 && currentMillis - previousMachineMillis > 5000)
        {
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Sugar Hopper: ");
            lcd.setCursor(0, 1);
            lcd.print("   Closed");

            val_sugarHopper = 1600; // close value
            debug_println("closed sugar hopper");
            previousMachineMillis = currentMillis;
            state = 6;
        }

        else if (state == 6 && currentMillis - previousMachineMillis > 5000)
        {
            // the servo should be intialized in resting position
            // the servo will go down now, dispensing sugar to cooking pan
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Sugar to Pan:");
            lcd.setCursor(0, 1);
            lcd.print("Dispensing...");

            val_arm1LDcell = 1700;
            val_arm2LDcell = 1700;
            debug_println("from hopper (sugar) to pan");
            previousMachineMillis = currentMillis;
            state = 7;
        }

        else if (state == 7 && currentMillis - previousMachineMillis > 5000)
        {
            // arm1LDcell go to resting pos, turn on induction cooker and set it to normal temp multitask
            val_arm1LDcell = 1500;
            val_arm2LDcell = 1500;
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Turning on");
            lcd.setCursor(0, 1);
            lcd.print("Cooker");

            debug_println("turn on induction cooker");
            stepFlagInduction_Med = 1; // start induction cooker sequence
            previousMachineMillis = currentMillis;
            state = 8;
        }

        else if (state == 8 && currentMillis - previousMachineMillis > 5000)
        {
            // let stirrer arm go down, adjust the time on testing, initialize it to be upward
            // capture the time
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Stirrer down");
            lcd.setCursor(0, 1);
            lcd.print("Timeleft: ");

            digitalWrite(stirrer_down, HIGH);
            digitalWrite(stirrer_up, LOW);
            bookmarkTime = millis() + 1000; // set clock timer for 1min 05 sec or 65000
            previousMachineMillis = currentMillis;
            state = 9;
        }

        else if (state == 9 && currentMillis - previousMachineMillis > 50)
        {

            // during

            digitalWrite(stirrer_down, HIGH);
            digitalWrite(stirrer_up, LOW);
            timeleft = bookmarkTime - millis();
            debug_println("down timeleft: " + String(timeleft / 1000));
            lcd.setCursor(10, 1);
            lcd.print(String(timeleft / 1000));
            if (timeleft <= 0)
            {
                // after
                // stay stirring arm, start rotating sequence for 2 minutes
                // timeleft = 0;
                toggleFlagStirring = 1;
                digitalWrite(stirrer_down, LOW);
                lcd.setCursor(10, 1);
                lcd.print("DONE");
                previousMachineMillis = currentMillis;
                state = 10;
            }
            previousMachineMillis = currentMillis;
        }

        else if (state == 10 && currentMillis - previousMachineMillis > 1000)
        {
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Boiling");
            lcd.setCursor(0, 1);
            lcd.print("Timeleft: ");

            debug_println("resseting clock");
            bookmarkTime = 0;
            timeleft = 0;
            bookmarkTime = millis() + 12000; // set clocktime for 2 minutes. or 120000
            toggleFlagStirring = 1;
            previousMachineMillis = currentMillis;
            state = 11;
        }

        else if (state == 11 && currentMillis - previousMachineMillis > 50)
        {
            // during
            timeleft = bookmarkTime - millis();
            debug_println(" boiling time left: " + String(timeleft / 1000));
            lcd.setCursor(10, 1);
            lcd.print(String(timeleft / 1000) + "s");
            previousMachineMillis = currentMillis;
            if (timeleft <= 0)
            {
                // after
                lcd.setCursor(10, 1);
                lcd.print("DONE");
                toggleFlagStirring = 0; // off stirring
                state = 12;
            }
        }

        else if (state == 12 && currentMillis - previousMachineMillis > 1500)
        {
            // toggle off stirrer, pilinuts dispensing

            static int runOnce = 0;
            if (runOnce == 0)
            {
                lcd.clear();
                lcd.setCursor(0, 0); // Set cursor position
                lcd.print("Nuts Hopper");
                lcd.setCursor(10, 1);
                lcd.print("grams");
                runOnce = 1;
            }

            toggleFlagStirring = 0;
            val_nutsHopper = 1600; // open hopper
            previousMachineMillis = currentMillis;
            int value = get_Loadcells();
            debug_println("pili nuts value: " + value);
            if (value > 500)
            {
                // state = 13;
            }
            lcd.setCursor(0, 1);
            lcd.print(String(value));
            state = 13;
        }

        else if (state == 13 && currentMillis - previousMachineMillis > 500)
        {
            // close pili nuts hopper, drop it to pan

            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Nuts hopper: ");
            lcd.setCursor(0, 1);
            lcd.print("   Closed");

            val_nutsHopper = 1500; // close hopper
            debug_println("Closing nuts hopper");
            previousMachineMillis = currentMillis;
            state = 14;
        }

        else if (state == 14 && currentMillis - previousMachineMillis > 5000)
        {
            // the servo should be intialized in resting position
            // the servo will go down now, dispensing nuts to cooking pan
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Nuts to pan:");
            lcd.setCursor(0, 1);
            lcd.print("  Dispensing");

            val_arm1LDcell = 756;
            val_arm2LDcell = 756;
            debug_println("from hopper (nuts) to pan");
            previousMachineMillis = currentMillis;
            state = 15;
        }

        else if (state == 15 && currentMillis - previousMachineMillis > 5000)
        {
            // servo arm to resting position
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Dispenser to");
            lcd.setCursor(0, 1);
            lcd.print("Resting");

            val_arm1LDcell = SERVOMIN;
            val_arm2LDcell = SERVOMIN;
            debug_println("LD cell arm back to resting pos");
            previousMachineMillis = currentMillis;
            state = 16;
        }

        else if (state == 16 && currentMillis - previousMachineMillis > 5000)
        {
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Boiling 3min");
            lcd.setCursor(0, 1);
            lcd.print("Timeleft: ");

            debug_println("resseting clock for boiling again");
            bookmarkTime = 0;
            timeleft = 0;
            bookmarkTime = millis() + 18000; // set clocktime for 3 minutes. or 180000
            previousMachineMillis = currentMillis;
            state = 17;
        }

        else if (state == 17 && currentMillis - previousMachineMillis > 50)
        {
            // during
            timeleft = bookmarkTime - millis();
            debug_println("boiling time left: " + String(timeleft / 1000));
            toggleFlagStirring = 1;
            lcd.setCursor(10, 1);
            lcd.print(String(timeleft / 1000) + "s");
            previousMachineMillis = currentMillis;
            if (timeleft <= 0)
            {
                // after
                lcd.setCursor(10, 1);
                lcd.print("DONE");
                toggleFlagStirring = 0; // off stirring
                state = 18;
            }
        }

        else if (state == 18 && currentMillis - previousMachineMillis > 2000)
        {

            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("stirrer up");
            lcd.setCursor(0, 1);
            lcd.print("Timeleft: ");
            // digitalWrite(IndCooker_ONOFF, HIGH);

            // make the stirrer up again
            debug_println("resseting clock for boiling again");
            bookmarkTime = 0;
            timeleft = 0;
            stepFlagInduction_Low = 1;      // Set the temperature to low or 180 degrees
            bookmarkTime = millis() + 1000; // set clocktime for 1min 11 secc minutes. or 71000
            previousMachineMillis = currentMillis;
            state = 19;
        }

        else if (state == 19 && currentMillis - previousMachineMillis > 50)
        {
            // make the stirrer up again
            digitalWrite(stirrer_down, LOW);
            digitalWrite(stirrer_up, HIGH);

            timeleft = bookmarkTime - millis();
            debug_println("stirrer time left: " + String(timeleft / 1000));
            toggleFlagStirring = 1; // on stirring
            lcd.setCursor(10, 1);
            lcd.print(String(timeleft / 1000) + "s");
            previousMachineMillis = currentMillis;
            if (timeleft <= 0)
            {
                // after
                lcd.setCursor(10, 1);
                lcd.print("DONE");
                toggleFlagStirring = 0;        // off stirring
                digitalWrite(stirrer_up, LOW); // off yung up
                // digitalWrite(IndCooker_ONOFF, LOW);
                state = 20;
            }
        }

        else if (state == 20 && currentMillis - previousMachineMillis > 1000)
        {
            // rotate the pan to the hopper below
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Moving pan");
            lcd.setCursor(0, 1);
            lcd.print("Rotating...");

            kawaliStepper.move(200); // dispensing position
            kawaliStepper.setSpeed(100);
            debug_println("pan is dispensing the cooked nuts");
            previousMachineMillis = currentMillis;
            state = 21;
        }

        else if (state == 21 && currentMillis - previousMachineMillis > 5000)
        {
            // rotate the pan to its resting position.
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Returning Pan");
            lcd.setCursor(0, 1);
            lcd.print("Resting...");

            kawaliStepper.move(-200); // resting position
            kawaliStepper.setSpeed(100);
            debug_println("finished for now");
            previousMachineMillis = currentMillis;
            state = 22;
        }

        else if (state == 22 && currentMillis - previousMachineMillis > 1000)
        {
            lcd.clear();
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Instruction sent");
            lcd.setCursor(0, 1);
            lcd.print("to Nano");

            Wire.beginTransmission(9); // Set slave address (0x70) or (9)
            Wire.write(startNano);     // Send character
            Wire.endTransmission();

            debug_println("Instruction sent to Nano");
            previousMachineMillis = currentMillis;
            state = 23;
        }

        else if (state == 23 && currentMillis - previousMachineMillis > 500)
        {
            debug_println("reseting states, done");
            previousMachineMillis = currentMillis;
            state = 0;
        }

        // *** SEQUENCES ***

        // stirring arm cc & cw
        if (toggleFlagStirring == 1 && stepFlagStirring == 0 && currentMillis - previousStirringMillis > 7500)
        {
            // set stirrer counterclockwise for 7.5 sec
            analogWrite(stirrer_speed, 75); // set speed enA
            digitalWrite(stirrer_cc, HIGH);
            digitalWrite(stirrer_cw, LOW);
            debug_println("stiriing to counterclockwise");
            previousStirringMillis = currentMillis;
            stepFlagStirring = 1;
        }

        else if (toggleFlagStirring == 1 && stepFlagStirring == 1 && currentMillis - previousStirringMillis > 1000)
        {
            // 1 second buffer
            previousStirringMillis = currentMillis;
            stepFlagStirring = 2;
        }
        else if (toggleFlagStirring == 1 && stepFlagStirring == 2 && currentMillis - previousStirringMillis > 7500)
        {
            // set stirrer clockwise for 7.5 sec
            analogWrite(stirrer_speed, 15); // set speed enA
            digitalWrite(stirrer_cc, LOW);
            digitalWrite(stirrer_cw, HIGH);
            debug_println("stiriing to clockwise");
            previousStirringMillis = currentMillis;
            stepFlagStirring = 3;
        }
        else if (toggleFlagStirring == 1 && stepFlagStirring == 3 && currentMillis - previousStirringMillis > 1000)
        {
            // 1 second buffer
            previousStirringMillis = currentMillis;
            stepFlagStirring = 0;
        }

        // Induction seq
        if (stepFlagInduction_Med == 1)
        {
            // turn on induction first, send a pulse thru a relay
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("set to med temp");
            digitalWrite(IndCooker_ONOFF, HIGH);
            stepFlagInduction_Med = 2;
            previousInductionMedMillis = currentMillis;
        }

        else if (stepFlagInduction_Med == 2 && currentMillis - previousInductionMedMillis > 500)
        {
            digitalWrite(IndCooker_ONOFF, LOW);
            stepFlagInduction_Med = 3;
            previousInductionMedMillis = currentMillis;
        }

        else if (stepFlagInduction_Med == 3 && currentMillis - previousInductionMedMillis > 250)
        {
            // simulate press function button 3 times
            previousInductionMedMillis = currentMillis;
            digitalWrite(IndCooker_FUNC, HIGH);
            stepFlagInduction_Med = 4;
        }

        else if (stepFlagInduction_Med == 4 && currentMillis - previousInductionMedMillis > 250)
        {
            previousInductionMedMillis = currentMillis;
            digitalWrite(IndCooker_FUNC, LOW);
            stepFlagInduction_Med = 5;
        }

        else if (stepFlagInduction_Med == 5 && currentMillis - previousInductionMedMillis > 250)
        {
            // simulate press function button 3 times
            previousInductionMedMillis = currentMillis;
            digitalWrite(IndCooker_FUNC, HIGH);
            stepFlagInduction_Med = 6;
        }

        else if (stepFlagInduction_Med == 6 && currentMillis - previousInductionMedMillis > 250)
        {
            previousInductionMedMillis = currentMillis;
            digitalWrite(IndCooker_FUNC, LOW);
            stepFlagInduction_Med = 7;
        }

        else if (stepFlagInduction_Med == 7 && currentMillis - previousInductionMedMillis > 250)
        {
            // simulate press function button 3 times
            previousInductionMedMillis = currentMillis;
            digitalWrite(IndCooker_FUNC, HIGH);
            stepFlagInduction_Med = 8;
        }

        else if (stepFlagInduction_Med == 8 && currentMillis - previousInductionMedMillis > 250)
        {
            previousInductionMedMillis = currentMillis;
            digitalWrite(IndCooker_FUNC, LOW);
            debug_println("setting to adjustable temp");
            stepFlagInduction_Med = 9;
        }

        else if (stepFlagInduction_Med == 9 && currentMillis - previousInductionMedMillis > 250)
        {
            previousInductionMedMillis = currentMillis;
            digitalWrite(IndCooker_negaTEMP, LOW);
            stepFlagInduction_Med = 10;
        }

        else if (stepFlagInduction_Med == 10 && currentMillis - previousInductionMedMillis > 250)
        {
            previousInductionMedMillis = currentMillis;
            digitalWrite(IndCooker_negaTEMP, HIGH);
            stepFlagInduction_Med = 11;
        }

        else if (stepFlagInduction_Med == 11 && currentMillis - previousInductionMedMillis > 250)
        {
            previousInductionMedMillis = currentMillis;
            digitalWrite(IndCooker_negaTEMP, LOW);
            stepFlagInduction_Med = 12;
        }

        else if (stepFlagInduction_Med == 12 && currentMillis - previousInductionMedMillis > 250)
        {
            previousInductionMedMillis = currentMillis;
            digitalWrite(IndCooker_negaTEMP, HIGH);
            stepFlagInduction_Med = 13;
        }

        else if (stepFlagInduction_Med == 13 && currentMillis - previousInductionMedMillis > 250)
        {
            previousInductionMedMillis = currentMillis;
            digitalWrite(IndCooker_negaTEMP, LOW);
            stepFlagInduction_Med = 14;
        }

        else if (stepFlagInduction_Med == 14 && currentMillis - previousInductionMedMillis > 250)
        {
            previousInductionMedMillis = currentMillis;
            digitalWrite(IndCooker_negaTEMP, HIGH);
            stepFlagInduction_Med = 15;
        }

        else if (stepFlagInduction_Med == 15 && currentMillis - previousInductionMedMillis > 250)
        {
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Boiling        ");
            debug_println("finished setting to (MED) 180 degrees C");
            digitalWrite(IndCooker_negaTEMP, LOW);
            previousInductionMedMillis = currentMillis;
            stepFlagInduction_Med = 0;
        }

        // setting to low temp seq
        if (stepFlagInduction_Low == 1)
        {
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("set to low temp");
            digitalWrite(IndCooker_negaTEMP, LOW);
            previousInductionLowMillis = currentMillis;
            stepFlagInduction_Low = 2;
        }

        else if (stepFlagInduction_Low == 2 && currentMillis - previousInductionLowMillis > 500)
        {
            digitalWrite(IndCooker_negaTEMP, HIGH);
            previousInductionLowMillis = currentMillis;
            stepFlagInduction_Low = 3;
        }

        else if (stepFlagInduction_Low == 3 && currentMillis - previousInductionLowMillis > 500)
        {
            digitalWrite(IndCooker_negaTEMP, LOW);
            previousInductionLowMillis = currentMillis;
            stepFlagInduction_Low = 4;
        }

        else if (stepFlagInduction_Low == 4 && currentMillis - previousInductionLowMillis > 500)
        {
            digitalWrite(IndCooker_negaTEMP, HIGH);
            previousInductionLowMillis = currentMillis;
            stepFlagInduction_Low = 5;
        }

        else if (stepFlagInduction_Low == 5 && currentMillis - previousInductionLowMillis > 500)
        {
            digitalWrite(IndCooker_negaTEMP, LOW);
            previousInductionLowMillis = currentMillis;
            stepFlagInduction_Low = 6;
        }

        else if (stepFlagInduction_Low == 6 && currentMillis - previousInductionLowMillis > 500)
        {
            lcd.setCursor(0, 0); // Set cursor position
            lcd.print("Boiling        ");
            digitalWrite(IndCooker_negaTEMP, HIGH);
            previousInductionLowMillis = currentMillis;
            stepFlagInduction_Low = 0;
        }
    }

    // filter values
    val_sugarHopperFiltered = filter(val_sugarHopper, val_sugarHopperFiltered, 20);
    val_nutsHopperFiltered = filter(val_nutsHopper, val_sugarHopperFiltered, 20);
    val_arm1LDcellFiltered = filter(val_arm1LDcell, val_arm1LDcellFiltered, 20);
    val_arm2LDcellFiltered = filter(val_arm2LDcell, val_arm2LDcellFiltered, 20);

    // write filter to servo with trim
    pwm.writeMicroseconds(sugarHopper, val_sugarHopperFiltered);
    pwm.writeMicroseconds(nutsHopper, val_nutsHopperFiltered);
    pwm.writeMicroseconds(arm1LDcell, val_arm1LDcellFiltered);
    pwm.writeMicroseconds(arm2LDcell, val_arm2LDcellFiltered);
}

int get_Loadcells()
{
    int value;
    // debug_print(count);
    // for (int i = 0; i < 2; i++)
    // {
    //   value = scales[i].read_average(10);
    //   debug_print("\t");
    //   debug_print(String(value) + calib[i]);
    // }
    value = scales[0].read_average(10);
    debug_print("\t");
    debug_print(value);
    debug_print("this is the value: " + String(value));
    return value;
}

int get_Loadcell2()
{
    int value;
    // debug_print(count);
    // for (int i = 0; i < 2; i++)
    // {
    //   value = scales[i].read_average(10);
    //   debug_print("\t");
    //   debug_print(String(value) + calib[i]);
    // }
    value = scales[1].read_average(10);
    debug_print("\t");
    debug_print("this is the value: " + String(value));
    return value;
}

void waterLevel()
{
    static unsigned long lastStateTime = millis();

    switch (Waterstate)
    {
    case IDLE:
        // Wait for a certain time before starting the process
        if (millis() - lastStateTime >= 1000)
        { // Wait for 1 second
            state = POWER_ON;
            lastStateTime = millis();
        }
        break;

    case POWER_ON:
        digitalWrite(WATER_LEVEL_POW, HIGH); // Turn the sensor ON
        state = READ_SENSOR;
        lastStateTime = millis();
        break;

    case READ_SENSOR:
        if (millis() - lastStateTime >= 20)
        {                                            // Wait for 10 milliseconds
            water_level = analogRead(WATER_LEVEL_S); // Read the analog value from sensor
            Serial.println("The water level is: " + String(water_level));
            state = POWER_OFF;
            lastStateTime = millis();
        }
        break;

    case POWER_OFF:
        digitalWrite(WATER_LEVEL_POW, LOW); // Turn the sensor OFF
        state = IDLE;
        lastStateTime = millis();
        break;
    }
}

int machine_buttons()
{
    // Read the state of the buttons
    int buttonSetState = digitalRead(START_PB);
    int buttonResetState = digitalRead(STOP_PB);

    // Check if the Set (green) button is pressed
    if (buttonSetState == LOW && !buttonStartPressed)
    {
        unsigned long currentTime = millis();
        if (currentTime - prevTime_StartPB > debounceDelay_PB)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            machineOnStatus = true;
            debug_println("MACHINE IS ON : ");
            debug_println(machineOnStatus);
            buttonStartPressed = true;
            state = 0;
            debug_println(state);
            // machine_statusnow = presetup_process;
        }
        prevTime_StartPB = currentTime;
    }
    else if (buttonSetState == HIGH)
    {
        buttonStartPressed = false;
    }

    // Check if the Reset (red) button is pressed
    if (buttonResetState == LOW && !buttonStopPressed)
    {
        unsigned long currentTime = millis();
        if (currentTime - prevTime_StopPB > debounceDelay_PB)
        {
            digitalWrite(LED_BUILTIN, LOW);
            debug_println("MACHINE IS OFF :");
            machineOnStatus = false;
            debug_println(machineOnStatus);
            buttonStopPressed = true;
            state = -1;
            debug_println(state);
        }
        prevTime_StopPB = currentTime;
    }
    else if (buttonResetState == HIGH)
    {
        buttonStopPressed = false;
        // waterPump_flag = false;
        // sugarHopper_flag = false;
        // loadCell_servos_flag = false;
        // panRotation_flag = false;
    }

    return state;
}

void water_dispensing()
{
    if (!waterPump_flag)
    { // Check if water pump hasn't been activated yet
        debug_println("flowing Waterpump");
        digitalWrite(WaterPump, HIGH);
        delay(5200); // Wait for 5200 milliseconds (5.2 seconds)
        digitalWrite(WaterPump, LOW);
        debug_println("Done Waterpump");
        waterPump_flag = true; // Set flag to indicate water pump has run
    }
}

void sugarHopper_servo()
{
    if (!sugarHopper_flag)
    {
        debug_println("opening hopper");
        for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
        {
            pwm.setPWM(sugarHopper, 0, pulselen);
        }
        int value = get_Loadcells();

        if (value > 500)
        {
            // do something
            debug_print("closing hoppper");
            for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
            {
                pwm.setPWM(sugarHopper, 0, pulselen);
            }
            sugarHopper_flag = true;
        }
        debug_println(value);
    }
}

void loadCell_servos()
{
    delay(1000);
    if (!loadCell_servos_flag)
    {
        debug_println("opening LDCELL ARM");
        for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
        {
            pwm.setPWM(arm1LDcell, 0, pulselen);
            pwm.setPWM(arm2LDcell, 0, pulselen);
        }
        delay(2000);
        debug_println("taktak LDCELL ARM");
        pwm.setPWM(arm1LDcell, 0, SERVOMAX / 2);
        pwm.setPWM(arm2LDcell, 0, SERVOMAX / 2);
        for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
        {
            pwm.setPWM(arm1LDcell, 0, pulselen);
            pwm.setPWM(arm2LDcell, 0, pulselen);
        }
        delay(1000);
        debug_println("default position LDCELL ARM");
        for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
        {
            pwm.setPWM(arm1LDcell, 0, pulselen);
            pwm.setPWM(arm2LDcell, 0, pulselen);
        }
        delay(1000);
        loadCell_servos_flag = true;
    }
}

void panRotation()
{
    if (!panRotation_flag)
    {
        // Move the kawaliStepper motor to the target position
        kawaliStepper.moveTo(targetPosition);
        while (kawaliStepper.distanceToGo() != 0)
        {
            kawaliStepper.run();
        }

        // Wait for 4 seconds
        delay(4000);

        // Move the kawaliStepper motor back to the default position
        kawaliStepper.moveTo(defaultPosition);
        while (kawaliStepper.distanceToGo() != 0)
        {
            kawaliStepper.run();
        }
        panRotation_flag = true;
    }
}
// int dispensing_specificprocess(int dispensing_switch)
// {
//   int value;
//   switch (dispensing_switch)
//   {
//   case 1:
//     // do something
//     dispensing_switch = 2;
//     debug_println("opening Waterpump");
//     digitalWrite(WaterPump, HIGH);
//     delay(4000);
//     digitalWrite(WaterPump, LOW);
//     break;

//   case 2:
//     // open hopper
//     debug_println("opening hopper");
//     for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
//     {
//       pwm.setPWM(sugarHopper, 0, pulselen);
//     }
//     value = get_Loadcells();

//     if (value > 500)
//     {
//       // do something
//       debug_print("closing hoppper");
//       for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
//       {
//         pwm.setPWM(sugarHopper, 0, pulselen);
//       }
//     }

//     debug_println(String(value));
//     dispensing_switch = 3;
//     break;

//   case 3:
//     debug_println("doing something3");
//     value = 0;
//     dispensing_switch = 0;
//     dispensing_flag = false;
//     break;

//   default:
//     debug_println("doing something default");
//     dispensing_switch = 0;
//     dispensing_flag = false;
//     break;
//   }
//   return dispensing_switch;
//   return dispensing_flag = false;
// }
