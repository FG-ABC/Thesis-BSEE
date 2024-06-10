//
//    FILE: Thesis-BSEE.cpp
//  AUTHOR: James Kyle S. Balolong
// PURPOSE: Arduino Mega for dispensing, cooking and cooling cooked pili nuts
//     URL: https://github.com/Jamsekun/Thesis-BSEE/blob/main/src/Thesis-BSEE.cpp
// Simulator: WOKWI, PlatformIO
// CODE INSPIRATION: multitasking code and state machine algorithm is inspired by https://github.com/XRobots/Furby/blob/main/Code/002/002.ino

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

//-------------address------------------- (IDK)
/*
0x27: LCD
0x40: Servo driver
0x70: nano
*/

//----------USER INTERFACE-------------------
unsigned int reqPilinutsLevel = 15;
int reqTubigLevel = 200;
unsigned int reqSugarLevel = 15; // required sugar level

//---------------------------------

//-----------I2C Communication----------------- (IDK)
char startNano = 's';

//-----------TIME MILLIS ----------------- (IDK)
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
// FG Set state here
int state = 0; // stage of automation //original position and number is -1
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
const uint8_t dataPin[2] = {22, 24}; // isa lang ginagamit ko dito sa HX711 kahit naka array, 22 nlng gamitin
const uint8_t clockPin = 23;
// TODO you need to adjust to your calibrate numbers
float calib[2] = {320, 559};
uint32_t count = 0;
//-------------------------------------------------
//-------FUNCTIONS INITIALIZE -------------
// unsigned int UltraS1();
// unsigned int UltraS2();
// void waterLevel();
// int machine_buttons();
// int dispensing_specificprocess(int dispensing_switch);
// void water_dispensing();
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
// tinanggal na to nila:
int IndCooker_FUNC = 8;
int IndCooker_negaTEMP = 9;
int IndCooker_posiTEMP = 10;
//--------------------------------

//---------Stirrer Arm---------------
const int stirrer_up = 30;   // 1 min and 11 secs pataas, test polarity of the motor first
const int stirrer_down = 31; // 1 min and 5 secs pababa, test polarity of the motor first
const int stirrer_cc = 34;   // in1
const int stirrer_cw = 35;   // in2
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
const int IL_NoSugar = 4;
const int IL_NoNuts = 2;
const int IL_NoWater = 3;
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
const int stpprPANen = 18;
const int targetPosition = 1000;
const int defaultPosition = 0;
AccelStepper kawaliStepper(AccelStepper::DRIVER, stpprPANstep, stpprPANdir);
//-----------------------------

// SCL - 20
// SDA - 21

// Eto ata yung Main
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
    //-----------------------------------

    //-----------stirrer arm------------------
    // 1st L298N- 12 Volts
    pinMode(stirrer_up, OUTPUT);   // in1
    pinMode(stirrer_down, OUTPUT); // in2

    // -==tiamng
    pinMode(stpprPANstep, OUTPUT);

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
    kawaliStepper.setEnablePin(18);
    kawaliStepper.setMaxSpeed(200);
    kawaliStepper.setAcceleration(100);
    kawaliStepper.setSpeed(200);
    kawaliStepper.enableOutputs();
    // kawaliStepper.move(0); // kawaliStepper.moveTo(200) i changed this since no initial position indicator in actual
    //---------------------------
}

void loop()
{
    currentMillis = millis();
    if (currentMillis - previousMillis >= 10) // If executes if 10ms has passed since last loop
    {
        // start event
        previousMillis = currentMillis;
        int buttonSetState = digitalRead(START_PB);
        int buttonResetState = digitalRead(STOP_PB);

        // start button logic

        // State 1 if start button is pressed
        if (buttonSetState == LOW && buttonStartPressed == 0)
        {
            // debug_println("nag-start ako");
            state = 1;
            buttonStartPressed = 1;
        }

        else if (buttonSetState == HIGH && buttonStartPressed == 1)
        {
            delay(250);
            debug_println("nag-start ako");
            buttonStartPressed = 0;
        }

        // stop button logic
        // State -1 if stop button is pressed
        if (buttonResetState == LOW && buttonStopPressed == 0)
        {
            // set all to resting or initial position
            state = -1;
            buttonStopPressed = 1;
        }

        else if (buttonResetState == HIGH && buttonStopPressed == 1)
        {
            delay(250);
            debug_println("nag-stop ako");
            buttonStopPressed = 0;
        }

        // State - 1, stop process
        if (state == -1 && machineOnStatus == false)
        {
            debug_println("State -1");
            // stop process is ran once.
            digitalWrite(IL_NoSugar, HIGH);
            digitalWrite(IL_NoWater, HIGH);
            digitalWrite(IL_NoNuts, HIGH);
            digitalWrite(IL_Running, HIGH);
            digitalWrite(IL_Stopped, LOW);
            bookmarkTime = 0; // reset timers to 0
            timeleft = 0;
            stepFlagInduction_Low = 0; // reset to initial position
            stepFlagInduction_Med = 0;
            toggleFlagStirring = 0;

            // state = 0;
            previousMachineMillis = currentMillis;
        }

        // State 0, presetup process
        else if (state == 0 && machineOnStatus == false)
        {
            debug_println("State 0");
            digitalWrite(stirrer_up, LOW);
            digitalWrite(stirrer_down, HIGH);
            previousMachineMillis = currentMillis;
            delay(5000);
        }

        // State 1, Running - HIGH, Stopped - LOW
        else if (state == 1)
        {
            debug_println("State 1");
            digitalWrite(IL_Running, LOW);
            digitalWrite(IL_Stopped, HIGH);
            water_level = analogRead(WATER_LEVEL_S);
            // debug_println(water_level);
            distances = HCSR04.measureDistanceCm();
            bool tuloy = true;
            if (distances[0] >= reqSugarLevel)
            {
                digitalWrite(IL_NoSugar, HIGH);
                tuloy = false;
            }
            else
            {
                digitalWrite(IL_NoSugar, LOW);
            }

            if (distances[1] >= reqPilinutsLevel)
            {
                digitalWrite(IL_NoNuts, HIGH);
                tuloy = false;
            }
            else
            {
                digitalWrite(IL_NoNuts, LOW);
            }

            if (water_level <= 200)
            {
                digitalWrite(IL_NoWater, HIGH);
                tuloy = false;
            }
            else
            {
                digitalWrite(IL_NoWater, LOW);
            }
            if (tuloy)
            {
                debug_println("tuloy");
                // state = 2;
            }

            previousMachineMillis = currentMillis;
        }

        // State 2, Turn water pump on
        else if (state == 2 && currentMillis - previousMachineMillis > 1000)
        {
            debug_println("State 2");
            digitalWrite(WaterPump, HIGH);
            debug_println("WaterPump is open");
            previousMachineMillis = currentMillis;
            state = 3;
        }

        // State 3, Turn water pump off after 2s
        else if (state == 3 && currentMillis - previousMachineMillis > 4750)
        {
            debug_println("State 3");
            digitalWrite(WaterPump, LOW);
            debug_println("WaterPump is closed");
            previousMachineMillis = currentMillis;
            state = 4;
        }

        // State 4, Open sugar hopper servos after 1.5s
        else if (state == 4 && currentMillis - previousMachineMillis > 1500)
        {
            // open sugar hopper servos
            debug_println("State 4");
            val_sugarHopper = 1500; // means open
            previousMachineMillis = currentMillis;
            int value2 = get_Loadcells();
            debug_println("opening sugar hopper, value: " + String(value2));
            if (value2 > 500)
            {
                state = 5;
            }
        }

        // State 5, Close sugar hopper after 5s
        else if (state == 5 && currentMillis - previousMachineMillis > 5000)
        {
            debug_println("State 5");
            val_sugarHopper = 1600; // close value
            debug_println("closed sugar hopper");
            previousMachineMillis = currentMillis;
            state = 6;
        }

        // State 6, Open load cell servos after 5s
        else if (state == 6 && currentMillis - previousMachineMillis > 5000)
        {
            // the servo should be intialized in resting position
            // the servo will go down now, dispensing sugar to cooking pan
            debug_println("State 6");
            val_arm1LDcell = 1700;
            val_arm2LDcell = 1700;
            debug_println("from hopper (sugar) to pan");
            previousMachineMillis = currentMillis;
            state = 7;
        }

        // State 7, Close load cell servos after 5s
        else if (state == 7 && currentMillis - previousMachineMillis > 5000)
        {
            // arm1LDcell go to resting pos, turn on induction cooker and set it to normal temp multitask
            debug_println("State 7");
            val_arm1LDcell = 1500;
            val_arm2LDcell = 1500;

            debug_println("turn on induction cooker");
            stepFlagInduction_Med = 1; // start induction cooker sequence
            previousMachineMillis = currentMillis;
            state = 8;
        }

        // State 8, Turn off induction cooker after 5s and put arm down. TODO: Adjust time
        else if (state == 8 && currentMillis - previousMachineMillis > 5000)
        {
            // let stirrer arm go down, adjust the time on testing, initialize it to be upward
            debug_println("State 8");
            digitalWrite(stirrer_down, HIGH);
            digitalWrite(stirrer_up, LOW);
            bookmarkTime = millis() + 1000; // set clock timer for 1min 05 sec or 65000
            previousMachineMillis = currentMillis;
            state = 9;
        }

        // State 9, Stirring arm sequence
        else if (state == 9 && currentMillis - previousMachineMillis > 50)
        {
            debug_println("State 9");
            digitalWrite(stirrer_cc, HIGH);
            digitalWrite(stirrer_cw, LOW);
            timeleft = bookmarkTime - millis();
            debug_println("down timeleft: " + String(timeleft / 1000));
            if (timeleft <= 0)
            {
                toggleFlagStirring = 1;
                digitalWrite(stirrer_down, LOW);
                previousMachineMillis = currentMillis;
                state = 10;
            }
            previousMachineMillis = currentMillis;
        }

        // State 10, Boiling
        else if (state == 10 && currentMillis - previousMachineMillis > 1000)
        {
            debug_println("State 10");
            debug_println("resseting clock");
            bookmarkTime = 0;
            timeleft = 0;
            bookmarkTime = millis() + 12000; // set clocktime for 2 minutes. or 120000
            toggleFlagStirring = 1;
            previousMachineMillis = currentMillis;
            state = 11;
        }

        // State 11, Boiling time left
        else if (state == 11 && currentMillis - previousMachineMillis > 50)
        {
            debug_println("State 11");
            timeleft = bookmarkTime - millis();
            debug_println(" boiling time left: " + String(timeleft / 1000));
            previousMachineMillis = currentMillis;
            if (timeleft <= 0)
            {
                toggleFlagStirring = 0; // off stirring
                state = 12;
            }
        }

        // State 12, Open nuts hopper after 1.5s
        else if (state == 12 && currentMillis - previousMachineMillis > 1500)
        {
            debug_println("State 12");
            toggleFlagStirring = 0;
            val_nutsHopper = 1600; // open hopper
            previousMachineMillis = currentMillis;
            int value = get_Loadcells();
            debug_println("pili nuts value: " + value);
            if (value > 500)
            {
                state = 13;
            }
        }

        // State 13, Close nuts hopper after 0.5s
        else if (state == 13 && currentMillis - previousMachineMillis > 500)
        {
            // close pili nuts hopper, drop it to pan
            debug_println("State 13");
            val_nutsHopper = 1500; // close hopper
            debug_println("Closing nuts hopper");
            previousMachineMillis = currentMillis;
            state = 14;
        }

        // State 14, Open load cell servos after 5s
        else if (state == 14 && currentMillis - previousMachineMillis > 5000)
        {
            // the servo should be intialized in resting position
            // the servo will go down now, dispensing nuts to cooking pan
            debug_println("State 14");
            val_arm1LDcell = 756;
            val_arm2LDcell = 756;
            debug_println("from hopper (nuts) to pan");
            previousMachineMillis = currentMillis;
            state = 15;
        }

        else if (state == 15 && currentMillis - previousMachineMillis > 5000)
        {
            // servo arm to resting position
            debug_println("State 15");
            val_arm1LDcell = SERVOMIN;
            val_arm2LDcell = SERVOMIN;
            debug_println("LD cell arm back to resting pos");
            previousMachineMillis = currentMillis;
            state = 16;
        }

        else if (state == 16 && currentMillis - previousMachineMillis > 5000)
        {
            debug_println("State 16");
            debug_println("resseting clock for boiling again");
            bookmarkTime = 0;
            timeleft = 0;
            bookmarkTime = millis() + 18000; // set clocktime for 3 minutes. or 180000
            previousMachineMillis = currentMillis;
            state = 17;
        }

        else if (state == 17 && currentMillis - previousMachineMillis > 50)
        {
            debug_println("State 17");
            // during
            timeleft = bookmarkTime - millis();
            debug_println("boiling time left: " + String(timeleft / 1000));
            toggleFlagStirring = 1;
            previousMachineMillis = currentMillis;
            if (timeleft <= 0)
            {
                toggleFlagStirring = 0;
                state = 18;
            }
        }

        else if (state == 18 && currentMillis - previousMachineMillis > 2000)
        {
            debug_println("State 18");
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
            debug_println("State 19");
            // make the stirrer up again
            digitalWrite(stirrer_down, LOW);
            digitalWrite(stirrer_up, HIGH);

            timeleft = bookmarkTime - millis();
            debug_println("stirrer time left: " + String(timeleft / 1000));
            toggleFlagStirring = 1; // on stirring
            previousMachineMillis = currentMillis;
            if (timeleft <= 0)
            {

                toggleFlagStirring = 0;        // off stirring
                digitalWrite(stirrer_up, LOW); // off yung up
                state = 20;
            }
        }

        else if (state == 20 && currentMillis - previousMachineMillis > 1000)
        {
            debug_println("State 20");
            // rotate the pan to the hopper below
            debug_println("rotating pan to hopper below");
            kawaliStepper.setSpeed(200);
            kawaliStepper.move(200);

            debug_println(boolean(kawaliStepper.isRunning()));
            debug_println(kawaliStepper.distanceToGo());
            debug_println(kawaliStepper.speed());
            previousMachineMillis = currentMillis;
            machineOnStatus = false;
            // state = ;
        }

        else if (state == 21 && currentMillis - previousMachineMillis > 5000)
        {

            debug_println("State 21");
            // rotate the pan to its resting position.
            kawaliStepper.move(-200); // resting position
            kawaliStepper.setSpeed(100);
            debug_println("finished for now");
            previousMachineMillis = currentMillis;
            state = 22;
        }

        else if (state == 22 && currentMillis - previousMachineMillis > 1000)
        {
            // send instruction to nano to reset all states
            debug_println("State 22");
            Wire.beginTransmission(9); // Set slave address (0x70) or (9)
            Wire.write(startNano);     // Send character
            Wire.endTransmission();

            debug_println("Instruction sent to Nano");
            previousMachineMillis = currentMillis;
            state = 23;
        }

        else if (state == 23 && currentMillis - previousMachineMillis > 500)
        {
            debug_println("State 23");
            debug_println("reseting states, done");
            previousMachineMillis = currentMillis;
            state = 0;
        }

        // *** SEQUENCES ***

        // stirring arm cc & cw
        if (toggleFlagStirring == 1 && stepFlagStirring == 0 && currentMillis - previousStirringMillis > 7500)
        {
            debug_println("Sequence 1");
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
            debug_println("Sequence 2");
            previousStirringMillis = currentMillis;
            stepFlagStirring = 2;
        }
        else if (toggleFlagStirring == 1 && stepFlagStirring == 2 && currentMillis - previousStirringMillis > 7500)
        {
            // set stirrer clockwise for 7.5 sec
            debug_println("Sequence 3");
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
            debug_println("Sequence 4");
            previousStirringMillis = currentMillis;
            stepFlagStirring = 0;
        }

        // Induction seq
        if (stepFlagInduction_Med == 1)
        {
            // turn on induction first, send a pulse thru a relay
            // lcd.setCursor(0, 0); // Set cursor position
            // lcd.print("set to med temp");
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
            // lcd.setCursor(0, 0); // Set cursor position
            // lcd.print("Boiling        ");
            debug_println("finished setting to (MED) 180 degrees C");
            digitalWrite(IndCooker_negaTEMP, LOW);
            previousInductionMedMillis = currentMillis;
            stepFlagInduction_Med = 0;
        }

        // setting to low temp seq
        if (stepFlagInduction_Low == 1)
        {
            // lcd.setCursor(0, 0); // Set cursor position
            // lcd.print("set to low temp");
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
            // lcd.setCursor(0, 0); // Set cursor position
            // lcd.print("Boiling        ");
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
    // Delay function
    // delay(5000);
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

// void waterLevel()
// {
//     static unsigned long lastStateTime = millis();

//     switch (Waterstate)
//     {
//     case IDLE:
//         // Wait for a certain time before starting the process
//         if (millis() - lastStateTime >= 1000)
//         { // Wait for 1 second
//             state = POWER_ON;
//             lastStateTime = millis();
//         }
//         break;

//     case POWER_ON:
//         digitalWrite(WATER_LEVEL_POW, HIGH); // Turn the sensor ON
//         state = READ_SENSOR;
//         lastStateTime = millis();
//         break;

//     case READ_SENSOR:
//         if (millis() - lastStateTime >= 20)
//         {                                            // Wait for 10 milliseconds
//             water_level = analogRead(WATER_LEVEL_S); // Read the analog value from sensor
//             Serial.println("The water level is: " + String(water_level));
//             state = POWER_OFF;
//             lastStateTime = millis();
//         }
//         break;

//     case POWER_OFF:
//         digitalWrite(WATER_LEVEL_POW, LOW); // Turn the sensor OFF
//         state = IDLE;
//         lastStateTime = millis();
//         break;
//     }
// }

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

// void water_dispensing()
// {
//     if (!waterPump_flag)
//     { // Check if water pump hasn't been activated yet
//         debug_println("flowing Waterpump");
//         digitalWrite(WaterPump, HIGH);
//         delay(5200); // Wait for 5200 milliseconds (5.2 seconds)
//         digitalWrite(WaterPump, LOW);
//         debug_println("Done Waterpump");
//         waterPump_flag = true; // Set flag to indicate water pump has run
//     }
// }

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
