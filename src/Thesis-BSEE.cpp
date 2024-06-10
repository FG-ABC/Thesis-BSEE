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
#include <Servo.h>

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
//------------------------sat

//--------------automation-------------
// FG Set state here
int state = 0; // stage of automation //original position and number is -1
//-------------------------------

//----------Servo Params------------------
#define servoPin1 10
#define servoPin2 11
#define servoPin3 12
#define servoPin4 13

Servo HopperServo;
Servo PiliServo;
Servo StoveServo;
Servo SugarServo;
//-------------------------------

//---------Stirrer Arm---------------
const int stirrer_cw = 38;    // 1 min and 11 secs pataas, test polarity of the motor first
const int stirrer_cc = 39;    // 1 min and 5 secs pababa, test polarity of the motor first
const int stirrer_down = 34;  // in1
const int stirrer_up = 35;    // in2
const int stirrer_speed = 36; // Set this to high when moving vertically.
//-----------------------------

//--------------ULTRASONIC-----------------------------
byte triggerPin = 45;
byte echoCount = 2;
byte *echoPins = new byte[echoCount]{46, 48};
double *distances;

//-------------------WATER LEVEL-----------------------
// Define the pins
const int WATER_LEVEL_S = A0; // Analog pin for water level sensor
// Define variables
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

//--------------STEPPER MOTORS-------------------
const int stpprPANstep = 8;
const int stpprPANdir = 9;
const int stpprPANen = 18;
const int targetPosition = 1000;
const int defaultPosition = 0;
AccelStepper kawaliStepper(AccelStepper::DRIVER, stpprPANstep, stpprPANdir);
//-----------------------------

// SCL - 20
// SDA - 21

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
    delay(10);
    //-----------ULTRASONIC-------------------
    HCSR04.begin(triggerPin, echoPins, echoCount);
    //-------------------------------------

    //--------------I2C Communication----------
    Wire.begin(); // Start I2C communication as master
    //-----------------------------------

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
    PiliServo.attach(servoPin4);
    StoveServo.attach(servoPin3);
    SugarServo.attach(servoPin2);
    HopperServo.attach(servoPin1);

    //--------------------------------

    //-------------STEPPER MOTORS--------
    kawaliStepper.setMaxSpeed(12);
    kawaliStepper.setAcceleration(1000);
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
            StoveServo.write(0);

            // state = 0;
            previousMachineMillis = currentMillis;
        }

        // State 0, ISO process
        else if (state == 0 && machineOnStatus == false)
        {

            debug_println("State 0");
            StoveServo.write(0);
            SugarServo.write(0);
            digitalWrite(stirrer_up, HIGH);
            digitalWrite(stirrer_down, LOW);
            digitalWrite(stirrer_speed, HIGH);
            // StoveServo.write(0);
            // delay(5000);
            // StoveServo.write(90);
            // delay(5000);

            // for (int angle = 0; angle <= 45; angle += 1)
            //
            //     WeighingServo.write(angle);
            //     delay(15);
            // }
            // delay(5000);
            // for (int angle = 45; angle >= 0; angle -= 1)
            // {
            //     WeighingServo.write(angle);
            //     delay(15);
            // }
            // delay(5000);
        }

        // State 1, Running - HIGH, Stopped - LOW
        else if (state == 1)
        {
            debug_println("State 1");
            digitalWrite(IL_Running, LOW);
            digitalWrite(IL_Stopped, HIGH);
            water_level = analogRead(WATER_LEVEL_S);
            debug_println(water_level);
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
                state = 2;
            }
            delay(1000);
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

        // State 3, Turn water pump off after 4.75s
        else if (state == 3 && currentMillis - previousMachineMillis > 4750)
        {
            debug_println("State 3");
            digitalWrite(WaterPump, LOW);
            StoveServo.write(90);
            debug_println("WaterPump is closed");
            previousMachineMillis = currentMillis;
            state = 4;
        }

        // State 4, Open sugar hopper servos after 1.5s
        else if (state == 4 && currentMillis - previousMachineMillis > 1500)
        {
            // open sugar hopper servos
            SugarServo.write(105);
            previousMachineMillis = currentMillis;
            state = 5;
        }

        // State 5, Close sugar hopper after 2s
        else if (state == 5 && currentMillis - previousMachineMillis > 2000)
        {
            debug_println("State 5");
            SugarServo.write(0);
            previousMachineMillis = currentMillis;
            state = 8;
        }

        // State 8,  TODO: Adjust time
        else if (state == 8 && currentMillis - previousMachineMillis > 5000)
        {
            // let stirrer arm go down, adjust the time on testing, initialize it to be upward
            debug_println("State 8");
            // digitalWrite(stirrer_down, HIGH);
            digitalWrite(stirrer_up, LOW);
            bookmarkTime = millis() + 0000; // set clock timer for 1min 05 sec or 65000
            previousMachineMillis = currentMillis;
            state = 9;
        }

        // State 9, Stirring arm sequence
        else if (state == 9 && currentMillis - previousMachineMillis > 50)
        {
            debug_println("State 9");
            timeleft = bookmarkTime - millis();
            debug_println("down timeleft: " + String(timeleft / 1000));
            if (timeleft <= 0)
            {
                digitalWrite(stirrer_down, LOW);
                previousMachineMillis = currentMillis;
                state = 10;
                digitalWrite(stirrer_cc, HIGH);
                digitalWrite(stirrer_cw, LOW);
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
            bookmarkTime = millis() + 180000; // set clocktime for 2 minutes. or 120000

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
                digitalWrite(stirrer_cc, LOW); // off stirring
                state = 12;
            }
        }

        // State 12, Open nuts hopper after 1.5s
        else if (state == 12 && currentMillis - previousMachineMillis > 1500)
        {
            debug_println("State 12");
            PiliServo.write(105); // open nuts hopper
            previousMachineMillis = currentMillis;
            state = 13;
        }

        // State 13, Close nuts hopper after 0.5s
        else if (state == 13 && currentMillis - previousMachineMillis > 2000)
        {
            // close pili nuts hopper, drop it to pan
            debug_println("State 13");
            PiliServo.write(0);
            debug_println("Closing nuts hopper");
            previousMachineMillis = currentMillis;
            state = 16;
        }

        else if (state == 16 && currentMillis - previousMachineMillis > 5000)
        {
            debug_println("State 16");
            debug_println("resseting clock for boiling again");
            bookmarkTime = 0;
            timeleft = 0;
            bookmarkTime = millis() + 120000; // set clocktime for 3 minutes. or 180000
            previousMachineMillis = currentMillis;
            state = 17;
        }

        else if (state == 17 && currentMillis - previousMachineMillis > 50)
        {
            debug_println("State 17");
            // during
            timeleft = bookmarkTime - millis();
            debug_println("boiling time left: " + String(timeleft / 1000));
            digitalWrite(stirrer_cc, HIGH);
            previousMachineMillis = currentMillis;
            if (timeleft <= 0)
            {
                StoveServo.write(90);
                state = 18;
            }
        }

        else if (state == 18 && currentMillis - previousMachineMillis > 2000)
        {
            debug_println("State 18");
            // make the stirrer up again
            debug_println("resseting clock for boiling again");
            bookmarkTime = 0;
            timeleft = 0;                      // Set the temperature to low or 180 degrees
            bookmarkTime = millis() + 4200000; // set clocktime for 1min 11 secc minutes. or 71000
            previousMachineMillis = currentMillis;
            state = 19;
        }

        else if (state == 19 && currentMillis - previousMachineMillis > 50)
        {
            debug_println("State 19");
            // make the stirrer up again

            timeleft = bookmarkTime - millis();
            debug_println("stirrer time left: " + String(timeleft / 1000));
            previousMachineMillis = currentMillis;
            if (timeleft <= 0)
            {
                digitalWrite(stirrer_cc, LOW);
                digitalWrite(stirrer_up, HIGH); // off yung up
                state = 41;
            }
        }

        else if (state == 41 && currentMillis - previousMachineMillis > 2000)
        {
            debug_println("State 18");
            // make the stirrer up again
            debug_println("resseting clock for boiling again");
            bookmarkTime = 0;
            timeleft = 0;                      // Set the temperature to low or 180 degrees
            bookmarkTime = millis() + 2300000; // set clocktime for 1min 11 secc minutes. or 71000
            previousMachineMillis = currentMillis;
            state = 42;
        }

        else if (state == 42 && currentMillis - previousMachineMillis > 50)
        {
            debug_println("State 19");
            // make the stirrer up again
            digitalWrite(stirrer_up, HIGH);
            timeleft = bookmarkTime - millis();
            debug_println("stirrer time left: " + String(timeleft / 1000));
            previousMachineMillis = currentMillis;
            if (timeleft <= 0)
            {
                digitalWrite(stirrer_up, LOW);
                state = 0;
            }
        }

        else if (state == 20 && currentMillis - previousMachineMillis > 1000)
        {
            debug_println("State 20");
            // rotate the pan to the hopper below
            debug_println("rotating pan to hopper below");
            // Set the target position:
            kawaliStepper.moveTo(-50);
            // Run to target position with set speed and acceleration/deceleration:
            kawaliStepper.runToPosition();
            debug_println(boolean(kawaliStepper.isRunning()));
            debug_println(kawaliStepper.distanceToGo());
            debug_println(kawaliStepper.speed());
            previousMachineMillis = currentMillis;
            machineOnStatus = false;
            state = 21;
        }

        else if (state == 21 && currentMillis - previousMachineMillis > 5000)
        {

            debug_println("State 21");
            // rotate the pan to its resting position.
            delay(10000);

            // Move back to zero:
            kawaliStepper.moveTo(0);
            kawaliStepper.runToPosition();

            delay(10000);
            debug_println("finished for now");
            previousMachineMillis = currentMillis;
            state = 0;
        }

        else if (state == 22 && currentMillis - previousMachineMillis > 1000)
        {
            // send instruction to nano to reset all states
            debug_println("State 22");
            // Wire.beginTransmission(9); // Set slave address (0x70) or (9)
            // Wire.write(startNano);     // Send character
            // Wire.endTransmission();

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

        // Delay function
        // delay(5000);
    }
}