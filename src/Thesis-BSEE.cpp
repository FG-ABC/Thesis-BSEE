#include <Arduino.h>
#include "HX711.h"
#include <NoDelay.h>
#include <HCSR04.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>
#include "AccelStepper.h"

#define DEBUG // Uncomment to disable debugging printing

#ifdef DEBUG
#define debug_print(x) Serial.print(x)
#define debug_println(x) Serial.println(x)
#else
#define debug_print(x)   // Empty statement
#define debug_println(x) // Empty statement
#endif

//----------USER INTERFACE-------------------
unsigned int reqPilinutsLevel = 17;
int reqTubigLevel = 200;
unsigned int reqSugarLevel = 17; // required sugar level

//---------------------------------

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
int state = -1; // stage of automation
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
int IndCooker_ONOFF = 7;
int IndCooker_FUNC = 8;
int IndCooker_negaTEMP = 9;
int IndCooker_posiTEMP = 10;
//--------------------------------

//---------Stirrer Arm---------------
int stirrer_up = 30;   // 1 min and 11 secs pataas
int stirrer_down = 31; // 1min and 5 secs pababa
int stirrer_cc = 32;
int stirrer_cw = 33;
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
const byte sugarHopper = 0;
const byte nutsHopper = 4;
const byte arm1LDcell = 2;
const byte arm2LDcell = 3;

float val_sugarHopper = 1500;
float val_nutsHopper = 1500;
float val_arm1LDcell = 1500;
float val_arm2LDcell = 1500;

float val_sugarHopperFiltered = 1500;
float val_nutsHopperFiltered = 1500;
float val_arm1LDcellFiltered = 1500;
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

    //-----------stirrer arm------------------
    pinMode(stirrer_up, OUTPUT);
    pinMode(stirrer_down, OUTPUT);
    pinMode(stirrer_cc, OUTPUT);
    pinMode(stirrer_cw, OUTPUT);
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
    kawaliStepper.moveTo(200);
    //---------------------------
}

void loop()
{
    currentMillis = millis();
    if (currentMillis - previousMillis >= 10)
    { // start event
        previousMillis = currentMillis;
        if (int prevstate = !state)
        {
            debug_println(state);
        }

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
        { // stop process
            digitalWrite(IL_NoSugar, LOW);
            digitalWrite(IL_NoWater, LOW);
            digitalWrite(IL_NoNuts, LOW);
            digitalWrite(IL_Running, LOW);
            digitalWrite(IL_Stopped, HIGH);
            state = 0;
            // debug_println("nag-stop");
            previousMachineMillis = currentMillis;
        }

        else if (state == 0 && machineOnStatus == true)
        {
            digitalWrite(WATER_LEVEL_POW, HIGH);
            state = 1;
            debug_println(state + " yeah!");
            previousMachineMillis = currentMillis;
        }
        else if (state == 1)
        {
            // idle or presetup process
            digitalWrite(IL_Running, HIGH); // REvise this code
            digitalWrite(IL_Stopped, LOW);  // REvise this code
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
            digitalWrite(WATER_LEVEL_POW, LOW);
            digitalWrite(WaterPump, HIGH);
            debug_println("WAterPump is open");
            previousMachineMillis = currentMillis;
            state = 3;
        }

        else if (state == 3 && currentMillis - previousMachineMillis > 4500)
        {
            // turn off Waterpump
            digitalWrite(WaterPump, LOW);
            debug_println("WAterPump is closed");
            previousMachineMillis = currentMillis;
            state = 4;
        }

        else if (state == 4 && currentMillis - previousMachineMillis > 250)
        {
            // open sugar hopper servos
            val_sugarHopper = 756;
            previousMachineMillis = currentMillis;
            int value = get_Loadcells();
            debug_println("opening sugar hopper, value: ");
            if (value > 500)
            {
                state = 5;
            }
        }

        else if (state == 5 && currentMillis - previousMachineMillis > 5000)
        {
            val_sugarHopper = SERVOMIN;
            debug_println("closed sugar hopper");
            previousMachineMillis = currentMillis;
            state = 6;
        }

        else if (state == 6 && currentMillis - previousMachineMillis > 5000)
        {
            // the servo should be intialized in resting position
            // the servo will go down now, dispensing sugar to cooking pan
            val_arm1LDcell = 756;
            val_arm2LDcell = 756;
            debug_println("from hopper (sugar) to pan");
            previousMachineMillis = currentMillis;
            state = 7;
        }

        else if (state == 7 && currentMillis - previousMachineMillis > 5000)
        {
            // arm1LDcell go to resting pos, turn on induction cooker and set it to normal temp multitask
            val_arm1LDcell = SERVOMIN;
            val_arm2LDcell = SERVOMIN;
            debug_println("turn on induction cooker");
            stepFlagInduction_Med = 1; // start induction cooker sequence
            previousMachineMillis = currentMillis;
            state = 8;
        }

        else if (state == 8 && currentMillis - previousMachineMillis > 5000)
        {
            // let stirrer arm go down, adjust the time on testing, initialize it to be upward
            // capture the time
            digitalWrite(stirrer_down, HIGH);
            digitalWrite(stirrer_up, LOW);
            bookmarkTime = millis() + 5000; // set clock timer for 1min 11 sec or 71000
            previousMachineMillis = currentMillis;
            state = 9;
        }

        else if (state == 9 && currentMillis - previousMachineMillis > 50)
        {

            // during
            digitalWrite(stirrer_down, HIGH);
            digitalWrite(stirrer_up, LOW);
            timeleft = bookmarkTime - millis();
            debug_println("time left: " + String(timeleft / 1000));
            if (timeleft <= 0)
            {
                // after
                // stay stirring arm, start rotating sequence for 2 minutes
                // timeleft = 0;
                toggleFlagStirring = 1;
                digitalWrite(stirrer_down, LOW);
                previousMachineMillis = currentMillis;
                state = 10;
            }
            previousMachineMillis = currentMillis;
        }

        else if (state == 10 && currentMillis - previousMachineMillis > 1000)
        {
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
            previousMachineMillis = currentMillis;
            if (timeleft <= 0)
            {
                // after
                toggleFlagStirring = 0; // off stirring
                state = 12;
            }
        }

        else if (state == 12 && currentMillis - previousMachineMillis > 1000)
        {
            // toggle off stirrer, pilinuts dispensing
            toggleFlagStirring = 0;
            val_nutsHopper = 756; // open hopper
            previousMachineMillis = currentMillis;
            int value = get_Loadcells();
            debug_println("pili nuts value: " + value);
            if (value > 500)
            {
                state = 13;
            }
        }

        else if (state == 13 && currentMillis - previousMachineMillis > 1000)
        {
            // close pili nuts dispensing, drop it to pan
            val_nutsHopper = SERVOMIN;
            debug_println("finished for now");
        }

        // *** SEQUENCES ***

        // stirring arm cc & cw
        if (toggleFlagStirring == 1 && stepFlagStirring == 0 && currentMillis - previousStirringMillis > 7500)
        {
            // set stirrer counterclockwise for 7.5 sec
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
            debug_println("finished setting induction to medium heat");
            stepFlagInduction_Med = 0;
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
