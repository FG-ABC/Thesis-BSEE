#include <Arduino.h>
#include "HX711.h"
#include <NoDelay.h>
#include <HCSR04.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>

//----------USER INTERFACE-------------------
unsigned int reqPilinutsLevel = 17;
int reqTubigLevel = 200;
unsigned int reqSugarLevel = 17; // required sugar level

//---------------------------------

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
void machine_buttons();
// int dispensing_specificprocess(int dispensing_switch);
void water_dispensing();
void sugarHopper_servo();
void loadCell_servos();
//-------------

//---------FLAGS----------
// make these false at stop
bool waterPump_flag = false;
bool sugarHopper_flag = false;
bool loadCell_servos_flag = false;
//---------------------

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
WaterLevelState state = IDLE;
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
#define SERVOMIN 150  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600  // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
const byte sugarHopper = 0;
const byte nutsHopper = 1;
const byte arm1LDcell = 2;
const byte arm2LDcell = 3;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//----------------------------------------------

void setup()
{
  Serial.begin(9600);
  Serial.println(__FILE__);
  Serial.print("LIBRARY VERSION: ");
  Serial.println(HX711_LIB_VERSION);
  Serial.println();

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
  pinMode(10, OUTPUT); // TEST ONLY, REMOVE AFTER
  //-------------------------------------

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
}

void loop()
{
  // Read the state of the buttons
  int buttonSetState = digitalRead(START_PB);
  int buttonResetState = digitalRead(STOP_PB);

  // Check if the Set button is pressed
  if (buttonSetState == LOW && !buttonStartPressed)
  {
    unsigned long currentTime = millis();
    if (currentTime - prevTime_StartPB > debounceDelay_PB)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      machineOnStatus = true;
      Serial.println("MACHINE IS ON : ");
      Serial.println(machineOnStatus);
      buttonStartPressed = true;
      machine_statusnow = presetup_process;
    }
    prevTime_StartPB = currentTime;
  }
  else if (buttonSetState == HIGH)
  {
    buttonStartPressed = false;
  }

  // Check if the Reset button is pressed
  if (buttonResetState == LOW && !buttonStopPressed)
  {
    unsigned long currentTime = millis();
    if (currentTime - prevTime_StopPB > debounceDelay_PB)
    {
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("MACHINE IS OFF :");
      machineOnStatus = false;
      Serial.println(machineOnStatus);
      buttonStopPressed = true;
      machine_statusnow = stop_process;
    }
    prevTime_StopPB = currentTime;
  }
  else if (buttonResetState == HIGH)
  {
    buttonStopPressed = false;
  }

  // Add any other code you want to execute here
  switch (machine_statusnow)
  {
  case stop_process:
    // code here
    digitalWrite(IL_NoSugar, LOW);
    digitalWrite(IL_NoWater, LOW);
    digitalWrite(IL_NoNuts, LOW);
    digitalWrite(IL_Running, LOW);
    digitalWrite(IL_Stopped, HIGH);
    machine_buttons();
    break;
  case presetup_process:
    digitalWrite(IL_Running, HIGH); // REvise this code
    digitalWrite(IL_Stopped, LOW);  // REvise this code
    machine_buttons();
    waterLevel();

    distances = HCSR04.measureDistanceCm();
    // Serial.print(distances[0]);
    // Serial.print(distances[1]);

    if (distances[1] <= 200 && distances[0] <= 200 && water_level > 200)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(10, HIGH); // TEST ONLY, REMOVE AFTER
      machine_statusnow = dispensing_process;
    }
    else
    {
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(10, LOW); // TEST ONLY, REMOVE AFTER
    }

    if (distances[0] >= reqSugarLevel)
    {
      digitalWrite(IL_NoSugar, HIGH);
      Serial.println("WALANG LAMAN ASUKAL");
    }
    else
    {
      digitalWrite(IL_NoSugar, LOW);
    }

    if (distances[1] >= reqPilinutsLevel)
    {
      digitalWrite(IL_NoNuts, HIGH);
      Serial.println("WALANG LAMAN NUTS");
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
    }
    break;
  case dispensing_process:
    digitalWrite(IL_Running, HIGH); // REvise this code
    digitalWrite(IL_Stopped, LOW);
    Serial.println(machine_statusnow);
    machine_buttons();
    // dispensing_switch = dispensing_specificprocess(dispensing_switch);
    water_dispensing();
    sugarHopper_servo();
    loadCell_servos();

    // Serial.println("Done yung water dispensing");

    // gagalaw na yung servo adafruit servo
    // loadCellvalues.update();
    // if loadcell values => setamount then titigil na yung servo.

    // if done na ung sugarHopper

    break;
  case cooking_process:
    digitalWrite(IL_Running, HIGH); // REvise this code
    digitalWrite(IL_Stopped, LOW);
    break;
  case cooling_process:
    break;
  case packaging_process:
    break;
  default:
    // nothing
    Serial.println("nothing");
    break;
  }
}

int get_Loadcells()
{
  float value;
  // Serial.print(count);
  // for (int i = 0; i < 2; i++)
  // {
  //   value = scales[i].read_average(10);
  //   Serial.print("\t");
  //   Serial.print(String(value) + calib[i]);
  // }
  value = scales[0].read_average(10);
  Serial.print("\t");
  Serial.print(value);
  Serial.print("this is the value: " + String(value));
  return value;
}

int get_Loadcell2()
{
  int value;
  // Serial.print(count);
  // for (int i = 0; i < 2; i++)
  // {
  //   value = scales[i].read_average(10);
  //   Serial.print("\t");
  //   Serial.print(String(value) + calib[i]);
  // }
  value = scales[1].read_average(10);
  Serial.print("\t");
  Serial.print("this is the value: " + String(value));
  return value;
}

void waterLevel()
{
  unsigned long currentTime = millis();

  switch (state)
  {
  case IDLE:
    // Wait for a certain time before starting the process
    if (currentTime - lastStateTime >= 1000)
    { // Wait for 1 second
      state = POWER_ON;
      lastStateTime = currentTime;
    }
    break;

  case POWER_ON:
    digitalWrite(WATER_LEVEL_POW, HIGH); // Turn the sensor ON
    state = READ_SENSOR;
    lastStateTime = currentTime;
    break;

  case READ_SENSOR:
    if (currentTime - lastStateTime >= 10)
    {                                          // Wait for 10 milliseconds
      water_level = analogRead(WATER_LEVEL_S); // Read the analog value from sensor
      Serial.println("The water level is: " + String(water_level));
      state = POWER_OFF;
      lastStateTime = currentTime;
    }
    break;

  case POWER_OFF:
    digitalWrite(WATER_LEVEL_POW, LOW); // Turn the sensor OFF
    state = IDLE;
    lastStateTime = currentTime;
    break;
  }
}

void machine_buttons()
{
  // Read the state of the buttons
  int buttonSetState = digitalRead(START_PB);
  int buttonResetState = digitalRead(STOP_PB);

  // Check if the Set button is pressed
  if (buttonSetState == LOW && !buttonStartPressed)
  {
    unsigned long currentTime = millis();
    if (currentTime - prevTime_StartPB > debounceDelay_PB)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      machineOnStatus = true;
      Serial.println("MACHINE IS ON : ");
      Serial.println(machineOnStatus);
      buttonStartPressed = true;
      machine_statusnow = presetup_process;
    }
    prevTime_StartPB = currentTime;
  }
  else if (buttonSetState == HIGH)
  {
    buttonStartPressed = false;
  }

  // Check if the Reset button is pressed
  if (buttonResetState == LOW && !buttonStopPressed)
  {
    unsigned long currentTime = millis();
    if (currentTime - prevTime_StopPB > debounceDelay_PB)
    {
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("MACHINE IS OFF :");
      machineOnStatus = false;
      Serial.println(machineOnStatus);
      buttonStopPressed = true;
    }
    prevTime_StopPB = currentTime;
  }
  else if (buttonResetState == HIGH)
  {
    buttonStopPressed = false;
  }
}

void water_dispensing()
{
  if (!waterPump_flag)
  { // Check if water pump hasn't been activated yet
    Serial.println("flowing Waterpump");
    digitalWrite(WaterPump, HIGH);
    delay(5200); // Wait for 5200 milliseconds (5.2 seconds)
    digitalWrite(WaterPump, LOW);
    Serial.println("Done Waterpump");
    waterPump_flag = true; // Set flag to indicate water pump has run
  }
}

void sugarHopper_servo()
{
  if (!sugarHopper_flag)
  {
    Serial.println("opening hopper");
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
    {
      pwm.setPWM(sugarHopper, 0, pulselen);
    }
    int value = get_Loadcells();

    if (value > 500)
    {
      // do something
      Serial.print("closing hoppper");
      for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
      {
        pwm.setPWM(sugarHopper, 0, pulselen);
      }
      sugarHopper_flag = true;
    }
    Serial.println(value);
  }
}

void loadCell_servos()
{
  delay(1000);
  if (!loadCell_servos_flag)
  {
    Serial.println("opening LDCELL ARM");
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
    {
      pwm.setPWM(arm1LDcell, 0, pulselen);
      pwm.setPWM(arm2LDcell, 0, pulselen);
    }
    delay(2000);
    Serial.println("taktak LDCELL ARM");
    pwm.setPWM(arm1LDcell, 0, SERVOMAX / 2);
    pwm.setPWM(arm2LDcell, 0, SERVOMAX / 2);
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
    {
      pwm.setPWM(arm1LDcell, 0, pulselen);
      pwm.setPWM(arm2LDcell, 0, pulselen);
    }
    delay(1000);
    Serial.println("default position LDCELL ARM");
    for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
    {
      pwm.setPWM(arm1LDcell, 0, pulselen);
      pwm.setPWM(arm2LDcell, 0, pulselen);
    }
    delay(1000);
    loadCell_servos_flag = true;
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
//     Serial.println("opening Waterpump");
//     digitalWrite(WaterPump, HIGH);
//     delay(4000);
//     digitalWrite(WaterPump, LOW);
//     break;

//   case 2:
//     // open hopper
//     Serial.println("opening hopper");
//     for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++)
//     {
//       pwm.setPWM(sugarHopper, 0, pulselen);
//     }
//     value = get_Loadcells();

//     if (value > 500)
//     {
//       // do something
//       Serial.print("closing hoppper");
//       for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--)
//       {
//         pwm.setPWM(sugarHopper, 0, pulselen);
//       }
//     }

//     Serial.println(String(value));
//     dispensing_switch = 3;
//     break;

//   case 3:
//     Serial.println("doing something3");
//     value = 0;
//     dispensing_switch = 0;
//     dispensing_flag = false;
//     break;

//   default:
//     Serial.println("doing something default");
//     dispensing_switch = 0;
//     dispensing_flag = false;
//     break;
//   }
//   return dispensing_switch;
//   return dispensing_flag = false;
// }