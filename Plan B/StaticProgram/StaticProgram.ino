/*
This is the static program of the project

Controls:
- Collection wheel spinning
- Colour detection and sorting
*/

// #define DEBUG_ENCODER_COUNT 1
#define DEBUG_DRIVE_SPEED 1

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

// Function declarations
void Indicator();                                // for mode/heartbeat on Smart LED
void setTarget(int dir, long pos, double dist);  // sets encoder position target for movement

// Port pin constants
#define LEFT_MOTOR_A 15        // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B 16        // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A 17       // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B 18       // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A 9       // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B 10      // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A 11     // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B 12     // right encoder B signal is connected to pin 20 GPIO12 (J12)
#define MODE_BUTTON 0          // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1 1               // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1
#define SMART_LED 21           // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT 1      // number of SMART LEDs in use

// Constants
const int cDisplayUpdate = 100;           // update interval for Smart LED in milliseconds
const int cPWMRes = 8;                    // bit resolution for PWM
const int cMinPWM = 150;                  // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;  // PWM value for maximum speed
const int cCountsRev = 1096;              // encoder pulses per motor revolution
const double cDistPerRev = 13.2;          // distance travelled by robot in 1 full revolution of the motor (1096 counts = 13.2 cm)

const int cSmartLED = 21;      // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount = 1;  // number of Smart LEDs in use
const int cSDA = 47;           // GPIO pin for I2C data
const int cSCL = 48;           // GPIO pin for I2C clock
const int cTCSLED = 14;        // GPIO pin for LED on TCS34725
const int cLEDSwitch = 46;     // DIP switch S1-2 controls LED on TCS32725


//=====================================================================================================================
//
// IMPORTANT: The constants in this section need to be set to appropriate values for your robot.
//            You will have to experiment to determine appropriate values.
// Port pin constants
#define SORTER_SERVO 41  // GPIO41 pin 34 (J41) Servo 1
#define GATE_SERVO 42    // GPIO42 pin 35 (J42) Servo 2

const int cGateServoOpen = 1700;     // Value for open position of claw
const int cGateServoClosed = 1000;   // Value for closed position of claw
const int cSorterServoRight = 1400;  // Value for shoulder of arm fully up
const int cSorterServoLeft = 1150;   // Value for shoulder of arm fully down

bool flag = true;            // delay flag
unsigned long pastTime = 0;  // var to store time

// VARIABLES FOR GREEN
const int rLow = 29;
const int rHigh = 31;

const int gLow = 33;
const int gHigh = 35;

const int bLow = 27;
const int bHigh = 30;

const int cLow = 90;
const int cHigh = 105;

//
//=====================================================================================================================

// Variables
bool motorsEnabled = true;  // motors enabled flag
bool firstPass = true;
bool timeUp2sec = false;
bool tc3Up = false;
bool tc2Up = false;  // 2 second timer elapsed flag
bool tc1Up = false;
unsigned char leftDriveSpeed;   // motor drive speed (0-255)
unsigned char rightDriveSpeed;  // motor drive speed (0-255)
unsigned int modePBDebounce;    // pushbutton debounce timer count
unsigned long tc3 = 0;          // 500ms second timer count in milliseconds
unsigned long tc2 = 0;          // 250ms second timer count in milliseconds
unsigned long tc1 = 0;
unsigned long timerCount2sec = 0;  // 2 second timer count in milliseconds
unsigned long displayTime;         // heartbeat LED update timer
unsigned long previousMicros;      // last microsecond count
unsigned long currentMicros;       // current microsecond count
double target;                     // target encoder count to keep track of distance travelled
unsigned long prevTime;            // Get the current time in milliseconds
float driveDistance = 80;          // Forward/backward drive distance
float turningDistance = 4.4;       // Turning distance counter
int driveCounter = 0;              // Counter for drive circles

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);

// smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0;
unsigned char LEDBrightnessLevels[] = { 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255,
                                        240, 225, 210, 195, 180, 165, 150, 135, 120, 105, 90, 75, 60, 45, 30, 15 };

int robotModeIndex = 0;  // robot operational state
int driveModeIndex = 0;
unsigned int modeIndicator[2] = {
  // colours for different modes
  SmartLEDs.Color(255, 0, 0),  //   red - stop
  SmartLEDs.Color(0, 255, 0),  //   green - run
};

// Motor, encoder, and IR objects (classes defined in MSE2202_Lib)
Motion Wheel = Motion();             // Instance of Motion for wheel control
Encoders LeftEncoder = Encoders();   // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders();  // Instance of Encoders for right encoder data

// Variables
uint16_t r, g, b, c;  // RGBC values from TCS34725

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;  // TCS34725 flag: 1 = connected; 0 = not found

void setup() {
#if defined DEBUG_DRIVE_SPEED || DEBUG_ENCODER_COUNT
  Serial.begin(115200);
#endif

  // Set up servos
  Wheel.servoBegin("S1", GATE_SERVO);    // set up claw servo
  Wheel.servoBegin("S2", SORTER_SERVO);  // set up shoulder servo

  Wire.setPins(cSDA, cSCL);           // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);           // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);  // configure GPIO to set state of TCS34725 LED

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }

  // Set up bot motors and encoders
  Wheel.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B);  // set up motors as Drive 1
  LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Wheel.iLeftMotorRunning);       // set up left encoder
  RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Wheel.iRightMotorRunning);   // set up right encoder

  // Set up SmartLED
  SmartLEDs.begin();                                     // initialize smart LEDs object (REQUIRED)
  SmartLEDs.clear();                                     // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 0, 0));  // set pixel colors to 'off'
  SmartLEDs.show();                                      // send the updated pixel colors to the hardware

  pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);  // set up motor enable switch with internal pullup
  pinMode(MODE_BUTTON, INPUT_PULLUP);          // Set up mode pushbutton
  modePBDebounce = 0;                          // reset debounce timer count
}

void loop() {

  // Colour Sensor Code:
  //=====================================================================================================================
  unsigned long currentTime = millis();  // Current time

  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));  // turn on onboard LED if switch state is low (on position)
  if (tcsFlag) {                                    // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                 // get raw RGBC values
#ifdef PRINT_COLOUR
    Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
#endif

    if (flag == true) {
      pastTime = millis();
      Wheel.ToPosition("S2", cSorterServoRight);  // Moves servo so stone slides into disposal tube
    }

    if ((r >= rLow && r <= rHigh) && (g >= gLow && g <= gHigh) && (b >= bLow && b <= bHigh) && (c >= cLow && c <= cHigh)) {  // Checks the green value reading /* REQUIRES TESTING AND ADJUSTMENTS */
      Wheel.ToPosition("S2", cSorterServoLeft);
      Serial.print("Green");  // Moves servo so stone slides into collection
      flag = false;           // reset flag
      pastTime = millis();
    } else {
      if ((millis() - pastTime) > 500) {
        Wheel.ToPosition("S2", cSorterServoRight);  // Moves servo so stone slides into disposal tube
        flag = true;
      }
    }
  }

  //=====================================================================================================================

  long pos[] = { 0, 0 };  // current motor positions
  int pot = 0;            // raw ADC value from pot

  currentMicros = micros();                        // get current time in microseconds
  if ((currentMicros - previousMicros) >= 1000) {  // enter when 1 ms has elapsed
    previousMicros = currentMicros;                // record current time in microseconds


    // 500ms second timer
    tc3 = tc3 + 1;    // increment 500ms second timer count
    if (tc3 > 150) {  // if 500ms seconds have elapsed
      tc3 = 0;        // reset 500ms second timer count
      tc3Up = true;   // indicate that 500ms seconds have elapsed
    }

    // 500ms second timer
    tc2 = tc2 + 1;    // increment 500ms second timer count
    if (tc2 > 150) {  // if 500ms seconds have elapsed
      tc2 = 0;        // reset 500ms second timer count
      tc2Up = true;   // indicate that 500ms seconds have elapsed
    }

    // 500ms second timer
    tc1 = tc1 + 1;   // increment 500ms second timer count
    if (tc1 > 80) {  // if 500ms seconds have elapsed
      tc1 = 0;       // reset 500ms second timer count
      tc1Up = true;  // indicate that 500ms seconds have elapsed
    }

    // 2 second timer, counts 2000 milliseconds
    timerCount2sec = timerCount2sec + 1;  // increment 2 second timer count
    if (timerCount2sec > 2000) {          // if 2 seconds have elapsed
      timerCount2sec = 0;                 // reset 2 second timer count
      timeUp2sec = true;                  // indicate that 2 seconds have elapsed
    }

    // Mode pushbutton debounce and toggle
    if (!digitalRead(MODE_BUTTON)) {  // if pushbutton GPIO goes LOW (nominal push)
      // Start debounce
      if (modePBDebounce <= 25) {             // 25 millisecond debounce time
        modePBDebounce = modePBDebounce + 1;  // increment debounce timer count
        if (modePBDebounce > 25) {            // if held for at least 25 mS
          modePBDebounce = 1000;              // change debounce timer count to 1 second
        }
      }
      if (modePBDebounce >= 1000) {  // maintain 1 second timer count until release
        modePBDebounce = 1000;
      }
    } else {                       // pushbutton GPIO goes HIGH (nominal release)
      if (modePBDebounce <= 26) {  // if release occurs within debounce interval
        modePBDebounce = 0;        // reset debounce timer count
      } else {
        modePBDebounce = modePBDebounce + 1;    // increment debounce timer count
        if (modePBDebounce >= 1025) {           // if pushbutton was released for 25 mS
          modePBDebounce = 0;                   // reset debounce timer count
          robotModeIndex++;                     // switch to next mode
          robotModeIndex = robotModeIndex & 1;  // keep mode index between 0 and 1
          timerCount2sec = 0;                   // reset 3 second timer count
          timeUp2sec = false;                   // reset 3 second timer
        }
      }
    }

    // check if drive motors should be powered
    motorsEnabled = !digitalRead(MOTOR_ENABLE_SWITCH);  // if SW1-1 is on (low signal), then motors are enabled

    // modes
    // 0 = Default after power up/reset. Robot is stopped.
    // 1 = Press mode button once to enter. Run robot

    switch (robotModeIndex) {
      case 0:  // Robot stopped
        Wheel.Stop("D1");
        LeftEncoder.clearEncoder();  // clear encoder counts
        RightEncoder.clearEncoder();
        timeUp2sec = false;  // reset 2 second timer
        break;

      case 1:  // Run robot
        switch (driveModeIndex) {
          case 0:
            if (timeUp2sec) {  // pause for 2 sec before running case 1 code
              leftDriveSpeed = cMaxPWM * 0.9;
              rightDriveSpeed = cMaxPWM * 0.9;
              driveModeIndex++;
              timeUp2sec = false;
              tc3 = 0;
              tc3Up = false;
            }
            break;

          case 1:
            Wheel.Forward("D1", leftDriveSpeed, rightDriveSpeed);  // Spin collection wheel

            if (tc3Up) {
              driveModeIndex++;
              tc2 = 0;
              tc2Up = false;
            }
            break;

          case 2:
            Wheel.Stop("D1");

            if (tc2Up) {
              driveModeIndex++;
              tc1 = 0;
              tc1Up = false;
            }
            break;

          case 3:
            Wheel.Reverse("D1", leftDriveSpeed, rightDriveSpeed);
            if (tc1Up) {
              driveModeIndex++;
              tc2 = 0;
              tc2Up = false;
            }
            break;

          case 4:
            Wheel.Stop("D1");

            if (tc2Up) {
              driveModeIndex = 1;
              tc3 = 0;
              tc3Up = false;
            }
            break;
        }
        break;
    }

    // Update brightness of heartbeat display on SmartLED
    displayTime++;                                             // count milliseconds
    if (displayTime > cDisplayUpdate) {                        // when display update period has passed
      displayTime = 0;                                         // reset display counter
      LEDBrightnessIndex++;                                    // shift to next brightness level
      if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) {  // if all defined levels have been used
        LEDBrightnessIndex = 0;                                // reset to starting brightness
      }
      SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);  // set brightness of heartbeat LED
      Indicator();                                                       // update LED
    }
  }
}

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator() {
  SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);  // set pixel colors to = mode
  SmartLEDs.show();                                           // send the updated pixel colors to the hardware
}