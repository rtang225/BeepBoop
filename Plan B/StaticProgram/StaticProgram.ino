#define PRINT_COLOUR  // uncomment to turn on output of colour sensor data

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"

// Function delcaration

void Indicator();  // for mode/heartbeat on Smart LED
void setMotor(int dir, int pwm, int in1, int in2);
long degreesToDutyCycle(int deg);

// Port pin constants
#define SORTER_SERVO 41   // GPIO41 pin 34 (J41) Servo 1
#define LEFT_MOTOR_A 35   // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B 36   // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A 37  // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B 38  // GPIO38 pin 31 (J38) Motor 2 B
#define MODE_BUTTON 0     // GPIO0  pin 27 for Push Button 1



// Constants
const int cPotPin = 1;                                   // when DIP switch S1-3 is on, pot (R1) is connected to GPIO1 (ADC1-0)
const int cDisplayUpdate = 100;                          // update interval for Smart LED in milliseconds
const int cNumMotors = 2;                                // number of DC motors
const int cIN1Pin[] = { LEFT_MOTOR_A, RIGHT_MOTOR_A };   // GPIO pin(s) for INT1
const int cIN1Chan[] = { 0, 1 };                         // PWM channe(s) for INT1
const int c2IN2Pin[] = { LEFT_MOTOR_B, RIGHT_MOTOR_B };  // GPIO pin(s) for INT2
const int cIN2Chan[] = { 2, 3 };                         // PWM channel(s) for INT2
const int cPWMRes = 8;                                   // bit resolution for PWM
const int cMinPWM = 150;                                 // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;                 // PWM value for maximum speed
const int cPWMFreq = 20000;                              // frequency of PWM signal
const int cCountsRev = 1096;                             // encoder pulses per motor revolution
const double cDistPerRev = 13.2;                         // distance travelled by robot in 1 full revolution of the motor (1096 counts = 13.2 cm)
const int cServoChannel = 5;                             // PWM channel used for the RC servo motor
const long cMinDutyCycle = 400;                          // duty cycle for 0 degrees
const long cMaxDutyCycle = 2100;                         // duty cycle for 180 degrees

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

const int cSorterServoRight = 960;  // Value for shoulder of arm fully up
const int cSorterServoLeft = 560;   // Value for shoulder of arm fully down

unsigned long pastTime = 0;  // var to store time
int count = 0;

// VARIABLES FOR GREEN
const int rLow = 26;
const int rHigh = 30;

const int gLow = 30;
const int gHigh = 34;

const int bLow = 20;
const int bHigh = 26;

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
int driveCounter = 0;              // Counter for drive program
int potPos = 0;                    // input value from the potentiometer

// Variables
uint16_t r, g, b, c;  // RGBC values from TCS34725

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

// smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0;
unsigned char LEDBrightnessLevels[] = { 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255,
                                        240, 225, 210, 195, 180, 165, 150, 135, 120, 105, 90, 75, 60, 45, 30, 15 };

int robotModeIndex = 0;  // robot operational state
int driveModeIndex = 0;
unsigned int modeIndicator[3] = {
  // colours for different modes
  SmartLEDs.Color(255, 0, 0),    //   red - stop
  SmartLEDs.Color(0, 100, 255),  //   cyan - tune servo pos
  SmartLEDs.Color(200, 0, 200),  //   purple - run
};

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;  // TCS34725 flag: 1 = connected; 0 = not found

void setup() {
  Serial.begin(115200);  // Standard baud rate for ESP32 serial monitor

  // Setup potentiometer
  pinMode(cPotPin, INPUT);  // configure potentiometer pin for input

  // Set up servo
  pinMode(SORTER_SERVO, OUTPUT);               // configure servo GPIO for output
  ledcSetup(cServoChannel, 50, 14);            // configure PWM channel frequency and resolution
  ledcAttachPin(SORTER_SERVO, cServoChannel);  // attach INT1 GPIO to PWM channel

  // Set up bot motors
  // Set up motors and encoders
  for (int k = 0; k < cNumMotors; k++) {
    ledcAttachPin(cIN1Pin[k], cIN1Chan[k]);     // attach INT1 GPIO to PWM channel
    ledcSetup(cIN1Chan[k], cPWMFreq, cPWMRes);  // configure PWM channel frequency and resolution
    ledcAttachPin(c2IN2Pin[k], cIN2Chan[k]);    // attach INT2 GPIO to PWM channel
    ledcSetup(cIN2Chan[k], cPWMFreq, cPWMRes);  // configure PWM channel frequency and resolution
  }

  // Set up SmartLED
  SmartLEDs.begin();                                     // initialize smart LEDs object
  SmartLEDs.clear();                                     // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 0, 0));  // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                            // set brightness [0-255]
  SmartLEDs.show();                                      // update LED

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
  pinMode(MODE_BUTTON, INPUT_PULLUP);  // Set up mode pushbutton
  modePBDebounce = 0;                  // reset debounce timer count
}

void loop() {
  // COLOUR CODE
  //=================================================================================================================================
  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));  // turn on onboard LED if switch state is low (on position)
  if (tcsFlag) {                                    // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                 // get raw RGBC values
    //Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);

    if ((r >= rLow && r <= rHigh) && (g >= gLow && g <= gHigh) && (b >= bLow && b <= bHigh) && (g - b > 3) && (g > b)) {  // Checks the green value reading /* REQUIRES TESTING AND ADJUSTMENTS */
      ledcWrite(cServoChannel, cSorterServoLeft);
      Serial.println("Green");  // Moves servo so stone slides into collection
      pastTime = millis();
    } else {
      if ((millis() - pastTime) > 600) {
        ledcWrite(cServoChannel, cSorterServoRight);  // Moves servo so stone slides into disposal tube
      }
    }
  }
  //=================================================================================================================================

  int pot = 0;  // raw ADC value from pot

  currentMicros = micros();                        // get current time in microseconds
  if ((currentMicros - previousMicros) >= 1000) {  // enter when 1 ms has elapsed
    previousMicros = currentMicros;                // record current time in microseconds

    // 500ms second timer
    tc3 = tc3 + 1;   // increment 500ms second timer count
    if (tc3 > 14) {  // if 500ms seconds have elapsed
      tc3 = 0;       // reset 500ms second timer count
      tc3Up = true;  // indicate that 500ms seconds have elapsed
    }

    // 500ms second timer
    tc2 = tc2 + 1;   // increment 500ms second timer count
    if (tc2 > 70) {  // if 500ms seconds have elapsed
      tc2 = 0;       // reset 500ms second timer count
      tc2Up = true;  // indicate that 500ms seconds have elapsed
    }

    // 500ms second timer
    tc1 = tc1 + 1;   // increment 500ms second timer count
    if (tc1 > 10) {  // if 500ms seconds have elapsed
      tc1 = 0;       // reset 500ms second timer count
      tc1Up = true;  // indicate that 500ms seconds have elapsed
    }

    // 2 second timer, counts 2000 milliseconds
    timerCount2sec = timerCount2sec + 1;  // increment 2 second timer count
    if (timerCount2sec > 500) {           // if 2 seconds have elapsed
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
          robotModeIndex = robotModeIndex % 3;  // keep mode index between 0 and 2
          timerCount2sec = 0;                   // reset 3 second timer count
          timeUp2sec = false;                   // reset 3 second timer
        }
      }
    }

    // modes
    // 0 = Default after power up/reset. Robot is stopped.
    // 1 = Press mode button once to enter. Run robot

    switch (robotModeIndex) {
      case 0:                                      // Robot stopped
        setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);  // stop left motor
        setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);  // stop right motor
        timeUp2sec = false;                        // reset 2 second timer
        break;

      case 1:
        mapPosition(analogRead(cPotPin));  // get desired servo postion from pot input
        break;

      case 2:  // Run robot
        switch (driveModeIndex) {
          case 0:
            if (timeUp2sec) {  // pause for 2 sec before running case 1 code
              leftDriveSpeed = cMaxPWM - 20;
              rightDriveSpeed = cMaxPWM - 20;
              driveModeIndex++;
              timeUp2sec = false;
              tc3 = 0;
              tc3Up = false;
            }
            break;

          case 1:
            setMotor(1, leftDriveSpeed, cIN1Chan[0], cIN2Chan[0]);    // left motor forward
            setMotor(-1, rightDriveSpeed, cIN1Chan[1], cIN2Chan[1]);  // right motor reverse (opposite dir from left)

            if (tc3Up) {
              driveModeIndex++;
              tc2 = 0;
              tc2Up = false;
            }
            break;

          case 2:
            setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);  // stop left motor
            setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);  // stop right motor

            if (tc2Up) {
              driveModeIndex++;
              tc3 = 0;
              tc3Up = false;
            }
            break;

          case 3:
            setMotor(1, leftDriveSpeed, cIN1Chan[0], cIN2Chan[0]);    // left motor forward
            setMotor(-1, rightDriveSpeed, cIN1Chan[1], cIN2Chan[1]);  // right motor reverse (opposite dir from left)

            if (tc3Up) {
              driveModeIndex++;
              tc2 = 0;
              tc2Up = false;
            }
            break;

          case 4:
            setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);  // stop left motor
            setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);  // stop right motor

            if (tc2Up) {
              driveModeIndex++;
              tc3 = 0;
              tc3Up = false;
            }
            break;

          case 5:
            setMotor(1, leftDriveSpeed, cIN1Chan[0], cIN2Chan[0]);    // left motor forward
            setMotor(-1, rightDriveSpeed, cIN1Chan[1], cIN2Chan[1]);  // right motor reverse (opposite dir from left)

            if (tc3Up) {
              driveModeIndex++;
              tc2 = 0;
              tc2Up = false;
            }
            break;

          case 6:
            setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);  // stop left motor
            setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);  // stop right motor

            if (tc2Up) {
              driveModeIndex++;
              tc3 = 0;
              tc3Up = false;
            }
            break;

          case 7:
            setMotor(1, leftDriveSpeed, cIN1Chan[0], cIN2Chan[0]);    // left motor forward
            setMotor(-1, rightDriveSpeed, cIN1Chan[1], cIN2Chan[1]);  // right motor reverse (opposite dir from left)

            if (tc3Up) {
              driveModeIndex++;
              tc2 = 0;
              tc2Up = false;
            }
            break;

          case 8:
            setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);  // stop left motor
            setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);  // stop right motor

            if (tc2Up) {
              driveModeIndex++;
              tc1 = 0;
              tc1Up = false;
            }
            break;

          case 9:
            setMotor(-1, leftDriveSpeed, cIN1Chan[0], cIN2Chan[0]);  // left motor reverse
            setMotor(1, rightDriveSpeed, cIN1Chan[1], cIN2Chan[1]);  // right motor forward (opposite dir from right)
            if (tc1Up) {
              driveModeIndex++;
              tc2 = 0;
              tc2Up = false;
            }
            break;

          case 10:
            setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);  // stop left motor
            setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);  // stop right motor

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

// send motor control signals, based on direction and pwm (speed)
void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {  // forward
    ledcWrite(in1, pwm);
    ledcWrite(in2, 0);
  } else if (dir == -1) {  // reverse
    ledcWrite(in1, 0);
    ledcWrite(in2, pwm);
  } else {  // stop
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator() {
  SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);  // set pixel colors to = mode
  SmartLEDs.show();                                           // send the updated pixel colors to the hardware
}

void mapPosition(int potPos) {
  long position = map(potPos, 0, 4096, cMinDutyCycle, cMaxDutyCycle);  // convert to duty cycle
  ledcWrite(cServoChannel, position);
  Serial.println("PotPos: ");
  Serial.println(position);

}
