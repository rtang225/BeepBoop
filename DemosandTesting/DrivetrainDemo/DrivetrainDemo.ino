// Drivetrain Demo from Milestone 2

// #define DEBUG_ENCODER_COUNT 1
// #define DEBUG_DRIVE_SPEED 1

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>
#include "NewPing.h"

// Function declarations
void Indicator();                               // for mode/heartbeat on Smart LED
void setTarget(int dir, long pos, double dist); // sets encoder position target for movement

// Port pin constants
#define MODE_BUTTON 0         // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3 // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1 1              // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1
#define SMART_LED 21          // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT 1     // number of SMART LEDs in use

// Port pin constants for Drive System Wheels
#define LEFT_MOTOR_A 35    // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B 36    // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A 37   // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B 38   // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A 9   // left encoder A signal is connected to GPIO9 (J9)
#define ENCODER_LEFT_B 10  // left encoder B signal is connected to GPIO10 (J10)
#define ENCODER_RIGHT_A 11 // right encoder A signal is connected to GPIO11 (J11)
#define ENCODER_RIGHT_B 12 // right encoder B signal is connected to GPIO12 (J12)

// Port pin constants for Picker Upper Wheel
#define LEFT_MOTOR_2A 15   // GPIO15 (J15) Motor 1 A
#define LEFT_MOTOR_2B 16   // GPIO16 (J16) Motor 1 B
#define RIGHT_MOTOR_2A 17  // GPIO17 (J17) Motor 2 A
#define RIGHT_MOTOR_2B 18  // GPIO18 (J18) Motor 2 B
#define ENCODER_LEFT_2A 4  // left encoder A signal is connected to GPIO4 (J4)
#define ENCODER_LEFT_2B 5  // left encoder B signal is connected to GPIO5 (J5)
#define ENCODER_RIGHT_2A 6 // right encoder A signal is connected to GPIO6 (J6)
#define ENCODER_RIGHT_2B 7 // right encoder B signal is connected to GPIO7 (J7)

// IR DETECTOR
#define IR_DETECTOR 4 // GPIO14 pin 17 (J14) IR detector input

// ULTRASONIC SENSOR
#define TRIGGER_PIN 48
#define ECHO_PIN 47

// Port pin constants for Picker Upper Wheel
#define LEFT_MOTOR_2A 15    // GPIO15 (J15) Motor 1 A
#define LEFT_MOTOR_2B 16    // GPIO16 (J16) Motor 1 B
#define RIGHT_MOTOR_2A 17   // GPIO17 (J17) Motor 2 A
#define RIGHT_MOTOR_2B 18   // GPIO18 (J18) Motor 2 B
#define ENCODER_LEFT_2A 4   // left encoder A signal is connected to GPIO4 (J4)
#define ENCODER_LEFT_2B 5   // left encoder B signal is connected to GPIO5 (J5)
#define ENCODER_RIGHT_2A 6  // right encoder A signal is connected to GPIO6 (J6)
#define ENCODER_RIGHT_2B 7  // right encoder B signal is connected to GPIO7 (J7)
// Constants
const int cDisplayUpdate = 100;          // update interval for Smart LED in milliseconds
const int cPWMRes = 8;                   // bit resolution for PWM
const int cMinPWM = 150;                 // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1; // PWM value for maximum speed
const int cCountsRev = 1096;             // encoder pulses per motor revolution
const double cDistPerRev = 13.2;         // distance travelled by robot in 1 full revolution of the motor (1096 counts = 13.2 cm)

//=====================================================================================================================
//
// IMPORTANT: The constants in this section need to be set to appropriate values for your robot.
//            You will have to experiment to determine appropriate values.

const int cClawServoOpen = 1000;   // Value for open position of claw
const int cClawServoClosed = 2150; // Value for closed position of claw
const int cArmServoUp = 2200;      // Value for shoulder of arm fully up
const int cArmServoDown = 1000;    // Value for shoulder of arm fully down
const int cLeftAdjust = 0;         // Amount to slow down left motor relative to right
const int cRightAdjust = 0;        // Amount to slow down right motor relative to left

const int detectionDistance = 400; // Ultrasonic range
//
//=====================================================================================================================

// Variables
boolean motorsEnabled = true;        // motors enabled flag
boolean timeUp3sec = false;          // 3 second timer elapsed flag
boolean timeUp2sec = false;          // 2 second timer elapsed flag
boolean timeUp200msec = false;       // 200 millisecond timer elapsed flag
unsigned char leftDriveSpeed;        // motor drive speed (0-255)
unsigned char rightDriveSpeed;       // motor drive speed (0-255)
unsigned char wheelLDriveSpeed;      // wheel drive speed (0-255)
unsigned char wheelRDriveSpeed;      // wheel drive soeed (0-255)
unsigned char driveIndex;            // state index for run mode
unsigned int modePBDebounce;         // pushbutton debounce timer count
unsigned long timerCount3sec = 0;    // 3 second timer count in milliseconds
unsigned long timerCount2sec = 0;    // 2 second timer count in milliseconds
unsigned long timerCount200msec = 0; // 200 millisecond timer count in milliseconds
unsigned long displayTime;           // heartbeat LED update timer
unsigned long previousMicros;        // last microsecond count
unsigned long currentMicros;         // current microsecond count
double target;                       // target encoder count to keep track of distance travelled
unsigned long prevTime;              // Get the current time in milliseconds
float driveDistance = 80;            // Forward/backward drive distance
float turningDistance = 4.4;         // Turning distance counter
int driveCounter = 0;                // Counter for drive circles
boolean beaconLocated = false;
float currentDistance = 0.00;

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
unsigned char LEDBrightnessLevels[] = {5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210, 225, 240, 255,
                                       240, 225, 210, 195, 180, 165, 150, 135, 120, 105, 90, 75, 60, 45, 30, 15};

unsigned int robotModeIndex = 0; // robot operational state
unsigned int modeIndicator[6] = {
    // colours for different modes
    SmartLEDs.Color(255, 0, 0), //   red - stop
    SmartLEDs.Color(0, 255, 0), //   green - run
};

// Motor, encoder, and IR objects (classes defined in MSE2202_Lib)
Motion Bot = Motion();              // Instance of Motion for motor control
Encoders LeftEncoder = Encoders();  // Instance of Encoders for left encoder data
Encoders RightEncoder = Encoders(); // Instance of Encoders for right encoder data

Motion LeftWheel = Motion();         // Instance of Motion for left wheel control
Motion RightWheel = Motion();        // Instance of Motion for right wheel control
Encoders LeftEncoder2 = Encoders();  // Instance of Encoders for left encoder data
Encoders RightEncoder2 = Encoders(); // Instance of Encoders for right encoder data

IR Scan = IR(); // instance of IR for detecting IR signals
NewPing sonar(TRIGGER_PIN, ECHO_PIN, detectionDistance);

void setup()
{
#if defined DEBUG_DRIVE_SPEED || DEBUG_ENCODER_COUNT
  Serial.begin(115200);
#endif

  // Set up bot motors and encoders
  Bot.driveBegin("D1", LEFT_MOTOR_A, LEFT_MOTOR_B, RIGHT_MOTOR_A, RIGHT_MOTOR_B); // set up motors as Drive 1
  LeftEncoder.Begin(ENCODER_LEFT_A, ENCODER_LEFT_B, &Bot.iLeftMotorRunning);      // set up left encoder
  RightEncoder.Begin(ENCODER_RIGHT_A, ENCODER_RIGHT_B, &Bot.iRightMotorRunning);  // set up right encoder
  // leftDriveSpeed = cMaxPWM - cLeftAdjust;                                          // Set left drive motor speed to max
  // rightDriveSpeed = cMaxPWM - cRightAdjust;                                        // Set right drive motor speed to max

  // Set up wheel motors and encoders
  LeftWheel.motorBegin("M1", LEFT_MOTOR_2A, LEFT_MOTOR_2B);    // set up motor as Motor 1
  RightWheel.motorBegin("M1", RIGHT_MOTOR_2A, RIGHT_MOTOR_2B); // set up motor as Motor 2
  // LeftEncoder2.Begin(ENCODER_LEFT_2A, ENCODER_LEFT_2B, &LeftWheel.iLeftMotorRunning);       // set up left encoder
  // RightEncoder2.Begin(ENCODER_RIGHT_2A, ENCODER_RIGHT_2B, &RightWheel.iRightMotorRunning);  // set up right encoder
  wheelLDriveSpeed = cMaxPWM - cLeftAdjust;
  wheelLDriveSpeed = cMaxPWM - cRightAdjust;

  Scan.Begin(IR_DETECTOR, 1200); // set up IR Detection @ 1200 baud

  // Set up SmartLED
  SmartLEDs.begin();                                    // initialize smart LEDs object (REQUIRED)
  SmartLEDs.clear();                                    // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 0, 0)); // set pixel colors to 'off'
  SmartLEDs.show();                                     // send the updated pixel colors to the hardware

  pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP); // set up motor enable switch with internal pullup
  pinMode(MODE_BUTTON, INPUT_PULLUP);         // Set up mode pushbutton
  modePBDebounce = 0;                         // reset debounce timer count
}

void loop()
{

  long pos[] = {0, 0}; // current motor positions
  int pot = 0;         // raw ADC value from pot

  currentMicros = micros(); // get current time in microseconds
  if ((currentMicros - previousMicros) >= 1000)
  {                                 // enter when 1 ms has elapsed
    previousMicros = currentMicros; // record current time in microseconds

    // 3 second timer, counts 3000 milliseconds
    timerCount3sec = timerCount3sec + 1; // increment 3 second timer count
    if (timerCount3sec > 3000)
    {                     // if 3 seconds have elapsed
      timerCount3sec = 0; // reset 3 second timer count
      timeUp3sec = true;  // indicate that 3 seconds have elapsed
    }

    // 2 second timer, counts 2000 milliseconds
    timerCount2sec = timerCount2sec + 1; // increment 2 second timer count
    if (timerCount2sec > 2000)
    {                     // if 2 seconds have elapsed
      timerCount2sec = 0; // reset 2 second timer count
      timeUp2sec = true;  // indicate that 2 seconds have elapsed
    }

    // 200 millisecond timer, counts 200 milliseconds
    timerCount200msec = timerCount200msec + 1; // Increment 200 millisecond timer count
    if (timerCount200msec > 200)
    {                        // If 200 milliseconds have elapsed
      timerCount200msec = 0; // Reset 200 millisecond timer count
      timeUp200msec = true;  // Indicate that 200 milliseconds have elapsed
    }

    // Mode pushbutton debounce and toggle
    if (!digitalRead(MODE_BUTTON))
    { // if pushbutton GPIO goes LOW (nominal push)
      // Start debounce
      if (modePBDebounce <= 25)
      {                                      // 25 millisecond debounce time
        modePBDebounce = modePBDebounce + 1; // increment debounce timer count
        if (modePBDebounce > 25)
        {                        // if held for at least 25 mS
          modePBDebounce = 1000; // change debounce timer count to 1 second
        }
      }
      if (modePBDebounce >= 1000)
      { // maintain 1 second timer count until release
        modePBDebounce = 1000;
      }
    }
    else
    { // pushbutton GPIO goes HIGH (nominal release)
      if (modePBDebounce <= 26)
      {                     // if release occurs within debounce interval
        modePBDebounce = 0; // reset debounce timer count
      }
      else
      {
        modePBDebounce = modePBDebounce + 1; // increment debounce timer count
        if (modePBDebounce >= 1025)
        {                                      // if pushbutton was released for 25 mS
          modePBDebounce = 0;                  // reset debounce timer count
          robotModeIndex++;                    // switch to next mode
          robotModeIndex = robotModeIndex & 7; // keep mode index between 0 and 7
          timerCount3sec = 0;                  // reset 3 second timer count
          timeUp3sec = false;                  // reset 3 second timer
        }
      }
    }

    // check if drive motors should be powered
    motorsEnabled = !digitalRead(MOTOR_ENABLE_SWITCH); // if SW1-1 is on (low signal), then motors are enabled

    // modes
    // 0 = Default after power up/reset. Robot is stopped.
    // 1 = Press mode button once to enter. Run robot

    switch (robotModeIndex)
    {
    case 0: // Robot stopped
      Bot.Stop("D1");
      LeftWheel.Stop("M1");
      RightWheel.Stop("M1");
      // clear encoder counts
      LeftEncoder.clearEncoder();
      RightEncoder.clearEncoder();
      // LeftEncoder2.clearEncoder();
      // RightEncoder2.clearEncoder();
      driveIndex = 0;     // reset drive index
      timeUp2sec = false; // reset 2 second timer
      break;

    case 1: // Run robot
      if (timeUp3sec)
      { // pause for 3 sec before running case 1 code
        // Read pot to update drive motor speed
        pot = analogRead(POT_R1);
        // wheelLDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cLeftAdjust;
        // wheelRDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cRightAdjust;
        leftDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cLeftAdjust;
        rightDriveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM) - cRightAdjust;
#ifdef DEBUG_DRIVE_SPEED
        Serial.print(F("Left Drive Speed: Pot R1 = "));
        Serial.print(pot);
        Serial.print(F(", mapped = "));
        Serial.print(leftDriveSpeed);
        Serial.print(F(", Left wheel speed = "));
        Serial.print(wheelLDriveSpeed);
        Serial.print(F(", Right wheel speed = "));
        Serial.println(wheelRDriveSpeed);
#endif
#ifdef DEBUG_ENCODER_COUNT
        if (timeUp200msec)
        {
          timeUp200msec = false;             // reset 200 ms timer
          LeftEncoder.getEncoderRawCount();  // read left encoder count
          RightEncoder.getEncoderRawCount(); // read right encoder count
          Serial.print(F("Left Encoder count = "));
          Serial.print(LeftEncoder.lRawEncoderCount);
          Serial.print(F("  Right Encoder count = "));
          Serial.print(RightEncoder.lRawEncoderCount);
          Serial.print(F("  Target = "));
          Serial.print(target);
          Serial.print(F("  Drive index = "));
          Serial.print(driveIndex);
          Serial.print("\n");
        }
#endif
        if (motorsEnabled)
        {                                                              // run motors only if enabled
          LeftWheel.Forward("M1", wheelLDriveSpeed, wheelRDriveSpeed); // Spin collection wheel
          RightWheel.Reverse("M1", wheelLDriveSpeed, wheelRDriveSpeed);
          if (timeUp2sec)
          {
            RightEncoder.getEncoderRawCount(); // read right encoder count
            switch (driveIndex)
            {                 // cycle through drive states
            case 0:           // Stop
              Bot.Stop("D1"); // drive ID

            case 1:                                               // Drive forward
              Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed

              if (RightEncoder.lRawEncoderCount >= target)
              {
                setTarget(-1, RightEncoder.lRawEncoderCount, turningDistance); // set next target to turn 90 degrees CCW
                driveIndex++;                                                  // next state: turn left
              }
              break;

            case 2:                                            // Turn left
              Bot.Left("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed

              if (RightEncoder.lRawEncoderCount <= target)
              {
                driveDistance -= 10;
                setTarget(1, RightEncoder.lRawEncoderCount, driveDistance); // set target to drive forward
                driveIndex = 1;                                             // next state: drive forward
              }
              break;

            case 3:                                               // Drive forward
              Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed

              if (RightEncoder.lRawEncoderCount >= target)
              {
                setTarget(-1, RightEncoder.lRawEncoderCount, turningDistance); // set next target to turn 90 degrees CCW
                driveIndex++;                                                  // next state: turn left
              }
              break;

            case 4:                                            // Turn left
              Bot.Left("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed

              if (RightEncoder.lRawEncoderCount <= target)
              {
                driveDistance -= 10;
                setTarget(1, RightEncoder.lRawEncoderCount, driveDistance); // set target to drive forward
                driveIndex++;                                               // next state: drive forward
              }
              break;

            case 5:                                               // Drive forward
              Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed

              if (RightEncoder.lRawEncoderCount >= target)
              {
                setTarget(-1, RightEncoder.lRawEncoderCount, turningDistance); // set next target to turn 90 degrees CCW
                driveIndex++;                                                  // next state: turn left
              }
              break;

            case 6:                                            // Turn left
              Bot.Left("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed

              if (RightEncoder.lRawEncoderCount <= target)
              {
                driveDistance -= 10;
                setTarget(1, RightEncoder.lRawEncoderCount, driveDistance); // set target to drive forward
                driveIndex++;                                               // next state: drive forward
              }
              break;

            case 7:                                               // Drive forward
              Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed

              if (RightEncoder.lRawEncoderCount >= target)
              {
                setTarget(-1, RightEncoder.lRawEncoderCount, turningDistance); // set next target to turn 90 degrees CCW
                driveIndex++;                                                  // next state: turn left
              }
              break;

            case 8:                                            // Turn left
              Bot.Left("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed

              if (RightEncoder.lRawEncoderCount <= target)
              {
                driveDistance -= 10;
                setTarget(1, RightEncoder.lRawEncoderCount, driveDistance); // set target to drive forward
                driveIndex++;                                               // next state: stop robot, exit drive mode
              }
              break;

            case 9:                                               // Drive forward
              Bot.Forward("D1", leftDriveSpeed, rightDriveSpeed); // drive ID, left speed, right speed

              if (RightEncoder.lRawEncoderCount >= target)
              {
                if (driveCounter < 100)
                {
                  driveCounter++;
                  driveDistance -= 10;
                  setTarget(1, RightEncoder.lRawEncoderCount, driveDistance); // set target to drive forward
                  driveIndex = 0;
                }
                else
                {
                  robotModeIndex = 0;
                }
              }
              break;
            }
          }
        }
        else
        { // stop when motors are disabled
          Bot.Stop("D1");
          LeftWheel.Stop("M1");
          RightWheel.Stop("M2");
        }
      }
      else
      { // stop when motors are disabled
        Bot.Stop("D1");
      }
      break;

    case 2:
      Bot.Stop("D1"); // drive ID
      if (beaconLocated = false)
      {
        Bot.Left("D1", leftDriveSpeed, rightDriveSpeed);
        if (Scan.Available() && Scan.Get_IR_Data() == 'U')
        {
          beaconLocated = true;
          Serial.println(Scan.Get_IR_Data());
          Bot.Stop("D1");
        }
      }
      if (beaconLocated = true)
      {
        Bot.Reverse("D1", leftDriveSpeed, rightDriveSpeed);
        currentDistance = sonar.ping_cm();
        if (currentDistance <= 5.00)
        {
          driveIndex++; // Move to next case
        }
      }
      break;
    case 3:
      if (Scan.Available())
      {                                     // if data is received
        Serial.println(Scan.Get_IR_Data()); // output received data to serial
      }
      break;
    }

    // Update brightness of heartbeat display on SmartLED
    displayTime++; // count milliseconds
    if (displayTime > cDisplayUpdate)
    {                       // when display update period has passed
      displayTime = 0;      // reset display counter
      LEDBrightnessIndex++; // shift to next brightness level
      if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels))
      {                         // if all defined levels have been used
        LEDBrightnessIndex = 0; // reset to starting brightness
      }
      SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]); // set brightness of heartbeat LED
      Indicator();                                                      // update LED
    }
  }
}

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator()
{
  SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]); // set pixel colors to = mode
  SmartLEDs.show();                                          // send the updated pixel colors to the hardware
}

// Set target of motor encoder
void setTarget(int dir, long pos, double dist)
{
  if (dir == 1)
  { // Forwards
    target = pos + ((dist / cDistPerRev) * cCountsRev);
  }
  if (dir == -1)
  { // Backwards
    target = pos - ((dist / cDistPerRev) * cCountsRev);
  }
}