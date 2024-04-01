#define PRINT_COLOUR // uncomment to turn on output of colour sensor data

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
#include <MSE2202_Lib.h>

// Function delcaration

void Indicator(); // for mode/heartbeat on Smart LED

// Port pin constants
#define SORTER_SERVO 41 // GPIO41 pin 34 (J41) Servo 1

// Constants
const int cSmartLED = 21;     // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount = 1; // number of Smart LEDs in use
const int cSDA = 47;          // GPIO pin for I2C data
const int cSCL = 48;          // GPIO pin for I2C clock
const int cTCSLED = 14;       // GPIO pin for LED on TCS34725
const int cLEDSwitch = 46;    // DIP switch S1-2 controls LED on TCS32725

//=====================================================================================================================
//
// IMPORTANT: The constants in this section need to be set to appropriate values for your robot.
//            You will have to experiment to determine appropriate values.

const int cSorterServoRight = 1400; // Value for shoulder of arm fully up
const int cSorterServoLeft = 1150;  // Value for shoulder of arm fully down

bool flag = true;           // delay flag
unsigned long pastTime = 0; // var to store time
int count = 0;

// VARIABLES FOR GREEN
const int rLow = 24;
const int rHigh = 30;

const int gLow = 26;
const int gHigh = 34;

const int bLow = 22;
const int bHigh = 28;

//
//=====================================================================================================================

// Variables
uint16_t r, g, b, c; // RGBC values from TCS34725

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

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0; // TCS34725 flag: 1 = connected; 0 = not found

// Motor, encoder, and IR objects (classes defined in MSE2202_Lib)
Motion Bot = Motion(); // Instance of Motion for motor control

void setup()
{
    Serial.begin(115200); // Standard baud rate for ESP32 serial monitor

    // Set up servos
    Bot.servoBegin("S1", SORTER_SERVO); // set up shoulder servo

    // Set up SmartLED
    SmartLEDs.begin();                                    // initialize smart LEDs object
    SmartLEDs.clear();                                    // clear pixel
    SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 0, 0)); // set pixel colours to black (off)
    SmartLEDs.setBrightness(0);                           // set brightness [0-255]
    SmartLEDs.show();                                     // update LED

    Wire.setPins(cSDA, cSCL);          // set I2C pins for TCS34725
    pinMode(cTCSLED, OUTPUT);          // configure GPIO to control LED on TCS34725
    pinMode(cLEDSwitch, INPUT_PULLUP); // configure GPIO to set state of TCS34725 LED

    // Connect to TCS34725 colour sensor
    if (tcs.begin())
    {
        Serial.printf("Found TCS34725 colour sensor\n");
        tcsFlag = true;
    }
    else
    {
        Serial.printf("No TCS34725 found ... check your connections\n");
        tcsFlag = false;
    }
}

void loop()
{
    unsigned long currentTime = millis(); // Current time

    digitalWrite(cTCSLED, !digitalRead(cLEDSwitch)); // turn on onboard LED if switch state is low (on position)
    if (tcsFlag)
    {                                   // if colour sensor initialized
        tcs.getRawData(&r, &g, &b, &c); // get raw RGBC values

        if ((r >= rLow && r <= rHigh) && (g >= gLow && g <= gHigh) && (b >= bLow && b <= bHigh) && (g > r) && (g > b))
        { // Checks the green value reading /* REQUIRES TESTING AND ADJUSTMENTS */
            Bot.ToPosition("S1", cSorterServoLeft);
            Serial.println("Green"); // Moves servo so stone slides into collection
            pastTime = millis();
        }
        else
        {
            if ((millis() - pastTime) > 500)
            {
                Bot.ToPosition("S1", cSorterServoRight); // Moves servo so stone slides into disposal tube
            }
        }
    }
}
