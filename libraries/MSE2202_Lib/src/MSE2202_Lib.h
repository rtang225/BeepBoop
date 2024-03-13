/*
 Western Engineering MSE 2202 Library
 2024 E J Porter

 
  
 */



#ifndef MSE2202_LIB_H
  #define MSE2202_LIB_H 1

#include <Arduino.h>

//************  Motion *********************************

#define LEDCMAXCHANNELS 8

class Motion
{
public:
        
		int iLeftMotorRunning;
		int iRightMotorRunning;

	    Motion();
	    ~Motion(){ end(); }
/*
  driveBegin with set up two motors as a drive base. DriveID = D1 or D2 ( only two Drives can be begun, this will limit number of servoBegin and motorBegin. Since there is only 8 channels total
 and driveBegin uses 4 channels each.
*/
	    void driveBegin(const char cDriveID[2], int iLeftMotorPin1, int iLeftMotorPin2,int iRightMotorPin1, int iRightMotorPin2); 
/*
  Set up one motor to be controlled.  cMotorID = M1 to M4 ( limited number of motors and will be lessened if servos or drives are used. Since there is only 8 channels total
 and MotorBegin uses 2 channels each.
*/		
		void motorBegin(const char cMotorID[2], int iMotorPin1, int iMotorPin2);
/*
  Set up one servo to be controlled.  cServoID = S1 to S8 ( limited number of servos and will be lessened if motors or drives are used. Since there is only 8 channels total
 and servoBegin uses 1 channel each.
*/			
		void servoBegin(const char cServoID[2], int iServoPin1);
	
		

/*
  Will run the motor(s) "Forward" at ucLeftSpeed( 0 to 255) for left motor and ucRightSpeed ( 0 to 255) for right motor.  Only for Drive ID = D1, D2
*/		
		void Forward(const char cID[2], unsigned char ucLeftSpeed, unsigned char ucRightSpeed );

/*
  Will run the motor(s) "Reverse" at ucLeftSpeed( 0 to 255) for left motor and ucRightSpeed ( 0 to 255) for right motor.  Only for Drive ID = D1, D2
*/		
		void Reverse(const char cID[2], unsigned char ucLeftSpeed, unsigned char ucRightSpeed );

/*
  Will run the motor(s) "Left" (Right motor forward at ucRightSpeed speed and  Left motor in reverse at ucLeftSpeed speed , will do sweep turn), Only for Drive ID = D1, D2
*/		
		void Left(const char cID[2], unsigned char ucLeftSpeed, unsigned char ucRightSpeed );

/*
  Will run the motor(s) "Right" (Right motor reverse at ucRightSpeed speed and  Left motor in forward at ucLeftSpeed speed , will do sweep turn), Only for Drive ID = D1, D2
*/		
		void Right(const char cID[2], unsigned char ucLeftSpeed, unsigned char ucRightSpeed );
/*
  Will run the Stop motor(s) 55), Can be Drive,  Motor or Servo ID = M1 to M4 or D1, D2
*/		
		void Stop(const char cID[2]);
/*
  Will run the Servo(s) to the raw position in ucServoPosition ( 0 to 16384), ID can only be servo ID = S1 to S8
*/			
		void ToPosition(const char cID[2], unsigned int uiServoPosition);
	
		void end();

	  
private:
        unsigned char ucLEDcLastUnUsedChannel;	
		unsigned char ucLEDcChannelError;
		unsigned char ucLEDcDriveChannels[8];
		unsigned char ucLEDcMotorChannels[8];
		unsigned char ucLEDcServoChannels[8];
		unsigned char Get_LEDcChannel();



};

//------------------------------------------------------------------------------------------------------------------------------------------------


class Encoders
{
public:

	int* iMotorRunning;
		

	    Encoders();
	    ~Encoders(){ end(); }
/*
  Begin with set up both encoders: 
  iEncoderAPin = GPIO pin encoder A is conneded to
  iEncoderBPin = GPIO pin encoder B is conneded to
   
*/
		void Begin(int iEncoderAPin, int iEncoderBPin, int* MotorRunnning);  
		
		void end();
		
		void getEncoderRawSpeed();
		void getEncoderRawCount();
		void clearEncoder();
		
		
		
		long lRawEncoderCount;
		long lRawEncoderSpeed;
	
		
private:
       
		
		int iEncoderPinB;
				
		void  Spd_Encoder_ISR();  
		
	
	   // void static ENC_isrLSpd();
		
        volatile long ENC_vlEncoderRawSpeed;
						
        volatile long ENC_vlOdometer;
	
		
};

class IR
{
public:
	    IR();
	    ~IR(){ end(); }
/*
  Set up IR Detector. int iIR_Pin = GPIO pin for spi reception ,int iBaud is baud rate of beacon/ 
*/			
		void Begin(int iIR_Pin,int iBaud);	
/*
  Will return the character IR Detector saw since last time Get_IR_Data was called. If no data was seen since last call will return 0 
  
*/		
		char Get_IR_Data();
/*
  Indicate that IR saw data 
*/		
		boolean Available();
		
		void end();	
		
private:

    char b_IR_ReceivedData;
};

#endif /* MSE2202_LIB_H */
