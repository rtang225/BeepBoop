/*
 Western Engineering MSE 2202 Library
 2024 E J Porter

 
  
 */

#include "MSE2202_Lib.h"

#define DEBUGPRINT 1
#define ACCELERATIONRATE 1;


#include "Arduino.h"
#include "FunctionalInterrupt.h"


Motion::Motion()
{
	ucLEDcLastUnUsedChannel = 0;  
}

unsigned char Motion::Get_LEDcChannel()
{
	unsigned char uc_NewChannel;
	
	uc_NewChannel = ucLEDcLastUnUsedChannel;
	ucLEDcLastUnUsedChannel++;
	if(ucLEDcLastUnUsedChannel >= LEDCMAXCHANNELS)
	{
		return(100);
	}
	return(uc_NewChannel);
	
}

void Motion::driveBegin(const char cDriveID[2], int iLeftMotorPin1, int iLeftMotorPin2,int iRightMotorPin1, int iRightMotorPin2)
{
	char c_driveID; 
	
	if((cDriveID[0] == 'd') || (cDriveID[0] == 'D'))
	{
		if(cDriveID[1] == '1')
		{
			c_driveID = 1;
		}
		else if(cDriveID[1] == '2')
		{
			c_driveID = 2;
		}
		else
		{
			Serial.printf("Incorrect ID Designator number %s\n", cDriveID);
		}
	
		//setup PWM for motors
		ucLEDcDriveChannels[(c_driveID * 4) - 4] = Get_LEDcChannel();
		ledcAttachPin(iLeftMotorPin1, ucLEDcDriveChannels[0]); // assign Motors pins to channels
		 // Initialize channels 
	  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
	  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
		ledcSetup(ucLEDcDriveChannels[(c_driveID * 4) - 4], 20000, 8); // 20mS PWM, 8-bit resolution
		
		ucLEDcDriveChannels[(c_driveID * 4) - 3] = Get_LEDcChannel();
		ledcAttachPin(iLeftMotorPin2, ucLEDcDriveChannels[1]);
		ledcSetup(ucLEDcDriveChannels[(c_driveID * 4) - 3], 20000, 8);
		
		ucLEDcDriveChannels[(c_driveID * 4) - 2] = Get_LEDcChannel();
		ledcAttachPin(iRightMotorPin1, ucLEDcDriveChannels[2]);
		ledcSetup(ucLEDcDriveChannels[(c_driveID * 4) - 2], 20000, 8);
		
		ucLEDcDriveChannels[(c_driveID * 4) - 1] = Get_LEDcChannel();
		ledcAttachPin(iRightMotorPin2, ucLEDcDriveChannels[3]);
		ledcSetup(ucLEDcDriveChannels[(c_driveID * 4) - 1], 20000, 8);
	}
	else
	{
		Serial.printf("Incorrect ID Designator %s\n", cDriveID);
	}
}

void Motion::motorBegin(const char cMotorID[2], int iMotorPin1, int iMotorPin2)
{
	char c_motorID; 
	
	if((cMotorID[0] == 'M') || (cMotorID[0] == 'M'))
	{
		if((cMotorID[1] >= '1') &&  (cMotorID[1] <= '4'))
		{
			c_motorID = cMotorID[1] - 0x30;
		}
		else 
		{
			Serial.printf("Incorrect ID Designator number %s\n", cMotorID);
		}
		//setup PWM for motors
		ucLEDcMotorChannels[(c_motorID * 2) - 2] = Get_LEDcChannel();
		ledcAttachPin(iMotorPin1, ucLEDcMotorChannels[0]); // assign Motors pins to channels
		 // Initialize channels 
	  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
	  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
		ledcSetup(ucLEDcMotorChannels[(c_motorID * 2) - 2], 20000, 8); // 20mS PWM, 8-bit resolution
		
		ucLEDcMotorChannels[(c_motorID * 2) - 1] = Get_LEDcChannel();
		ledcAttachPin(iMotorPin2, ucLEDcMotorChannels[1]);
		ledcSetup(ucLEDcMotorChannels[(c_motorID * 2) - 1], 20000, 8);
		

		
	}
	else
	{
		Serial.printf("Incorrect ID Designator %s\n", cMotorID);
	}
}

void Motion::servoBegin(const char cServoID[2], int iServoPin1)
{
	
	char c_servoID; 
	
	if((cServoID[0] == 'S') || (cServoID[0] == 's'))
	{
		if((cServoID[1] >= '1') &&  (cServoID[1] <= '8'))
		{
			c_servoID = cServoID[1] - 0x30;
		}
		else 
		{
			Serial.printf("Incorrect ID Designator number %s\n", cServoID);
		}
		//setup PWM for motors
		ucLEDcServoChannels[(c_servoID - 1)] = Get_LEDcChannel();
		ledcAttachPin(iServoPin1, ucLEDcServoChannels[(c_servoID - 1)]); // assign Motors pins to channels
		 // Initialize channels 
	  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
	  // ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
		ledcSetup(ucLEDcServoChannels[c_servoID  - 1],  50,14);// channel 1, 50 Hz, 14-bit width
	}
	else
	{
		Serial.printf("Incorrect ID Designator number %s\n", cServoID);
	}
}

void Motion::ToPosition(const char cID[2], unsigned int uiServoPosition)
{
	char c_ID; 
	
	switch(cID[0])
	{
		
		case 's':
		case 'S':
		{
			if((cID[1] >= '1') &&  (cID[1] <= '8'))
			{
				c_ID = cID[1] - 0x30;
			}
			else 
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			ledcWrite(ucLEDcServoChannels[c_ID - 1],uiServoPosition);
			break;
		}
		default:
		{
			Serial.printf("Incorrect ID Designator %s\n", cID);
		}
	}
}



void Motion::Forward(const char cID[2], unsigned char ucLeftSpeed,unsigned char ucRightSpeed)
{
	 char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == '1')
			{
				c_ID = 1;
			}
			else if(cID[1] == '2')
			{
				c_ID = 2;
			}
			else
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			iLeftMotorRunning = 5;
			//Left Motor
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],ucLeftSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],0);
			//Right Motor
			iRightMotorRunning = 5;
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],ucRightSpeed);
			break;
		}
		
		default:
		{
			Serial.printf("Incorrect ID Designator %s\n", cID);
		}
	}
}


void Motion::Reverse(const char cID[2], unsigned char ucLeftSpeed,unsigned char ucRightSpeed)
{
	 char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == '1')
			{
				c_ID = 1;
			}
			else if(cID[1] == '2')
			{
				c_ID = 2;
			}
			else
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			
			//Left Motor
			iLeftMotorRunning = 10;
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],ucLeftSpeed);
			//Right Motor
			iRightMotorRunning = 10;
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],ucRightSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		
		default:
		{
			Serial.printf("Incorrect ID Designator %s\n", cID);
		}
	}
}


void Motion::Left(const char cID[2], unsigned char ucLeftSpeed,unsigned char ucRightSpeed)
{
	char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == '1')
			{
				c_ID = 1;
			}
			else if(cID[1] == '2')
			{
				c_ID = 2;
			}
			else
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			
			//Left Motor
			iLeftMotorRunning = 20;
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],ucLeftSpeed);
			//Right Motor
			iRightMotorRunning = 20;
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],ucRightSpeed);
			break;
		}
		
		default:
		{
			Serial.printf("Incorrect ID Designator %s\n", cID);
		}
	}
}


void Motion::Right(const char cID[2], unsigned char ucLeftSpeed,unsigned char ucRightSpeed)
{
	char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == '1')
			{
				c_ID = 1;
			}
			else if(cID[1] == '2')
			{
				c_ID = 2;
			}
			else
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			
			//Left Motor
			iLeftMotorRunning = 30;
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],ucLeftSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],0);
			//Right Motor
			iRightMotorRunning = 30;
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],ucRightSpeed);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		
		default:
		{
			Serial.printf("Incorrect ID Designator %s\n", cID);
		}
	}
}

void Motion::Stop(const char cID[2])
{
	 char c_ID; 
	
	switch(cID[0])
	{
		case 'd':
		case 'D':
		{
			if(cID[1] == '1')
			{
				c_ID = 1;
			}
			else if(cID[1] == '2')
			{
				c_ID = 2;
			}
			else
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			
			//Left Motor
			iLeftMotorRunning = 100;
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 4],255);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 3],255);
			//Right Motor
			iRightMotorRunning = 100;
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],255);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],255);
			break;
		}
		case 'm':
		case 'M':
		{
			if((cID[1] >= '1') &&  (cID[1] <= '4'))
			{
				c_ID = cID[1] - 0x30;
			}
			else 
			{
				Serial.printf("Incorrect ID Designator number %s\n", cID);
			}
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 2],0);
			ledcWrite(ucLEDcDriveChannels[(c_ID * 4) - 1],0);
			break;
		}
		default:
		{
			Serial.printf("Incorrect ID Designator %s\n", cID);
		}
	}
}


void Motion::end()
{
	for(unsigned char ucLEDcIndex = 0; ucLEDcIndex < ucLEDcLastUnUsedChannel; ucLEDcIndex++)
	{
		ledcWrite(ucLEDcDriveChannels[ucLEDcIndex],0);
	}
	ucLEDcLastUnUsedChannel = 0; 
}

//----------------------------------------------------------------------------------------------------------
//Encoders
//Encoders* Encoders::anchorLeftSpd = 0;
  #include "esp_log.h"
  
 


Encoders::Encoders()
{
	this->ENC_vlOdometer = 0;
	
	lRawEncoderCount = 0;
	lRawEncoderSpeed = 0;
	this->ENC_vlEncoderRawSpeed = 0;
	
}



void Encoders::Begin(int iEncoderAPin, int iEncoderBPin, int* MotorRunnning)
{
	iEncoderPinB = iEncoderBPin;
	
	pinMode(iEncoderAPin, INPUT);   
	pinMode(iEncoderBPin, INPUT); 
	
    iMotorRunning = MotorRunnning;
	
	attachInterrupt(digitalPinToInterrupt(iEncoderAPin),  std::bind(&Encoders::Spd_Encoder_ISR, this), RISING);
	
	
	
}


void Encoders::Spd_Encoder_ISR()
{

	
	
   volatile static int32_t ENC_vsi32LastTime;
   volatile static int32_t ENC_vsi32ThisTime;
   volatile static int32_t ENC_vsi32LastOdometer;
   
   if(*iMotorRunning < 100)
   {
	  //if Read Left direction pin if low count up otherwise wheel is going backwards count down
	  //odometer reading
	  if(digitalRead(iEncoderPinB))  // MSE-Dunio port pin
	  {
		this->ENC_vlOdometer += 1;
	  }
	  else
	  {
		this->ENC_vlOdometer -= 1;
	  }
  
   } 
  asm volatile("esync; rsr %0,ccount":"=a" (ENC_vsi32ThisTime )); // @ 240mHz clock each tick is ~4nS 
 
  
  if(ENC_vsi32ThisTime - ENC_vsi32LastTime >= 25000000 ) //~ every 100 mSec
  {
	  ENC_vsi32LastTime = ENC_vsi32ThisTime;
	  this->ENC_vlEncoderRawSpeed = this->ENC_vlOdometer - ENC_vsi32LastOdometer;
	  ENC_vsi32LastOdometer = this->ENC_vlOdometer;
  }

	this->ENC_vlEncoderRawSpeed = ENC_vsi32ThisTime;

}

  

void Encoders::getEncoderRawCount()
{
	lRawEncoderCount = this->ENC_vlOdometer;
	
}
	
void Encoders::getEncoderRawSpeed()
{
	lRawEncoderSpeed = this->ENC_vlEncoderRawSpeed;
	
}
	
void Encoders::clearEncoder()
{
	this->ENC_vlOdometer = 0;
	lRawEncoderCount = 0;
	
}




void Encoders::end()
{
	
}





IR::IR()
{
	b_IR_ReceivedData = 0;  
}


void IR::Begin(int iIR_Pin, int iBaud)
{
    Serial2.begin(iBaud, SERIAL_8N1, iIR_Pin); 

}

boolean IR::Available()
{
	if (Serial2.available() > 0)
    { 
	  b_IR_ReceivedData = Serial2.read();
      return(1);          // read the incoming byte

    }
	else
	{
		b_IR_ReceivedData = 0xEE;
		return(0x00);
		
	}
	
}

char IR::Get_IR_Data()
{
	
   return(b_IR_ReceivedData);
   
}

void IR::end()
 {
	
 }