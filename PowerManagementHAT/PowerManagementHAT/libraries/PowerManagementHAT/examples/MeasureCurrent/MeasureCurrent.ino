/******************************************************************************
 * File Name          : MeasureCurrent.ino
 * Description        : Readme file
 * Date               : 2019-04-08
 ******************************************************************************
 *
 * Copyright (c) 2019 Waveshare
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
 * SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

//this is the logical:
//when press,then check that Raspberry Pi'Power,if power off,then power on.
//when release,then check chat Raspberry Pi'Power,if power on，then power off(soft shutdown or cut the power).
 
 
#include "PowerManagementHAT.h"
#include "Time.h"
#include "LowPower.h"
#include "PCF8523.h"
#include <Wire.h>

//-------------------------------------------------------------------------------------

//when key pressed more than SHUTDOWN_TIME_MS,start soft shudown
#define SHUTDOWN_TIME_MS    2000

//when key pressed more than CUT_POWER_TIME_MS,just cut the power
#define CUT_POWER_TIME_MS   8000

//-------------------------------------------------------------------------------------

// States
typedef enum {
  buttonState_Wait = 0,
  buttonState_Pressed,
  buttonState_Released
}eBUTTONSTATE;

typedef enum {
   piState_OFF = 0,
   piState_BOOTING,
   piState_ON,
   piState_SHUTTING_DOWN
}ePISTATE;



//define whether button pressed value
volatile bool  buttonPressed = false;

//define button pressed state
eBUTTONSTATE   buttonState = buttonState_Released;

//define pi working state
ePISTATE       piState = piState_OFF;

//define time to calculate
unsigned long  time, timePress;

//define user LED
const int USER_LED = 13;

void button_isr()
{
    buttonPressed = true;
}


void setup()
{
  //turn off the USER_LED
  pinMode(USER_LED, OUTPUT);		
  digitalWrite(USER_LED,HIGH);

  power.connectPiPower(false);  
    
  //Initialize serial communication:
  Serial.begin(115200);
  Serial.println("Measure the Raspberry Pi Current");
  delay(100); 
  
  //button interrupt
  attachInterrupt(1, button_isr, LOW);
  power.RTCInit(true);
  
}

void loop() 
{
    bool   piRunFlag;
    float  piRunCurrent; 

	//Cut Power if we detect Pi not running
    piRunFlag = power.getPiStatus(true);
    if(piRunFlag == false){ 
		//power.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
    }
    
    // Button State changed
    if(buttonPressed == true){
        detachInterrupt(1);      
        buttonPressed = false;  
        switch(buttonState) { 
			case buttonState_Released:          
			  //get the button pressed time
			  timePress = millis();
			  //check is the pi is running
			  piRunFlag = power.getPiStatus(false);
			  if(piRunFlag == false){  
				  //if pi is not running,then just cut the pi power
				  power.connectPiPower(true);
			  }
			  buttonState = buttonState_Pressed;
			  digitalWrite(USER_LED,LOW);

			  //wait for the button to released
			  attachInterrupt(1, button_isr, HIGH);                    
			  break;
			case buttonState_Pressed:
			  unsigned long buttonTime;
			  //get the button released
			  time = millis();
			  buttonState = buttonState_Released;
			  piRunFlag = power.getPiStatus(false);
			  if(piRunFlag == true){
				  //check how long the button has been pressed
				  buttonTime = time - timePress;
				  
				  //hold so long,then just cut the power
				  if(buttonTime > CUT_POWER_TIME_MS){
					  // cut the power
					 power.connectPiPower(false);
				  }
				  //hold just a little time,then just start a soft shutdown
				  else if (buttonTime > SHUTDOWN_TIME_MS){
					  // start a soft shutdown
					  power.shutPiDown();
								  
				  } else { 
					 //hols not long enough,then do nothing.
				  } 
			  }else{
				  //why? before you button released, the Raspberry Pi power has already power off ?
				  //everytime,when RPI has't power on,when button released,program goes here,so you can't add this.
				  //power.connectPiPower(false);
			  }
			  digitalWrite(USER_LED,HIGH);
			  
			  // button pin，
			  attachInterrupt(1, button_isr, LOW);       
			  break;
			default:
			  break;
        }
    }
	//if no button,then just print the current
    else {
        piRunCurrent = power.measureCurrent();
        Serial.print("The Raspberry Pi running current: ");
        Serial.print(piRunCurrent);  
        Serial.println(" mA");        
        delay(500);             
    }
}
