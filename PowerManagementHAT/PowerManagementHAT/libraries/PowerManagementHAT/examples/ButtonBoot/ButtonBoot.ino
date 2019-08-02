/******************************************************************************
 * File Name          : ButtonBoot.ino
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

//when use current to check the status of pi(warning:default use pin to check pi status)
#define USE_CURRENT_TO_CHECK_PI_STATUS 0

#if(USE_CURRENT_TO_CHECK_PI_STATUS)
	#define CURRENT_THRESHOLD_MA   85
#endif

//-------------------------------------------------------------------------------------

//buttonState
typedef enum{
	buttonState_Wait = 0,
	buttonState_Pressed,
	buttonState_WaitRelease,
	buttonState_Released
}buttonStateEnmu;

//piState
typedef enum{
	piState_OFF = 0,
	piState_BOOTING,
	piState_ON,
	piState_SHUTTING_DOWN
}piStateEnmu;

//define whether button pressed value
volatile bool  buttonPressed = false;

//define button pressed state
buttonStateEnmu  buttonState = buttonState_Wait;

//define pi working state
piStateEnmu      piState = piState_OFF;

//define time to calculate
unsigned long  time, timePress;

//define user LED
const int USER_LED = 13;

//time for RTC to wake pi up periodically when buttonState in buttonState_Wait state
eTIMER_TIMEBASE  Timer_Timebase     = eTB_SECOND;
uint8_t          Timer_Value        = 5;

//button press interrupt
void button_isr()
{
    buttonPressed = true;
}

//alarm press interrupt
void alarm_isr()
{
	//do something
}

void setup()
{
  //turn off the USER_LED
  pinMode(USER_LED, OUTPUT);		
  digitalWrite(USER_LED,HIGH);
  
  power.connectPiPower(false);  
  
  //button interrupt
  attachInterrupt(1, button_isr, LOW);
  power.RTCInit(true);
}

void loop() 
{
    bool piRunFlag;
    int buttonPressedTime;

    switch(buttonState)
    {
        case buttonState_Wait:
        
            power.RTCClearInterrupts();
            
            attachInterrupt(0, alarm_isr, FALLING);
            
            power.setTimer1(Timer_Timebase, Timer_Value);
			
            //arduino goes to low power model
            power.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
            
			//trigger by the alarm every 5s
           if(buttonPressed == false)
           {
				//Light the LED,just for observe the state change
                digitalWrite(USER_LED,LOW);
				#if(!USE_CURRENT_TO_CHECK_PI_STATUS)
					//default use pin to check pi status
					piRunFlag = power.getPiStatus(false);
				#else
					//when use current to check pi status
					piRunFlag = power.getPiStatus(CURRENT_THRESHOLD_MA,false);
				#endif
                switch(piState)
                {            
                  case piState_BOOTING:
                       if(piRunFlag == true)
                       {
                           piState = piState_ON;                  
                       }
                       else 
                       {
                           piState = piState_BOOTING;                   
                       } 
                       break;
                  case piState_ON:
                       if(piRunFlag == false)
                       {
                           //maybe user power pi off, So cut the power now          
                           power.connectPiPower(false);
                           piState = piState_OFF;                           
                       }
                       else 
                       {
                           piState = piState_ON;                   
                       } 
                       break;
                  case piState_SHUTTING_DOWN:
                      // Is it still shutting down? 
                       if(piRunFlag == false)
                       {
                           //not shutdown,so just cut the power.               
                           power.connectPiPower(false);
                           piState = piState_OFF;                           
                       }
                       else 
                       {
                           // Still shutting down
                           piState = piState_SHUTTING_DOWN;                   
                       } 
                      break;
                  case piState_OFF:             
                  default:
					 //The Raspberry Pi is still poweroff,so wait again
                     delay(10);
                     piState = piState_OFF;
                     break; 
                }
                buttonState = buttonState_Wait;
                digitalWrite(USER_LED,HIGH);
				
                //Alarm,and then disable the interrupt
                detachInterrupt(0);
				
				//Ack the alarm
                power.ackTimer1();                 
           }
		   //button pressed now:
           else
           {
              buttonPressed = false;
			  
              //Disable RTC interrupt
              detachInterrupt(0);              
              //Disable external pin(button) interrupt 
              detachInterrupt(1);
			  
              buttonState = buttonState_Pressed; 
           }
           break;
        case buttonState_Pressed:
            buttonPressed = false;
			//log the button pressed time
            timePress = millis();              
            #if(!USE_CURRENT_TO_CHECK_PI_STATUS)
				//default use pin to check pi status
				piRunFlag = power.getPiStatus(false);
			#else
				//when use current to check pi status
				piRunFlag = power.getPiStatus(CURRENT_THRESHOLD_MA,false);
			#endif
            if(piRunFlag == false)
            {  
                // Switch on the Pi
                power.connectPiPower(true);
                piState = piState_BOOTING;   
            }          
            buttonState = buttonState_WaitRelease;
            digitalWrite(USER_LED,LOW);
			//when released,caused a interrupt
            attachInterrupt(1, button_isr, HIGH);          
            break;
        case buttonState_WaitRelease:
            if(buttonPressed == true)
            {
				//Disable external pin(button) interrupt 
                detachInterrupt(1); 
                buttonPressed = false;
				//Log the button released time
                time = millis();
                buttonState = buttonState_Released;
            }
            else
            {
                //button has not released yet,wait again
                buttonState = buttonState_WaitRelease;  
            }
            break;
        case buttonState_Released:           
            #if(!USE_CURRENT_TO_CHECK_PI_STATUS)
				//default use pin to check pi status
				piRunFlag = power.getPiStatus(false);
			#else
				//when use current to check pi status
				piRunFlag = power.getPiStatus(CURRENT_THRESHOLD_MA,false);
			#endif
            if(piRunFlag == true)
            {
                //Check how long the key has pressed
                buttonPressedTime = time - timePress;
				
				//hold so long,then just cut the power
                if(buttonPressedTime > CUT_POWER_TIME_MS)
                {
                   //cut the power             
                   power.connectPiPower(false);
                   piState = piState_OFF;          
                } 
				//hold just a little time,then just start a soft shutdown
                else if (buttonPressedTime > SHUTDOWN_TIME_MS)
                {
                    //start a soft shutdown
                    piState = piState_SHUTTING_DOWN;
					#if(!USE_CURRENT_TO_CHECK_PI_STATUS)
						//default use pin to check pi has shut down
						power.shutPiDown();
					#else
						//also,you can use current to check pi has shut down
						power.shutPiDown(CURRENT_THRESHOLD_MA);
					#endif
                } 
                else 
                { 
                     //hols not long enough,then do nothing.
                } 
            } 
            else 
            {
                //why button released, the Raspberry Pi power has already power off ? 
            }
            digitalWrite(USER_LED,HIGH);
			//when pressed,cause a interrupt
            attachInterrupt(1, button_isr, LOW);
            buttonState = buttonState_Wait;        
            break;
        default:
            break;
    }
}
