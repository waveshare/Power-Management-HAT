/******************************************************************************
 * File Name          : MeasureVoltage_ShutdownInLowVoltage.ino
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

#define SHUTDOWN_TIME_MS    2000
#define CUT_POWER_TIME_MS   8000

//                          1S(not support)   2s      3s 	  4S
//ON_VOLTAGE_VOLT                   (3.3)          6.6    9.9	 13.2
//OFF_VOLTAGE_VOLT                  (3.15)         6.3    9.45 	 12.6
//FORCE_OFF_VOLTAGE_VOLT            (2.9)          5.8    8.7 	 11.6

//
#define	ON_VOLTAGE_VOLT          6.6

//
#define OFF_VOLTAGE_VOLT         6.3

//
#define FORCE_OFF_VOLTAGE_VOLT   5.8

//very low voltage more than this time will start a shudown
#define LOW_VOLTAGE_TIME_MS      30000ul

//if botton pressed,then you will have more time
#define OVERRIDE_TIME_MS         3600000ul

//-------------------------------------------------------------------------------------

// States
typedef enum {
    buttonState_Wait = 0,
    buttonState_Pressed,
    buttonState_Released
}buttonStateEnmu;

typedef enum {
    piState_OFF = 0,
    piState_BOOTING,
    piState_ON,
    piState_SHUTTING_DOWN
}piStateEnmu;

const int USER_LED = 13;

volatile bool  alarmFired = false;
volatile bool  buttonPressed = false;

//define button pressed state
buttonStateEnmu   buttonState = buttonState_Released;

//define pi working state
piStateEnmu       piState = piState_OFF;


bool state = LOW;
unsigned long  time,
               timeLow = 0,
               timeVeryLow = 0,
               timePress = 0;

			   
//time for RTC to wake pi up periodically
eTIMER_TIMEBASE  Timer_Timebase     = eTB_SECOND;
uint8_t          Timer_Value        = 10;


void button_isr()
{
    buttonPressed = true;
}

void alarm_isr()
{
    alarmFired = true;
}


void setup()
{
    //configure The LED Output
    pinMode(USER_LED, OUTPUT);
    digitalWrite(USER_LED,HIGH);

    power.connectPiPower(false);
    
    //configure alarm pin interrupt
    attachInterrupt(0, alarm_isr, FALLING);

    //configure button pin interrupt
    attachInterrupt(1, button_isr, FALLING);
    
    // Initialize serial communication:
    Serial.begin(115200);
    Serial.println("Measure the Raspberry Pi Voltage");
    delay(50);
    
	//Init RTC clock
    power.RTCInit(true);
    
	// FIXME: Not sure why we need this line
    power.RTCClearInterrupts();
	
    //Set the RTC interrupt Timer
    power.setTimer1(Timer_Timebase, Timer_Value);
}

void loop()
{
    bool   pi_running;
    float  supply_voltage;

	//Cut Power if we detect Pi not running
    pi_running = power.getPiStatus(true);
    if(pi_running == false){
        delay(500);
        power.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
        pi_running = power.getPiStatus(false);
    }

    time = millis();
	
    //just roll the clock
    if(time < timeLow ||
       time < timeVeryLow ||
       time < timePress){
        timeLow = time;
        timeVeryLow = time;
        timePress = 0;
    }

    if(alarmFired == true){
        power.ackTimer1();
        alarmFired = false;
    }    

    //once button pressed,the interrupt occur,and buttonPressed become true
    if(buttonPressed == true){
		//disable Pin interrupt,and buttonPressed flag become false
        detachInterrupt(1);
        buttonPressed = false;
        switch(buttonState){
            case buttonState_Released:
                // Button pressed
                timePress = millis();
                pi_running = power.getPiStatus(false);
				// if pi hasn't power on,then power on
                if(pi_running == false){
                    power.connectPiPower(true);    
                }
                buttonState = buttonState_Pressed;
				//when pressed,the LED turn on
                digitalWrite(USER_LED,LOW);
				//when pin level goes high,the interrupt occur
                attachInterrupt(1, button_isr, HIGH);
                break;
            case buttonState_Pressed:
                // Button Released
                unsigned long buttonTime;
                buttonState = buttonState_Released;
                pi_running = power.getPiStatus(false);
                if(pi_running == true){
                    //check for how long the button has been pressed
                    buttonTime = time - timePress;
					
					//hold so long,then just cut the power
                    if(buttonTime > CUT_POWER_TIME_MS){
                        //cut the power 
                        power.connectPiPower(false);
                        
                    }
					//hold just a little time,then just start a soft shutdown
					else if (buttonTime > SHUTDOWN_TIME_MS){
                        //start a soft shutdown
                        power.shutPiDown();
                        
                    } else {
                        //hols not long enough,then do nothing.
                    }
                } else {
                    //why button released, the Raspberry Pi power has already power off ? 
                }
                digitalWrite(USER_LED,HIGH);
                attachInterrupt(1, button_isr, FALLING);    // button pin       
                break;
            default:
                break;
        }
    }

    //measure the voltage and decide to power on or off
    delay(30);
    supply_voltage = power.measureVoltage();
    if(pi_running == true){
        if(supply_voltage > OFF_VOLTAGE_VOLT){
            //Voltage is normal,reset the POWER_OFF counter
            timeLow = time;
        }
        if(supply_voltage > FORCE_OFF_VOLTAGE_VOLT){
			//voltage is normal,reset the FORCE_POWER_OFF counter 
            timeVeryLow = time;
        }
        
		//check the pi power:
		//if from timeVeryLow to now longer than LOW_VOLTAGE_TIME_MS,then shutdown the pi
		//if from timeLow to now longer LOW_VOLTAGE_TIME_MS and (button hasn't pressed or from pressed to now longer than OVERRIDE_TIME_MS)
		//then shutdown the pi
        if(time - timeVeryLow > LOW_VOLTAGE_TIME_MS || (
           time - timeLow > LOW_VOLTAGE_TIME_MS &&
           (timePress == 0 || time - timePress > OVERRIDE_TIME_MS))){
            // Start a soft shutdown
            power.shutPiDown();
            
        }
        //print the Raspberry Pi voltage to PC Host
        Serial.println(supply_voltage);
        delay(500);
    } else {
        //if the power voltage recover over ON_VOLTAGE_VOLT again
        if(supply_voltage >= ON_VOLTAGE_VOLT){
            //then connect the power to Pi
            power.connectPiPower(true);
            
        }
    }
}
