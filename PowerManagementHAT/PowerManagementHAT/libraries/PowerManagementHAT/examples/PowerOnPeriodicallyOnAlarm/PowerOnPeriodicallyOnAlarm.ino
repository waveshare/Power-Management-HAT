/******************************************************************************
 * File Name          : PowerOnPeriodicallyOnAlarm.ino
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

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
}; 

const int USER_LED = 13;

//-------------------------------------------------------------------------------------

//Raspberry Pi awake time in minute 
#define  RPI_WAKE_UP_CYCLE_MINUTE   10

//Raspberry Pi Stay Awake time in ms
#define  RPI_STAY_AWAKE_TIME_MS     60000

//When then current is below this, It is considered to be shutdown
#define  CURRENT_THRESHOLD_MA       110

//-------------------------------------------------------------------------------------

tmElements_t tm;
uint8_t   nextWakeTime;


void alarm_isr()
{
	//alarm interrupt
}

void setup()
{
	//turn off the USER_LED
	pinMode(USER_LED, OUTPUT);   
	digitalWrite(USER_LED,HIGH);

	//initialize serial
	Serial.begin(115200);
	Serial.println("I am going to wake up periodically by alarm clock");
	delay(500);

	power.connectPiPower(false);
	
	power.RTCInit(true);

	//Just set the time(Compiled) to RTC clock.
	if (getDate(__DATE__) && getTime(__TIME__)) {
	// and configure the RTC with this info
	  power.setTime(DateTime(F(__DATE__), F(__TIME__)));
	}  

	//print the time in serial monitor
	printTimeNow();

	Serial.print("Raspberry Pi Wakeup Cycle is:");
	Serial.print(RPI_WAKE_UP_CYCLE_MINUTE);     
	Serial.println("minutes");         

	//Calculate the next time to power on
	nextWakeTime = CalcNextWakeTime();
}

void loop() 
{
    unsigned long TimeOutStart, ElapsedTimeMs,TimeOutEnd ;
    bool pi_running;
  
    //RTC alarm interrupt in interrupt0
    attachInterrupt(0, alarm_isr, FALLING);

    power.RTCAlarmClockEnable(true);
    
    //Set up the alarm time
    power.setAlarm(nextWakeTime);     
    
	//Just for debug
    //PrintRTCRegisters();
    
    delay(500);

    //just like enter the low power mode
    //it will wake up util the wakeup pin is low(just like the alert rings)
    power.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
    
    //Alarm,and then disable the interrupt
    detachInterrupt(0); 
    
	//Ack the alarm
    power.ackAlarm();

    //Calculate the next time to power pi
    nextWakeTime = CalcNextWakeTime();

	//Power pi on
    power.connectPiPower(true);    

	//Light the LED
    digitalWrite(USER_LED,LOW);
	
	
    Serial.println("I've Just woken up on the Alarm!");
	
    //Print the time
    printTimeNow();
	
	//print the time requires time,so delay
    delay(50);
	
	//turn off the LED
    digitalWrite(USER_LED,HIGH); 

	//you can do some other task here
	
	//wait to boot up a bit,of course,this is included in RPI_STAY_AWAKE_TIME_MS
    delay(10000);
    TimeOutStart = millis();
    ElapsedTimeMs = TimeOutStart;
    TimeOutEnd = TimeOutStart + RPI_STAY_AWAKE_TIME_MS;
    pi_running = power.getPiStatus(CURRENT_THRESHOLD_MA,false); 

	//this time,which is pi wakeup time in RPI_STAY_AWAKE_TIME_MS
    while((pi_running == true) && (ElapsedTimeMs < TimeOutEnd))
    {
        pi_running = power.getPiStatus(CURRENT_THRESHOLD_MA,false); 
        ElapsedTimeMs = millis();
        delay(1000);
		
		//just for debug and tuning to see current change
		Serial.print("state:");
        Serial.print(pi_running);
		Serial.print(" ");
        Serial.print("current:");
        Serial.println(power.measureCurrent());
    }      

    //after run in RPI_STAY_AWAKE_TIME_MS minutes,still running
    if(pi_running == true){
        //then start a shutdown,this will cut the power
        power.shutPiDown();
    }
    else {
        //already shutdown(user shutdown),then just cut the power
        power.connectPiPower(false);   
    }
}



uint8_t CalcNextWakeTime(void)
{
  DateTime now = power.readTime();
    
  nextWakeTime = now.minute() + RPI_WAKE_UP_CYCLE_MINUTE;
  if(nextWakeTime == 60){
      nextWakeTime = 0; 
  }
  else if (nextWakeTime > 60){
      nextWakeTime = nextWakeTime - 60;  
  }
 
  return nextWakeTime;
}


void printTimeNow()
{
    // Read the time
    DateTime now = power.readTime();
    
    // Print out the time
    Serial.print("Ok, Time = ");
    print2digits(now.hour());
    Serial.write(':');
    print2digits(now.minute());
    Serial.write(':');
    print2digits(now.second());
    Serial.print(", Date (D/M/Y) = ");
    Serial.print(now.day());
    Serial.write('/');
    Serial.print(now.month()); 
    Serial.write('/');
    Serial.print(now.year(), DEC);
    Serial.println();

    return;
}


bool getTime(const char *str)
{
	int Hour, Min, Sec;

	if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3){
		return false;
	}
	tm.Hour = Hour;
	tm.Minute = Min;
	tm.Second = Sec;
	return true;
}

bool getDate(const char *str)
{
	char Month[12];
	int Day, Year;
	uint8_t monthIndex;

	if(sscanf(str, "%s %d %d", Month, &Day, &Year) != 3){
		return false;
	}
	for (monthIndex = 0; monthIndex < 12; monthIndex++) {
		if (strcmp(Month, monthName[monthIndex]) == 0){
			break;
		}
	}
	if (monthIndex >= 12){
		return false;
	}
	tm.Day = Day;
	tm.Month = monthIndex + 1;
	tm.Year = CalendarYrToTm(Year);
	return true;
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

void PrintRTCRegisters(void)
{
  
      // Debug
      uint8_t reg_value;
      reg_value = power.rtcReadReg(PCF8523_CONTROL_1);
      Serial.print("Control 1: 0x");
      Serial.println(reg_value,HEX);
      reg_value = power.rtcReadReg(PCF8523_CONTROL_2);
      Serial.print("Control 2: 0x");
      Serial.println(reg_value, HEX);      
      reg_value = power.rtcReadReg(PCF8523_CONTROL_3);
      Serial.print("Control 3: 0x");
      Serial.println(reg_value,HEX); 
      
      reg_value = power.rtcReadReg(PCF8523_SECONDS);
      Serial.print("Seconds: ");
      Serial.println(reg_value,HEX);
      reg_value = power.rtcReadReg(PCF8523_MINUTES);
      Serial.print("Minutes: ");
      Serial.println(reg_value,HEX);  
      reg_value = power.rtcReadReg(PCF8523_HOURS);
      Serial.print("Hours: ");
      Serial.println(reg_value,HEX);  
      reg_value = power.rtcReadReg(PCF8523_DAYS);
      Serial.print("Days: ");
      Serial.println(reg_value,HEX);   
      reg_value = power.rtcReadReg(PCF8523_WEEKDAYS);
      Serial.print("Week Days: ");
      Serial.println(reg_value,HEX);    
      reg_value = power.rtcReadReg(PCF8523_MONTHS);
      Serial.print("Months: ");
      Serial.println(reg_value,HEX);  
      reg_value = power.rtcReadReg(PCF8523_YEARS);
      Serial.print("Years: ");
      Serial.println(reg_value,HEX); 
      
      reg_value = power.rtcReadReg(PCF8523_MINUTE_ALARM);
      Serial.print("Minute Alarm: ");
      Serial.println(reg_value,HEX);      
      reg_value = power.rtcReadReg(PCF8523_HOUR_ALARM);
      Serial.print("Hour Alarm: ");
      Serial.println(reg_value,HEX);  
      reg_value = power.rtcReadReg(PCF8523_DAY_ALARM);
      Serial.print("Day Alarm: ");
      Serial.println(reg_value,HEX);      
      reg_value = power.rtcReadReg(PCF8523_WEEKDAY_ALARM);
      Serial.print("Weekday Alarm: ");
      Serial.println(reg_value,HEX); 
      
      reg_value = power.rtcReadReg(PCF8523_OFFSET);
      Serial.print("Offset: 0x");
      Serial.println(reg_value,HEX); 
      reg_value = power.rtcReadReg(PCF8523_TMR_CLKOUT_CTRL);
      Serial.print("TMR_CLKOUT_CTRL: 0x");
      Serial.println(reg_value,HEX);  
      reg_value = power.rtcReadReg(PCF8523_TMR_A_FREQ_CTRL);
      Serial.print("TMR_A_FREQ_CTRL: 0x");
      Serial.println(reg_value,HEX); 
      reg_value = power.rtcReadReg(PCF8523_TMR_A_REG);
      Serial.print("TMR_A_REG: 0x");
      Serial.println(reg_value,HEX);     
      reg_value = power.rtcReadReg(PCF8523_TMR_B_FREQ_CTRL);
      Serial.print("TMR_B_FREQ_CTRL: 0x");
      Serial.println(reg_value,HEX);    
      reg_value = power.rtcReadReg(PCF8523_TMR_B_REG);
      Serial.print("TMR_B_REG: 0x");
      Serial.println(reg_value,HEX); 
}

