/******************************************************************************
 * File Name          : PowerOnInAPeriodTime.ino
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

//Wakeup Hour in 24 hour clock
#define  WAKE_UP_START_HOUR      8
//Wakeup Minute
#define  WAKE_UP_START_MINUTE    30

//Sleep Houe in 24 hour clock
#define  SLEEP_START_HOUR       18
//Sleep Minute
#define  SLEEP_START_MINUTE     30 
//default,your pi will wakeUp in 8:30，Sleep in 18:30

//-------------------------------------------------------------------------------------

tmElements_t tm;
bool  piAwake = false;

void alarm_isr()
{
	//RTC clock alarm interrupt
}

void setup()
{
	//Configure output pin
	pinMode(USER_LED, OUTPUT);
	
	//turn off the light
	digitalWrite(USER_LED,HIGH);
	
	//initialize serial communication
	Serial.begin(115200);
	Serial.println("System is woking now");
	delay(60);

	//RTC init with a reset
	power.RTCInit(true);

	// Just set the time(Compiled) to RTC clock.
	if (getDate(__DATE__) && getTime(__TIME__)) {
		power.setTime(DateTime(F(__DATE__), F(__TIME__)));
	}

	//print the time in serial monitor
	printTimeNow();

	Serial.print("Alarm Set for: ");
	Serial.print(WAKE_UP_START_HOUR);
	Serial.print(":");
	Serial.println(WAKE_UP_START_MINUTE);

	Serial.print("Bedtime Set for: ");
	Serial.print(SLEEP_START_HOUR);
	Serial.print(":");
	Serial.println(SLEEP_START_MINUTE);
}

void loop() 
{
	if(piAwake == false)
	{
		power.RTCClearInterrupts();
	  
		//allow RTC alarm pin to trigger interrupt
		attachInterrupt(0, alarm_isr, FALLING);

		//enable RTC alarm clock
		power.RTCAlarmClockEnable(true);
		
		//configue the wake up time
		power.setAlarm(WAKE_UP_START_HOUR,WAKE_UP_START_MINUTE);      
		
		//when clock is set,then print the register of the RTC
		//PrintRTCRegisters();
		//Serial print requires time,so delay here
		//delay(500);
				
		//just like enter the low power mode
		//it will wake up util the wakeup pin is low(just like the alert rings)
		power.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
		
		//Alarm,and then disable the interrupt
		detachInterrupt(0);

		//ack the alarm clock.
		power.ackAlarm();
		
		//NOW,the alarm clock rings.
		//in here,you can do some task
		digitalWrite(USER_LED,LOW);
		Serial.println("wake up");
		
		//Print the time
		printTimeNow();
		
		//Print the time requires time
		delay(50);
		
		//turn off the LED
		digitalWrite(USER_LED,HIGH);
		
		//Connect the power to wakeup the Pi
		power.connectPiPower(true); 
		piAwake = true;      
	}
	else
	{		 
		power.RTCClearInterrupts();

		//allow RTC alarm pin to trigger interrupt
		attachInterrupt(0, alarm_isr, FALLING);

		//enable RTC alarm clock
		power.RTCAlarmClockEnable(true);

		//configue the sleep time
		power.setAlarm(SLEEP_START_HOUR,SLEEP_START_MINUTE);   

		//when clock is set,then print the register of the RTC
		//PrintRTCRegisters();
		//Serial print requires time,so delay here
		//delay(500);
		   
		//just like enter the low power mode
		//it will wake up util the wakeup pin is low(just like the alert rings)
		power.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 

		//Disable external pin interrupt on wake up pin.
		detachInterrupt(0); 

		//wait for the alert rings,and then ack the alarm clock.
		power.ackAlarm();              

		//NOW,the alarm clock rings.
		//in here,you can do some task
		digitalWrite(USER_LED,LOW);
		Serial.println("go to bed now!");

		//Print the time
		printTimeNow();

		//print the time requires time
		delay(50);

		digitalWrite(USER_LED,HIGH);

		//Start a soft shutdown to the pi
		power.shutPiDown();         
		piAwake = false;       
	}
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

	if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
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

	if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
	for (monthIndex = 0; monthIndex < 12; monthIndex++) {
	if (strcmp(Month, monthName[monthIndex]) == 0) break;
	}
	if (monthIndex >= 12) return false;
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
	//This message is just for debug
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

