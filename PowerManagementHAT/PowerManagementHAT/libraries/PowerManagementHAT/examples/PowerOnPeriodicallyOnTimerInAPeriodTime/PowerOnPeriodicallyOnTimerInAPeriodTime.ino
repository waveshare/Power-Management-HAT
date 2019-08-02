/******************************************************************************
 * File Name          : PowerOnPeriodicallyOnTimerInAPeriodTime.ino
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

const int LED_PIN = 13;

//-------------------------------------------------------------------------------------

//Timebase set to seconds. Other options: eTB_SECOND, eTB_MINUTE, eTB_HOUR
//eTIMER_TIMEBASE  PERIODIC_TIMER_TIMEBASE = eTB_MINUTE;

#define  PERIODIC_TIMER_TIMEBASE  eTB_MINUTE

//Raspberry Pi awake time in minute 
#define  PERIODIC_TIMER_VALUE     5

//Raspberry Pi Stay Awake time in ms
#define  RPI_STAY_AWAKE_TIME_MS   60000

//Wakeup Hour in 24 hour clock
#define  WAKE_UP_START_HOUR       8
//Wakeup Minute
#define  WAKE_UP_START_MINUTE     30
//Sleep Houe in 24 hour clock
#define  SLEEP_START_HOUR        18
//Sleep Minute
#define  SLEEP_START_MINUTE      30

//-------------------------------------------------------------------------------------

tmElements_t tm;

void alarm_isr()
{
	//RTC clock alarm interrupt
}

void setup()
{ 
  //Configure output pin
  pinMode(LED_PIN, OUTPUT);

  //turn off the light
  digitalWrite(LED_PIN,HIGH);

  // initialize serial communication
  Serial.begin(115200);
  Serial.println("I am goning to WakePiPeriodicallyInSpecifiedTime");
  delay(60);
  
  //RTC init with a reset
  power.RTCInit(true);

  // Just set the time(Compiled) to RTC clock.
  if (getDate(__DATE__) && getTime(__TIME__)){
      power.setTime(DateTime(F(__DATE__), F(__TIME__)));
  } 
  
  //print the time in serial monitor
  printTimeNow();   

  Serial.print("Periodic Interval Set for: ");
  Serial.print(PERIODIC_TIMER_VALUE);
  switch(PERIODIC_TIMER_TIMEBASE)
  {
    case eTB_SECOND:
      Serial.print(" seconds");
      break;
    case eTB_MINUTE:
      Serial.print(" minutes");
      break;
    case eTB_HOUR:
      Serial.print(" hours");
    default:
        Serial.print(" unknown timebase");
        break;
  }
  Serial.print(" + ");
  Serial.print(RPI_STAY_AWAKE_TIME_MS / 1000);
  Serial.println(" seconds");

}

void loop() 
{
    DateTime now;
  
    power.RTCClearInterrupts();

    //allow RTC alarm pin to trigger interrupt
    attachInterrupt(0, alarm_isr, FALLING);

    //set the periodically wake timer
    power.setTimer1(PERIODIC_TIMER_TIMEBASE, PERIODIC_TIMER_VALUE);

	//when clock is set,then print the register of the RTC
	//PrintRTCRegisters();
	//Serial print requires time,so delay here
	//delay(500);

	//just like enter the low power mode
	//it will wake up util the wakeup pin is low(just like the alert rings)
    power.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 

    //Disable external pin interrupt on wake up pin.
    detachInterrupt(0);
    
	//ack the timer1
    power.ackTimer1();

	//NOW,the alarm clock rings.
	//in here,you can do some task
    digitalWrite(LED_PIN,LOW);
    Serial.println("I wake up periodically during the specified time");
	
    //Print the time
    printTimeNow();
	
	//print the time requires time
    delay(50);
	
	//turn off LED
    digitalWrite(LED_PIN,HIGH);

    now = power.readTime();
		
	//check whether the time is in specified time
	if((now.hour()*60 + now.minute()) > (WAKE_UP_START_HOUR*60 + WAKE_UP_START_MINUTE)){
		if((now.hour()*60 + now.minute()) < (SLEEP_START_HOUR*60 + SLEEP_START_MINUTE)){

			//if it is,then power on the pi
			power.connectPiPower(true);
			
			//the pi is powered on,and delay for RPI_STAY_AWAKE_TIME_MS ms
			delay(RPI_STAY_AWAKE_TIME_MS);    
			
			//has already wake for that period,so shutdown
			power.shutPiDown();
		}
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
