/******************************************************************************
 * File Name          : Power-Management-HAT.cpp
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

#define ARD_ENABLE_PI		16				// PC2|______|O/P|______|take high to enable the RaspPi-------------------______|Active High
#define ARD_PI_SHUT_DOWN	17				// PC3|______|0/P|______|Handshake to request the Pi to shutdown----------______|Active high
#define ARD_PI_IS_RUNNING	7				// PD7|______|I/P|______|Handshake to show that the Pi is running---------______|Active High  
#define ARD_USER_BUTTON		3				// PD3|______|I/P|______|User Power-on Button (INT1)----------------------______|Active Low
#define ARD_RTC_ALARM		2				// PD2|______|I/P|______|Pin that pulses when the alarm has expired (INT0)______|Active Low

#define VOLTAGE				A6				// I/P|______|A/I|______|Supply monitoring pin----------------------------______|ADC Pin
#define CURRENT				A7				// I/P|______|A/I|______|Current monitoring pin---------------------------______|ADC Pin	

//Constructor
PowerManagement::PowerManagement(){
	
	//----------------some value-------------------------
		
	//pi statue
	isPiRunning = false;
	
	//when ARD_ENABLE_PI become high level,then powwerOn become true
	powerOn = false;

	//----------------output pin-------------------------
	
	//Configure Pi Power
	pinMode(ARD_ENABLE_PI, OUTPUT);
	
	//Pi is Off at Startup
	PowerManagement::connectPiPower(false);
	
	//configure ARD_PI_SHUT_DOWN pin to output,pull up will shutdown pi
	pinMode(ARD_PI_SHUT_DOWN, OUTPUT);
	
	//at the start up pull down mean don't shutdown pi
	digitalWrite(ARD_PI_SHUT_DOWN,LOW);	
	
	//----------------input pin-------------------------
	
	//Configure Pi Shutdown handshake from Pi goes high when Pi is running
	pinMode(ARD_PI_IS_RUNNING, INPUT);    
	
	//Initialize the button pin as a input,also can be used as an interrupt INT1
	pinMode(ARD_USER_BUTTON, INPUT);
	
	//Initialize the alarm as a input,also can be used as an interrupt INT0
	pinMode(ARD_RTC_ALARM, INPUT);	
}

/* 
 * brief : enable normal power
 * param : true   if enable normal power
 *		   false  disable normal power
 */
void PowerManagement::connectPiPower(bool enable){
	if(enable == true){
		digitalWrite(ARD_ENABLE_PI,HIGH);
		powerOn = true;
	}
	else {
		digitalWrite(ARD_ENABLE_PI,LOW);
		powerOn = false;
	}
}

/*
 *brief:
 *		shutDown the Raspberry Pi by pull ARD_PI_SHUT_DOWN up
 *		wait ARD_PI_IS_RUNNING become low level means Pi has shudown
 *		when ARD_PI_IS_RUNNING become low level or timeout,then cut the power
 */
void PowerManagement::shutPiDown(void){
	int	handShake;
	unsigned long timeStart, timeNow, testTime;

	//pull up to send shutdown commander
	digitalWrite(ARD_PI_SHUT_DOWN,HIGH);		
	
	//wait for max 60s for Pi to shutdown 
	timeStart = millis();
	testTime = 0;
	handShake = digitalRead(ARD_PI_IS_RUNNING);
	while((handShake > 0) && (testTime < MAX_SHUT_DOWN_TIME)){
		handShake = digitalRead(ARD_PI_IS_RUNNING);  
		delay(50);
		timeNow = millis();
		testTime = timeNow - timeStart;
	}
	//when program comes to here,means has wait for max 60s(MAX_SHUT_DOWN_TIME)
	delay(6000);
	//so directly to cut the power
	PowerManagement::connectPiPower(false);
	digitalWrite(ARD_PI_SHUT_DOWN,LOW);
}

/*
 *brief:
 *		shutDown the Raspberry Pi by pull ARD_PI_SHUT_DOWN up
 *		wait currtnt become below thresholdCurrent_mA
 *		when current become below thresholdCurrent_mA or timeout,then cut the power
 */
void PowerManagement::shutPiDown(long thresholdCurrent_mA){
	bool isPiRunning;
	unsigned long timeStart, timeNow, testTime;
	
	// Command the Sleepy Pi to shutdown
	digitalWrite(ARD_PI_SHUT_DOWN,HIGH);		
		
	// Wait for max 60s for Pi to shutdown
	timeStart = millis();
	testTime = 0;
	isPiRunning = PowerManagement::getPiStatus(thresholdCurrent_mA,false);
	while((isPiRunning == true) && (testTime < MAX_SHUT_DOWN_TIME)){
		isPiRunning = PowerManagement::getPiStatus(thresholdCurrent_mA,false); 
		delay(50);
		timeNow = millis();
		testTime = timeNow - timeStart;
	}
	//when program comes to here,means has wait for max 60s(MAX_SHUT_DOWN_TIME)
	delay(6000);
	//so directly cut the power
	PowerManagement::connectPiPower(false);
	digitalWrite(ARD_PI_SHUT_DOWN,LOW);
}

/* 
 * brief : Check if Pi is running
 * param : forceShutdownIfNotWorking:
 * if pi is not running,then shutdown 	   
 */
bool PowerManagement::getPiStatus(bool forceShutdownIfNotWorking){
	
	int	handShake;
	
	handShake = digitalRead(ARD_PI_IS_RUNNING);	
	
	//if handShake>0,means pi is still running
	if(handShake > 0) {
		isPiRunning = true;
		return true;
	}else{
		//need to force shutdown?
		if(forceShutdownIfNotWorking == true){
			//it was running
			if(isPiRunning == true){
				//but now,isn't running,cut the power directly
				PowerManagement::connectPiPower(false);
				isPiRunning = false;
			}
		}
		return false;
	}
}

/* 
 * brief : check Pi status by current
 * param : 
 *		   thresholdCurrent_mA:
 *		   if working current is more than thresholdCurrent_mA, then it is working
 *
 *         forceShutdownIfNotWorking:
 *         need to shutdown ,if it is not running
 */
bool PowerManagement::getPiStatus(long thresholdCurrent_mA, bool forceShutdownIfNotWorking){

	float electricCurrent = 0.0;

	electricCurrent = PowerManagement::measureCurrent();
	if(electricCurrent >= (float)thresholdCurrent_mA){
		isPiRunning = true;
		return true;
	}else{
		if(forceShutdownIfNotWorking == true){
			PowerManagement::connectPiPower(false);
			isPiRunning = false;
		}
		return false;
	}
}

/*
 * brief: read the voltage to predict the power voltage
 */
float PowerManagement::measureVoltage(void){
	int 	tempVoltage;
	float   powerVoltage;
	
	tempVoltage = analogRead(VOLTAGE);   
	
	// 3.3V/1024 = 3.22mV
	powerVoltage = 3.22*(float)tempVoltage;
	// covert it to reallt voltage
	powerVoltage /= 52;
	
	return powerVoltage;
}

/*
 * brief: read the voltage to predict the power voltage
 */
float PowerManagement::measureCurrent(void){
	int 	tempCurrent;
	float   powerCurrent;

	tempCurrent = analogRead(CURRENT);  
	//filter the noise
	if(tempCurrent <= 3){
		tempCurrent = 0;
	}
	
	//3.3V/1024 = 3.22mV
	powerCurrent = 3.22*(float)tempCurrent;
	
	return  powerCurrent;           
}

/*
 *brief: Init the RTC clock
 *params: reset
 *		true: reset the RTC clock
 *		false: does't reset the RTC clock
 */
bool PowerManagement::RTCInit(bool reset){

	uint8_t tmp_reg;
	begin();
	
	//do this to check whether PCF8523 is exist
	tmp_reg = rtcReadReg(PCF8523_CONTROL_3);
	if(tmp_reg == 0xFF){
		//means PCF8523 is not detected
		return false;
	}

	//then reset the clock now
	if(reset){
		PowerManagement::reset();
	}
	
	//do some deal with it RTC clock
	PowerManagement::stop_32768_clkout();  
	PowerManagement::setBatterySwitchover();
	PowerManagement::rtcCapSelect(eCAP_12_5pF);
	PowerManagement::clearRtcInterruptFlags();

	return true;
}

/* 
 * brief : enable RTC alarm
 * param : true   if enable RTC alarm
 *		   false  disable RTC alarm
 * This interface is used for ino files
 */
void PowerManagement::RTCAlarmClockEnable(bool enable){
	enableAlarm(enable);
}

/*
 * brief: clear RTC interrupt
 * Thia interface is used for ino files
 */
uint8_t PowerManagement::RTCClearInterrupts(){	
	return clearRtcInterruptFlags();
}

PowerManagement power;