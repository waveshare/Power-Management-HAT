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

#ifndef PowerManagementHAT_H
#define PowerManagementHAT_H

#include "Arduino.h"
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <PCF8523.h>
#include <Time.h>
#include <Wire.h>
#include <LowPower.h>

//The max time to shutdown the Raspberry Pi
#define MAX_SHUT_DOWN_TIME		60000

/*PowerManagement class description*/
class PowerManagement : public PCF8523 , public LowPowerClass {

	//public function for user to use it
	public:
		bool isPiRunning;
		bool powerOn;

		/*Initialize the constructor*/
		PowerManagement();

		/*Basic control of the turn on the power*/
		void connectPiPower(bool enable);

		/*Basic control of shut down the power*/
		void  shutPiDown(void);
		void  shutPiDown(long thresholdCurrent_mA);
				
		/*Check status*/
		bool  getPiStatus(bool forceShutdownIfNotWorking);
		bool  getPiStatus(long thresholdCurrent_mA, bool forceShutdownIfNotWorking);

		/*Measure voltage and current*/
		float measureVoltage(void);
		float measureCurrent(void);
		
		/*Some function with PCF8523 RTC*/
		bool RTCInit(bool reset); 
		void RTCAlarmClockEnable(bool enable);
		uint8_t RTCClearInterrupts(void);
};

extern PowerManagement power;
//end of PowerManagementHAT_H
#endif
