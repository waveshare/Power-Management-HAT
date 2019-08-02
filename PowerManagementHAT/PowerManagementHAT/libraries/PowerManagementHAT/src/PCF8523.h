/*
 * PCF8523.h - library for Power Management HAT

Inspiration
============
This library is built on the PCF8523 Arduino library by 
Alfredo Prado of radikalbytes [at] gmail [dot] com

License
  =======
  The library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 */

#ifndef _PCF8523_H_
#define _PCF8523_H_



// comment this out if you already have a JeeLabs' DateTime class in your code
// or if you don't need DateTime functionality
#define PCF8523_INCLUDE_DATETIME_CLASS

// comment this out if you don't need DateTime functionality
#define PCF8523_INCLUDE_DATETIME_METHODS

#define PCF8523_ADDRESS				0x68 //0xD0
#define PCF8523_DEFAULT_ADDRESS		0x68 //0xD0

#define PCF8523_CONTROL_1			0x00
#define PCF8523_CONTROL_2			0x01
#define PCF8523_CONTROL_3			0x02
#define PCF8523_SECONDS				0x03
#define PCF8523_MINUTES				0x04
#define PCF8523_HOURS				0x05
#define PCF8523_DAYS				0x06
#define PCF8523_WEEKDAYS			0x07
#define PCF8523_MONTHS				0x08
#define PCF8523_YEARS				0x09
#define PCF8523_MINUTE_ALARM		0x0A
#define PCF8523_HOUR_ALARM			0x0B
#define PCF8523_DAY_ALARM			0x0C
#define PCF8523_WEEKDAY_ALARM		0x0D
#define PCF8523_OFFSET				0x0E
#define PCF8523_TMR_CLKOUT_CTRL		0x0F
#define PCF8523_TMR_A_FREQ_CTRL		0x10
#define PCF8523_TMR_A_REG			0x11
#define PCF8523_TMR_B_FREQ_CTRL		0x12
#define PCF8523_TMR_B_REG			0x13

#define PCF8523_CONTROL_1_CAP_SEL_BIT	7
#define PCF8523_CONTROL_1_T_BIT			6
#define PCF8523_CONTROL_1_STOP_BIT		5
#define PCF8523_CONTROL_1_SR_BIT		4
#define PCF8523_CONTROL_1_1224_BIT		3
#define PCF8523_CONTROL_1_SIE_BIT		2
#define PCF8523_CONTROL_1_AIE_BIT		1
#define PCF8523_CONTROL_1CIE_BIT		0

#define PCF8523_CONTROL_2_WTAF_BIT		7
#define PCF8523_CONTROL_2_CTAF_BIT		6
#define PCF8523_CONTROL_2_CTBF_BIT		5
#define PCF8523_CONTROL_2_SF_BIT		4
#define PCF8523_CONTROL_2_AF_BIT 		3
#define PCF8523_CONTROL_2_WTAIE_BIT		2
#define PCF8523_CONTROL_2_CTAIE_BIT		1
#define PCF8523_CONTROL_2_CTBIE_BIT		0

#define PCF8523_SECONDS_OS_BIT			7
#define PCF8523_SECONDS_10_BIT       	6
#define PCF8523_SECONDS_10_LENGTH   	3
#define PCF8523_SECONDS_1_BIT        	3
#define PCF8523_SECONDS_1_LENGTH     	4

#define PCF8523_MINUTES_10_BIT       	6
#define PCF8523_MINUTES_10_LENGTH    	3
#define PCF8523_MINUTES_1_BIT        	3
#define PCF8523_MINUTES_1_LENGTH     	4

#define PCF8523_HOURS_MODE_BIT  	    3 // 0 = 24-hour mode, 1 = 12-hour mode
#define PCF8523_HOURS_AMPM_BIT      	5 // 2nd HOURS_10 bit if in 24-hour mode
#define PCF8523_HOURS_10_BIT        	4
#define PCF8523_HOURS_1_BIT          	3
#define PCF8523_HOURS_1_LENGTH       	4

#define PCF8523_WEEKDAYS_BIT 	        2
#define PCF8523_WEEKDAYS_LENGTH         3

#define PCF8523_DAYS_10_BIT          5
#define PCF8523_DAYS_10_LENGTH       2
#define PCF8523_DAYS_1_BIT           3
#define PCF8523_DAYS_1_LENGTH        4

#define PCF8523_MONTH_10_BIT         4
#define PCF8523_MONTH_1_BIT          3
#define PCF8523_MONTH_1_LENGTH       4

#define PCF8523_YEAR_10H_BIT         7
#define PCF8523_YEAR_10H_LENGTH      4
#define PCF8523_YEAR_1H_BIT          3
#define PCF8523_YEAR_1H_LENGTH       4

const uint8_t RTC_CLKOUT_DISABLED = ((1<<3) | (1<<4) | (1<<5));

#define PCF8523_TMR_CLKOUT_CTRL_TAM_BIT		7
#define PCF8523_TMR_CLKOUT_CTRL_TBM_BIT		6
#define PCF8523_TMR_CLKOUT_CTRL_TBC_BIT		0

typedef enum {
	eSUNDAY = 0,
	eMONDAY,
	eTUESDAY,
	eWEDNESDAY,
	eTHURSDAY,
	eFRIDAY,
	eSATURDAY
} eWEEKDAYS;

typedef enum {
	eTB_4KHZ = 0,
	eTB_64HZ,
	eTB_SECOND,
	eTB_MINUTE,
	eTB_HOUR
} eTIMER_TIMEBASE;

typedef enum {
	eCAP_7pF = 0,
	eCAP_12_5pF
}eCAP_SEL;

typedef enum {
	eAM = 0,
	ePM
}eAM_PM;

typedef enum {
	eTWENTYFOURHOUR = 0,
	eTWELVEHOUR
}e12_24;

typedef struct {
	uint8_t		minutes;
	uint8_t		hours;
	uint8_t     days;
	eWEEKDAYS	weekdays;
	e12_24		twentyFourHour;
	eAM_PM		AmPm;
	bool		minutesEnabled;
	bool 		hoursEnabled;	
	bool 		daysEnabled;
	bool 		weekdaysEnabled;
}ALARM_SETTINGS;

class TimeSpan;

// Simple general-purpose date/time class (no TZ / DST / leap second handling!)
class DateTime {
public:
    DateTime (uint32_t t =0);
    DateTime (uint16_t year, uint8_t month, uint8_t day,
                uint8_t hour =0, uint8_t min =0, uint8_t sec =0);
    DateTime (const DateTime& copy);
    DateTime (const char* date, const char* time);
    DateTime (const __FlashStringHelper* date, const __FlashStringHelper* time);
    uint16_t year() const       { return 2000 + yOff; }
    uint8_t month() const       { return m; }
    uint8_t day() const         { return d; }
    uint8_t hour() const        { return hh; }
    uint8_t minute() const      { return mm; }
    uint8_t second() const      { return ss; }
    uint8_t dayOfWeek() const;

    // 32-bit times as seconds since 1/1/2000
    long secondstime() const;   
    // 32-bit times as seconds since 1/1/1970
    uint32_t unixtime(void) const;

    DateTime operator+(const TimeSpan& span);
    DateTime operator-(const TimeSpan& span);
    TimeSpan operator-(const DateTime& right);

protected:
    uint8_t yOff, m, d, hh, mm, ss;
};

// Timespan which can represent changes in time with seconds accuracy.
class TimeSpan {
public:
    TimeSpan (int32_t seconds = 0);
    TimeSpan (int16_t days, int8_t hours, int8_t minutes, int8_t seconds);
    TimeSpan (const TimeSpan& copy);
    int16_t days() const         { return _seconds / 86400L; }
    int8_t  hours() const        { return _seconds / 3600 % 24; }
    int8_t  minutes() const      { return _seconds / 60 % 60; }
    int8_t  seconds() const      { return _seconds % 60; }
    int32_t totalseconds() const { return _seconds; }

    TimeSpan operator+(const TimeSpan& right);
    TimeSpan operator-(const TimeSpan& right);

protected:
    int32_t _seconds;
};

class PCF8523{

	public:
		static bool begin(void);
		static void setTime(const DateTime& dt);

		static DateTime readTime();

		// Alarms
		void setAlarm(uint8_t minute_alarm ); 
		void setAlarm(uint8_t hour_alarm,uint8_t minute_alarm );
		void setAlarm(uint8_t day_alarm, uint8_t hour_alarm,uint8_t minute_alarm );
		void setWeekDayAlarm(eWEEKDAYS weekday_alarm, uint8_t hour_alarm,uint8_t minute_alarm);
		void getAlarm(uint8_t* buf);
		void getAlarm(ALARM_SETTINGS* settings);
		void enableAlarm(bool enable);
		void ackAlarm(void);

		// Periodic Timers
		void setTimer1(eTIMER_TIMEBASE timebase, uint8_t value);
		void ackTimer1(void);
		uint8_t getTimer1(void);
		void setTimer2(eTIMER_TIMEBASE timebase,uint8_t value);
		void ackTimer2(void);
		uint8_t getTimer2(void);

		// Utility
		void reset();
		uint8_t isrunning(void);
		void rtcStop(void);
		void rtcStart(void);
		bool rtcBatteryLow(void);
		void setTwelveTwentyFourHour(e12_24 mode);
		e12_24 getTwelveTwentyFourHour(void);	
		
		// Register Access
		uint8_t rtcReadReg(uint8_t address);
		void rtcReadReg(uint8_t* buf, uint8_t size, uint8_t address);
		void rtcWriteReg(uint8_t address, uint8_t data);
		void rtcWriteReg(uint8_t address, uint8_t* buf, uint8_t size);

		// void startCounter_1(uint8_t value);
		void setBatterySwitchover(void);
	protected:
		void stop_32768_clkout();
		uint8_t clearRtcInterruptFlags();
		void 	rtcCapSelect(eCAP_SEL value);


};

// RTC using the internal millis() clock, has to be initialized before use
// NOTE: this clock won't be correct once the millis() timer rolls over (>49d?)
class RTC_Millis {
public:
    static void begin(const DateTime& dt) { adjust(dt); }
    static void adjust(const DateTime& dt);
    static DateTime now();

protected:
    static long offset;
};

#endif //_PCF8523_H_  