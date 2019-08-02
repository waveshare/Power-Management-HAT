
/*
 * PCF8523.cpp
  
  Copyright (c) Jon Watkins 2016
  http://spellfoundry.com

  This is a library of functions for use with the PCF8523 RTC.

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

  Releases
  ========
  V1_0 - 27 April 2016  - Initial release

 */

#include <Wire.h>
#include "PCF8523.h"

#ifdef __AVR__
 #include <avr/pgmspace.h>
 #define WIRE Wire
#else
 #define PROGMEM
 #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
 #define WIRE Wire1
#endif


#define SECONDS_PER_DAY 86400L

#define SECONDS_FROM_1970_TO_2000 946684800

#if (ARDUINO >= 100)
 #include <Arduino.h> // capital A so it is error prone on case-sensitive filesystems
 // Macro to deal with the difference in I2C write functions from old and new Arduino versions.
 #define _I2C_WRITE write
 #define _I2C_READ  read
#else
 #include <WProgram.h>
 #define _I2C_WRITE send
 #define _I2C_READ  receive
#endif


////////////////////////////////////////////////////////////////////////////////
// utility code, some of this could be exposed in the DateTime API if needed

const uint8_t daysInMonth [] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 };

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
    if (y >= 2000)
        y -= 2000;
    uint16_t days = d;
    for (uint8_t i = 1; i < m; ++i)
        days += pgm_read_byte(daysInMonth + i - 1);
    if (m > 2 && y % 4 == 0)
        ++days;
    return days + 365 * y + (y + 3) / 4 - 1;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
    return ((days * 24L + h) * 60 + m) * 60 + s;
}

////////////////////////////////////////////////////////////////////////////////
// DateTime implementation - ignores time zones and DST changes
// NOTE: also ignores leap seconds, see http://en.wikipedia.org/wiki/Leap_second

DateTime::DateTime (uint32_t t) {
  t -= SECONDS_FROM_1970_TO_2000;    // bring to 2000 timestamp from 1970

    ss = t % 60;
    t /= 60;
    mm = t % 60;
    t /= 60;
    hh = t % 24;
    uint16_t days = t / 24;
    uint8_t leap;
    for (yOff = 0; ; ++yOff) {
        leap = yOff % 4 == 0;
        if (days < 365 + leap)
            break;
        days -= 365 + leap;
    }
    for (m = 1; ; ++m) {
        uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
        if (leap && m == 2)
            ++daysPerMonth;
        if (days < daysPerMonth)
            break;
        days -= daysPerMonth;
    }
    d = days + 1;
}

DateTime::DateTime (uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
    if (year >= 2000)
        year -= 2000;
    yOff = year;
    m = month;
    d = day;
    hh = hour;
    mm = min;
    ss = sec;
}

DateTime::DateTime (const DateTime& copy):
  yOff(copy.yOff),
  m(copy.m),
  d(copy.d),
  hh(copy.hh),
  mm(copy.mm),
  ss(copy.ss)
{}

static uint8_t conv2d(const char* p) {
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}

// A convenient constructor for using "the compiler's time":
//   DateTime now (__DATE__, __TIME__);
// NOTE: using F() would further reduce the RAM footprint, see below.
DateTime::DateTime (const char* date, const char* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    yOff = conv2d(date + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec 
    switch (date[0]) {
        case 'J': m = date[1] == 'a' ? 1 : m = date[2] == 'n' ? 6 : 7; break;
        case 'F': m = 2; break;
        case 'A': m = date[2] == 'r' ? 4 : 8; break;
        case 'M': m = date[2] == 'r' ? 3 : 5; break;
        case 'S': m = 9; break;
        case 'O': m = 10; break;
        case 'N': m = 11; break;
        case 'D': m = 12; break;
    }
    d = conv2d(date + 4);
    hh = conv2d(time);
    mm = conv2d(time + 3);
    ss = conv2d(time + 6);
}

// A convenient constructor for using "the compiler's time":
// This version will save RAM by using PROGMEM to store it by using the F macro.
//   DateTime now (F(__DATE__), F(__TIME__));
DateTime::DateTime (const __FlashStringHelper* date, const __FlashStringHelper* time) {
    // sample input: date = "Dec 26 2009", time = "12:34:56"
    char buff[11];
    memcpy_P(buff, date, 11);
    yOff = conv2d(buff + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (buff[0]) {
        case 'J': m = buff[1] == 'a' ? 1 : m = buff[2] == 'n' ? 6 : 7; break;
        case 'F': m = 2; break;
        case 'A': m = buff[2] == 'r' ? 4 : 8; break;
        case 'M': m = buff[2] == 'r' ? 3 : 5; break;
        case 'S': m = 9; break;
        case 'O': m = 10; break;
        case 'N': m = 11; break;
        case 'D': m = 12; break;
    }
    d = conv2d(buff + 4);
    memcpy_P(buff, time, 8);
    hh = conv2d(buff);
    mm = conv2d(buff + 3);
    ss = conv2d(buff + 6);
}

uint8_t DateTime::dayOfWeek() const {    
    uint16_t day = date2days(yOff, m, d);
    return (day + 6) % 7; // Jan 1, 2000 is a Saturday, i.e. returns 6
}

uint32_t DateTime::unixtime(void) const {
  uint32_t t;
  uint16_t days = date2days(yOff, m, d);
  t = time2long(days, hh, mm, ss);
  t += SECONDS_FROM_1970_TO_2000;  // seconds from 1970 to 2000

  return t;
}

long DateTime::secondstime(void) const {
  long t;
  uint16_t days = date2days(yOff, m, d);
  t = time2long(days, hh, mm, ss);
  return t;
}

DateTime DateTime::operator+(const TimeSpan& span) {
  return DateTime(unixtime()+span.totalseconds());
}

DateTime DateTime::operator-(const TimeSpan& span) {
  return DateTime(unixtime()-span.totalseconds());
}

TimeSpan DateTime::operator-(const DateTime& right) {
  return TimeSpan(unixtime()-right.unixtime());
}

////////////////////////////////////////////////////////////////////////////////
// TimeSpan implementation

TimeSpan::TimeSpan (int32_t seconds):
  _seconds(seconds)
{}

TimeSpan::TimeSpan (int16_t days, int8_t hours, int8_t minutes, int8_t seconds):
  _seconds(days*86400L + hours*3600 + minutes*60 + seconds)
{}

TimeSpan::TimeSpan (const TimeSpan& copy):
  _seconds(copy._seconds)
{}

TimeSpan TimeSpan::operator+(const TimeSpan& right) {
  return TimeSpan(_seconds+right._seconds);
}

TimeSpan TimeSpan::operator-(const TimeSpan& right) {
  return TimeSpan(_seconds-right._seconds);
}



////////////////////////////////////////////////////////////////////////////////
// PCF8523 implementation

static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }

bool PCF8523::begin(void) {
	Wire.begin();
	return true;
}

// Example: bool a = PCF8523.isrunning();
// Returns 1 if RTC is running and 0 it's not 
uint8_t PCF8523::isrunning(void) {
  WIRE.beginTransmission(PCF8523_ADDRESS);
  WIRE._I2C_WRITE(0);
  WIRE.endTransmission();

  WIRE.requestFrom(PCF8523_ADDRESS, 1);
  uint8_t ss = WIRE._I2C_READ();
  ss = ss & 32;
  return !(ss>>5);
}

// Example: PCF8523.setTime (DateTime(2014, 8, 14, 1, 49, 0))
// Sets RTC time to 2014/14/8 1:49 a.m.
void PCF8523::setTime(const DateTime& dt) {
  WIRE.beginTransmission(PCF8523_ADDRESS);
  WIRE._I2C_WRITE(0x03);
  WIRE._I2C_WRITE(bin2bcd(dt.second()));
  WIRE._I2C_WRITE(bin2bcd(dt.minute()));
  WIRE._I2C_WRITE(bin2bcd(dt.hour()));
  WIRE._I2C_WRITE(bin2bcd(dt.day()));
  WIRE._I2C_WRITE(bin2bcd(0));
  WIRE._I2C_WRITE(bin2bcd(dt.month()));
  WIRE._I2C_WRITE(bin2bcd(dt.year() - 2000));
  WIRE._I2C_WRITE(0);
  WIRE.endTransmission();
}

// Example: DateTime now = PCF8523.now();
// Returns Date and time in RTC in:
// year = now.year()
// month = now.month()
// day = now.day()
// hour = now.hour()
// minute = now.minute()
// second = now.second()
DateTime PCF8523::readTime() {
  WIRE.beginTransmission(PCF8523_ADDRESS);
  WIRE._I2C_WRITE(3);   
  WIRE.endTransmission();

  WIRE.requestFrom(PCF8523_ADDRESS, 7);
  uint8_t ss = bcd2bin(WIRE._I2C_READ() & 0x7F);
  uint8_t mm = bcd2bin(WIRE._I2C_READ());
  uint8_t hh = bcd2bin(WIRE._I2C_READ());
  uint8_t d = bcd2bin(WIRE._I2C_READ());
  WIRE._I2C_READ();
  uint8_t m = bcd2bin(WIRE._I2C_READ());
  uint16_t y = bcd2bin(WIRE._I2C_READ()) + 2000;
  
  return DateTime (y, m, d, hh, mm, ss);
}


// Example: PCF8523.readReg(buf,size,address);
// Returns:   buf[0] = &address
//            buf[1] = &address + 1
//      ..... buf[size-1] = &address + size
void PCF8523::rtcReadReg(uint8_t* buf, uint8_t size, uint8_t address) {
  int addrByte = address;
  WIRE.beginTransmission(PCF8523_ADDRESS);
  WIRE._I2C_WRITE(addrByte);
  WIRE.endTransmission();
  
  WIRE.requestFrom((uint8_t) PCF8523_ADDRESS, size);
  for (uint8_t pos = 0; pos < size; ++pos) {
    buf[pos] = WIRE._I2C_READ();
  }
}

// Example: PCF8523.writeReg(address,buf,size);
// Write:     buf[0] => &address
//            buf[1] => &address + 1
//      ..... buf[size-1] => &address + size
void PCF8523::rtcWriteReg(uint8_t address, uint8_t* buf, uint8_t size) {
  int addrByte = address;
  WIRE.beginTransmission(PCF8523_ADDRESS);
  WIRE._I2C_WRITE(addrByte);
  for (uint8_t pos = 0; pos < size; ++pos) {
    WIRE._I2C_WRITE(buf[pos]);
  }
  WIRE.endTransmission();
}

// Example: val = PCF8523.readReg(0x08);
// Reads the value in register addressed at 0x08
// and returns data
uint8_t PCF8523::rtcReadReg(uint8_t address) {
  uint8_t data;
  rtcReadReg(&data, 1, address);
  return data;
}

// Example: PCF8523.writeReg(0x08, 0x25);
// Writes value 0x25 in register addressed at 0x08
void PCF8523::rtcWriteReg(uint8_t address, uint8_t data) {
  rtcWriteReg(address, &data, 1);
}

// Example: PCF8523.set_alarm(10,5,45)
// Set alarm at day = 5, 5:45 a.m.
void PCF8523::setAlarm(uint8_t day_alarm, uint8_t hour_alarm, uint8_t minute_alarm ) {
 
  WIRE.beginTransmission(PCF8523_ADDRESS);
  WIRE._I2C_WRITE(0x0A);
	// Enable Minute
	WIRE._I2C_WRITE(bin2bcd(minute_alarm) & ~0x80 );
  // Enable Hour
	WIRE._I2C_WRITE(bin2bcd(hour_alarm) & ~0x80 );
	// Enable Day
	WIRE._I2C_WRITE(bin2bcd(day_alarm) & ~0x80);
  WIRE._I2C_WRITE(0x80);	// Disable WeekDay
  WIRE.endTransmission();
}

void PCF8523::setAlarm(uint8_t hour_alarm,uint8_t minute_alarm ) {
  WIRE.beginTransmission(PCF8523_ADDRESS);
  WIRE._I2C_WRITE(0x0A);
	// Enable Minute
	WIRE._I2C_WRITE(bin2bcd(minute_alarm) & ~0x80 );
	// Enable Hour
	WIRE._I2C_WRITE(bin2bcd(hour_alarm) & ~0x80 );
  WIRE._I2C_WRITE(0x80);	// Disable Day	
  WIRE._I2C_WRITE(0x80);	// Disable WeekDay
  WIRE.endTransmission();
}

void PCF8523::setAlarm(uint8_t minute_alarm ) {
  WIRE.beginTransmission(PCF8523_ADDRESS);
  WIRE._I2C_WRITE(0x0A);
	// Enable Minute
	WIRE._I2C_WRITE(bin2bcd(minute_alarm) & ~0x80 );
  WIRE._I2C_WRITE(0x80);	// Disable Hour	
  WIRE._I2C_WRITE(0x80);	// Disable Day	
  WIRE._I2C_WRITE(0x80);	// Disable WeekDay
  WIRE.endTransmission();
}

void PCF8523::setWeekDayAlarm(eWEEKDAYS weekday_alarm,uint8_t hour_alarm, uint8_t minute_alarm)
{
  WIRE.beginTransmission(PCF8523_ADDRESS);
  WIRE._I2C_WRITE(0x0A);
	// Enable Minute
	WIRE._I2C_WRITE(bin2bcd(minute_alarm) & ~0x80 );
	// Enable Hour
	WIRE._I2C_WRITE(bin2bcd(hour_alarm) & ~0x80 );
  WIRE._I2C_WRITE(0x80);			// Disable Day
  // Enable Weekday
  WIRE._I2C_WRITE(bin2bcd(weekday_alarm) & ~0x80);
  WIRE.endTransmission();
}

// Example: PCF8523.getAlarm(a);
// Returns a[0] = alarm minutes, a[1] = alarm hour, a[2] = alarm day, a[3] = alarm weekday
void PCF8523::getAlarm(uint8_t* buf) {
  WIRE.beginTransmission(PCF8523_ADDRESS);
  WIRE._I2C_WRITE(0x0A);
  WIRE.endTransmission();
  WIRE.requestFrom((uint8_t) PCF8523_ADDRESS, (uint8_t)4);
  for (uint8_t pos = 0; pos < 4; ++pos) {
//   buf[pos] = bcd2bin((WIRE._I2C_READ() & 0x7F));  // remove the enable / disable
     buf[pos] = bcd2bin(WIRE._I2C_READ());
  }
}
void PCF8523::getAlarm(ALARM_SETTINGS* settings) {
  uint8_t register_value;
  uint8_t buf[4];
  e12_24  twentyFourHour;

  WIRE.beginTransmission(PCF8523_ADDRESS);
  WIRE._I2C_WRITE(0x0A);
  WIRE.endTransmission();
  WIRE.requestFrom((uint8_t) PCF8523_ADDRESS, (uint8_t)4);
  // Returns a[0] = alarm minutes, a[1] = alarm hour, a[2] = alarm day, a[3] = alarm weekday
  for (uint8_t pos = 0; pos < 4; ++pos) {
     buf[pos] = WIRE._I2C_READ();
  }
  // get the 24 hour mode
  PCF8523::rtcReadReg(&register_value, 1, PCF8523_CONTROL_1);
  twentyFourHour = (e12_24)!(register_value & 0x08);

  // Get the minutes
  settings->minutes = bcd2bin(buf[0] & 0x7F);
  if(buf[0] & 0x80){
    settings->minutesEnabled = false;
  }
  else{
    settings->minutesEnabled = true; 
  }
  // Get the hours
  if(twentyFourHour == eTWENTYFOURHOUR){
    settings->hours = bcd2bin(buf[1] & 0x3F); 
    settings->AmPm  = eAM;           
  }
  else {
    settings->hours = bcd2bin(buf[1] & 0x1F);  
    settings->AmPm  = (eAM_PM)(buf[1] & 0x20);       
  }
  if(buf[1] & 0x80) {
    settings->hoursEnabled = false;
  }
  else {
    settings->hoursEnabled = true; 
  }
  // Get the Days
  settings->days = bcd2bin(buf[2] & 0x3F);
  if(buf[2] & 0x80) {
    settings->daysEnabled = false;
  }
  else {
    settings->daysEnabled = true; 
  }
  // Get the weekdays
  settings->weekdays = (eWEEKDAYS)(buf[3] & 0x07);
  if(buf[3] & 0x80){
    settings->weekdaysEnabled = false;
  }
  else{
    settings->weekdaysEnabled = true; 
  }

}

void PCF8523::enableAlarm(bool enable)
{
	uint8_t tmp;

	tmp = rtcReadReg(PCF8523_CONTROL_1);
	if(enable){
		// Disable Clockout & other Timers
		rtcWriteReg(PCF8523_TMR_CLKOUT_CTRL , 0x38);

		// Clear any existing flags
		ackAlarm();	
		// Enable the AIE bit
		tmp |= _BV(PCF8523_CONTROL_1_AIE_BIT);	

	}
	else {
		tmp &= ~_BV(PCF8523_CONTROL_1_AIE_BIT);	// Disable the AIE bit
	}
	rtcWriteReg(PCF8523_CONTROL_1 , tmp);

}
void PCF8523::ackAlarm(void)
{
	uint8_t tmp;
  tmp = rtcReadReg(PCF8523_CONTROL_2);
	
	tmp &= ~_BV(PCF8523_CONTROL_2_AF_BIT);	// Clear the AF bit	

	rtcWriteReg(PCF8523_CONTROL_2 , tmp);
	return; 
}

//void PCF8523::startCounter_1(uint8_t value){	
    // Set timer freq at 1Hz
//    rtcWriteReg(PCF8523_TMR_A_FREQ_CTRL , 2);
    // Load Timer value
//    rtcWriteReg(PCF8523_TMR_A_REG,value); 
    // Set counter mode TAC[1:0] = 01 
    // Disable Clockout
//    uint8_t tmp;
//    tmp = rtcReadReg(PCF8523_TMR_CLKOUT_CTRL);
//   tmp |= (1<<7)|(1<<5)|(1<<4)|(1<<3)|(1<<1);
//    tmp &= ~(1<<2);
//    rtcWriteReg(PCF8523_TMR_CLKOUT_CTRL , tmp);
    // Set countdown flag CTAF
    // Enable interrupt CTAIE
//    tmp = rtcReadReg(PCF8523_CONTROL_2);
//    tmp|=_BV(PCF8523_CONTROL_2_CTAF_BIT)|_BV(PCF8523_CONTROL_2_CTAIE_BIT);
//   rtcWriteReg(PCF8523_CONTROL_2,tmp);
//}



void PCF8523::rtcStop(void)
{
	uint8_t tmp;
    tmp = rtcReadReg(PCF8523_CONTROL_1);
	
	tmp |= _BV(PCF8523_CONTROL_1_STOP_BIT);	// Freeze the Counter

	rtcWriteReg(PCF8523_CONTROL_1 , tmp);
	
	return; 
}

void PCF8523::rtcStart(void)
{
	uint8_t tmp;
    tmp = rtcReadReg(PCF8523_CONTROL_1);
	
	tmp &= ~_BV(PCF8523_CONTROL_1_STOP_BIT);	// Start the Counter

	rtcWriteReg(PCF8523_CONTROL_1 , tmp);
	
	return; 
}

void PCF8523::setTimer1(eTIMER_TIMEBASE timebase, uint8_t value)
{
	uint8_t tmp;

	// Set the timebase
	rtcWriteReg(PCF8523_TMR_A_FREQ_CTRL , timebase);

	// Set the value
	rtcWriteReg(PCF8523_TMR_A_REG , value);

	// Clear any Timer A flags
    tmp = rtcReadReg(PCF8523_CONTROL_2);
	
	tmp &= ~_BV(PCF8523_CONTROL_2_CTAF_BIT);	// Clear the Timer A flag
	tmp |= _BV(PCF8523_CONTROL_2_CTAIE_BIT);	// Enable Timer A interrupt

	rtcWriteReg(PCF8523_CONTROL_2 , tmp);

	// Set Timer A as Countdown and Enable
    tmp = rtcReadReg(PCF8523_TMR_CLKOUT_CTRL);

	tmp |= _BV(PCF8523_TMR_CLKOUT_CTRL_TAM_BIT);	// /INT line is pulsed
	tmp |= 0x02;									// Set as a Countdown Timer	and Enable	

	rtcWriteReg(PCF8523_TMR_CLKOUT_CTRL , tmp);

}
void PCF8523::ackTimer1(void)
{
	uint8_t tmp;

	// Clear any Timer A flags
    tmp = rtcReadReg(PCF8523_CONTROL_2);
	
	tmp &= ~_BV(PCF8523_CONTROL_2_CTAF_BIT);	// Clear the Timer A flag

	rtcWriteReg(PCF8523_CONTROL_2 , tmp);

	return;
}
uint8_t PCF8523::getTimer1(void)
{
	return rtcReadReg(PCF8523_TMR_A_REG);
}
void PCF8523::setTimer2(eTIMER_TIMEBASE timebase,uint8_t value)
{
	uint8_t tmp;

	// Set the timebase
	rtcWriteReg(PCF8523_TMR_B_FREQ_CTRL , timebase);

	// Set the value
	rtcWriteReg(PCF8523_TMR_B_REG , value);

	// Clear any Timer B flags
  tmp = rtcReadReg(PCF8523_CONTROL_2);
	
	tmp &= ~_BV(PCF8523_CONTROL_2_CTBF_BIT);	// Clear the Timer B flag
	tmp |= _BV(PCF8523_CONTROL_2_CTBIE_BIT);	// Enable Timer B interrupt

	rtcWriteReg(PCF8523_CONTROL_2 , tmp);

	// Set Timer A as Countdown and Enable
    tmp = rtcReadReg(PCF8523_TMR_CLKOUT_CTRL);

	tmp |= _BV(PCF8523_TMR_CLKOUT_CTRL_TBM_BIT);	// /INT line is pulsed
	tmp |= 0x01;									// Enable	

	rtcWriteReg(PCF8523_TMR_CLKOUT_CTRL , tmp);


}
void PCF8523::ackTimer2(void)
{
	uint8_t tmp;

	// Clear any Timer B flags
  tmp = rtcReadReg(PCF8523_CONTROL_2);
	
	tmp &= ~_BV(PCF8523_CONTROL_2_CTBF_BIT);	// Clear the Timer A flag

	rtcWriteReg(PCF8523_CONTROL_2 , tmp);

	return;
}
uint8_t PCF8523::getTimer2(void)
{
	return rtcReadReg(PCF8523_TMR_B_REG);
}


// Example: PCF8523.reset();
// Reset the PCF8523
void PCF8523::reset(){
    rtcWriteReg(PCF8523_CONTROL_1, 0x58);
}

uint8_t PCF8523::clearRtcInterruptFlags() {
  uint8_t rc2 = rtcReadReg(_BV(PCF8523_CONTROL_2) & (_BV(PCF8523_CONTROL_2_SF_BIT) | _BV(PCF8523_CONTROL_2_AF_BIT)));
  rtcWriteReg(PCF8523_CONTROL_2, 0);  // Just zero the whole thing
  return rc2 != 0;
}

// Stop the default 32.768KHz CLKOUT signal on INT1.
void PCF8523::stop_32768_clkout() {
    uint8_t tmp = (rtcReadReg (PCF8523_TMR_CLKOUT_CTRL))|RTC_CLKOUT_DISABLED;

    rtcWriteReg(PCF8523_TMR_CLKOUT_CTRL , tmp);
}
void PCF8523::setBatterySwitchover() {

	rtcWriteReg(PCF8523_CONTROL_3 , 0x00);
	return;

}

bool PCF8523::rtcBatteryLow(void)
{
	uint8_t tmp;

    tmp = rtcReadReg(PCF8523_CONTROL_3);
	
	if(tmp & 0x04){
		return true;
	}
	else {
		return false;
	}
}

void PCF8523::rtcCapSelect(eCAP_SEL value)
{
  uint8_t tmp;
  tmp = rtcReadReg(PCF8523_CONTROL_1);
  if(value == eCAP_7pF)
  {
      // Clear Cap_sel bit
      tmp &= ~_BV(PCF8523_CONTROL_1_CAP_SEL_BIT); 

  } 
  else
  {
      // Set Cap_Sel bit
      tmp |= _BV(PCF8523_CONTROL_1_CAP_SEL_BIT);

  }
  rtcWriteReg(PCF8523_CONTROL_1 , tmp);
  return;
}

void PCF8523::setTwelveTwentyFourHour(e12_24 mode)
{
  uint8_t tmp;
  tmp = rtcReadReg(PCF8523_CONTROL_1);
  if(mode == eTWENTYFOURHOUR) 
  {
      // Clear 12_24 flag
      tmp &= ~_BV(PCF8523_CONTROL_1_1224_BIT);  
  }
  else 
  {
      // 12 hour Mode
      // Set 12_24 bit
      tmp |= _BV(PCF8523_CONTROL_1_1224_BIT);

  }
  rtcWriteReg(PCF8523_CONTROL_1 , tmp);
  return;
}

e12_24 PCF8523::getTwelveTwentyFourHour(void)
{
  uint8_t tmp;
  tmp = rtcReadReg(PCF8523_CONTROL_1);
  if(tmp & _BV(PCF8523_CONTROL_1_1224_BIT))
  {
     return eTWELVEHOUR;
  }
  else
  {
     return eTWENTYFOURHOUR; 
  }
}


////////////////////////////////////////////////////////////////////////////////
// RTC_Millis implementation

long RTC_Millis::offset = 0;

void RTC_Millis::adjust(const DateTime& dt) {
    offset = dt.unixtime() - millis() / 1000;
}

DateTime RTC_Millis::now() {
  return (uint32_t)(offset + millis() / 1000);
}

////////////////////////////////////////////////////////////////////////////////


