/* 
  Copyright (c) 2023 Hudson Sonoma LLC
  tim@hudson-sonoma.com
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include <Wire.h>
#include <avr/sleep.h>
#include <SensirionI2CSht4x.h>
#include <SensirionErrors.h>
#include "Lorawan.hpp"

// Lorawan transmissions
// BOOT           PORT=8 boot flags 8bit, 16bit software_version, 32bit SHT40id,
// REJOIN/STATUS  PORT=7 uptime millis 32bit, battery voltage 16bit, RSSI 8bit, SNR 8bit, 
// SAMPLE         PORT=1 temperature 16bit, humidity 16bit


// power draw with no sleep: 15.7mA during waiting with > 22mA peaks during transmission - probably much higher during transmission.

float _convertTicksToCelsius(uint16_t ticks); 
float _convertTicksToPercentRH(uint16_t ticks); 
uint16_t readSupplyVoltage();
void RTC_init();
void printResetReason();

SensirionI2CSht4x sht4x;
Lorawan e5;  // defaults to Serial1. See defines in Lorawan.hpp to change serial port.
uint16_t software_version = 0x0001;
uint16_t message_failures = 0;

struct RTC_Event {
  uint16_t count = 0;
  uint16_t period = 0;
  volatile uint8_t flag = 0;
};

struct Events {
  // boot
  RTC_Event sample = {0, 30, 0}; // sample every 30 tick (seconds)
  RTC_Event status = {0, 60, 0};
  RTC_Event rejoin = {0, 120, 0};
};

Events rtc_event;

// RTC
ISR(RTC_PIT_vect)
{
  RTC.PITINTFLAGS = RTC_PI_bm;          /* Clear interrupt flag by writing '1' (required) */  

  if (rtc_event.sample.count == (rtc_event.sample.period-1))
  {
    rtc_event.sample.count = 0;
    rtc_event.sample.flag = 1;
  } else { rtc_event.sample.count++; }

  if (rtc_event.status.count == rtc_event.status.period-1)
  {
    rtc_event.status.count = 0;
    rtc_event.status.flag = 1;
  } else { rtc_event.status.count++; }

  if (rtc_event.rejoin.count == rtc_event.rejoin.period-1)
  {
    rtc_event.rejoin.count = 0;
    rtc_event.rejoin.flag = 1;
  } else { rtc_event.rejoin.count++; }

}

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_PA5,OUTPUT);
  digitalWrite(PIN_PA5,HIGH);
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  } 

  LoraE5.begin(115200);

  Wire.begin();

  delay(3000);

  printResetReason();

  // SHT40
  uint16_t error;
  char errorMessage[256];
  sht4x.begin(Wire);
  sht4x.softReset();
  uint32_t serialNumber;
  error = sht4x.serialNumber(serialNumber);
  if (error) {
      Serial.println("Error trying to execute SerialNumber()");
  } else {
      Serial.print("SHT40 Serial Number: ");
      Serial.println(serialNumber);
  }

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  /* Set sleep mode to SLEEP_MODE_STANDBY SLEEP_MODE_IDLE SLEEP_MODE_PWR_DOWN mode */
  sleep_enable();                       /* Enable sleep mode, but not going to sleep yet */
  
  bool s;
  // If Wio E5 module is new, set to 9600 baud and reset.
  if (!e5.sendATCommand("AT",  "+AT: OK", "+AT: ERROR",300)) {
    LoraE5.begin(9600);
    s= e5.sendATCommand("AT+UART=BR, 115200", "+UART: BR",         "+AT: ERROR",300); Serial.println(s);
    s= e5.sendATCommand("AT+RESET",           "+RESET",            "+AT: ERROR",300); Serial.println(s);
    LoraE5.begin(115200);
    delay(2000);
    s= e5.sendATCommand("AT",                 "+AT: OK",           "+AT: ERROR",300); Serial.println(s);   // Always is an error after reset, not sure why.
  }

  // Set to low power mode, turn on debugging info.
  s= e5.sendATCommand("AT+LOWPOWER=AUTOON",  "+LOWPOWER: AUTOON", "+AT: ERROR",300); Serial.println(s);
  //s= e5.sendATCommand("ÿÿÿÿAT+LOWPOWER=AUTOOFF", "+LOWPOWER: AUTOOFF","+AT: ERROR",300,1000); Serial.println(s);
  s= e5.sendATCommand("AT+LOG=DEBUG",    "+LOG: DEBUG",           "+AT: ERROR",300); Serial.println(s);
  s= e5.sendATCommand("AT+ID",           "+ID: AppEui",           "+AT: ERROR",300); Serial.println(s);

  // Set up for US915.
  s= e5.sendATCommand("AT+DR=US915",     "+DR: US915",            "+AT: ERROR",300); Serial.println(s);
  s= e5.sendATCommand("AT+CH=NUM,8-15",  "+CH: NUM,",             "+AT: ERROR",300); Serial.println(s);
  s= e5.sendATCommand("AT+DR=1",         "+DR:",                  "+AT: ERROR",300); Serial.println(s); // Set to >=DR1 so as to have packets up to 53 bytes.

  // OTA, ADR ON.
  s= e5.sendATCommand("AT+LW=LEN",       "+LW:",                  "+AT: ERROR",300); Serial.println(s);
  s= e5.sendATCommand("AT+MODE=LWOTAA",  "+MODE:",                "+AT: ERROR",300); Serial.println(s);
  s= e5.sendATCommand("AT+ADR=ON",       "+ADR: ON",              "+AT: ERROR",300); Serial.println(s);
  s= e5.sendATCommand("AT+JOIN",         "+JOIN: Network joined", "+JOIN: Join failed",10000); Serial.println(s);

  Serial.flush();
  RTC_init();

  digitalWrite(PIN_PA5,LOW);
  pinMode(PIN_PA5,INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  float temperature;
  float humidity;
  uint16_t error;
  char errorMessage[256];
  uint16_t temperatureTicks;
  uint16_t humidityTicks;
  int16_t supply_mv;

  if (rtc_event.sample.flag) {
    rtc_event.sample.flag = 0;

    bool status, message_sent;
    if (message_failures > 5 ) {
          status= e5.sendATCommand("AT+JOIN",         "+JOIN: Network joined", "+JOIN: Join failed",10000); Serial.println(status);
          if (status) { message_failures = 0; }
    }

    supply_mv = readSupplyVoltage();
    error = sht4x.measureHighPrecisionTicks(temperatureTicks, humidityTicks);
    if (error) {
        Serial.print("Error trying to execute measureHighPrecisionTicks(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        temperature = _convertTicksToCelsius(temperatureTicks);
        humidity = _convertTicksToPercentRH(humidityTicks); 

        Serial.print("Supply voltage: "); Serial.print(supply_mv); Serial.print("\t"); Serial.print("Temperature:"); Serial.print(temperature); Serial.print("\t");
        Serial.print("Humidity:"); Serial.print(humidity); Serial.println();
        
        uint8_t buf[64];
        sprintf((char*)buf,"AT+PORT=%d",1);
        status=e5.sendATCommand(buf,           "+PORT:",            "+AT: ERROR",300); Serial.println(status);
        status=e5.sendATCommand("AT+DR",       "+DR:",              "+AT: ERROR",300); Serial.println(status);
        sprintf((char*)buf,"AT+MSGHEX=\"%02X %02X %02X\"",supply_mv,temperatureTicks,humidityTicks);
        message_sent=e5.sendATCommand(buf,     "+MSGHEX: Done",     "+MSGHEX: Please join network first",10000); Serial.println(message_sent);
        status=e5.sendATCommand("AT+DR",       "+DR:",              "+AT: ERROR",300); Serial.println(status);
        Serial.println();

        //AT+CMSGHEX="xx xx xx xx" // confirmed message.
        // uint8_t buf[64];
        // sprintf((char*)buf,"AT+CMSGHEX=\"%02X %02X %02X\"",supply_mv,temperatureTicks,humidityTicks);
        // bool status;
        // status = e5.sendATCommand(buf,         "+CMSGHEX: ACK Received",     "+CMSGHEX: Please join network first",10000); Serial.println(status);    

        if (!message_sent) {
          message_failures++;
        } 

    }
    Serial.flush();
  }

  ADC0.CTRLA &= ~ADC_ENABLE_bm; // disable ADC
  sleep_cpu();
  ADC0.CTRLA |= ADC_ENABLE_bm; // enable ADC

}

float _convertTicksToCelsius(uint16_t ticks) 
{
    return static_cast<float>(ticks * 175.0 / 65535.0 - 45.0);
}


float _convertTicksToPercentRH(uint16_t ticks) 
{
    return static_cast<float>(ticks * 125.0 / 65535.0 - 6);
}

// attiny 2-series supply voltage measurement
// https://www.microchip.com/forums/m5200/m5200-attiny-2-series-supply-voltage-measurement.html
// https://github.com/SpenceKonde/megaTinyCore/tree/master/megaavr/libraries/megaTinyCore/examples/readTempVcc
uint16_t readSupplyVoltage() 
{ // returns value in millivolts to avoid floating point
  #if MEGATINYCORE_SERIES!=2
  analogReference(VDD);
  VREF.CTRLA = VREF_ADC0REFSEL_1V5_gc;
  // there is a settling time between when reference is turned on, and when it becomes valid.
  // since the reference is normally turned on only when it is requested, this virtually guarantees
  // that the first reading will be garbage; subsequent readings taken immediately after will be fine.
  // VREF.CTRLB|=VREF_ADC0REFEN_bm;
  // delay(10);
  uint16_t reading = analogRead(ADC_INTREF);
  Serial.print(reading);
  Serial.println(" (discarded)");
  reading = analogRead(ADC_INTREF);
  Serial.println(reading);
  uint32_t intermediate = 1023UL * 1500;
  reading = intermediate / reading;
  return reading;
  #else
  analogReference(INTERNAL1V024);
  int32_t vddmeasure = analogReadEnh(ADC_VDDDIV10, 12); // Take it at 12 bits
  vddmeasure *= 10; // since we measured 1/10th VDD
  int16_t returnval = vddmeasure >> 2; // divide by 4 to get into millivolts.
  if (vddmeasure & 0x02) {
    // if last two digits were 0b11 or 0b10 we should round up
    returnval++;
  }
  return returnval;
  #endif
}


void RTC_init(void)
{
  /* Initialize RTC: */
  while (RTC.STATUS > 0)
  {
    ;                                   /* Wait for all register to be synchronized */
  }
  RTC.CTRLA |= RTC_RUNSTDBY_bm;            /* Run RTC in RUNSTANDBY */
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;    /* 32.768kHz Internal Ultra-Low-Power Oscillator (OSCULP32K) */

  RTC.PITINTCTRL = RTC_PI_bm;           /* PIT Interrupt: enabled */

  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc /* RTC_PERIOD_CYC32768_gc= 1Hz  */
  | RTC_PITEN_bm;                       /* Enable PIT counter: enabled */
 /* RTC_PERIOD_CYC32768_gc= 1Hz  RTC_PERIOD_CYC8192_gc=4Hz RTC_PERIOD_CYC4096_gc=8HZ   RTC_PERIOD_CYC1024_gc=32HZ*/
}

void printResetReason() 
{
  uint8_t reset_flags = GPIOR0;
  if (reset_flags & RSTCTRL_UPDIRF_bm) {
    Serial.println("Reset by UPDI (code just uploaded now)");
  }
  if (reset_flags & RSTCTRL_WDRF_bm) {
    Serial.println("reset by WDT timeout");
  }
  if (reset_flags & RSTCTRL_SWRF_bm) {
    Serial.println("reset at request of user code. OR CRASH!");
  }
  if (reset_flags & RSTCTRL_EXTRF_bm) {
    Serial.println("Reset because reset pin brought low");
  }
  if (reset_flags & RSTCTRL_BORF_bm) {
    Serial.println("Reset by voltage brownout");
  }
  if (reset_flags & RSTCTRL_PORF_bm) {
    Serial.println("Reset by power on");
  }
}


