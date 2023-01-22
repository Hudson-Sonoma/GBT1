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
#include <SensirionI2CSht4x.h>
#include <SensirionErrors.h>
#include "Lorawan.hpp"

// power draw with no sleep: 15.7mA during waiting with > 22mA peaks during transmission - probably much higher during transmission.

float _convertTicksToCelsius(uint16_t ticks); 
float _convertTicksToPercentRH(uint16_t ticks); 
uint16_t readSupplyVoltage();

SensirionI2CSht4x sht4x;
Lorawan e5;  // default Serial1. See defines in Lorawan.hpp to change serial port.

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_PA5,OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  } 

  LoraE5.begin(9600);
  while (!LoraE5) {
    delay(100);
  }

  Wire.begin();

  delay(3000);

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

  // LoraE5
  
  //Serial.println("AT+ID"); LoraE5.println("AT+ID"); delay(300); while (LoraE5.available()) { Serial.write(LoraE5.read()); } Serial.println();
  //Serial.println("AT+DR=US915"); LoraE5.println("AT+DR=US915"); delay(300); while (LoraE5.available()) { Serial.write(LoraE5.read()); } Serial.println();
  //Serial.println("AT+CH=NUM,8-15"); LoraE5.println("AT+CH=NUM,8-15"); delay(300); while (LoraE5.available()) { Serial.write(LoraE5.read()); } Serial.println();
  //Serial.println("AT+MODE=LWOTAA"); LoraE5.println("AT+MODE=LWOTAA"); delay(300); while (LoraE5.available()) { Serial.write(LoraE5.read()); } Serial.println();
  //Serial.println("AT+JOIN"); LoraE5.println("AT+JOIN"); delay(300); while (LoraE5.available()) { Serial.write(LoraE5.read()); } Serial.println();

  bool s;
  s= e5.sendATCommand("AT+LOWPOWER=AUTOON",  "+LOWPOWER: AUTOON", "+AT: ERROR",300,1000); Serial.println(s);
  //s= e5.sendATCommand("ÿÿÿÿAT+LOWPOWER=AUTOOFF", "+LOWPOWER: AUTOOFF","+AT: ERROR",300,1000); Serial.println(s);
  s= e5.sendATCommand("AT+ID",           "+ID: AppEui",           "+AT: ERROR",300,1000); Serial.println(s);
  s= e5.sendATCommand("AT+DR=US915",     "+DR: US915",            "+AT: ERROR",300,1000); Serial.println(s);
  s= e5.sendATCommand("AT+CH=NUM,8-15",  "+CH: NUM,",             "+AT: ERROR",300,1000); Serial.println(s);
  s= e5.sendATCommand("AT+MODE=LWOTAA",  "+MODE:",                "+AT: ERROR",300,1000); Serial.println(s);
  s= e5.sendATCommand("AT+JOIN",         "+JOIN: Network joined", "+JOIN: Join failed",7000,8000); Serial.println(s);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(PIN_PA5,CHANGE);
  //Serial.println("Hello World");

  float temperature;
  float humidity;
  uint16_t error;
  char errorMessage[256];
  uint16_t temperatureTicks;
  uint16_t humidityTicks;
  int16_t supply_mv;

  supply_mv = readSupplyVoltage();
  error = sht4x.measureHighPrecisionTicks(temperatureTicks, humidityTicks);
 if (error) {
      Serial.print("Error trying to execute measureHighPrecision(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
  } else {
      temperature = _convertTicksToCelsius(temperatureTicks);
      humidity = _convertTicksToPercentRH(humidityTicks); 

      Serial.print("Supply voltage: ");
      Serial.print(supply_mv);
      Serial.print("\t");
      Serial.print("Temperature:");
      Serial.print(temperature);
      Serial.print("\t");
      Serial.print("Humidity:");
      Serial.print(humidity);
      Serial.print(" Lorawan bytes: ");
      Serial.printHex(supply_mv); Serial.printHex(temperatureTicks); Serial.printHex(humidityTicks);  // LoraWAN bytes: 3300mV 2 bytes, 2 bytes temperature, 2 bytes humidity
      Serial.println();

      // unconfirmed messages are not working in the E5 module.
      
      // //AT+MSGHEX="xx xx xx xx"
      // uint8_t buf[64];
      // sprintf((char*)buf,"AT+MSGHEX=\"%02X %02X %02X\"",supply_mv,temperatureTicks,humidityTicks);
      // bool status;
      // status = e5.sendATCommand(buf,         "+MSGHEX: RXWIN",     "+MSGHEX: Please join network first",13000,15000); Serial.println(status);

      //AT+CMSGHEX="xx xx xx xx" // confirmed message.
      uint8_t buf[64];
      sprintf((char*)buf,"AT+CMSGHEX=\"%02X %02X %02X\"",supply_mv,temperatureTicks,humidityTicks);
      bool status;
      status = e5.sendATCommand(buf,         "+CMSGHEX: ACK Received",     "+CMSGHEX: Please join network first",13000,15000); Serial.println(status);    

  }


  // every 10 seconds
  delay(30000);
  

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

