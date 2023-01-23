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
#ifndef LORAWAN_H
#define LORAWAN_H

#include <Arduino.h>

#define LoraE5 Serial1
#define DebugSerial Serial
#define LORAWAN_MAX_RESPONSE_LINE 256

bool startsWith(const char *str, const char *ref);


class Lorawan {

public:
    char     bufResponse[LORAWAN_MAX_RESPONSE_LINE];
    uint16_t respIndex; // processing response, index in buffer
                        // Function for processing AT response line
    bool statusOfCommand;        // state of the AT command execution

    bool sendATCommand(const char *cmd, const char *okResp, const char *errResp, uint32_t timeoutMs);
    bool processATResponse(const char *okResp, const char *errResp, uint32_t timeoutMs, uint32_t startTime);


};

  /**
 * Execute an AT command with a timeout
 * startsWith okResp or errResp to determine is the command is a success or a fail
 * okResp and errResp can use wild char '*'
 */
bool Lorawan::sendATCommand(const char *cmd, const char *okResp, const char *errResp, uint32_t timeoutMs) {

 
    LoraE5.printf("ÿÿÿÿ%s\r\n", cmd);   // 4 bytes of 0xff to wake up the module. In ascii, 'ÿÿÿÿ'
    DebugSerial.printf("%s\r\n", cmd);

    this->statusOfCommand = false;
    uint32_t startTime = millis();
    while (!processATResponse(okResp, errResp, timeoutMs,startTime))
    { ; }
    delay(300);
    while(LoraE5.available() > 0) {
        uint8_t c = LoraE5.read();
        if (c < 255) { DebugSerial.write(c); } // echo any left over input except 0xFF
    }
    return this->statusOfCommand;

}


/**
 * Process the AT response
 * Return true when the response is completed
 */ 
/**
 * Process command response
 * return true when nothing more to be done
 */
bool Lorawan::processATResponse(const char *okResp, const char *errResp, uint32_t timeoutMs, uint32_t startTime) {


    // process serial line response
    uint32_t min_duration = millis() - startTime;
    while (min_duration < timeoutMs) { 
      min_duration = millis() - startTime; 
      while ((LoraE5.available() > 0) ) {
          char c;
          c = LoraE5.read();
          if ((c == '\0' || c == '\r' || c == '\n')) {
              if (this->respIndex > 0) {
                  // process line response
                  this->bufResponse[this->respIndex] = '\0';
                  DebugSerial.print(this->bufResponse);
                  int i;
                  if (strlen(errResp) > 0 &&
                      startsWith(this->bufResponse, errResp)) {
                      // Error String found
                      this->respIndex     = 0;
                      DebugSerial.print("  ERROR! ");
                      return true;
                  } else if (strlen(okResp) > 0 && startsWith(this->bufResponse, okResp)) {
                      // Success String found
                      this->respIndex     = 0;
                      DebugSerial.print("  SUCCESS ");
                      this->statusOfCommand = true;
                      return true;
                  } else {
                      DebugSerial.println();
                  }
              }
              this->respIndex = 0;
          } else {
              if (c >= 32 && c <= 127) {
                  if (this->respIndex < LORAWAN_MAX_RESPONSE_LINE) {
                      this->bufResponse[this->respIndex] = c;
                      this->respIndex++;
                  } else {
                      DebugSerial.print("LoRaE5 - Response size overflow ");
                      this->respIndex = 0;
                  }
              }
          }
      }
    }
    if (min_duration < timeoutMs) {
        return false;
    } else {
        DebugSerial.println("LoRaE5 - Response timeout ");
        return true;
    }
}

// compare str with a ref string and return true when
// str starts with ref. Case sensitive. ref can contain
// a wild card char '*' that match any single char
bool startsWith(const char *str, const char *ref) 
{
    int i = 0;
    while (ref[i] != '\0') {
        if (ref[i] == '*') {
            i++;
            if (ref[i] == '\0')
                return true;
            while (str[i] != '\0' && str[i] != ref[i])
                i++;
        } else {
            if (str[i] != ref[i])
                return false;
            i++;
        }
    }
    return true;
}

#endif // LORAWAN_H
