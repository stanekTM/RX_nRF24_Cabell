/*
  *******************************************************************************************************************
  RC receiver with "Cabell" protocol
  **********************************
  RC receiver from my repository https://github.com/stanekTM/RX_nRF24_Cabell
  
  Hardware includes nRF24L01+ transceiver, ATmega328P/PB processor and motor driver.
  
  Works with RC transmitters:
  OpenAVRc      https://github.com/Ingwie/OpenAVRc_Dev
  Multiprotocol https://github.com/pascallanger/DIY-Multiprotocol-TX-Module
  *******************************************************************************************************************
  
  Copyright 2017 by Dennis Cabell
  KE8FZX
  
  To use this software, you must adhere to the license terms described below, and assume all responsibility for the use
  of the software.  The user is responsible for all consequences or damage that may result from using this software.
  The user is responsible for ensuring that the hardware used to run this software complies with local regulations and that 
  any radio signal generated from use of this software is legal for that user to generate.  The author(s) of this software 
  assume no liability whatsoever.  The author(s) of this software is not responsible for legal or civil consequences of 
  using this software, including, but not limited to, any damages cause by lost control of a vehicle using this software.  
  If this software is copied or modified, this disclaimer must accompany all copies.
  
  This project is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  
  RC_RX_CABELL_V3_FHSS is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with RC_RX_CABELL_V3_FHSS.  If not, see http://www.gnu.org/licenses.
*/

#include <EEPROM.h>
#include <Servo.h>     // v1.2.2
#include <DigitalIO.h> // v1.0.1
#include "RX.h"
#include "PWM.h"

//*********************************************************************************************************************
// Program setup
//*********************************************************************************************************************
void setup(void)
{
  //Serial.begin(9600);
  
  servo_setup();
  motor_setup();
  
  pinMode(PIN_BUTTON_BIND, INPUT_PULLUP);
  
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  
  // Initial analog reads for A6/A7. Initial call returns bad value so call 3 times to get a good starting value from each pin
  ADC_Processing();
  // Wait for conversion
  while (!bit_is_clear(ADCSRA, ADSC));
  ADC_Processing();
  // Wait for conversion
  while (!bit_is_clear(ADCSRA, ADSC));
  ADC_Processing();
  
  setupReciever();
}

//*********************************************************************************************************************
// Program loop
//*********************************************************************************************************************
void loop()
{
  while (true)
  {
    // Loop forever without going back to Arduino core code
    if (getPacket())
    {
      output_rc_channels();
    }
    
    ADC_Processing(); // Process ADC to asynchronously read A6/A7 for telemetry analog values. Non-blocking read
  }
}
 
