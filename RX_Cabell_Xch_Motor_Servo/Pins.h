/*
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

#ifndef __have__RC_RX_PINS_h__
#define __have__RC_RX_PINS_h__

#include <Arduino.h>
#include "RX.h"

// ATmega328P/PB pins overview
// PD0 - D0   PWM  328PB
// PD1 - D1   PWM  328PB
// PD2 - D2   PWM  328PB
// PD3 - D3   PWM
// PD4 - D4
// PD5 - D5   PWM
// PD6 - D6   PWM
// PD7 - D7
// PB0 - D8
// PB1 - D9   PWM
// PB2 - D10  PWM
// PB3 - D11  PWM  MOSI
// PB4 - D12       MISO
// PB5 - D13       SCK
// PC0 - D14 / A0
// PC1 - D15 / A1
// PC2 - D16 / A2
// PC3 - D17 / A3
// PC4 - D18 / A4   SDA
// PC5 - D19 / A5   SCL
// PB6 - D20        XTAL1
// PB7 - D21        XTAL2
// PC6 - D22        RESET
// PE0 - D23        328PB
// PE1 - D24        328PB
// PE2 - D25 / A6   328PB
// PE3 - D26 / A7   328PB
// ADC6   -    A6
// ADC7   -    A7

// PWM pins for motor 1 (possible combination, max. 2)
#if defined(MOTOR1)
const byte pins_motor1[] = {5, 6};
#endif

// PWM pins for motor 2 (possible combination, max. 2)
#if defined(MOTOR2)
const byte pins_motor2[] = {3, 11};
#endif

// Pins for servos (possible combination, max. 8)
#if defined(SERVO_8CH)
const byte pins_servo[] = {2, 3, 4, 5, 6, 7, 8, 9};
#endif

// Pins for servos (possible combination, max. 7)
#if defined(SERVO_7CH_MOTOR1)
const byte pins_servo[] = {2, 3, 4, 7, 8, 9, 10};
#endif

// Pins for servos (possible combination, max. 7)
#if defined(SERVO_7CH_MOTOR2)
const byte pins_servo[] = {2, 4, 5, 6, 7, 8, 9};
#endif

// Pins for servos (possible combination, max. 6)
#if defined(SERVO_6CH_MOTOR12)
const byte pins_servo[] = {2, 4, 7, 8, 9, 10};
#endif

#define PIN_BUTTON_BIND        12
#define PIN_LED                13

#define PIN_RX_BATT_A1         A6
#define PIN_RX_BATT_A2         A7

// Pins for nRF24L01+
#define PIN_CE                 A0
#define PIN_CSN                A1

// Software SPI https://nrf24.github.io/RF24/md_docs_2arduino.html
//          SCK                A2
//          MOSI               A3
//          MISO               A4

// Configuring pin A5 for radio IRQ 
#define RADIO_IRQ_PIN          A5
#define RADIO_IRQ_PIN_bit      5 // PC5
#define RADIO_IRQ_port         PORTC
#define RADIO_IRQ_ipr          PINC
#define RADIO_IRQ_ddr          DDRC
#define RADIO_IRQ_PIN_MASK     _BV(RADIO_IRQ_PIN_bit)
#define RADIO_IRQ_SET_INPUT    RADIO_IRQ_ddr &= ~RADIO_IRQ_PIN_MASK
#define RADIO_IRQ_SET_OUTPUT   RADIO_IRQ_ddr |=  RADIO_IRQ_PIN_MASK
#define RADIO_IRQ_SET_PULLUP   RADIO_IRQ_port |= RADIO_IRQ_PIN_MASK
#define IS_RADIO_IRQ_on        ((RADIO_IRQ_ipr & RADIO_IRQ_PIN_MASK) == 0x00)

#endif
 
