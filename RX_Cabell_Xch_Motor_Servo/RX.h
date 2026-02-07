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

#ifndef __have__RC_RX_TX_RX_h__
#define __have__RC_RX_TX_RX_h__

#include <Arduino.h>
#include "My_RF24.h"

// Setting PWM
// Pin D5 and D6 (8-bit Timer/Counter 0, functions delay, millis, micros and delayMicroseconds)
// 1024 = 61Hz
// 256 = 244Hz
// 64 = 976Hz(default)
// 8 = 7812Hz
// 1 = 62500Hz
#define PWM_TIMER0_5_6  64

// Pin D9 and D10 (16-bit Timer/Counter 1, Servo library)
// 1024 = 30Hz
// 256 = 122Hz
// 64 = 488Hz(default)
// 8 = 3906Hz
// 1 = 31250Hz
//#define PWM_TIMER1_9_10  64

// Pin D3 and D11 (8-bit Timer/Counter 2, ServoTimer2, Tone library)
// 1024 = 30Hz
// 256 = 122Hz
// 128 = 244Hz
// 64 = 488Hz(default)
// 32 = 976Hz
// 8 = 3906Hz
// 1 = 31250Hz
#define PWM_TIMER2_3_11  64

// Pin D0(RX) (328PB 16-bit Timer/Counter 3)
// 1024 = 30Hz
// 256 = 122Hz
// 64 = 488Hz(default)
// 8 = 3906Hz
// 1 = 31250Hz
//#define PWM_TIMER3_0  64

// Pin D1(TX) and D2 (328PB 16-bit Timer/Counter 4)
// 1024 = 30Hz
// 256 = 122Hz
// 64 = 488Hz(default)
// 8 = 3906Hz
// 1 = 31250Hz
//#define PWM_TIMER4_1_2  64

// Setting the reaction of the motor to be rotated after the lever has been moved. Settings (0-255)
#define ACCELERATE_MOTOR1  0
#define ACCELERATE_MOTOR2  0

// Setting the maximum motor power. Suitable for TX transmitters without endpoint setting. Settings (0-255)
#define MAX_FORWARD_MOTOR1  255
#define MAX_REVERSE_MOTOR1  255

#define MAX_FORWARD_MOTOR2  255
#define MAX_REVERSE_MOTOR2  255

// Brake setting, no brake 0, maximum brake 255. Settings (0-255)
#define BRAKE_MOTOR1  0
#define BRAKE_MOTOR2  0

// Setting the dead zone of poor quality joysticks TX for the motor controller
#define DEAD_ZONE        15

#define MIN_CONTROL_VAL  1000
#define MID_CONTROL_VAL  ((MIN_CONTROL_VAL + MAX_CONTROL_VAL) / 2)
#define MAX_CONTROL_VAL  2000

//*********************************************************************************************************************
// Uncomment only one output option that combines motors, servos and pins.
// (e.g. uncomment only output for 1 motor, no output for servos)
//*********************************************************************************************************************
//#define MOTOR1
//#define MOTOR2
//#define SERVO_8CH
//#define SERVO_7CH_MOTOR1
//#define SERVO_7CH_MOTOR2
#define SERVO_6CH_MOTOR12

#if defined(SERVO_8CH)
  #define SERVO_CHANNELS  8
#endif

#if defined(SERVO_7CH_MOTOR1)
  #define SERVO_CHANNELS  7
  #define MOTOR1
#endif

#if defined(SERVO_7CH_MOTOR2)
  #define SERVO_CHANNELS  7
  #define MOTOR2
#endif

#if defined(SERVO_6CH_MOTOR12)
  #define SERVO_CHANNELS  6
  #define MOTOR1
  #define MOTOR2
#endif

#define MAX_RC_CHANNELS   16 // The maximum number of RC channels that can be sent in one packet
#define MIN_RC_CHANNELS   4  // The minimum number of channels that must be included in a packet, the number of channels cannot be reduced any further than this
#define RF_PAYLOAD_BYTES  24 // 12 bits per value * 16 channels

#define BIND_RF_ADDR      0xA4B7C123F7LL

#define RF_CHANNELS       9 // This is 1/5 of the total number of radio channels used for FHSS
#define MIN_RF_CHANNEL    3 // Channel 0 is right on the boarder of allowed frequency range, so move up to avoid bleeding over

#define RESERVED_MASK_RF_CHANNEL  0x3F

#define OPTION_MASK_MAX_RF_POWER_OVERRIDE  0x40
#define OPTION_MASK_RF_CHANNEL_REDUCTION   0x0F

#define RX_CONNECTION_TIMEOUT  1000000 // If no packet received in this time frame apply failsafe settings. In microseconds

// FHSS parameters
#define DEFAULT_PACKET_INTERVAL     ((uint32_t) 3000) 
#define MAX_PACKET_INTERVAL         ((uint32_t) 4000) // Max packet interval - used with telemetry and 16 channels
#define INITIAL_PACKET_TIMEOUT_ADD  200ul
#define RESYNC_TIME_OUT             ((uint32_t) 2000000) // Go to re-sync if no packet received in 3 seconds
#define RESYNC_WAIT_MICROS          (((((uint32_t) RF_CHANNELS) * 5ul) + 8ul) * MAX_PACKET_INTERVAL) // when syncing listen on each channel for slightly longer than the time it takes to cycle through all channels

#define INITIAL_TELEMETRY_PACKETS_TO_SKIP  1000 // Dont send initial telemetry packets to avoid anoying warnings at startup

#define DO_NOT_SOFT_REBIND             0xAA
#define BOUND_WITH_FAILSAFE_NO_PULSES  0x99

typedef struct
{
  enum RxMode_t : uint8_t
  {
    normal              = 0,
    bind                = 1,
    setFailSafe         = 2,
    normalWithTelemetry = 3,
    telemetryResponse   = 4,
    bindFalesafeNoPulse = 5,
    unBind              = 127
  }
  RxMode;
  
  uint8_t reserved = 0; // Contains the channel number that the packet was sent on in bits 0-5
  
  uint8_t option; // mask 0x0F    : Channel reduction.  The number of channels to not send (subtracted from the 16 max channels) at least 4 are always sent
  //                 mask 0x40>>6 : Contains max power override flag for Multi-protocol TX module. Also sent to RX
  
  uint8_t  modelNum;
  uint8_t  checkSum_LSB; // Checksum least significant byte
  uint8_t  checkSum_MSB; // Checksum most significant byte
  uint8_t  payloadValue[RF_PAYLOAD_BYTES] = {0}; // 12 bits per channel value, unsigned
}
RxTxPacket_t;   


void attach_servo_pins();
void servo_control();
void motor_control();
void output_rc_channels();
void ADC_Processing();
void setupReciever();
void setNextRadioChannel(bool missedPacket);
bool getPacket();
void checkFailsafeDisarmTimeout(unsigned long lastPacketTime, bool inititalGoodPacketRecieved);
void outputFailSafeValues(bool callOutputChannels);
void unbindReciever();
void bindReciever(uint8_t modelNum, uint16_t tempHoldValues[], RxTxPacket_t :: RxMode_t RxMode);
void setFailSafeDefaultValues();
void loadFailSafeDefaultValues();
void setFailSafeValues(uint16_t newFailsafeValues[]);
bool validateChecksum(RxTxPacket_t const& packet, uint8_t maxPayloadValueIndex);
bool readAndProcessPacket();
bool processRxMode(uint8_t RxMode, uint8_t modelNum, uint16_t tempHoldValues[]);
bool decodeChannelValues(RxTxPacket_t const& RxPacket, uint8_t channelsRecieved, uint16_t tempHoldValues[]);
unsigned long sendTelemetryPacket(); // Returns micros of when the transmit is expected to be complete
bool failSafeButtonHeld();
void setTelemetryPowerMode(uint8_t option);
void initializeRadio();

#endif
 
