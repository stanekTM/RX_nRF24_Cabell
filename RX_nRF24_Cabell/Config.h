
#ifndef __Config_h__
#define __Config_h__

#include <Arduino.h>

//#define SERVO_8CH
//#define SERVO_7CH_MOTOR1
//#define SERVO_7CH_MOTOR2
#define SERVO_6CH_MOTOR1_2

#if defined(SERVO_8CH)
  #define SERVO_CHANNELS  8
#endif

#if defined(SERVO_7CH_MOTOR1)
  #define SERVO_CHANNELS  7
  #define MOTOR_CHANNELS  1
  
  #define MOTOR1
  #define PWM_122HZ_TIMER2_3_11
  #define REACTION_MOTOR1  0
  #define MAX_FORWARD_MOTOR1  255
  #define MAX_REVERSE_MOTOR1  255
  #define BRAKE_MOTOR1  0
#endif

#if defined(SERVO_7CH_MOTOR2)
  #define SERVO_CHANNELS  7
  #define MOTOR_CHANNELS  1
  
  #define MOTOR2
  #define PWM_976HZ_TIMER0_5_6_DEFAULT
  #define REACTION_MOTOR2  0
  #define MAX_FORWARD_MOTOR2  255
  #define MAX_REVERSE_MOTOR2  255
  #define BRAKE_MOTOR2  0
#endif

#if defined(SERVO_6CH_MOTOR1_2)
  #define SERVO_CHANNELS  6
  #define MOTOR_CHANNELS  2
  
  #define MOTOR1
  #define PWM_122HZ_TIMER2_3_11
  #define REACTION_MOTOR1  0
  #define MAX_FORWARD_MOTOR1  255
  #define MAX_REVERSE_MOTOR1  255
  #define BRAKE_MOTOR1  0
  
  #define MOTOR2
  #define PWM_976HZ_TIMER0_5_6_DEFAULT
  #define REACTION_MOTOR2  0
  #define MAX_FORWARD_MOTOR2  255
  #define MAX_REVERSE_MOTOR2  255
  #define BRAKE_MOTOR2  0
#endif

// Setting the dead zone of poor quality joysticks TX for the motor controller
#define DEAD_ZONE  15

#endif // End __Config_h__
 
