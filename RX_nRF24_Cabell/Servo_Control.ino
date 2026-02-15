
//*********************************************************************************************************************
// Servo setup
//*********************************************************************************************************************
Servo servo[SERVO_CHANNELS]; // Class driver

void servo_setup()
{
#if defined(SERVO_8CH) || defined(SERVO_7CH_MOTOR1) || defined(SERVO_7CH_MOTOR2) || defined(SERVO_6CH_MOTOR1_2)
  for (byte i = 0; i < SERVO_CHANNELS; i++)
  {
    servo[i].attach(pins_servo[i]);
  }
#endif
}

//*********************************************************************************************************************
// Servo control
//*********************************************************************************************************************
void servo_control()
{
#if defined(SERVO_8CH)
  for (byte i = 0; i < SERVO_CHANNELS; i++)
  {
    servo[i].writeMicroseconds(rc_packet[i]);
  }
#endif

#if defined(SERVO_7CH_MOTOR1)
  for (byte i = 0; i < SERVO_CHANNELS; i++)
  {
    servo[i].writeMicroseconds(rc_packet[i + MOTOR_CHANNELS]);
  }
#endif

#if defined(SERVO_7CH_MOTOR2)
  for (byte i = 0; i < SERVO_CHANNELS; i++)
  {
    servo[i].writeMicroseconds(rc_packet[i + MOTOR_CHANNELS]);
  }
#endif

#if defined(SERVO_6CH_MOTOR1_2)
  for (byte i = 0; i < SERVO_CHANNELS; i++)
  {
    servo[i].writeMicroseconds(rc_packet[i + MOTOR_CHANNELS]);
  }
#endif
}
 
