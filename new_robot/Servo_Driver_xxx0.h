
#ifndef _Servo_Driver_xxx0_H_
#define _Servo_Driver_xxx0_H_

/*Servo*/
#include <Servo.h>
class Servo_Driver
{
public:
  void Init(unsigned int Position_angle);
#if _Test_Servo_Driver
  void Servo_Driver_Test(void);
#endif
  void control(unsigned int Position_angle);

private:
#define PIN_Servo_z 10
};

#endif
