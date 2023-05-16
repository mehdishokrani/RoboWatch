
#include "Servo_Driver_xxx0.h"
#include <arduino.h>
/*Servo*/

Servo myservo; // create servo object to control a servo
void Servo_Driver::Init(unsigned int Position_angle)
{
  myservo.attach(PIN_Servo_z, 500, 2400); //500: 0 degree  2400: 180 degree
  myservo.attach(PIN_Servo_z);
  myservo.write(Position_angle); //sets the servo position according to the 90（middle）
  delay(500);

}
#if _Test_Servo_Driver
void Servo_Driver::Servo_Driver_Test(void)
{
  for (;;)
  {
    myservo.attach(PIN_Servo_z);
    myservo.write(180);
    delay(500);
    myservo.write(0);
    delay(500);
  }
}
#endif

/*0.17sec/60degree(4.8v)*/
void Servo_Driver::control(unsigned int Position_angle)
{
  myservo.attach(PIN_Servo_z);
  myservo.write(Position_angle);
  delay(450);
  myservo.detach();
}
