#ifndef _Ultra_xxx0_H_
#define _Ultra_xxx0_H_
#include <Arduino.h>
/*ULTRASONIC*/

//#include <NewPing.h>
class Ultra
{
public:
  void Init(void);
  void Test(void);
  unsigned int Get();

private:
#define TRIG_PIN 13      // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN 12      // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
};

#endif
