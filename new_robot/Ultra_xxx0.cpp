#include "Ultra_xxx0.h"

/*ULTRASONIC*/
//#include <NewPing.h>
// NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
void Ultra::Init(void)
{
  pinMode(ECHO_PIN, INPUT); //Ultrasonic module initialization
  pinMode(TRIG_PIN, OUTPUT);
}
unsigned int Ultra::Get()
{
  unsigned int tempda_x = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempda_x = ((unsigned int)pulseIn(ECHO_PIN, HIGH) / 58);
  return tempda_x;
  // sonar.ping() / US_ROUNDTRIP_CM; // Send ping, get ping time in microseconds (uS).
}

void Ultra::Test(void)
{

  unsigned int tempda = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempda = ((unsigned int)pulseIn(ECHO_PIN, HIGH) / 58);

  // if (tempda_x > 50)
  // {
  //   tempda_x = 50;
  // }

  // // return tempda;
  // return tempda_x;

  Serial.print("ULTRASONIC=");
  Serial.print(tempda); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.println("cm");
}
