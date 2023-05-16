//#include <Smartcar.h>

#include "Driver_xxx0.h"

extern Driver_Motor AppMotor;


enum direction {
  Forward,        //(1)
  Backward,       //(2)
  Left,           //(3)
  Right,          //(4)
  LeftForward,    //(5)
  LeftBackward,   //(6)
  RightForward,   //(7)
  RightBackward,  //(8)
  stop_it         //(9)
};

struct App_xxx {
  direction Motion;
};

extern App_xxx app_motorsxxx0;

static void function_direction(direction direction, uint8_t is_speed) {
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;
  switch (direction) {
    case /* constant-expression */
      Forward:
      /* code */
      AppMotor.Control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                       /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable);  //Motor control
      break;
    case /* constant-expression */ Backward:
      /* code */

      AppMotor.Control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                       /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable);  //Motor control
      break;
    case /* constant-expression */ Left:

      AppMotor.Control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                       /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable);  //Motor control
      break;
    case /* constant-expression */ Right:

      AppMotor.Control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                       /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable);  //Motor control
      break;
    case /* constant-expression */ LeftForward:
      /* code */
      AppMotor.Control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                       /*direction_B*/ direction_just, /*speed_B*/ speed / 2, /*controlED*/ control_enable);  //Motor control
      break;
    case /* constant-expression */ LeftBackward:
      AppMotor.Control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                       /*direction_B*/ direction_back, /*speed_B*/ speed / 2, /*controlED*/ control_enable);  //Motor control
      break;
    case /* constant-expression */ RightForward:
      AppMotor.Control(/*direction_A*/ direction_just, /*speed_A*/ speed / 2,
                       /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable);  //Motor control
      break;
    case /* constant-expression */ RightBackward:
      AppMotor.Control(/*direction_A*/ direction_back, /*speed_A*/ speed / 2,
                       /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable);  //Motor control
      break;
    case /* constant-expression */ stop_it:
      AppMotor.Control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                       /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable);  //Motor control
      break;
    default:
      break;
  }
}
