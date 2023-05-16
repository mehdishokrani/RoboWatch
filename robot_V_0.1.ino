#include <Servo.h>            //to run servo motor
#include <IRremote.hpp>       //Library for IR remote
#include <dht11.h>            //for DHT11 Temperature and Humidity Sensor
#include <FastLED.h>          //Used for INbuilt RGB LED  WS2812
#include "Driver_xxx0.h"      //DC motor Driver
#include "function_xxx0.cpp"  //DC motor Function

//Define servo pin and make an servo instance from library
#define servo_pin 10
Servo myservo;

//Define pin number for fron and back ultrasonic sensors also added to variable for return functio
#define trig_front 13
#define echo_front 12
#define trig_back 53
#define echo_back 52
int distance1;
int distance2;
int distance3;
int distance_back;
//long duration;

//make an istance of motor driver
Driver_Motor Motor;

//define pin for Remote and also enum for different positions to move
#define IR_RECEIVE_PIN 49
enum directions {
  forward = 70,   //Forward
  backward = 21,  //Backward
  left = 68,
  right = 67,
  leftforward = 25,    //2
  rightforward = 13,   //3
  leftbackward = 24,   //5
  rightbackward = 94,  //6
  stop = 64,
  automatic = 22,  //1
  nothing = 0      //default taulerance zero which received some times
};
int command = 64;  //command received from IR remote

//Define pin number for temp DHT11 sensor and also make an instance of that
#define DHT11PIN A1
dht11 DHT11;

//Define PIR motion Sensor
#define pir_sensor A2

// Define number of INbuilt RGB LED and also its pin WS2812
#define NUM_LEDS 1
#define DATA_PIN 4
CRGB leds[NUM_LEDS];  // Define the array of leds

//Photocell Pin
#define light_sensor A15

//RGB LED Pins
#define red_pin 37
#define green_pin 29
#define blue_pin 23
bool flag_white_rgb = false;  //this flag shows that there is no light

//Sound Sensor PIN
#define sound_sensor A0

//Active Buzzer PIN
#define buzzer_pin 51

int i = 0;             //as counter to for loop
int previous_command;  //to remove 0 error
int servo_angle = 90;  //angle of servo motor

void setup() {
  Serial.begin(9600);

  //Set pin mode for Servo and attach it to pin and Initialize it to 90 degree
  pinMode(servo_pin, OUTPUT);
  myservo.attach(servo_pin);
  myservo.write(servo_angle);

  //pinmode and Initialize remote
  pinMode(IR_RECEIVE_PIN, INPUT);
  IrReceiver.begin(IR_RECEIVE_PIN);

  //Pin mode for DHT11 temperature sensor
  pinMode(DHT11PIN, INPUT);

  //Pin mode for PIR motion sensor
  pinMode(pir_sensor, INPUT);

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // CRGB initialization for INbuilt RGB LED

  //Define Pin Mode for RGB LED and LIGHT Sensor (Photocell)
  pinMode(light_sensor, INPUT);
  pinMode(red_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);
  pinMode(blue_pin, OUTPUT);

  //Initialize RGB LED to OFF Mode
  digitalWrite(red_pin, LOW);
  digitalWrite(green_pin, LOW);
  digitalWrite(blue_pin, LOW);

  //Define Pin Mode for Sound Sensor
  pinMode(sound_sensor, INPUT);

  //Define Buzzer Pin and Initialize to LOW
  pinMode(buzzer_pin, OUTPUT);
  digitalWrite(buzzer_pin, LOW);

  //initialize DC motors
  Motor.Init();
  // Forward,       //(0)
  // Backward,      //(1)
  // Left,          //(2)
  // Right,         //(3)
  // LeftForward,   //(4)
  // LeftBackward,  //(5)
  // RightForward,  //(6)
  // RightBackward, //(7)
  // stop_it        //(8)


  //Define ultrasonic fron and back sensor pins
  pinMode(trig_front, OUTPUT);
  pinMode(echo_front, INPUT);
  pinMode(trig_back, OUTPUT);
  pinMode(echo_back, INPUT);

  delay(5000);
}

void loop() {
  //delay(300);
  //Serial.println("Loop Begin");
  if (analogRead(light_sensor) < 200) {  //In dark Place turn on Flash White
    flag_white_rgb = true;
    digitalWrite(red_pin, HIGH);
    digitalWrite(green_pin, HIGH);
    digitalWrite(blue_pin, HIGH);
    delay(300);
  } else {  // Else Blink Red
    flag_white_rgb = false;
    digitalWrite(red_pin, HIGH);
    digitalWrite(green_pin, LOW);
    digitalWrite(blue_pin, LOW);
    delay(300);
    digitalWrite(red_pin, LOW);
    digitalWrite(green_pin, LOW);
    digitalWrite(blue_pin, LOW);
  }
  //Serial.println(analogRead(sound_sensor));
  // if (analogRead(sound_sensor) > 600) {  //Check if there is a sound in area  ***it should be trimed depend on area noise***
  //   for (i = 0; i < 6; i++) {
  //     //Yellow CRGB For Sound Detection
  //     leds[0] = CRGB(255, 255, 0);
  //     FastLED.show();
  //     if (!flag_white_rgb) {
  //       digitalWrite(red_pin, HIGH);
  //       digitalWrite(green_pin, LOW);
  //       digitalWrite(blue_pin, LOW);
  //     }
  //     digitalWrite(buzzer_pin, HIGH);
  //     delay(100);
  //     if (!flag_white_rgb) {
  //       digitalWrite(red_pin, LOW);
  //       digitalWrite(green_pin, LOW);
  //       digitalWrite(blue_pin, LOW);
  //     }
  //     digitalWrite(buzzer_pin, LOW);
  //     //delay(200);
  //   }
  // } else {
  //   delay(100);
  // }

  // DHT11.read(DHT11PIN);  //Initialize DHT11 Sensor and check if temperature is too hot
  // //Serial.println(DHT11.temperature);
  // if (DHT11.temperature >= 60) {
  //   leds[0] = CRGB::Blue;
  //   FastLED.show();
  //   if (flag_white_rgb) {
  //     digitalWrite(red_pin, HIGH);
  //     digitalWrite(green_pin, HIGH);
  //     digitalWrite(blue_pin, HIGH);
  //   }
  //   digitalWrite(buzzer_pin, LOW);
  //   digitalWrite(buzzer_pin, HIGH);
  //   function_direction(8, 100);
  //   function_direction(1, 100);
  //   delay(2000);
  //   function_direction(8, 100);
  // }
  // Serial.println(analogRead(pir_sensor));
  // if (analogRead(pir_sensor) < 10) {  // Check if there is any movement with PIR sensor   **set this later to 100**
  //   leds[0] = CRGB::Yellow;
  //   FastLED.show();
  //   if (flag_white_rgb) {
  //     digitalWrite(red_pin, HIGH);
  //     digitalWrite(green_pin, HIGH);
  //     digitalWrite(blue_pin, HIGH);
  //   }
  //   digitalWrite(buzzer_pin, HIGH);
  //   delay(100);
  //   function_direction(8, 100);
  //   //delay(1000);
  // }
  // function_direction(8, 100); ///******* NEW
  //Check status of the remote and move accordingly
  //Serial.println(get_distance_front(servo_angle));
  if (IrReceiver.decode()) {
    //read pushed key on remote and switch according to enum that is defined at begining
    command = IrReceiver.decodedIRData.command;
    IrReceiver.resume();
    servo_angle = 90;
    myservo.write(servo_angle);
    //delay(50);
    //Serial.println(command);
    Serial.print("In Remote");
    Serial.println(command);
    switch (command) {
      case forward:
        previous_command = command;
        function_direction(8, 120);
        delay(300);
        function_direction(0, 80);
        break;
      case backward:
        previous_command = command;
        function_direction(8, 120);
        delay(300);
        function_direction(1, 80);
        break;
      case left:
        previous_command = command;
        function_direction(8, 120);
        delay(300);
        function_direction(2, 120);
        delay(200);
        function_direction(8, 120);
        break;
      case right:
        previous_command = command;
        function_direction(8, 120);
        delay(300);
        function_direction(3, 120);
        delay(200);
        function_direction(8, 120);
        break;
      case leftforward:
        previous_command = command;
        function_direction(8, 120);
        delay(300);
        function_direction(4, 120);
        delay(200);
        function_direction(8, 120);
        break;
      case rightforward:
        previous_command = command;
        function_direction(8, 120);
        delay(300);
        function_direction(6, 120);
        delay(200);
        function_direction(8, 120);
        break;
      case leftbackward:
        previous_command = command;
        function_direction(8, 120);
        delay(300);
        function_direction(5, 120);
        delay(200);
        function_direction(8, 120);
        break;
      case rightbackward:
        previous_command = command;
        function_direction(8, 120);
        delay(300);
        function_direction(7, 120);
        delay(200);
        function_direction(8, 120);
        break;
      case stop:
        previous_command = command;
        function_direction(8, 120);
        delay(300);
        break;
      case automatic:  // Autmatic functions will be played here
        // if (previous_command == 22) {
        //   command = 22;
        // }

        //***** Go in Automatic Mode   ********
        break;
      // case nothing:
      //   if (previous_command == 22) {
      //     command = 22;
      //   }
      //   break;
    }
  }

  //Go in Automatic Mode until another key is pressed
  if (command == 22 || (previous_command == 22 && command == 0)) {
    if(command == 22)
    previous_command = command;
    automatic_movement();
  }


  digitalWrite(buzzer_pin, LOW);
  leds[0] = CRGB::Black;
  FastLED.show();
  distance1 = 3;
  distance2 = 3;
  distance3 = 3;
  distance_back = 3;
  servo_angle = 90;
}

void automatic_movement() {
  Serial.println("In Auto");
  servo_angle = 90;
  distance3 = get_distance_front(servo_angle);
  distance1 = get_distance_front(servo_angle + 40);
  distance2 = get_distance_front(servo_angle - 40);
  // myservo.write(servo_angle);
  // if((distance1 > 800 && distance2 > 800) || distance3>800){
  //   distance3 = get_distance_front(servo_angle);
  //   distance1 = get_distance_front(servo_angle + 40);
  //   distance2 = get_distance_front(servo_angle - 40);
  // }
  if (distance1 > 30 && distance2 > 30 && distance3>30) {  //(distance1 > 40 && distance2 > 40) || (distance1>40 && distance3>40) || (distance2>40 && distance3>40)
    function_direction(0, 60);
  } 
  else if (((distance1 > 20 && distance1 <= 30) && (distance2 > 20 && distance2 <= 30)) || ((distance1 > 20 && distance1 <= 30) && (distance3 > 20 && distance3 <= 30)) || ((distance3 > 20 && distance3 <= 30) && (distance2 > 20 && distance2 <= 30)) ) {
    function_direction(0, 50);
  } 
  // else if (distance1 < 2 || distance2 < 2 || distance3 < 2) {
  //   function_direction(8, 80);
  // } 
  else if ( distance1 <= 20 || distance2 <= 20 ||  distance3 <= 20) {
    function_direction(8, 80);
    servo_angle = 45;
    myservo.write(servo_angle);
    distance2 = get_distance_front(servo_angle);
    distance1 = get_distance_front(servo_angle - 30);
    //distance2 = get_distance_front(servo_angle - 40);
    servo_angle = 90;
    myservo.write(servo_angle);  //NEW
    if (distance1 > 40 && distance2 > 40) {
      function_direction(3, 80);
      delay(750);
      // function_direction(0, 50);
      function_direction(8, 80);
    } 
    // else if (distance1 < 2 || distance2 < 2) {
    //   function_direction(8, 80);
    // } 
    else if ((distance1 > 20 && distance1 <= 40) && (distance2 > 20 && distance2 <= 40)) {
      function_direction(3, 80);
      delay(750);
      // function_direction(0, 50);
      function_direction(8, 80);
    } else if (distance1 <= 20 || distance2 <= 20 ) {
      // function_direction(8, 80);
      servo_angle = 135;
      myservo.write(servo_angle);
      distance1 = get_distance_front(servo_angle + 30);
      distance2 = get_distance_front(servo_angle);
      servo_angle = 90;
      myservo.write(servo_angle);
      if (distance1 > 40 && distance2 > 40) {
        function_direction(2, 80);
        delay(750);
        // function_direction(0, 50);
        function_direction(8, 80);
      } 
      // else if (distance1 < 2 || distance2 < 2) {
      //   function_direction(8, 80);
      // }
      else if ((distance1 > 20 && distance1 <= 40) && (distance2 > 20 && distance2 <= 40) ) {
        function_direction(2, 80);
        delay(750);
        // function_direction(0, 50);
        function_direction(8, 80);
      } 
      else if (distance1 <= 20 || distance2 <= 20) {
        // function_direction(8, 80);
        servo_angle = 180;
        myservo.write(servo_angle);
        distance1 = get_distance_front(servo_angle);
        distance2 = get_distance_front(servo_angle - 30);
        servo_angle = 90;
        myservo.write(servo_angle);
        if (distance1 > 40 && distance2 > 40) {
          function_direction(2, 80);
          delay(800);
          // function_direction(0, 50);
          function_direction(8, 80);
        } 
        else if ((distance1 > 20 && distance1 <= 40) && (distance2 > 20 && distance2 <= 40) ) {
          function_direction(2, 80);
          delay(1000);
          // function_direction(0, 50);
          function_direction(8, 80);
        } 
        // else if (distance1 < 2 || distance2 < 2) {
        //   function_direction(8, 80);
        // }
        else if (distance1 <= 20 || distance2 <= 20) {
          // function_direction(8, 80);
          servo_angle = 0;
          myservo.write(servo_angle);
          distance1 = get_distance_front(servo_angle + 30);
          distance2 = get_distance_front(servo_angle);
          servo_angle = 90;
          myservo.write(servo_angle);
          if (distance1 > 40 && distance2 > 40) {
            function_direction(3, 80);
            delay(800);
            // function_direction(0, 50);
            function_direction(8, 80);
          } else if ((distance1 > 20 && distance1 <= 40) && (distance2 > 20 && distance2 <= 40) ) {
            function_direction(3, 80);
            delay(1000);
            // function_direction(0, 50);
            function_direction(8, 80);
          } else if (distance1 <= 20 || distance2 <= 20) {
            // function_direction(8, 80);
            // servo_angle = 90;
            // myservo.write(servo_angle);
            distance_back = get_distance_back();
            if (distance_back > 40) {
              function_direction(1, 80);
              delay(1500);
              function_direction(2, 80);
              delay(1000);
              //function_direction(0, 80);
              // function_direction(8, 80);
              function_direction(8, 80);
            } 
            // else if (distance_back < 2) {
            //   function_direction(8, 80);
            // } 
            else if (distance_back > 20 && distance_back <= 40) {
              function_direction(1, 80);
              delay(800);
              function_direction(2, 80);
              delay(1200);
              function_direction(8, 80);
            } else{
              function_direction(2, 80);
              delay(1600);
              function_direction(8, 80);
            }
          }
        }
      }
    }
  }
  else {
    function_direction(3, 80);
    delay(750);
    function_direction(8, 80);
  }
  if (IrReceiver.decode()) {
    //read pushed key on remote and switch according to enum that is defined at begining
    command = IrReceiver.decodedIRData.command;
    IrReceiver.resume();
    
    if(command>0)
    previous_command = command;
  }
  return 0;
}

int get_distance_front(int angle) {
  bool flag = true;
  if (IrReceiver.decode()) {
    //read pushed key on remote and switch according to enum that is defined at begining
    command = IrReceiver.decodedIRData.command;
    IrReceiver.resume();
    
    if(command>0)
    previous_command = command;
  }
  // delay(100);
  myservo.write(angle);
  delay(200);
  int i =0 ;
  long duration_f = 0;
  int dist_front = 1;
  while(flag && i<6){
    
    digitalWrite(trig_front, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_front, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_front, LOW);
    duration_f = pulseIn(echo_front, HIGH);
    dist_front = duration_f * 0.034 / 2;
    i++;
    if (dist_front <950)
    flag=false;
  }
  // myservo.write(servo_angle);
  Serial.println(dist_front);
  // if (IrReceiver.decode()) {
  //   //read pushed key on remote and switch according to enum that is defined at begining
  //   command = IrReceiver.decodedIRData.command;
  //   IrReceiver.resume();
    
  //   if(command>0)
  //   previous_command = command;
  // }
  return dist_front;
}

int get_distance_back() {
  long duration_b = 0;
  int dist_back = 1;
  for (int j = 0; j < 5; j++) {
    digitalWrite(trig_back, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_back, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_back, LOW);
    duration_b = pulseIn(echo_back, HIGH);
  }
  dist_back = duration_b * 0.034 / 2;
  Serial.print("Back: ");
  Serial.println(dist_back);
  return dist_back;
}
