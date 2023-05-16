#include <avr/wdt.h>
#include "Driver_xxx0.h"
#include "function_xxx0.cpp"
#include "Servo_Driver_xxx0.h"
#include "Ultra_xxx0.h"
#include <IRremote.h>
#include <DHT.h>

//Desfine pins for pir motion sensore and temperature sensor
#define pirPin A2
#define tempSensor A1

//Attach temperature sensor to its library
DHT dht(A1,DHT11);

//IR remote receiver
#define IR_RECEIVE_PIN 9
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
  automatic = 22  //1
};
int command = 22;  //command received from IR remote

//Ultrasonic sensor object
Ultra ultra;

//Servo Object
Servo_Driver Servo;
//Servo.control(Degree);


//Motor Object
Driver_Motor Motor;
App_xxx app_motorsxxx0;


void setup() {
  Serial.begin(9600);

  //initialize dc motors
  // Forward,       //(0)
  // Backward,      //(1)
  // Left,          //(2)
  // Right,         //(3)
  // LeftForward,   //(4)
  // LeftBackward,  //(5)
  // RightForward,  //(6)
  // RightBackward, //(7)
  // stop_it        //(8)
  //function_direction(diection_number,speed);       ************
  Motor.Init();
  delay(2000);
  //function_direction(2,100);

  //Initialize ultrasonic sensor  ********
  ultra.Init();

  //make pir pin to inpute mode   ******
  pinMode(pirPin, INPUT);
  // start temperature sensor 
  dht.begin();
  //can use dht.readHumidity() and dht.readTemperature() to read humidity and temperature from sensor wich temperature is degree cellcious already  *******

  //Servo Intial value by 90 Degree   *******
  Servo.Init(90);
  delay(1000);
  // Servo.control(angle); to set servo motor rotation angle      *****

  //set pin mode for PIR and temp sensore   ****
  pinMode(IR_RECEIVE_PIN, INPUT);
  IrReceiver.begin(IR_RECEIVE_PIN);


}

void loop() {
  //To Read ultrasonic sensor use ultra.Get(); which retirns unsigne integer which should be used inside the loop  ****
  
  //it goes manually with IR command until it receive key 1 on remote then goea to automatic mode  ******
  
  if (IrReceiver.decode()) {
    command = IrReceiver.decodedIRData.command;
    Serial.println(command);
    IrReceiver.resume();
    switch (command) {
      case forward:
        Serial.println("Forward");
        function_direction(0, 100);
        break;
      case backward:
        Serial.println("BackWard");
        function_direction(1, 100);
        break;
      case left:
        Serial.println("LEFT");
        function_direction(2, 150);
        delay(100);
        function_direction(8, 150);
        break;
      case right:
        Serial.println("RIGHT");
        function_direction(3, 150);
        delay(100);
        function_direction(8, 150);
        break;
      case stop:
        Serial.println("STOP");
        function_direction(8, 150);
        break;
      case automatic: // Autmatic functions will be played here
        function_direction(8, 0);
        Serial.println("WE are in Automatic mode to run auto funstions");
        break;
    }
  }
}
