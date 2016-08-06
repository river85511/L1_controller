
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <SoftwareSerial.h>

#include <ros.h>
#include <std_msgs/UInt16.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

ros::NodeHandle  nh;

Servo servo; 

int motorPin = 3;
int speed;
int pos = 90;

void steering( const std_msgs::Float32MultiArray& cmd_msg){
  servo.write(90-cmd_msg.data[0]*1000); //set servo angle, should be from 0-180

  if(cmd_msg.data[1] == 0){
              speed = 180;  
                analogWrite(motorPin, speed);
                delay(2);
  }else if(cmd_msg.data[1] != 0){
                analogWrite(motorPin, 180);
                delay(2);
                speed = 197+cmd_msg.data[1]*15;
                analogWrite(motorPin, speed);
                delay(2);
  }
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::Float32MultiArray> sub("l1control", steering);

void setup(){
  Serial.begin(9600);

  //nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);

  delay(1000);

  pinMode(13, OUTPUT);
  pinMode(motorPin, OUTPUT);
  
  servo.attach(9); //attach it to pin 9//UP DOWN
}

void loop(){
  nh.spinOnce();
  delay(0.1);
}
