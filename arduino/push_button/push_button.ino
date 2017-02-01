/* 
 * Button Example for Rosserial
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <fydp/MoveData.h>
#include <Servo.h>

// Front motors
Servo motor1;
Servo motor2;
// Back motors
Servo motor3;
Servo motor4;

const int motor1Pin = 3;
const int motor2Pin = 4;
const int motor3Pin = 5;
const int motor4Pin = 6;

// 0 - 93 Reverse, > 93 Forward
const int motor1StopPos = 93;
const int motor3StopPos = 93;
// 0 - 93 Forward, > 93 Reverse
const int motor2StopPos = 93;
const int motor4StopPos = 93;

const int thresX = 256;
const int thresA = 22000;
const int margX = 250;
const int margA = 2000;

bool flag = false;

ros::NodeHandle nh;

void followCallback(const fydp::MoveData& msg) {
  int inX = msg.x;
  String sx = String(inX);
  nh.loginfo(sx.c_str());
  
  int inY = msg.y;
  String sy = String(inY);
  nh.loginfo(sy.c_str());
  
  int inArea = msg.area;
  String sArea = String(inArea);
  nh.loginfo(sArea.c_str());
  
  if (inArea <= (thresA+margA) && inArea >= (thresA-margA)) {
      stop();
  }
  else {
     if (inX < (thresX-margX))
       turnSharpLeft(10);
     else if(inX > (thresX+margX))
       turnSharpRight(10);
     else {
       if (inArea > (thresA+margA))
         drive(-3);
       if (inArea < (thresA-margA))
         drive(3);
     }
  }  
}

std_msgs::Bool pushed_msg;
ros::Publisher pub_button("pushed", &pushed_msg);
ros::Subscriber<fydp::MoveData> sub("follower", &followCallback);

const int button_pin = 2;
const int led_pin = 13;

bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;

void initializePins() {
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);
}

void drive(int speed) {
  setSpeed(speed);
}

void stop() {
  setSpeed(93);
}

void setSpeed(int speed) {
  setLeftWingSpeed(speed);
  setRightWingSpeed(speed);
}

void setLeftWingSpeed(int speed) {
  motor1.write(motor1StopPos + speed);
  motor3.write(motor3StopPos + speed);
}

void setRightWingSpeed(int speed) {
  motor2.write(motor2StopPos - speed);
  motor4.write(motor4StopPos - speed);
}

void turnSharpLeft(int speed) {
  setLeftWingSpeed(-speed);
  setRightWingSpeed(speed);
}

void turnSharpRight(int speed) {
  setLeftWingSpeed(speed);
  setRightWingSpeed(-speed);
}

void setup()
{
  nh.initNode();
  nh.advertise(pub_button);
  nh.subscribe(sub);
  //initialize an LED output pin 
  //and a input pin for our push button
  pinMode(led_pin, OUTPUT);
  pinMode(button_pin, INPUT);
  
  //Enable the pullup resistor on the button
  digitalWrite(button_pin, HIGH);
  
  //The button is a normally button
  last_reading = ! digitalRead(button_pin);
  initializePins();
  stop();
 
}

void loop()
{
  bool reading = ! digitalRead(button_pin);
  
  if (last_reading!= reading){
      last_debounce_time = millis();
      published = false;
  }
  
  //if the button value has not changed for the debounce delay, we know its stable
  if ( !published && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(led_pin, reading);
    pushed_msg.data = reading;
    pub_button.publish(&pushed_msg);
    published = true;
  }

  last_reading = reading;
  
  nh.spinOnce();
}
