
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
Servo lock;
Servo lock2;

//Pin initialization
const int motor1Pin = 3;
const int motor2Pin = 4;
const int motor3Pin = 5;
const int motor4Pin = 6;
const int lockPin = 9;
const int lock2Pin = 12;
const int door1Pin = 10;
const int door2Pin = 11;

// 0 - 93 Reverse, > 93 Forward
const int motor1StopPos = 93;
const int motor3StopPos = 93;
// 0 - 93 Forward, > 93 Reverse
const int motor2StopPos = 93;
const int motor4StopPos = 93;

// Centroid thresholds
int thresX = 250;
int thresY = 0;
long thresA = 0;

// margins
int margX = 20;
long margA = 0;
int sharpX = 100;

int prevSpeed = 0;


bool flag = false;
bool locked = false;

// Declaration of ROS node handle
ros::NodeHandle nh;

// Initialize required thresholds based on initial centroid data
void initCallback(const fydp::MoveData& msg) {
   thresA = msg.area;
   margA = thresA/10;
   thresY = msg.y;
   
   nh.loginfo("Initial Area:");
   String a = String(thresA);
   nh.loginfo(a.c_str());
   
   nh.loginfo("Area Margin:");
   String b = String(margA);
   nh.loginfo(b.c_str());
   
   nh.loginfo("Y-THRESHOLD:");
   String c = String(thresY);
   nh.loginfo(c.c_str());
}


void followCallback(const fydp::MoveData& msg) {
  int inX = msg.x;
  String sx = String(inX);
  nh.loginfo(sx.c_str());
  
  int inY = msg.y;
  String sy = String(inY);
  nh.loginfo(sy.c_str());
  
  unsigned long inArea = msg.area;
  String sArea = String(inArea);
  nh.loginfo(sArea.c_str());
  
  // operation range: follow if the person gets further from initial position
  // TODO: might have to make it more difficult to get into this loop (i.e., thresY+larger number)
  if (inArea <= thresA-margA && inY>thresY+30 &&  inArea >= 5000) {
    	follow(inX, inY, inArea);
  }
  // otherwise, stop
//  else if (inY<thresY+10) {
//    if (inX > (thresX+sharpX)) {
//      turnSharpRight(8);
//    }
//    else if (inX < (thresX-sharpX)) {
//      turnSharpLeft(8); 
//    }
//    else {
//      nh.loginfo("STOP");
////    setSpeed(-2);
//      stop();
//    }
//  }
  else {
    nh.loginfo("STOP");
    stop();
  } 
}


void follow(int inX, int inY, unsigned long inArea) {
    //Right turn
    if (inX > (thresX+margX)) {
	turnRight(inX, inY, inArea);  
    }
    //Left turn
    else if (inX < (thresX-margX)) {
      	turnLeft(inX, inY, inArea); 
    }
    //Move forward
    else {
    	driveForward(inX, inY, inArea); 
    }

}

void turnRight(int inX, int inY, unsigned long inArea) {
	//Sharp Turning
      if (inX > thresX+sharpX) {
        int sharpXDiff = inX-thresX-sharpX;
        if (sharpXDiff > 30)
          turnSharpRight(11);
        else
          turnSharpRight(10);
        nh.loginfo("turning sharp right");
      }
      //Smooth Turning
      else {
      	 //scale to take distance into account as well. i.e., when turning, we want to move it at same prev speed
         int scaledSpeed = (int)(1.0*(inX-thresX-margX)/20 + 10);
         float distanceAdjustment = 1.0*(inY-thresY)/thresY*9;
         setLeftWingSpeed((int)(scaledSpeed+distanceAdjustment));
         setRightWingSpeed((int)(3+distanceAdjustment));
         nh.loginfo("turning right");
         String ss = String(scaledSpeed);
         nh.loginfo("*****SPEED*****");
         nh.loginfo(ss.c_str());
      }
}

void turnLeft(int inX, int inY, unsigned long inArea) {
	//Sharp turning
      if (inX < thresX-sharpX) {
        int sharpXDiff = thresX-sharpX-inX;
        if (sharpXDiff > 30)
          turnSharpLeft(11);
        else
          turnSharpLeft(10);
         nh.loginfo("turning sharp left");
      }
      //Smooth turning
      else {
      	//scale to take distance into account as well i.e., when turning, we want to move it at approx same prev speed
        // max scaledSpeed = 16;
        int scaledSpeed = (int)(1.0*(thresX-margX-inX)/20 + 10);
        float distanceAdjustment = 1.0*(inY-thresY)/thresY*9;
        setRightWingSpeed((int)(scaledSpeed+distanceAdjustment));
        setLeftWingSpeed((int)(3+distanceAdjustment));
        nh.loginfo("turning left");
        String ss = String(scaledSpeed);
        nh.loginfo("*****SPEED*****");
        nh.loginfo(ss.c_str());
      }
}

void driveForward(int inX, int inY, unsigned long inArea) {
      int scaledSpeed = (int)((1.0*(inY-thresY)/thresY)*13 + 6);
      
      if (scaledSpeed > 15) {
          scaledSpeed = 15;
      }
      
      if (scaledSpeed - prevSpeed >= 6) {
        while (prevSpeed < scaledSpeed) {
          setSpeed(prevSpeed+2);
          delay(200);
          prevSpeed = prevSpeed + 2;
        }
      }
      setSpeed(scaledSpeed);
      prevSpeed = scaledSpeed;
      String ss = String(scaledSpeed);
      nh.loginfo("*****SPEED*****");                                
      nh.loginfo(ss.c_str());
      nh.loginfo("driving forward");
}

std_msgs::Bool pushed_msg;

//Initialization of ROS publishers and subscribers
ros::Subscriber<fydp::MoveData> init_sub("init", &initCallback);
ros::Publisher pub_button("pushed", &pushed_msg);
ros::Subscriber<fydp::MoveData> sub("follower", &followCallback);

const int button_pin = 2;
const int led_pin = 13;

bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;


// MOTOR HANDLING
void initializePins() {
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);
}

void stop() {
  setSpeed(90);
  prevSpeed = 0;
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
  delay(50);
  stop();
}

void turnSharpRight(int speed) {
  setLeftWingSpeed(speed);
  setRightWingSpeed(-speed);
  delay(50);
  stop();
}

void setup()
{
  nh.initNode();
  nh.advertise(pub_button);
  nh.subscribe(sub);
  nh.subscribe(init_sub);
  //initialize an LED output pin 
  //and a input pin for our push button
  pinMode(led_pin, OUTPUT);
  pinMode(button_pin, INPUT);
  pinMode(door1Pin, INPUT);
  pinMode(door2Pin, INPUT);
  // Remove this line in prod, testing only
  //Serial.begin(9600);
  
  //Enable the pullup resistor on the button
  digitalWrite(button_pin, HIGH);
  
  //The button is a normally button
  last_reading = ! digitalRead(button_pin);
  initializePins();
  lock.attach(lockPin);
  lock.write(5);
  lock2.attach(lock2Pin);
  lock2.write(10);
  stop();
 
}

void lockTest() {
  lock.write(5);
  delay(2000);
  lock.write(90);
  delay(2000);

}

void lock2Test() {
  lock2.write(10);
  delay(2000);
  lock2.write(95);
  delay(2000);
}

void runLocksTest() {
  lock.write(5);
  lock2.write(10);
  delay(500);
  lock.write(90);
  lock2.write(95);
  delay(500);
}

boolean isDoorLocked() {
  return (digitalRead(door1Pin) == HIGH && digitalRead(door2Pin) == HIGH);
}

void lockDoors() {
  if (isDoorLocked()) {
    lock.write(90);
    lock2.write(95);
    delay(500);
    locked = true;
  }
}

void unlockDoors() {
    lock.write(5);
    lock2.write(10);
    delay(500);
    locked = false;
}

void loop()
{
//  if (!flag) {
//    runLocksTest();
//    flag = true;
//  }
    
//    if (digitalRead(button_pin) == HIGH) {
//      if (!locked) 
//        lockDoors();
//      else
//        unlockDoors();
//    }

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

