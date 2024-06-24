#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 11


// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);


// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;
 

////////////////// Motor Controller Variables and Constants ///////////////////
 
// Motor A connections
const int el1=6;
const int z1 = 12;
const int vr1 = 5;

//const int el1=9;
//const int z1 = 13;
//const int vr1 = 10;




// Motor B connections
//const int el2=6;
//const int z2 = 12;
//const int vr2 = 5;

const int el2=9;
const int z2 = 13;
const int vr2 = 10;


// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;

// Number of ticks per wheel revolution. We won't use this in this code.*****
const int TICKS_PER_REVOLUTION = 47;

// Wheel radius in meters*****
const double WHEEL_RADIUS = 0.081;

 
// Distance from center of the left tire to the center of the right tire in m *****
const double WHEEL_BASE = 0.279;

// Number of ticks a wheel makes moving a linear distance of 1 meter *****
// This value was measured manually.
const double TICKS_PER_METER = 95; // Originally 2880


// Proportional constant, which was measured by measuring the ****
// PWM-Linear Velocity relationship for the robot.
const int K_P = 45;

// Y-intercept for the PWM-Linear Velocity relationship for the robot****
const int b = 18;

// Correction multiplier for drift. Chosen through experimentation.****
const int DRIFT_MULTIPLIER = 20;

// Turning PWM output (0 = min, 255 = max for PWM values)
const int PWM_TURN = 15;

// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 15; // about 0.1 m/s
const int PWM_MAX = 25; // about 0.172 m/s

// Set linear velocity and PWM variable values for each wheel
double velLeftWheel = 0;
double velRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;

// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;



/////////////////////// Tick Data Publishing Functions ////////////////////////
 
// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
  
 
  if (val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
  
 
  if (val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}


/////////////////////// Motor Controller Functions ////////////////////////////
 
// Calculate the left wheel linear velocity in m/s every time a 
// tick count message is rpublished on the /left_ticks topic. 
void calc_vel_left_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 + left_wheel_tick_count.data - prevLeftCount) % 65535;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velLeftWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
 
  // Keep track of the previous tick count
  prevLeftCount = left_wheel_tick_count.data;
 
  // Update the timestamp
  prevTime = (millis()/1000);
 
}
 
// Calculate the right wheel linear velocity in m/s every time a 
// tick count message is published on the /right_ticks topic. 
void calc_vel_right_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevRightCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 + right_wheel_tick_count.data - prevRightCount) % 65535;
 
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velRightWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
 
  prevRightCount = right_wheel_tick_count.data;
   
  prevTime = (millis()/1000);
 
}

// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
   
  // Record timestamp of last velocity command received
  lastCmdVelReceived = (millis()/1000);
   
  // Calculate the PWM value given the desired velocity 

  pwmLeftReq = (cmdVel.linear.x+((cmdVel.angular.z*0.279)/2))*(255);
  pwmRightReq = (cmdVel.linear.x-((cmdVel.angular.z*0.279)/2))*(255);

  
  
  // Handle low PWM values
//  if (abs(pwmLeftReq) < PWM_MIN) {
//    pwmLeftReq = 0;
//  }
//  if (abs(pwmRightReq) < PWM_MIN) {
//    pwmRightReq = 0;  
//  }  
}


void set_pwm_values() {
 
  // These variables will hold our desired PWM values
  static int pwmLeftOut = 0;
  static int pwmRightOut = 0;
 
  // If the required PWM is of opposite sign as the output PWM, we want to
  // stop the car before switching direction
  static bool stopped = false;
  if ((pwmLeftReq * velLeftWheel < 0 && pwmLeftOut != 0) ||
      (pwmRightReq * velRightWheel < 0 && pwmRightOut != 0)) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }
 
  // Set the direction of the motors
  if (pwmLeftReq > 0) { // Left wheel forward
    digitalWrite(z1,LOW);
    digitalWrite(el1,HIGH);
//    Serial.println("frl");
//    Serial.print(pwmRightOut);
//    Serial.println();
    
    
  }
  else if (pwmLeftReq < 0) { // Left wheel reverse

    digitalWrite(z1,HIGH);
    digitalWrite(el1,HIGH);
//    Serial.println("rvl");
//    Serial.print(pwmRightOut);
//    Serial.println();
    
    
  }
  else if (pwmLeftReq == 0 && pwmLeftOut == 0 ) { // Left wheel stop
  
    digitalWrite(el1,LOW);
  }
  else { // Left wheel stop
   
    digitalWrite(el1,LOW); 
  }
 
  if (pwmRightReq > 0) { // Right wheel forward
     digitalWrite(z2,HIGH);
    digitalWrite(el2,HIGH);
//    Serial.println("frr");
//    Serial.print(pwmLeftOut);
//    Serial.println();
  }
  else if(pwmRightReq < 0) { // Right wheel reverse
    digitalWrite(z2,LOW);
    digitalWrite(el2,HIGH);
//    Serial.println("rvr");
//    Serial.print(pwmLeftOut);
//    Serial.println();
  }
  else if (pwmRightReq == 0 && pwmRightOut == 0) { // Right wheel stop
    
    digitalWrite(el2,LOW);
  }
  else { // Right wheel stop
    
    digitalWrite(el2,LOW); 
  }
 
  // Increase the required PWM if the robot is not moving
  if (pwmLeftReq != 0 && velLeftWheel == 0) {
    pwmLeftReq *= 1.5;
  }
  if (pwmRightReq != 0 && velRightWheel == 0) {
    pwmRightReq *= 1.5;
  }
 
  // Calculate the output PWM value by making slow changes to the current value
//  if (abs(pwmLeftReq) > pwmLeftOut) {
//    pwmLeftOut += PWM_INCREMENT;
//  }
//  else if (abs(pwmLeftReq) < pwmLeftOut) {
//    pwmLeftOut -= PWM_INCREMENT;
//  }
//  else{}
//   
//  if (abs(pwmRightReq) > pwmRightOut) {
//    pwmRightOut += PWM_INCREMENT;
//  }
//  else if(abs(pwmRightReq) < pwmRightOut) {
//    pwmRightOut -= PWM_INCREMENT;
//  }
//  else{}
 
  // Conditional operator to limit PWM output at the maximum 
  pwmLeftOut = (pwmLeftReq > PWM_MAX) ? PWM_MAX : pwmLeftReq;
  pwmRightOut = (pwmRightReq > PWM_MAX) ? PWM_MAX : pwmRightReq;
 
  // PWM output cannot be less than 0
  pwmLeftOut = (pwmLeftReq < PWM_MIN) ? PWM_MIN: pwmLeftReq;
  pwmRightOut = (pwmRightReq < PWM_MIN) ? PWM_MIN : pwmRightReq;
 
  // Set the PWM value on the pins
  analogWrite(vr1, pwmLeftOut); 
  analogWrite(vr2, pwmRightOut); 
}


// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );



void setup() {
 
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
   
  // Motor control pins are outputs
  pinMode(el1, OUTPUT);
  pinMode(el2, OUTPUT);
  pinMode(vr1, OUTPUT);
  pinMode(vr2, OUTPUT);
  pinMode(z1, OUTPUT);
  pinMode(z2, OUTPUT);
//  Serial.begin(9600);


// Turn off motors - Initial state
  digitalWrite(el1, LOW);
  digitalWrite(el2, LOW);


  // Set the motor speed
  analogWrite(vr1, 0); 
  analogWrite(vr2, 0);


 // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
}


void loop() {
   
  nh.spinOnce();
   
  // Record the time
  currentMillis = millis();
 
  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
 
    // Publish tick counts to topics
//    leftPub.publish( &left_wheel_tick_count );
//    rightPub.publish( &right_wheel_tick_count );
       leftPub.publish( &right_wheel_tick_count );
    rightPub.publish( &left_wheel_tick_count );
 
    // Calculate the velocity of the right and left wheels
    calc_vel_right_wheel();
    calc_vel_left_wheel();
     
  }
   
  // Stop the car if there are no cmd_vel messages
  if((millis()/1000) - lastCmdVelReceived > .5) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }
 
  set_pwm_values();
}
