/*
 * Author: Brad Bazemore
 * Licence: MIT
 */

#include <FastGPIO.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>

/*
 * Tire is 30.48cm radius
 * Circumference is 191.51cm
 * Encoder P/R is 1024
 * Distance is 0.18cm per pulse 
 * Distance per window 128P 23.9 cm
 * 
 * TODO Add dead band
 */


//////////////////////////////////////
//Config
//////////////////////////////////////
// Digital pins
#define A 5 // pin number of A pulse
#define B 6 // pin number of B pulse
#define ACTUATOR_PIN 3 // pin for the linear servo singnal
#define THROTTLE_PIN 4 // pin for throttle servo

// Analog pins
#define PIVOT_PIN 0 // pin for the potentiometer

// Limits
#define THROTTLE_MAX 150
#define THROTTLE_MIN 90
#define ACTUATOR_MAX 120
#define ACTUATOR_MIN 0


//////////////////////////////////////
//ROS
//////////////////////////////////////
// ros node object
ros::NodeHandle nh;

std_msgs::Float64 vel; // the ros message for out velocity
ros::Publisher encoder_pub("driver/encoder_velocity", &vel); // the publisher of the velocty

std_msgs::Float64 pivot; // ros meesage for the amount of pivot
ros::Publisher pivot_pub("driver/pivot", &pivot); // publisher of pivot

Servo actuator; // servo object for the linear actuator
void actuator_callback(const std_msgs::Float64 &cmd_msg); // method def used for actuator call back 
ros::Subscriber<std_msgs::Float64> actuator_sub("driver/linear_drive_actuator", actuator_callback);

Servo throttle; // servo object for the engine throttle
void throttle_callback(const std_msgs::UInt16 &cmd_msg); // method def used for actuator call back 
ros::Subscriber<std_msgs::UInt16> throttle_sub("driver/throttle", throttle_callback);


//////////////////////////////////////
//Global Variables
//////////////////////////////////////
// last value of the a and b pulses
bool A_last_val;
bool B_last_val;

char state;  // directional state of the encoder
int count;  // number of times a pulse has been seen
unsigned long last_time; // last time we saw 128 pulses
unsigned long loop_time; // time that the entire loop has ran
float distance; // distance we have gone
float velocity; // out velocity
unsigned long ros_rate; // the rate in milliseconds to refresh ros
float angle; // the angle of pivot


void setup() {
  // start ros, start publisher, and subscriber
  nh.initNode();
  nh.advertise(encoder_pub);
  nh.advertise(pivot_pub);
  nh.subscribe(actuator_sub);
  nh.subscribe(throttle_sub);

  actuator.attach(ACTUATOR_PIN); // set pin to be used for actuator
  throttle.attach(THROTTLE_PIN); // set the pin for the throttle servo

  // add pins A and B to the fast GPIO, we use this to not miss encoder pulses
  FastGPIO::Pin<A>::setInput();
  FastGPIO::Pin<B>::setInput();

  // init values for the A and B last pulses
  A_last_val = false;
  B_last_val = false;

  // set the rate for ros to update at 100ms
  ros_rate = 100;

  // more init values
  state = 'F';
  count = 0;
  distance = 0;
  velocity = 0;
  angle = 0;

  // init the loop timers
  last_time = millis();
  loop_time = millis();
}


/*
 * parms:
 * 
 * return: void
 * 
 * This is the main loop of the program. It will get the velocity, direction
 * and distance from the encoder. It will then update ros every ros_rate
 */
void loop() {
  if(millis()-loop_time < ros_rate){
    bool A_val = FastGPIO::Pin<A>::isInputHigh();
    bool B_val = FastGPIO::Pin<B>::isInputHigh();
    
    if((A_last_val != A_val) || (B_last_val != B_val)){
      count++; 
      direction(A_val,B_val);
    }

// TODO need to figure out how to clear velocity  
    if(count > 128){
      distance += 23.9;
      if(state == 'F'){
        velocity = (distance / (millis()-last_time))/100;
      }else{
        velocity = ((distance / (millis()-last_time))/100)*-1;
      }
      count = 0;
      last_time = millis();
    }
    //Update data
    vel.data=velocity;
    get_angle();
    pivot.data = angle;
    
  }else{
    loop_time = millis();
    encoder_pub.publish(&vel);
    pivot_pub.publish(&pivot);
    nh.spinOnce();
  }
}


/*
 * params:
 * bool A_val: this is the current value of the A pulse
 * bool B_val: this is the current value of the B pulse
 * 
 * return: void
 * 
 * This method finds the direction of the encoder. This is done by finding if the A
 * or B puls leads. If A leads then we are doing clock wise, if B leads then we are 
 * going counter clock wise.
 */
void direction(bool A_val, bool B_val){
  if((A_last_val && !B_last_val) && (A_val && B_val)){
    state = 'F';
  }else if((!A_last_val && B_last_val) && (A_val && B_val)){
    state = 'B';
  }

  A_last_val = A_val;
  B_last_val = B_val;
}


/*
 * param:
 * 
 * return: void
 * 
 * Read the pivot sensor. Conver to angle in degreese.
 * Thanks to Tim for find out this equation.
 */
void get_angle(){
  float temp = analogRead(PIVOT_PIN);
  angle = 30.275 * temp - 45.503;
}


/*
 * param:
 * std_msgs::Float64 &cmd_msgs: data from ROS
 * 
 * Move the actuator on the hydrolic pump
 */
void actuator_callback(const std_msgs::Float64 &cmd_msg){
  if(cmd_msg.data >= ACTUATOR_MAX){
    actuator.write(ACTUATOR_MAX);
  }else if(cmd_msg.data <= ACTUATOR_MIN){
    actuator.write(ACTUATOR_MIN);
  }else{
    actuator.write(cmd_msg.data);
  }
}


/*
 * param:
 * std_msgs::UInt16 &cmd_msgs: this is the message coming from ROS 
 * 
 * return: void
 * 
 * Change the throttle
 */
void throttle_callback(const std_msgs::UInt16 &cmd_msg){
  if(cmd_msg.data >= THROTTLE_MAX){
    throttle.write(THROTTLE_MAX);
  }else if(cmd_msg.data <= THROTTLE_MIN){
    throttle.write(THROTTLE_MIN);
  }else{
    throttle.write(cmd_msg.data);
  }
}


/*
 * param: 
 * 
 * return: void
 * 
 * TODO idk how this should be done as turning is booblean operation.
 * Maybe we should look at having this be set up to have a number come in but
 * it just turns on and off based on a threshold? IDk, just a thought.
 */
void articulation_callback(const std_msgs::){

}

