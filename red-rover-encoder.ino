#include <FastGPIO.h>
#include <ros.h>
#include <std_msgs/Float64.h>
/*
 * Tire is 30.48cm radius
 * Circumference is 191.51cm
 * Encoder P/R is 1024
 * Distance is 0.18cm per pulse 
 * Distance per window 128P 23.9 cm
 * 
 * Add dead band
 */

#define A 2 // pin number of A pulse
#define B 3 // pin number of B pulse

// ros node object
ros::NodeHandle nh;

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

std_msgs::Float64 vel; // the ros message for out velocity

ros::Publisher p("encoder_velocity", &vel); // the publisher of the velocty


void setup() {
  // start ros and start publisher
  nh.initNode();
  nh.advertise(p);

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

// need to figure out how to clear velocity  
    if(count > 128){
      distance += 23.9;
      velocity = (distance / (millis()-last_time))/100;
      count = 0;
      last_time=millis();
    }
  
    vel.data=velocity;
  }else{
    loop_time = millis();
    p.publish(&vel);
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
