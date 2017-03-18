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

#define A 2
#define B 3

ros::NodeHandle nh;

bool A_last_val;
bool B_last_val;

char state;
int count;
unsigned long last_time;
unsigned long loop_time;
float distance;
float velocity;
unsigned long ros_rate;

std_msgs::Float64 vel;

ros::Publisher p("encoder_velocity", &vel);


void setup() {
  nh.initNode();
  nh.advertise(p);
  
  FastGPIO::Pin<A>::setInput();
  FastGPIO::Pin<B>::setInput();

  A_last_val = false;
  B_last_val = false;

  ros_rate = 100;

  state = 'F';
  count = 0;
  distance = 0;
  velocity = 0;
  last_time = millis();
  loop_time = millis();
}


void loop() {
  if(millis()-loop_time < ros_rate){
    bool A_val = FastGPIO::Pin<A>::isInputHigh();
    bool B_val = FastGPIO::Pin<B>::isInputHigh();
    
    if((A_last_val != A_val) || (B_last_val != B_val)){
      count++; 
      direction(A_val,B_val);
    }
  
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


void direction(bool A_val, bool B_val){
  if((A_last_val && !B_last_val) && (A_val && B_val)){
    state = 'F';
  }else if((!A_last_val && B_last_val) && (A_val && B_val)){
    state = 'B';
  }

  A_last_val = A_val;
  B_last_val = B_val;
}
