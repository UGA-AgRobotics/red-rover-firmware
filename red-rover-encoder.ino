#include <FastGPIO.h>

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

bool A_last_val;
bool B_last_val;

char state;

int count;

float distance;

unsigned long last_time;

void setup() {
  Serial.begin(9600);
  
  FastGPIO::Pin<A>::setInput();
  FastGPIO::Pin<B>::setInput();

  A_last_val = false;
  B_last_val = false;

  state = 'F';

  count = 0;

  distance = 0;

  last_time = millis();
}

void loop() {
  bool A_val = FastGPIO::Pin<A>::isInputHigh();
  bool B_val = FastGPIO::Pin<B>::isInputHigh();
  
  if((A_last_val != A_val) || (B_last_val != B_val)){
    count++; 
    direction(A_val,B_val);
  }

  if(count > 128){
    distance += 23.9;
    //Serial.print(state);
    //Serial.print(" ");    
    //Serial.print(distance);
    //Serial.print(" ");
    Serial.println((distance / (millis()-last_time))/100);
    count = 0;
    last_time=millis();
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
