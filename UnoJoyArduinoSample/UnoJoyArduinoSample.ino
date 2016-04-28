#include <Wire.h>


#include <tilt.h>


#include "UnoJoy.h"

tilt tilt1;

unsigned long lastupdate = millis();
unsigned long lastupdateY = millis();
unsigned long timebet = 0;
unsigned long timebetlast = 0;
int pitchOff = 0;
bool active = true;
//float zero;

void setup(){
  setupPins();
  setupUnoJoy();
  tilt1.init_accel();
  tilt1.Init_hmc();
//  Serial.begin(9600);

}

void loop(){
  // Always be getting fresh data
  if((millis() - lastupdate) >= 10)
  {
    timebetlast = timebet;
    timebet = millis();    
    tilt1.update_global(timebet - timebetlast,0);
    tilt1.get_yaw(true);
    int output = 0;
    
//    Serial.print(map(tilt1.get_pitch(),-90,90,0,255));
//      Serial.print(tilt1.get_pitch());
//      Serial.print(" , ");
      

//    Serial.print(abs((tilt1.get_pitch() - pitchOff)));
//    Serial.print(" , ");
//    Serial.println(output);    
//    Serial.print(" , ");
//    Serial.print("out: ");
//    Serial.println(tilt1.get_yaws());
    //Serial.print(" , ");
//    Serial.println(tilt1.get_roll_gyro());
//    Serial.println((int)(((tilt1.get_yaws())*6) + 128));  
//    Serial.println(timebet - timebetlast);
    lastupdate = millis();
  }
  
  if((millis() - lastupdateY) >= 100)
  {
    //tilt1.get_yaw();
  } 
 
  if(!digitalRead(10))
 {
   tilt1.syncup();
   pitchOff = tilt1.get_pitch();
 } 
  if(!digitalRead(9))
 {
   //tilt1.syncoff();
   active = !active;
   while(!digitalRead);
   delay(100);
   
 } 
 if(!active)
 {
   dataForController_t controllerData = getControllerDataOff();
   setControllerData(controllerData);
 }
 else
 {
  dataForController_t controllerData = getControllerData();
  setControllerData(controllerData);
 }
  
}

void setupPins(void){
  // Set all the digital pins as inputs
  // with the pull-up enabled, except for the 
  // two serial line pins
  for (int i = 2; i <= 12; i++){
    pinMode(i, INPUT);
    digitalWrite(i, HIGH);
  }
  pinMode(A4, INPUT);
  digitalWrite(A4, HIGH);
  pinMode(A5, INPUT);
  digitalWrite(A5, HIGH);
}

dataForController_t getControllerData(void){
  
  // Set up a place for our controller data
  //  Use the getBlankDataForController() function, since
  //  just declaring a fresh dataForController_t tends
  //  to get you one filled with junk from other, random
  //  values that were in those memory locations before
  dataForController_t controllerData = getBlankDataForController();
  // Since our buttons are all held high and
  //  pulled low when pressed, we use the "!"
  //  operator to invert the readings from the pins
  controllerData.triangleOn = !digitalRead(2);
  controllerData.circleOn = !digitalRead(3);
  controllerData.squareOn = !digitalRead(4);
  controllerData.crossOn = !digitalRead(5);
  controllerData.dpadUpOn = !digitalRead(6);
  controllerData.dpadDownOn = !digitalRead(7);
  controllerData.dpadLeftOn = !digitalRead(8);
  controllerData.dpadRightOn = !digitalRead(9);
  controllerData.l1On = !digitalRead(10);
  controllerData.r1On = !digitalRead(11);
  controllerData.selectOn = !digitalRead(12);
  controllerData.startOn = !digitalRead(A4);
  controllerData.homeOn = !digitalRead(A5);
  
  // Set the analog sticks
  //  Since analogRead(pin) returns a 10 bit value,
  //  we need to perform a bit shift operation to
  //  lose the 2 least significant bits and get an
  //  8 bit number that we can use  
  //if(abs(tilt1.get_yaws()) < 3.0)
  //{
  //  controllerData.leftStickX = (unsigned int) constrain(    (((tilt1.get_yaws())* 1 +abs(tilt1.get_yaws())) + 128),0,254);
  //}
  //else
  //{
    controllerData.leftStickX = (unsigned int) constrain(    (((tilt1.get_yaws())*4) + 128),0,254);
  //}
//  if(abs((tilt1.get_pitch() - pitchOff)) < 2.0)
//  {
//    controllerData.leftStickY = (int)constrain((((tilt1.get_pitch() - pitchOff)* abs((tilt1.get_pitch()- pitchOff)*4)) +128.0),0,254);;//analogRead(A1) >> 2;
//  }
//  else
//  {
    controllerData.leftStickY = (int)constrain((((tilt1.get_pitch() - pitchOff)*8)+128.0),0,254);//analogRead(A1) >> 2;;
//  }
  controllerData.rightStickX = 128; //analogRead(A2) >> 2;
  controllerData.rightStickY = 128;//analogRead(A3) >> 2;
  // And return the data!
  return controllerData;
}

dataForController_t getControllerDataOff(void){
  
  // Set up a place for our controller data
  //  Use the getBlankDataForController() function, since
  //  just declaring a fresh dataForController_t tends
  //  to get you one filled with junk from other, random
  //  values that were in those memory locations before
  dataForController_t controllerData = getBlankDataForController();
  // Since our buttons are all held high and
  //  pulled low when pressed, we use the "!"
  //  operator to invert the readings from the pins
  controllerData.triangleOn = !digitalRead(2);
  controllerData.circleOn = !digitalRead(3);
  controllerData.squareOn = !digitalRead(4);
  controllerData.crossOn = !digitalRead(5);
  controllerData.dpadUpOn = !digitalRead(6);
  controllerData.dpadDownOn = !digitalRead(7);
  controllerData.dpadLeftOn = !digitalRead(8);
  controllerData.dpadRightOn = !digitalRead(9);
  controllerData.l1On = !digitalRead(10);
  controllerData.r1On = !digitalRead(11);
  controllerData.selectOn = !digitalRead(12);
  controllerData.startOn = !digitalRead(A4);
  controllerData.homeOn = !digitalRead(A5);
  
  // Set the analog sticks
  //  Since analogRead(pin) returns a 10 bit value,
  //  we need to perform a bit shift operation to
  //  lose the 2 least significant bits and get an
  //  8 bit number that we can use  
  controllerData.leftStickX = 127;
  controllerData.leftStickY = 127;//analogRead(A1) >> 2;
  controllerData.rightStickX = 128; //analogRead(A2) >> 2;
  controllerData.rightStickY = 128;//analogRead(A3) >> 2;
  // And return the data!
  return controllerData;
}
