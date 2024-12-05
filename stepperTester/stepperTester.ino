/*
 * Simple demo, should work with any driver board
 *
 * Connect STEP, DIR as indicated
 *
 * Copyright (C)2015-2017 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <Arduino.h>
#include "BasicStepperDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 100

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// All the wires needed for full functionality
const int stepS = 33, dirS = 32;
const int stepE = 4, dirE = 2;
const int stepK = 14, dirK = 27;

// Stepper motor instances
BasicStepperDriver dampStepper(MOTOR_STEPS, dirS, stepS);
BasicStepperDriver elevatorStepper(MOTOR_STEPS, dirE, stepE);
BasicStepperDriver sliderStepper(MOTOR_STEPS, dirK, stepK);
//Uncomment line to use enable/disable functionality
//#define SLEEP 13

// 2-wire basic config, microstepping is hardwired on the driver

//Uncomment line to use enable/disable functionality
//BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, SLEEP);
bool magnet = 0 ;
int d1 = 0 ,d2 = 0 ,d3 = 0 ;
void setup() {
    sliderStepper.begin(180, MICROSTEPS);
    elevatorStepper.begin(60, MICROSTEPS);
    dampStepper.begin(120, MICROSTEPS);
    Serial.begin(115200);
    pinMode(15,OUTPUT);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    // stepper.setEnableActiveState(LOW);
}
void loop() {
if (Serial.available() > 0 ){
  char inputData = Serial.read() ;
  if (inputData == '0'){
    d1 = 0 ;
    d2 = 0 ;
    d3 = 0 ;
  }
   else if (inputData == '1') {
    d1 = 1 ;
    d2 = 0 ;
    d3 = 0 ;
    // elevatorStepper.stop();
    // dumpStepper.stop( );
  }

  else if (inputData == '2') {
    d1 = -1 ;
    d2 = 0 ;
    d3 = 0 ;
    // elevatorStepper.stop();
    // dumpStepper.stop( );
  }

  else if (inputData == '3') {
    d1 = 0 ;
    d2 = 1 ;
    d3 = 0 ;
    // sliderStepper.stop();
    // dumpStepper.stop( );
  }
  else if (inputData == '4') {
    d1 = 0 ;
    d2 = -1 ;
    d3 = 0 ;
  }
  else if (inputData == '5') {
    d1 = 0 ;
    d2 = 0 ;
    d3 = 1 ;
  }
  else if (inputData == '6') {
    d1 = 0 ;
    d2 = 0 ;
    d3 = -1 ;
  }
  else if (inputData == '7') {
    d1 = 0 ;
    d2 = 0 ;
    d3 = 0 ;
    magnet =  1 ;
  }

  else if (inputData == '8') {
    d1 = 0 ;
    d2 = 0 ;
    d3 = 0 ; 
    magnet = 0 ;
  }
}

    dampStepper.move(d3*MOTOR_STEPS*MICROSTEPS);
    elevatorStepper.move(d2*MOTOR_STEPS*MICROSTEPS);
    sliderStepper.move(d1*MOTOR_STEPS*MICROSTEPS);

    digitalWrite(15 , magnet) ;

    // pause and allow the motor to be moved by hand
    // stepper.disable();

    //delay(5000);
}
