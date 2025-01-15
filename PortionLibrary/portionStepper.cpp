#include <Stepper.h>
#include <arduino.h>
#include "portionStepper.h"

portionStepper::portionStepper(int A, int B, int C, int D){
  this->outA = A;
  this->outB = B;
  this->outC = C;
  this->outD = D;
  myStepper = Stepper(200, outA, outB, outC, outD);
}

void portionStepper::init(){
  Stepper myStepper(stepsPerRevolution, outA, outB, outC, outD);
  
}

