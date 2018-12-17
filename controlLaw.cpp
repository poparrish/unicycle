#include <Arduino.h>
#include "controlLaw.h"


controlLaw::controlLaw(){
  Hz = 15;
  pulsesPerRot = 45;
  DerRPMerror = 0;
  lastCycleSpeedCheck = 0;
  lastCycleRPM = 0;
  cutoff = 10;
  speedCheck = 0;
  speedCheckFloat = 0.0;
  Pgain = 0.0;
  Dgain = 0.0;
  RPMerror = 0;
  lastHubPerror = 0;
  lastDesiredRPM = 0;
  backToZero = false;
  lastCycleRPM = 0;
}

returnVariables controlLaw::calculate(float phi,int newDesiredRPM,bool lastInterruptTime,float avgDeltaTime) {
  using namespace std;

  /*
 * The next section looks to see if the desired RPM is in the same direction or not. IF it is not, it will set the 
 *  desired RPM to 0 this pass and then next pass if will switch the forward and back variable and the desired RPM.
 *  By setting the desried RPM to 0, we also we restart the interrupt logic and go open loop on the initial speedcheck
 *  value (set later on). 
 */

  if (newDesiredRPM < 0)
  {
    if (lastDesiredRPM > 0) {
      // current positive new negative
      desiredRPM = 0;
    }
    else {
      // current negative new negative
      forwardBackward = LOW;
      desiredRPM =newDesiredRPM*-1;
    }
  }
  else {
    if (lastDesiredRPM < 0) {
      // current negative new positive
      desiredRPM = 0; 
    }
    else {
      //current positive new positive
      desiredRPM = newDesiredRPM;
      forwardBackward = HIGH;
    }
  }

  lastDesiredRPM = newDesiredRPM;

/* 
*  This if statement is looking for the case that the wheel has stopped and we are not getting any more interrupts, 
*  and averagedeltatime is not valid (old). So, in this case we will set the current RPM to zero. If not, then we will
*  calcultate the RPM - the 60,000 is to convert from millisec to minutes.
*/
  if (( millis() - lastInterruptTime) > 100) {
    curRPM = 0;
  }
  else {
    curRPM = (1 / ((float)avgDeltaTime * pulsesPerRot)) * 60000;
  }
  // If the wheel has stopped, we need to do a new start up sequence.
  if (curRPM <= cutoff) {
    backToZero = true;
  }else{
    backToZero = false;
  }

  // Calculate the delta (error) in RPM and in the derivative of RPM.
  RPMerror = desiredRPM - curRPM;
  DerRPMerror = curRPM - lastCycleRPM;

//  speedCheck = phi;
//    HUB PD START//
  if(desiredRPM != 0){
    speedCheck = desiredRPM+cutoff;
  }else{
    speedCheck = 0;
  }
  //keep within 255char size
  if(speedCheck > 255){
    speedCheck =255;
  }
 
   
// return the desired parameters  
  struct returnVariables returnStruct;
  returnStruct.speedCheck = speedCheck;
  returnStruct.forwardBackward = forwardBackward;
  returnStruct.currentWheelRPM = curRPM;;
  returnStruct.backToZero = backToZero;
  return returnStruct;   
}
