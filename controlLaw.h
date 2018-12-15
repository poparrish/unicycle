#include "returnedVariables.h"
class controlLaw {
  public:
  int Hz;
  int pulsesPerRot;
  float DerRPMerror;
  float lastCycleSpeedCheck;
  float lastCycleRPM;
  float curRPM;
  float desiredRPM;
  int cutoff;
  int speedCheck;
  float speedCheckFloat;
  float lastSpeedCheckFloat;
  float Pgain;
  float Dgain;
  float RPMerror;
  float Perror, Derror;
  float kP, kD;
  float lastHubPerror;
  float lastDesiredRPM;
  bool forwardBackward;
  bool backToZero;
  
returnVariables calculate(float phi, int newDesiredRPM,bool lastInterruptTime,float avgDeltaTime);
controlLaw(); 
};

