//#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>

/*INSTRUCTIONS
This isn't finished yet but can calculate the RPM of the wheel for angular rotation and implements the basic functionality
of the motorcontroller. No PID control on desired RPM yet.(may be able to get away with not using one since voltage/rpm scaling
appears linear. To write a speed just change desiredRPM and upload. It will be about 1.5times faster than the value entered at 
25V. The current RPM is printed to the serial monitor.*/

//pins
int vrPin = 5;
int tachSignal = 3;//must use either pins 2 or 3 for interrupts if on arduino Uno
int zfPin = 2;
int Hz = 5;

//inits for speed+torque calcs
int pulsesPerRot = 45;
float DerRPMerror = 0;
float lastCycleSpeedCheck = 0;
float lastCycleRPM = 0;
float curRPM = 0;
float desiredRPM = 0;
int cutoff = 5;
int speedCheck = 0;
String inputString = "";
float totalTics = 0;
float distanceTraveled = 0;

//inits for hub interrupt routine
unsigned long beginningTic = millis();
int countWheel = 0;
float sumWheelTime = 0;
unsigned long currentTime = 0;
unsigned long lastIntTime = millis();
boolean firstPass = true;
boolean backToZero = false;
float avgDeltaTime = 10000001;
int iWheel;
float deltaTime[40];
unsigned long lastIntTimeCp;

//gyro
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void hubEncoder(){
  // Need first pass logic to set the last interrupt time to get an accurate time between interrupts on the second pass
  if(firstPass or backToZero){
    lastIntTime = millis();
    avgDeltaTime = 10000001;
    firstPass = false;
    backToZero = false;
  }else{
    currentTime=millis();
    deltaTime[countWheel] = (float)(currentTime-lastIntTime);
    lastIntTime=currentTime;

    for(iWheel = 0; iWheel <= countWheel; iWheel++){
      sumWheelTime = sumWheelTime+deltaTime[iWheel];
    }
    avgDeltaTime=sumWheelTime/(float)(countWheel+1);
    sumWheelTime=0;
    countWheel++;
  }
}
void setup() {
  //setup the pins
  Serial.begin(9600);
  pinMode(vrPin, OUTPUT);
  pinMode(zfPin, OUTPUT);
  pinMode(tachSignal,INPUT);
  attachInterrupt(digitalPinToInterrupt(tachSignal),hubEncoder,RISING);

  //setup the gyro
  bno.setExtCrystalUse(true);
  if(!bno.begin()){
    Serial.print("gyro fukd");
  }
  
  
  interrupts();  

  
}


void loop() {
  desiredRPM = 30;
  if (millis() - beginningTic > 1000 / Hz){
    beginningTic = millis();//reset the last time loop executed
    
    //TODO gyro stuff
    sensors_event_t event;
    bno.getEvent(&event);
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
    Serial.println("");
  
    //shut ISR off to copy data avoiding read/write corruption
    noInterrupts();
    float avgDeltaTimeCp = avgDeltaTime;
    lastIntTimeCp = lastIntTime;
    interrupts();
    
    //TODO calculate torque
    
    //update speed. need desiredRPM, avgDeltaTime, lastIntTime
    if ((millis() - lastIntTimeCp) > 100){
      curRPM = 0;
    }else{
      curRPM = (1/((float)avgDeltaTimeCp*pulsesPerRot))*60000;
    }
    //If wheel stopped
    if(curRPM <=cutoff){
      backToZero = true;
    }else{
      backToZero = false;
    }

    //write new speed
    if(desiredRPM !=0){
      speedCheck = desiredRPM;
    }else{
      speedCheck = 0;
    }
    countWheel = 0;
    float circumference = 0.78539848;
    //write speed to wheel    
    Serial.print("distanceTraveled: ");
    Serial.println(distanceTraveled);
    Serial.print("curRPM: ");
    Serial.println(curRPM);
    Serial.print("desiredRPM: ");
    Serial.println(desiredRPM);
    analogWrite(vrPin,speedCheck);
    digitalWrite(zfPin,HIGH);

  
  }


  
  
  
}
