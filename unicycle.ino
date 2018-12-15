#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "controlLaw.h"

//pins
int vrPin = 5;
int tachSignal = 3;//must use either pins 2 or 3 for interrupts if on arduino Uno
int zfPin = 2;
//Gyro connections for Adafruit driver
//   ===========
//   Connect SCL to analog 5
//   Connect SDA to analog 4
//   Connect VDD to 3.3V DC
//   Connect GROUND to common ground

int Hz = 30;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)
Adafruit_BNO055 bno = Adafruit_BNO055();

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

/*
 * This method is called when the interrupt for wheel 1's hub (hall sensor) goes high. It is calculating the average time 
 * between interrupts to get an RPM. However, it needs to handle the case when the wheel is stopped and starts up. Hence 
 * the first interrupt after it has been stopped can't be used to calculate a time bewtween interrups (we need two), so  
 * teh first time we will calculate a time between intrrupts and get an RPM is during the second interrupt.
 */
void hubEncoder(){
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

//pointer to wheel object
controlLaw* wheel; 

//arduino setup, runs ones
void setup() {
  //setup the pins&set baudrate
  Serial.begin(115200);
  pinMode(vrPin, OUTPUT);
  pinMode(zfPin, OUTPUT);
  pinMode(tachSignal,INPUT);
  attachInterrupt(digitalPinToInterrupt(tachSignal),hubEncoder,RISING);
  interrupts();  

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);
  
  //instantiate new wheel object
  wheel = new controlLaw();  
}

//main loop inits
float AverageDeltaTime = 10000001;
unsigned long lastInterruptTime;

//arduino main loop
void loop() {
  
  if (millis() - beginningTic > 1000 / Hz){
    beginningTic = millis();//reset the last time loop executed
    
   
  
    //shut ISR off to copy data avoiding read/write corruption
    noInterrupts();
    float avgDeltaTimeCp = avgDeltaTime;
    lastIntTimeCp = lastIntTime;
    interrupts();

    //get euler angle
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float phi = euler.x();

    //calculate new desiredRPM
    float newDesiredRPM;
    
    //TODO calculate torque

    //call the method for wheel calculations
    struct returnVariables returnVariablesWheel = wheel->calculate(phi,newDesiredRPM,lastInterruptTime,avgDeltaTime);
    
    

    
    //write speed to wheel    
//    Serial.print("distanceTraveled: ");
//    Serial.println(distanceTraveled);
//    Serial.print("curRPM: ");
//    Serial.println(curRPM);
//    Serial.print("desiredRPM: ");
//    Serial.println(desiredRPM);
//    analogWrite(vrPin,speedCheck);
//    digitalWrite(zfPin,HIGH);

  
  }


  
  
  
}
