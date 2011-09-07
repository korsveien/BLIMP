#include <Servo.h>

Servo elevator, throttle, tail;

double dState; //Last position input 
double iState; //Integrator state

double iMax, iMin; //Maximum and minimum allowable integrator state

double iGain, //integral gain
       pGain, //proportional gain
       dGain; //derivative gain
       
double pTerm, dTerm, iTerm;

double error, targetPos, currentPos;

double acceleration, defaultAcceleration;

static int altitudePin = 7;
static int elevatorPin = 9;
static int throttlePin = 10;
static int tailPin = 11;

void setup()
{
Serial.begin(9600);


defaultAcceleration = 102;

elevator.attach(elevatorPin);
throttle.attach(throttlePin);
tail.attach(tailPin);

targetPos = 50;
iMax = 100000;
iMin = 0;

iGain = 0.01;
pGain = 70;
//dGain = 7000; 

}

void loop(){ 
  Serial.begin(9600);

  currentPos = analogRead(altitudePin);
  acceleration = updatePID();
  
  Serial.print("currentPos (sensor reading) is: ");
  Serial.println(currentPos);
  
  Serial.print("acceleration before mapping is: ");
  Serial.println(acceleration);
  
  acceleration = map(acceleration, 0, 1023, 100, 180);
  
  Serial.print("acceleration after mapping is: ");
  Serial.println(acceleration);
  
  /*if(acceleration < 0){*/
  /*  acceleration = -acceleration;*/
  /*  acceleration = constrain(acceleration, 100, 180);*/
  /*  accelerateDown(acceleration);*/
  /*}*/
  /*else{*/
  /*  acceleration = constrain(acceleration, 100, 180);*/
  /*  accelerateUp(acceleration);*/
  /*}*/
  
  delay(100); //sampling rate of 10Hz
  
}

double updatePID()
{
  error = targetPos - currentPos;
  
  pTerm = pGain * error; // calculate the proportional term
  
  //calculate integral state with appropriate limiting
  iState += error;
  
  if(iState > iMax){
    iState = iMax;
  }
  else if(iState < iMin){
    iState = iMin;
  }
  
  dTerm = dGain * (dState - currentPos);
  dState = currentPos;
  
  return pTerm + dTerm + iTerm;
}

//deadspot ca. mellom 84-102
void accelerateUp(double acceleration){
  
  Serial.print("--- Accelerating up with acceleration: ");
  Serial.println(acceleration);
  
  elevator.write(30);
  throttle.write(acceleration);
  tail.write(90);
}

void accelerateDown(double acceleration){
  
   Serial.print("--- Accelerating down with acceleration: ");
   Serial.println(acceleration);
   
  elevator.write(122);
  throttle.write(acceleration);
  tail.write(90);
}

void defaultGlide(double acceleration){
  
  Serial.print("--- DefaultGlide with default acceleration: ");
  Serial.println(acceleration);
  
  elevator.write(77);
  throttle.write(acceleration);
  tail.write(90);
}

void testFlight(){
  accelerateUp(102);
  delay(3000);
  accelerateDown(102);
  delay(3000);
  defaultGlide(102);
  delay(3000);
}
