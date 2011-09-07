
#include <PID_v1.h>
#include <Servo.h>

Servo elevator, throttle, tail;

double acceleration, defaultAcceleration,thrust;
double targetPos, currentPos;

static int altitudePin = 7;
static int elevatorPin = 9;
static int throttlePin = 10;
static int tailPin = 11;

PID pid(&currentPos, &acceleration, &targetPos,0.3,0,0,DIRECT);

void setup()
{
Serial.begin(9600);

pid.SetMode(AUTOMATIC);
pid.SetOutputLimits(-10, 10); //acceleration




defaultAcceleration = 102;
targetPos = 40
;

elevator.attach(elevatorPin);
throttle.attach(throttlePin);
tail.attach(tailPin);



currentPos = analogRead(altitudePin);
}

void loop(){ 
  Serial.begin(9600);
  

  currentPos = analogRead(altitudePin);
  pid.Compute();
  
  Serial.print("--- Sensor reading is: ");
  Serial.println(currentPos);
  Serial.print(acceleration);
  
  
  if(acceleration < 0){
    
    thrust = (-acceleration) + 100;
  }
  else{
    
    thrust = acceleration + 100;
  }
  
  //if we are below target go up, if not go down
  if(currentPos < targetPos - 2){
    accelerateUp(thrust);
  }
  else if(currentPos > targetPos +2) {
    accelerateDown(thrust);
  }
  
  else { defaultGlide(102);
  }
  
  





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
  
  accelerateUp(115);
  delay(3000);
  accelerateDown(115);
  delay(3000);
  defaultGlide(115);
  delay(3000);
}
