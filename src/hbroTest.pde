
#include <PID_v1.h>
#include <Servo.h>

Servo elevator;

double acceleration, defaultAcceleration,thrust;
double targetPos, currentPos;

static int elevatorPin = 4;
static int motor1Pin = 9;    // H-bridge leg 1 (pin 2, 1A)
static int motor2Pin = 11;    // H-bridge leg 2 (pin 7, 2A)
static int enablePin = 10;    // H-bridge enable pin
static int ledPin = 13;      // LED 
static int tailPin1 = 7;     // tailMotor leg 1
static int tailPin2 = 6;    // talMotor leg 2
static int enablePin2 = 5;  //enables side 2 of the H-bridge


static int altitudePin = A1

; // HIGHT - sensor input




PID pid(&currentPos, &acceleration, &targetPos,9,0,0,DIRECT);

void setup()


{
Serial.begin(9600);


pid.SetMode(AUTOMATIC);
pid.SetOutputLimits(-255, 255
); //acceleration

pinMode(motor1Pin, OUTPUT); 
pinMode(motor2Pin, OUTPUT); 
pinMode(enablePin, OUTPUT);
pinMode(enablePin2, OUTPUT);
pinMode(tailPin1, OUTPUT);
pinMode(tailPin2, OUTPUT);
pinMode(ledPin, OUTPUT);
digitalWrite(enablePin, HIGH);  //fire up H-bridge
digitalWrite(enablePin2, HIGH); // fire up H-bridge




defaultAcceleration = 102;
targetPos 
= 40;

elevator.attach(elevatorPin);


}

void loop(){ 
  
 

  currentPos = analogRead(altitudePin);
  pid.Compute();
  
  Serial.print("--- Sensor reading is: ");
  Serial.println(currentPos);
  delay(10);
  Serial.print(acceleration);
  
  
  if(acceleration < 0){
    
    thrust = (-acceleration);
  }
  else{
    
    thrust = acceleration;
    
  }
  
  //if we are below target go up, if not go down
  if(currentPos < targetPos - 2){
    accelerateUp(thrust);
  }
  else if(currentPos > targetPos +2) {
    accelerateDown(thrust);
  }
  
  else { defaultGlide(130);
  }
  
  
  

}

//deadspot ca. mellom 84-102
void accelerateUp(double acceleration){
  
  Serial.print("--- Accel
  erating up with acceleration: ");
  Serial.println(acceleration);
  
  
  elevator.write(30);
  analogWrite(motor2Pin, acceleration);
  digitalWrite(motor1Pin, LOW);
  
}

void accelerateDown(double acceleration){
  
   Serial.print("--- Accelerating down with acceleration: ");
   Serial.println(acceleration);
   
  elevator.write(30);
  analogWrite(motor1Pin, acceleration);
  digitalWrite(motor2Pin, LOW);
  
  
  
}

void defaultGlide(double acceleration){
  
  Serial.print("--- DefaultGlide with default acceleration: ");
  Serial.println(acceleration);
  
  elevator.write(77
  );
  analogWrite(motor2Pin, acceleration);
  digitalWrite(motor1Pin,LOW);
  delay(200);


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
