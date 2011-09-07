/*******************************************************************************
* Authors: Fredrik Cappelen and Nils Peder Korsveien                           *
*                                                                              *
* SOURCE CODE AND DATA SHEETS                                                  *
* H-bridge data sheet : www.sparkfun.com/datasheets/IC/SN754410.pdf            *
* H-bridge how-to     : http://itp.nyu.edu/physcomp/Labs/DCMotorControl#toc8   *
* PID-library         : http://arduino.cc/playground/Code/PIDLibrary           *
********************************************************************************/

#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>

#define SMOOTHED 1
#define DEBUG 1
#define TESTDOUBLES 0
#define TESTTAIL 0
#define TESTRIGHTTAIL 0
#define TESTDOWN 0

Servo elevator;
double acceleration, defaultAcceleration,thrust;
double tailAcceleration, tailDefaulAcceleration, tailThrust;
double targetAltitude, voltage, leftRange, rightRange, forwardRange, altitudeRange;

static int elevatorPin = 4;  // "staget" styres over denne
static int motor1Pin   = 9;  // H-bridge leg 1 (pin 2, 1A)
static int motor2Pin   = 11; // H-bridge leg 2 (pin 7, 2A)
static int enablePin   = 10; // H-bridge enable pin
static int ledPin      = 13; // LED
static int tailPin1    = 7;  // tailMotor leg 1
static int tailPin2    = 6;  // talMotor leg 2
static int enablePin2  = 5;  // enables side 2 of the H-bridge

// LED'S
//int red = 2; //this sets the red led pin
//int green = 8; //this sets the green led pin
//int blue = 3; //this sets the blue led pin

//range sensors
static int altitudePin     = A0;
static int rightRangePin   = A1;
static int leftRangePin    = A2;
static int forwardRangePin = A3;

//compass setup
int HMC6352SlaveAddress = 0x42; 

//compass I2C variables
int HMC6352ReadAddress = 0x41; //"A" in hex, A command is: 
int headingValue; //kompass kurs variabel

//smoothing filter variables
int sensVal;        // for raw sensor values
float filterVal;    // this determines smoothness  - .0001 is max  1 is off (no smoothing)
float smoothedVal;  // this holds the last loop value just use a unique variable for every different sensor that needs smoothing
float smoothedVal2; // this would be the buffer value for another sensor if you needed to smooth two different sensors - not used in this sketch

float heading;
double d_heading, course;

// PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
// Input    : Variable we are trying to control(double)
// Output   : The variable that will be adjusted by the PID(double)
// Setpoint : The value we want to Input to maintain(double)
PID altitudePID(&altitudeRange, &acceleration, &targetAltitude,4,0,0,DIRECT); 
PID tailPID(&d_heading, &tailAcceleration, &course,4,0,0,DIRECT); 

void setup() {
    Serial.begin(9600); //skjermutskrift på

    altitudePID.SetMode(AUTOMATIC);
    altitudePID.SetOutputLimits(-255, 255); 
    altitudePID.SetSampleTime(5);

    //enabling all motorpins
    pinMode(motor1Pin, OUTPUT); 
    pinMode(motor2Pin, OUTPUT); 
    pinMode(enablePin, OUTPUT);
    pinMode(enablePin2, OUTPUT);
    pinMode(tailPin1, OUTPUT);
    pinMode(tailPin2, OUTPUT);
    pinMode(ledPin, OUTPUT);

    digitalWrite(enablePin, HIGH);  //fire up H-bridge
    digitalWrite(enablePin2, HIGH); // fire up H-bridge

    elevator.attach(elevatorPin); // binder servo-bibl til staget

    //kompass start:
    HMC6352SlaveAddress = HMC6352SlaveAddress >> 1; // I know 0x42 is less than 127, but this is still required
    Wire.begin();

    targetAltitude = 50;
    filterVal = 0.9;
}

void printHeading(){
    Serial.print("--- HEADING: ");
    Serial.println(heading);
}

void printAltitude(){
    Serial.print("--- ALTITUDE: ");
    Serial.println(altitudeRange);
}

void printAcceleration(){
    Serial.print("--- ACCELERATION: ");
    Serial.println(acceleration);
}

void readAllSensors(){
    altitudeRange = analogRead(leftRangePin);
    leftRange     = analogRead(leftRangePin);
    rightRange    = analogRead(rightRangePin);
    forwardRange  = analogRead(forwardRangePin);
}

void smoothInput(){
    smooth(altitudeRange, filterVal, altitudeRange);
    smooth(leftRange, filterVal, leftRange);
    smooth(rightRange, filterVal, rightRange);
    smooth(forwardRange, filterVal, forwardRange);
}

void accelerate(){
    if(acceleration < 0){

        thrust = (-acceleration);
    }
    else{

        thrust = acceleration;
    }

    //if we are below target go up, if not go down
    if(acceleration > 65){
        accelerateUp(thrust);
    }
    else if(acceleration < - 65) {
        accelerateDown(thrust);
    }
    else { 
        defaultGlide(120);
    }
}


void loop(){ 
    if(TESTDOUBLES == 1){
        testDoubleEngines();
    }
    else if(TESTTAIL == 1){
        testTailEngine();
    }
    else if(TESTRIGHTTAIL == 1){
        testRightTail();
    }
    else if(TESTDOWN ==1 ){
        testDownAcceleration();
    }
    else{
        heading = getHeading();
        readAllSensors();

        if(SMOOTHED == 1){
            smoothInput();
        }

        altitudePID.Compute();

        //delay(10); //bremser koden/utskrift? Mister respons men mer stabil oppførel? Jeg syntes vi bør glatte verdiene vi får inn f.eks fjerne verdier 
        if(DEBUG == 1){
            printHeading();
            printAltitude();
            printAcceleration();
        }
        accelerate();
        //batteryMonitor();
    }
}
//deadspot ca. mellom 84-102

void turnLeft(double acceleration) {
  if(DEBUG == 1){
      Serial.print("--- Turning left with acceleration: ");
      Serial.println(acceleration);
  }
  digitalWrite(tailPin1, LOW);
  digitalWrite(tailPin2, HIGH);
  analogWrite(enablePin2, acceleration);
}

void turnRight(double acceleration){
  if(DEBUG == 1){
      Serial.print("--- Turning right with acceleration: ");
      Serial.println(acceleration);
  }
  digitalWrite(tailPin1, HIGH);
  digitalWrite(tailPin2, LOW);
  analogWrite(enablePin2, acceleration);
}


void accelerateUp(double acceleration){
  if(DEBUG == 1){
      Serial.print("--- Accelerating up with acceleration: ");
      Serial.println(acceleration);
  }
  
  analogWrite(motor2Pin, 0);            // slår av motorer, mens servo kjører pga strøm/forstyrrelser
  digitalWrite(motor1Pin, LOW);
  elevator.write(30);                   // snur stag i riktig posisjon
  delay(5);                             // venter litt på stag
  analogWrite(motor2Pin, acceleration); // starter motorer med PID akselerasjon
  digitalWrite(motor1Pin, LOW);
  
}

void accelerateDown(double acceleration){
  
  if(DEBUG == 1){
      Serial.print("--- Accelerating down with acceleration: ");
      Serial.println(acceleration);
  }
  analogWrite(motor2Pin, 0);            // slår av motorer, mens servo kjører pga strøm/forstyrrelser
  digitalWrite(motor1Pin, LOW);
  elevator.write(122);                   // snur stag i riktig posisjon
  delay(5);                             // venter litt på stag
  analogWrite(motor2Pin, acceleration); // starter motorer med PID akselerasjon
  digitalWrite(motor1Pin, LOW);
}

void defaultGlide(double acceleration){
  if(DEBUG == 1){
      Serial.print("--- DefaultGlide with default acceleration: ");
      Serial.println(acceleration);
  }
  
  analogWrite(motor2Pin, 0);            // slår av motorer, mens servo kjører pga strøm/forstyrrelser
  digitalWrite(motor1Pin, LOW);
  elevator.write(77);                   // snur stag
  delay(5);                             // venter på stag
  analogWrite(motor2Pin, acceleration); // kjører motor med PID akselerasjon
  digitalWrite(motor1Pin,LOW);
  //delay(200); //legger inn et delay for å vente å se om fremover aksen stabiliserer høyden?
}

void land(double acceleration) {
   
  if(DEBUG == 1){
      Serial.print("--- LANDING: ");
      Serial.println(acceleration);
  }
  analogWrite(motor2Pin, 0);            // slår av motorer, mens servo kjører pga strøm/forstyrrelser
  digitalWrite(motor1Pin, LOW);
  elevator.write(122);                  // snur stag
  delay(5);                             // venter på stag
  analogWrite(motor2Pin, acceleration); // starter motorer med PID akselerasjon
  digitalWrite(motor1Pin, LOW);
}

/*
int batteryMonitor () {
  
    int reading = analogRead(3); //tilpass til riktig port
    
    voltage = reading * 5,0;
    ; //regner ut spenning
    
    if (voltage > 3990 ) {
      batteryLed(green, 150);
      
    } else if (voltage > 3500
    ) {
       batteryLed(blue, 150);
       
    } else {
        batteryLed(red, 200);
        // void landing();
    }        
    return voltage; //returnerer spenning til batteriet

}

*/
void batteryLed(int colour, int strength) {
    analogWrite(colour, 200);
}

float getHeading() {  
  //"Get Data. Compensate and Calculate New Heading"
  Wire.beginTransmission(HMC6352SlaveAddress);
  Wire.send(HMC6352ReadAddress);              // The "Get Data" command
  Wire.endTransmission();

  //time delays required by HMC6352 upon receipt of the command
  //Get Data. Compensate and Calculate New Heading : 6ms
  delay(6);

  Wire.requestFrom(HMC6352SlaveAddress, 2); //get the two data bytes, MSB and LSB

  //"The heading output data will be the value in tenths of degrees
  //from zero to 3599 and provided in binary format over the two bytes."
  byte MSB = Wire.receive();
  byte LSB = Wire.receive();

  float headingSum = (MSB << 8) + LSB; //(MSB / LSB sum)
  float headingInt = headingSum / 10; 
  
  
  return(headingInt);
  
  ///Serial.print(headingInt);
  //Serial.println(" degrees");
}


int smooth(int data, float filterVal, float smoothedVal){

  if (filterVal > 1){ // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);

  return (int)smoothedVal;
}

void testDoubleEngines(){
  accelerateUp(102);
  delay(3000);
  accelerateDown(102);
  delay(3000);
  defaultGlide(102);
  delay(3000);
  
  accelerateUp(180);
  delay(3000);
  accelerateDown(180);
  delay(3000);
  defaultGlide(180);
  delay(3000);

  accelerateUp(255);
  delay(3000);
  accelerateDown(255);
  delay(3000);
  defaultGlide(255);
  delay(3000);

}

void testTailEngine(){
  turnRight(102);
  delay(3000);
  turnLeft(102);
  delay(3000);

  turnRight(180);
  delay(3000);
  turnLeft(180);
  delay(3000);

  turnRight(255);
  delay(3000);
  turnLeft(255);
  delay(3000);
}

void testRightTail(){
  turnRight(0);
  delay(3000);
  turnRight(20);
  delay(3000);
  turnRight(40);
  delay(3000);
  turnRight(60);
  delay(3000);
  turnRight(80);
  delay(3000);
  turnRight(102);
  delay(3000);
  turnRight(130);
  delay(3000);
  turnRight(180);
  delay(3000);
  turnRight(220);
  delay(3000);
  turnRight(255);
  delay(3000);
}

void testDownAcceleration(){
    accelerateDown(0);
    delay(3000);
    accelerateDown(20);
    delay(3000);
    accelerateDown(40);
    delay(3000);
    accelerateDown(60);
    delay(3000);
    accelerateDown(80);
    delay(3000);
    accelerateDown(102);
    delay(3000);
    accelerateDown(130);
    delay(3000);
    accelerateDown(180);
    delay(3000);
    accelerateDown(220);
    delay(3000);
    accelerateDown(255);
    delay(3000);
}
