/****************************************************************************************
* Authors: Fredrik Cappelen and Nils Peder Korsveien                                    *
* University of Oslo 2011
*                                                                                       *
* SOURCE CODE AND DATA SHEETS                                                           *
* H-bridge data sheet : www.sparkfun.com/datasheets/IC/SN754410.pdf                     *
* H-bridge how-to     : http://itp.nyu.edu/physcomp/Labs/DCMotorControl#toc8            *
* PID-library         : http://arduino.cc/playground/Code/PIDLibrary                    *
* Compass product page: http://www.robonor.no/mag/default/compass-module-hmc6352.html   *
* Compass data sheet  : http://www.sparkfun.com/datasheets/Components/HMC6352.pdf       *
* Compass library     : http://mbed.org/cookbook/HMC6352-Digital-Compass                *
*****************************************************************************************/

//Fredrik endret smoothing formelen 15.9 på kvelden,  nå er høyere filter mer smoothing
//vi må finne ut av runtime for koden. Det er avjgørende for å sette riktig smoothing osv. Kjøretid bør være over 50 x sek.
//endret også startingverdiene for smoothing for å unngå feil kolisjon ved oppstart.
//vi bør jobbe med et og et problem. F.eks deaktivere kolisjoner fra siden og få til riktig oppførsel ved sensor foran.
//Vi trenger en litt spessiell PID justering, vi ønsker stort vindu  å defaultglide i. Defaultglide børe noe nedover så høyden holdes uten å gass rett opp.
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>


#define SMOOTHED 1
#define DEBUG 0
#define TESTDOUBLES 0
#define TESTTAIL 0
#define TESTRIGHTTAIL 0
#define TESTDOWN 0
#define BATTERYMONITOR 0
#define FRONTCOLLISION 1
#define LEFTCOLLISION 1
#define RIGHTCOLLISION 1
#define ELEVATORTEST 0
#define ALTITUDEDEBUG 0
#define TIMECOUNT 0
#define SMOOTHINGDEBUG 0
#define ENABLETAIL 0
#define DEBUGDIFF 0

Servo elevator; // servo pointer "elevator.write"
double acceleration, defaultAcceleration,thrust; //acceleration variables for main propellers
double tailAcceleration, tailDefaulAcceleration, tailThrust;  //acceleration variables for tail propeller
double leftRange, rightRange, forwardRange, altitudeRange; //variables for sensors, targets & voltage reading
double minLeftRange, minRightRange, minForwardRange, minAltitudeRange; // self explained
double targetAltitude, voltage;

float heading;                           // the current direction in degrees
double d_heading, course, currentCourse; // d__heading is heading converted to double, course is the desired direction
double target = 180;                     // target for compass differens
double diff;                             // diff to target

int timeCount = 0;

static int elevatorPin = 4;  // "staget" styres over denne
static int motor1Pin   = 9;  // H-bridge leg 1 (pin 2, 1A)
static int motor2Pin   = 11; // H-bridge leg 2 (pin 7, 2A)
static int enablePin   = 10; // H-bridge enable pin
static int ledPin      = 13; // LED
static int tailPin1    = 7;  // tailMotor leg 1
static int tailPin2    = 6;  // talMotor leg 2
static int enablePin2  = 5;  // enables side 2 of the H-bridge

// LED'S
int red = 2; //this sets the red led pin
int green = 8; //this sets the green led pin
int blue = 3; //this sets the blue led pin

//range sensors
static int altitudeRangePin = A0;
static int rightRangePin    = A1;
static int leftRangePin     = A7;
static int forwardRangePin  = A3;

//compass setup
int HMC6352SlaveAddress = 0x42; 

//compass I2C variables
int HMC6352ReadAddress = 0x41; //"A" in hex, A command is: 
int headingValue; //kompass kurs variabel

//smoothing filter variables
int sensVal;                 // for raw sensor values
float filterVal;             // this determines smoothness  - .0001 is max  1 is off (no smoothing)
float filterCollisionVal;
float smoothedAltitudeRange; // this holds the last loop value just use a unique variable for every different sensor that needs smoothing
float smoothedForwardRange;
float smoothedLeftRange;
float smoothedRightRange;    // this would be the buffer value for another sensor if you needed to smooth two different sensors - not used in this sketch
double d_smoothedAltitudeRange;
// variable used for collision detection
static double MINRANGE = 200.0;
boolean collisionDetected = false;

float smoothedAltitudeRangeFinal, smoothedLeftRangeTmp, smoothedRightRangeTmp, smoothedForwardRangeTmp;

// PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
// Input    : Variable we are trying to control(double)
// Output   : The variable that will be adjusted by the PID(double)
// Setpoint : The value we want to Input to maintain(double)
PID altitudePID(&d_smoothedAltitudeRange, &acceleration, &targetAltitude,2.9,0,0,DIRECT); 
PID tailPID(&diff, &tailAcceleration, &target,0.5,0,0,DIRECT); 

//for testing multiple PID's using forward sensor
/*double targetTestRange = 50.0;*/
/*PID testPID(&forwardRange, &tailAcceleration, &targetTestRange,4,0,0,DIRECT);*/

unsigned long t0;
void setup() {

    Serial.begin(9600); //skjermutskrift på
    t0 = millis();

    altitudePID.SetMode(AUTOMATIC);
    altitudePID.SetOutputLimits(-255, 255); 
    altitudePID.SetSampleTime(1);
    
    //vi må jobbe med å finne riktig maks outpoot for proppelen.
    tailPID.SetMode(AUTOMATIC);
    tailPID.SetOutputLimits(-100, 150); 
    tailPID.SetSampleTime(1);

    /*testPID.SetMode(AUTOMATIC);*/
    /*testPID.SetOutputLimits(-255, 255); */
    /*testPID.SetSampleTime(5);*/

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
    
    readAllSensors(); //leser alle sensorer en gang for å 

    //setter startverdier for avstandsbegrensninger og start kurs.
    course = 180;
    targetAltitude = 120;
    minForwardRange = 180;
    minRightRange = 180;
    minLeftRange = 180;
    smoothedForwardRange = minForwardRange + 50; //forwardRange;
    smoothedLeftRange = minLeftRange + 50; //leftRange;
    smoothedRightRange = minRightRange + 50; //rightRange;
    smoothedAltitudeRange = targetAltitude; //altitudeRange; 

    filterVal = 0.03;  //høy smoothing gir treg respons om loopen kjører sakte, finn ut hvor fort loopen kjører
    filterCollisionVal = 0.1;
}

void readAllSensors(){
    altitudeRange = analogRead(altitudeRangePin) + 0.01;
    leftRange     = analogRead(leftRangePin) + 0.01;
    rightRange    = analogRead(rightRangePin) + 0.01;
    forwardRange  = analogRead(forwardRangePin) + 0.01;
}

void smoothInput(){
    smoothedAltitudeRange = smooth(altitudeRange, filterVal, smoothedAltitudeRange);
    smoothedAltitudeRangeFinal = smooth(smoothedAltitudeRange, filterVal, smoothedAltitudeRangeFinal);
    d_smoothedAltitudeRange = (double)smoothedAltitudeRangeFinal;

    smoothedLeftRangeTmp = smooth(leftRange, filterCollisionVal, smoothedLeftRangeTmp);
    smoothedLeftRange = smooth(smoothedLeftRangeTmp, filterCollisionVal, smoothedLeftRange);

    smoothedRightRangeTmp = smooth(rightRange, filterCollisionVal, smoothedRightRangeTmp);
    smoothedRightRange = smooth(rightRange, filterCollisionVal, smoothedRightRange);

    smoothedForwardRangeTmp = smooth(forwardRange, filterCollisionVal, smoothedForwardRangeTmp);
    smoothedForwardRange = smooth(forwardRange, filterCollisionVal, smoothedForwardRange);

    if(SMOOTHINGDEBUG == 1){
        printAltitude();
    }
}

void accelerate(){
    if(acceleration < 0){
        thrust = (-acceleration);
    }
    else{
        thrust = acceleration;
    }

    /*printAcceleration();*/
    //if we are below target go up, if not go down
    if(acceleration > 115){
        accelerateUp(thrust);
    }
    else if(acceleration < -115){
        accelerateDown(thrust);
    }
    else { 
        defaultGlide(170);
    }
}

void turnToCourse(double course){

  if(tailAcceleration < 0){

        tailThrust = (-tailAcceleration);
        turnLeft(tailThrust);
    }
    else{
        tailThrust = tailAcceleration;
        turnRight(tailThrust);
    }
}

/*TODO:blimp is on collison course in forward direction. We then check right and left*/
/*sensor and determine which has the most available space for*/
/*maneuvering. We then add or subtract degrees to the current course*/
/*depending on which way we want to turn*/

//Return new course if collision is detected
//Leaves the course unaltered if no collision is detected
boolean detectCollision()
{
    // if colliton is up front:
    if(FRONTCOLLISION == 1){
        if (smoothedForwardRange < minForwardRange) {
            collisionDetected = true;

            if(DEBUG ==1){
                Serial.println("!!!!!!!!!!!!!!!");
                Serial.print("Kolisjon oppdaget forut, ny kurs: ");
                Serial.println("!!!!!!!!!!!!!!!");
            }

            //check wich way to turn:
            if (smoothedLeftRange > smoothedRightRange) { //turn LEFT
                course = course - 150;
                if (course > 360) {
                    course = course - 360;
                }
                if (course < 0) {
                    course = course + 360;
                }
                if(DEBUG == 1){
                    Serial.print("new course: ");
                    Serial.println(course);
                }
                return true;

            } else { //turn RIGHT
                course = d_heading + 150;
                // er kurs større enn 360, har vi gått rundt og 360 trekkes fra
                if (course > 360) {
                    course = course - 360;
                }
                //hvis ny krurs blir mindre enn 0 har vi gått rundt og 360 må legges til, feks -20 + 360 = 340
                if (course < 0) {
                    course = course + 360;
                }
                if(DEBUG == 1){
                    Serial.print("new course: ");
                    Serial.println(course);
                }
                return true;
            }
        }
    }

    if(LEFTCOLLISION == 1){

        if (smoothedLeftRange < minLeftRange) {
            if(DEBUG == 1){
                Serial.println("!!!!!!!!!!!!!!!");
                Serial.print("Kolisjon oppdaget til venstre!");
                Serial.println("!!!!!!!!!!!!!!!");
            }
            collisionDetected = true; // sørger for at vi ikke oppdager samme kolisjon mange ganger
            course = d_heading + 70;

            if (course > 360) {
                course = course - 360;
            }
            if (course < 0) {
                course = course + 360;
            }
            if(DEBUG == 1){
                Serial.print("new course: ");
                Serial.println(course);
            }
        }
    }

    if(RIGHTCOLLISION == 1){
        //kolisjon oppdaget til høyre:
        if (smoothedRightRange < minRightRange) { 
            collisionDetected = true; //se opp
            if(DEBUG == 1){
                Serial.println("!!!!!!!!!!!!!!!");
                Serial.print("Kolisjon oppdaget til høyre, ny kurs: ");
                Serial.println("!!!!!!!!!!!!!!!");
            }
            //turn left with X degrees
            course = d_heading - 70;
            if (course > 360) {
                course = course - 360;
            }
            if (course < 0) {
                course = course + 360;

            }
            if(DEBUG == 1){
                Serial.print("new course: ");
                Serial.println(course);
            }
            return true;
        }
    }
    return false;
}

//fungerer diff med tanke på negative verdier osv?
void calculateDiff() {  
  diff = course - d_heading;
  diff = diff + 180;


  if(DEBUGDIFF == 1){
      Serial.print("*** HEADING: ");
      Serial.print(d_heading);
      Serial.print("*** COURSE: ");
      Serial.print(course);
      Serial.print("*** DIFF: ");
      Serial.println(diff);
  }

  if (diff > 360) {
    diff = diff - 360;
  } 
  else if (diff < 180) {
    diff = diff + 360;
  }
}

void dirtyCollisionDetection(){
    if(smoothedForwardRange < minForwardRange){
        if(smoothedLeftRange > smoothedRightRange){
                turnLeft(75);
               // turnLeft(0);
        }
        else if(smoothedLeftRange < smoothedRightRange){
                turnRight(80);
               // turnRight(0);
        }
       
    }
    else if(smoothedLeftRange < minLeftRange){
            turnRight(80);
            //turnRight(0);
    }
    else if(smoothedRightRange < minRightRange){
            turnLeft(60);
           // turnLeft(0);
    }
    else { turnRight(0);
    }
}
          
// return new course based on collision info
double stakeOutCourse(){return 0.0;}

void loop(){ 
    if((millis() - t0) > 15000){
        accelerateDown(255);
    }
    if(ELEVATORTEST == 1){
        elevatorTest();
    }
    if(BATTERYMONITOR == 1){
        batteryMonitor();
    }
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
        d_heading = (double)heading;
        calculateDiff();
        readAllSensors();

        if(SMOOTHED == 1){
            smoothInput();
        }
        
        altitudePID.Compute();
        
        if(ENABLETAIL == 1){
            tailPID.Compute();
            //testPID.Compute();

            if(collisionDetected == false) { //hvis vi allerede ikke har oppdaget kolisjon
                detectCollision();
            }
            turnToCourse(course); //svinger med akselerasjon mot korrekt kurs.

            //FIXME: vil slå av collision-flagget mens piden aksellerer
            //forbi kursen
            if (diff > -3 &&  diff < 3) {
                collisionDetected = false;
                if(DEBUG == 1){
                    Serial.println("!!!! kolisjon avverget !!! ");
                    Serial.println();        
           }
         }
        }

        dirtyCollisionDetection();

        accelerate();              
        if(ALTITUDEDEBUG == 1){
            printAltitude();
        }
         
        printSmoothedRanges();
         if(TIMECOUNT == 1){
             timeCount = timeCount +1;
             if (timeCount == 100) {
                 Serial.print("100 cycles took: ");
                 Serial.println(millis());
                 delay(5000);
                 timeCount = 0;
             }
         }
           
        if(DEBUG == 1){
            printHeading();
            printCourse();
            printDiff();
          
            //printAltitude();
            printRanges();
            printAcceleration();
            printTailAcceleration();
        } 
    }
}

//TODO: kutt av litt i toppen på haleproppellen
void turnLeft(double acceleration) {
  if(DEBUG == 1){
      Serial.print("*** Turning left with acceleration: ");
      Serial.println(acceleration);
  }
  digitalWrite(tailPin1, HIGH);
  digitalWrite(tailPin2, LOW);
  analogWrite(enablePin2, acceleration);
}

//TODO: kutt av litt i toppen på haleproppellen
void turnRight(double acceleration){
  if(DEBUG == 1){
      Serial.print("*** Turning right with acceleration: ");
      Serial.println(acceleration);
  }
  digitalWrite(tailPin1, LOW);
  digitalWrite(tailPin2, HIGH);
  analogWrite(enablePin2, acceleration);
}

void accelerateUp(double acceleration){
  if(DEBUG == 1){
      Serial.print("*** Accelerating up with acceleration: ");
      Serial.println(acceleration);
  }
  
  analogWrite(motor2Pin, 0);            // slår av motorer, mens servo kjører pga strøm/forstyrrelser
  digitalWrite(motor1Pin, LOW);
  elevator.write(40);                   // snur stag i riktig posisjon
  delay(1);                             // venter litt på stag
  analogWrite(motor2Pin, acceleration); // starter motorer med PID akselerasjon
  digitalWrite(motor1Pin, LOW);
  
}

void accelerateDown(double acceleration){
  
  if(DEBUG == 1){
      Serial.print("*** Accelerating down with acceleration: ");
      Serial.println(acceleration);
  }
  analogWrite(motor2Pin, 0);            // slår av motorer, mens servo kjører pga strøm/forstyrrelser
  digitalWrite(motor1Pin, LOW);
  elevator.write(110);                   // snur stag i riktig posisjon
  delay(1);                             // venter litt på stag
  analogWrite(motor2Pin, acceleration); // starter motorer med PID akselerasjon
  digitalWrite(motor1Pin, LOW);
}

void defaultGlide(double acceleration){
  if(DEBUG == 1){
      Serial.print("*** DefaultGlide with default acceleration: ");
      Serial.println(acceleration);
  }
  
  analogWrite(motor2Pin, 0);            // slår av motorer, mens servo kjører pga strøm/forstyrrelser
  digitalWrite(motor1Pin, LOW);
  elevator.write(74);                   // snur stag
  delay(1);                             // venter på stag
  analogWrite(motor2Pin, acceleration); // kjører motor med PID akselerasjon
  digitalWrite(motor1Pin,LOW);
}

// emergency landing
void land() {
    while(1){
        Serial.println("*** EMERGENCY LANDING!");
        accelerateDown(255);
    }
}

//checks the mV of the battery, performs emergency landing if voltage is
//too low
void batteryMonitor () {
  
    double reading = (double) analogRead(7); //tilpass til riktig port
    
    voltage = reading * 5.0;
    ; //regner ut spenning
    
    if(DEBUG == 1){
        printVoltage();
    }
    if (voltage > 1500) {
      batteryLed(green, 150);
      
    } else if (voltage > 1000) {
       batteryLed(blue, 150);
       
    } else {
        batteryLed(red, 200);
        land();
    }        
}

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
  delay(1);

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

int smooth(float data, float filterVal, float smoothedVal){

  if (filterVal > 1){ // check to make sure param's are within range
    filterVal = .99;
  }
  else if (filterVal <= 0){
    filterVal = 0;
  }
  smoothedVal = smoothedVal + (data - smoothedVal)*filterVal;
  /*smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);*/

  return smoothedVal;
}

void testDoubleEngines(){
    accelerateUp(102);
    delay(3000);
    accelerateDown(102);
    delay(3000);
    defaultGlide(102);
    delay(3000);

    delay(3000);
    accelerateUp(180);
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

void elevatorTest(){
    elevator.write(30);
    delay(3000);
    elevator.write(90);
    delay(3000);
    elevator.write(130);
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

void printDiff() {
  Serial.print("*** DIFF: ");
  Serial.println(diff);
}

void printCourse(){
    Serial.print("*** COURSE: ");
    Serial.println(course);
}
    
void printRanges(){
    Serial.print("*** FORWARD RANGE: ");
    Serial.print("RAW: ");
    Serial.println(forwardRange);
    Serial.print("*** Smoothed: ");
    Serial.println(smoothedForwardRange);
    Serial.print("*** RIGHT RANGE: ");
    Serial.print("RAW: ");
    Serial.println(rightRange);
    Serial.print("*** Smoothed: ");
    Serial.println(smoothedRightRange);
    Serial.print("*** LEFT RANGE: ");
    Serial.print("RAW: ");
    Serial.println(leftRange);
    Serial.print("*** Smoothed: ");
    Serial.println(smoothedLeftRange);
    Serial.print("*** ALTITUDE RANGE: ");
    Serial.print("RAW: ");
    Serial.println(altitudeRange);
    Serial.print("*** Smoothed: ");
    Serial.println(smoothedAltitudeRange);
}

void printVoltage(){
    Serial.print("*** VOLTAGE: ");
    Serial.println(voltage);
}

void printTailAcceleration(){
    Serial.print("*** TAILACCELERATION: ");
    Serial.println(tailAcceleration);
}

void printHeading(){
    Serial.print("*** HEADING: ");
    Serial.println(heading);
}

void printAltitude(){
    Serial.print("*** ALTITUDE RANGE: ");
    Serial.println(altitudeRange);
    Serial.print("***SMOOTHED: ");
    Serial.println(smoothedAltitudeRangeFinal);
}

void printAcceleration(){
    Serial.print("*** ACCELERATION: ");
    Serial.println(acceleration);
}

void printSmoothedRanges() {
  
  Serial.print("Foran: ");
  Serial.println(forwardRange),
  Serial.println(smoothedForwardRange);
  Serial.print("Venstre: ");
  Serial.println(leftRange);
  Serial.println(smoothedLeftRange);
  Serial.println("Hoyre");
  Serial.println(rightRange);
  Serial.println(smoothedRightRange);
}
