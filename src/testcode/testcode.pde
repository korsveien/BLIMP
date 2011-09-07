
#include <Servo.h>
int range = 0; 
Servo elevator;
Servo throttle;
Servo tail;// create servo object to control a servo
                // a maximum of eight servo objects can be created
 
int pos = 30;    // variable to store the servo position
int posThrottle = 0;
int posTail =0;
 
void setup()
{
  Serial.begin(9600);
  elevator.attach(9);  // attaches the servo on pin 9 to the servo object
  throttle.attach(10);
  tail.attach(11);

}
 
 
 
void loop()
{
  range = analogRead(7
  );
  Serial.println(range);
  int push = 125;
  if (range > 70) {
  tail.write(95);  
  throttle.write(103);
  } else {
    throttle.write(95);
    tail.write(105);
  }
  
  //tail.write(75);
  //delay(3000);
  //elevatorTest();
  //elevator.write(75); // 77 = rett fremover, 30 = opp, 122 = ned
  //throttle.write(104); // 84 = stop //102 = start
  //tail.write(110);
  //
  //delay(3000);
  //throttle.write(109);
  /*
throttle.write(posThrottle);
Serial.print(posThrottle);
delay(2000);
throttle.write(posThrottle+=30);
Serial.print(posThrottle);
delay(2000);
throttle.write(posThrottle+=30);
Serial.print(posThrottle);
delay(2000);
throttle.write(posThrottle+=30);
Serial.print(posThrottle);
delay(2000);
throttle.write(posThrottle+=30);
Serial.print(posThrottle);
delay(2000);
throttle.write(posThrottle+=30);
Serial.print(posThrottle);
delay(2000);
throttle.write(posThrottle+=30);
Serial.print(posThrottle);
delay(2000);
throttle.write(posThrottle = 0);
Serial.print(posThrottle);

delay(2000);
*/


// går opp


  
  /*
  for(pos = 70; pos < 170; pos += 1)  // goes from 0 degrees to 180 degrees
  {                                  // in steps of 1 degree
    elevator.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);                      // waits 15ms for the servo to reach the position
  }
  delay(1000);
  // går helt ned
  for(pos = 170; pos>=70; pos-=1)     // goes from 180 degrees to 0 degrees
  {                                
    elevator.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);                     // waits 15ms for the servo to reach the position
  }
  delay(1000);
  //rett fram
  for( pos = 70; pos < 120; pos+=1)
  {
    elevator.write(pos);
    delay(5);
  }
  delay(1500);
  for (pos = 120; pos >= 70; pos-=1)
  {
    elevator.write(pos);
    delay(5);
  }
  delay(200);

  */
}




void elevatorTest() {
  
  for(pos = 30; pos < 77; pos+= 1)  // goes from 0 degrees to 180 degrees
  {                                  // in steps of 1 degree
    elevator.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                      // waits 15ms for the servo to reach the position
  }
  delay(1000);
  // går helt ned
  for(pos = 77; pos>=30; pos-=1)     // goes from 180 degrees to 0 degrees
  {                                
    elevator.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                     // waits 15ms for the servo to reach the position
  }
  delay(1000);
  //rett fram
  for( pos = 30; pos < 122; pos+=1)
  {
    elevator.write(pos);
    delay(3);
  }
  delay(1500);
  for (pos = 122; pos >= 30; pos-=1)
  {
    elevator.write(pos);
    delay(3);
  }
  delay(200);

}


