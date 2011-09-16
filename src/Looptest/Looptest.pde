
int i;
void setup(){
  Serial.begin(9600);
  i = 0;
}

void loop(){
  i = i + 1;
  if(i == 1000){
  Serial.println(millis());
  }
}
  
