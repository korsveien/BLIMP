unsigned long t0,t1;

bool alarmSounded = false;

void setup(){
    Serial.begin(9600);
    t0 = millis();
}

void loop(){
    if((millis() - t0) > 10000 && alarmSounded == false){
        soundTheAlarm();
        alarmSounded = true;
    }
}

void soundTheAlarm(){
    Serial.println("CUCKOOO!");
}
