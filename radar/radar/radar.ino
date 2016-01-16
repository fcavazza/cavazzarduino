//HC RS04 Sensore ultrasuoni

#include <Servo.h>

Servo myServo;


int triggerPort = 8;
int echoPort = 7;
int ANGLE = 90;
int SERVO_DIRECTION = 1;
int STEP_ANGLE = 1;
int MIN_ANGLE = 20;
int MAX_ANGLE = 160;

void setup() {
    myServo.attach(5);
    pinMode( triggerPort, OUTPUT );
    pinMode( echoPort, INPUT );
    Serial.begin( 250000 );
    //Serial.println( "Sensore ultrasuoni: ");
    myServo.write(ANGLE);
    delay(5000);
}
 
void loop() {
    //porta bassa l'uscita del trigger
    digitalWrite( triggerPort, LOW );
 
    //invia un impulso di 10microsec su trigger
    digitalWrite( triggerPort, HIGH );
    delayMicroseconds( 10 );
    digitalWrite( triggerPort, LOW );
    long r;
    long duration = pulseIn( echoPort, HIGH , 38001);
    if( duration > 38000 ) r = -1; // out of range
    else r = 0.034 * duration / 2;

    int oldangle = ANGLE;
    ANGLE = oldangle + STEP_ANGLE * SERVO_DIRECTION;
    if (ANGLE<= MIN_ANGLE){
        SERVO_DIRECTION = SERVO_DIRECTION * -1;
        ANGLE = MIN_ANGLE;
    } else if (ANGLE >= MAX_ANGLE){
        SERVO_DIRECTION = SERVO_DIRECTION * -1;
        ANGLE = MAX_ANGLE;
    }
    myServo.write(ANGLE);
    
    int inizio = millis();
    writeSerial(oldangle, r);
    readSerial();
    //if (millis() - inizio < 10) delay( 10 ); // time for servo to move
    delay(50);
}

void writeSerial(int oldangle, int r){
    Serial.print(oldangle);
    Serial.print(';');
    Serial.println( r );
}

void readSerial(){
  if(Serial.available()){
    STEP_ANGLE = Serial.parseInt();
    Serial.read();
    Serial.print("# ricevuto: STEP_ANGLE ");
    Serial.println(STEP_ANGLE);
  }
}
