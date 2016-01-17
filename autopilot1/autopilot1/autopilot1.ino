#include <Servo.h>
#include <NewPing.h>
#include <SoftwareSerial.h>

#define A_motor_pwm 3
#define A_motor_brake 9
#define A_motor_dir 12
#define A_motor_curr A0

#define B_motor_pwm 11
#define B_motor_brake 8
#define B_motor_dir 13
#define B_motor_curr A1

#define Steering_motor 5

#define Dist_echo 2
#define Dist_trig 4
#define max_distance_cm 200

#define Pot_input A2
#define Btn_1 7
#define Btn_2 8

#define MILLIS_SPEED_CHANGE 100 // accelerate every x millis, to let last acceleration have effect
#define STEP_SPEED_CHANGE 10 // how much acceleration for each step
#define MIN_SPEED 30 // minimum speed for motor to move
#define TIME_TO_BRAKE 2000 // time to stop from top speed

#define MAX_STEER 55 // maximum steering angle for servo
#define STEER_TRIM 25// adjust 0 angle

#define BT_RX A5 // bluetooth
#define BT_TX A4

int last_brake = 0;
int last_curr = 0;

int last_btn_1 = LOW;
int last_btn_2 = LOW;


int dir_A = HIGH;
bool brake_A = false;

int speed_A = 0;
int target_speed_A = 0;
unsigned long last_speed_change; // time since strategy start

int cur_steering = 0;
long last_dist;
long last_dist_check = millis();

int consumo;

bool strategyLock = false;
char strategy = 'Z'; // current strategy
char strategyState = 'Z'; // current strategy state
unsigned long strategyStart; // time since strategy start
unsigned long strategyStateStart; // time since strategy state start

SoftwareSerial bluetooth(BT_RX, BT_TX);

Servo steering_servo;
NewPing sonar(Dist_trig, Dist_echo, max_distance_cm);

void setup() {
  Serial.begin( 250000 );
  bluetooth.begin(9600); //set baud rate

  // put your setup code here, to run once:
  pinMode(A_motor_pwm, OUTPUT);
  pinMode(A_motor_brake, OUTPUT);
  pinMode(A_motor_dir, OUTPUT);

  pinMode(B_motor_pwm, OUTPUT);
  pinMode(B_motor_brake, OUTPUT);
  pinMode(B_motor_dir, OUTPUT);

  pinMode(Steering_motor, OUTPUT);

  pinMode(Dist_trig, OUTPUT);
  pinMode(Dist_echo, INPUT);

  pinMode(Btn_1, INPUT);
  pinMode(Btn_2, INPUT);

  steering_servo.attach(Steering_motor);

  digitalWrite(A_motor_dir, dir_A);

  forward();
  stopEngine();
}

void loop_test() {
  int steer = analogRead(Pot_input);
  if (steer != cur_steering) {
    int val = map(steer, 0, 1023, 0, 180);
    steering_servo.write(val);

    cur_steering = steer;
    bluetooth.println(val);
  }


}

void loop() {
  long dist;
  if ((millis() - last_dist_check) > 100) {
    dist = checkDistanceNew();
    if (dist==0){
      dist = checkDistanceNew();
    }
    consumo = analogRead(A0);
    last_dist_check = millis();
  } else  {
    dist = last_dist;
  }

  int delta_dist = dist - last_dist;
  last_dist = dist;

  checkManualStrategy();

  if (strategyLock == false) {
    if (consumo > 100 ) {
      bluetooth.println("consumo elevato");
      brakeAndBack();
    } else if (dist >= 100) { // current direction is good
      goAhead();
    }
    else if (dist < 20) { // emergency brake
      brakeAndBack();
    } else if (dist < 50) { // full steer
      steerFull();
    }
    else if (dist < 100) { // half steer
      steerHalf();
    }

  } else {
    switch (strategy) {
      case 'B':
        brakeAndBack();
        break;
      case 'G':
        goAhead();
        break;
      case 'H':
        steerHalf();
        break;
      case 'F':
        steerFull();
        break;
      case 'S':
        stopEngine();
        break;
      case 'X':
        manualSteer();
        break;
      case 'Y':
        manualSpeed();
        break;
    }
  }
  motorLoop();

}

void manualSteer() {
  bool start = startStrategy('X');
  strategyLock = true;
  int steer_ = map(analogRead(Pot_input), 0, 1023, -MAX_STEER, MAX_STEER);
  if (steer_ != cur_steering) {
    accelerate(0);
    steer(steer_);
  }
}

void manualSpeed() {
  bool start = startStrategy('Y');
  strategyLock = true;
  int speed_ = map(analogRead(Pot_input), 0, 1023, 0, 100);
  if (speed_ != target_speed_A) {
    accelerate(speed_);
    steer(0);
  }
}



void checkManualStrategy() {
  if (digitalRead(Btn_1) == HIGH && last_btn_1 == LOW && digitalRead(Btn_2) == HIGH && last_btn_2 == LOW) {
    if (strategy == 'X') {
      strategyLock = false;
    } else {
      stopEngine();
    }
  }

  if (digitalRead(Btn_1) == HIGH && last_btn_1 == LOW) {
    last_btn_1 == HIGH;
    if (strategy == 'X') {
      strategyLock = false;
    } else {
      manualSteer();
    }
  }
  if (digitalRead(Btn_1) == LOW) {
    last_btn_1 == LOW;
    delay(100);
  }

  if (digitalRead(Btn_2) == HIGH && last_btn_2 == LOW) {
    last_btn_2 == HIGH;
    if (strategy == 'Y') {
      strategyLock = false;
    } else {
      manualSpeed();
    }
  }
  if (digitalRead(Btn_1) == LOW) {
    last_btn_1 == LOW;
    delay(100);
  }


}

bool startStrategy(char st) {
  if (st == strategy) {
    return false; // strategy is in esecution
  } else {
    strategy = st;
    strategyStart = millis();
    strategyStateStart = millis();
    strategyState = 'Z';
    bluetooth.print("New strategy: ");
    bluetooth.print(st);
    bluetooth.print(" - dist: ");
    bluetooth.println(last_dist);
    return true;
  }

}

bool startStrategyState(char st) {
  if (st == strategyState) {
    return false; // strategy is in esecution
  } else {
    strategyState = st;
    strategyStateStart = millis();
    bluetooth.print("New strategy state: ");
    bluetooth.println(st);
    return true;
  }
}

void goAhead() {
  bool start = startStrategy('G');
  if (start) {
    accelerate(100);
    steer(0);
  }
}

void stopEngine() {
  bool start = startStrategy('S');
  if (start) {
    accelerate(0);
    steer(0);
  }
}

void brakeAndBack() {
  bool start = startStrategy('B');
  if (start) {
    strategyLock = true;
    bluetooth.println("strategyLock = true");
    accelerate(-1);
    steer(0);
    startStrategyState('A');
    return;
  }
  //bluetooth.println(strategyState);
  unsigned long t = millis();
  if (strategyState == 'A') { // braking
    if ((t - strategyStateStart) < TIME_TO_BRAKE) { // let time to brake
      //bluetooth.println("let time to brake ");
      return;
    }
    startStrategyState('B');
    reverse();
    accelerate(50);
    int newdir = random(0, 2);
    if (newdir == 1) {
      steer(MAX_STEER);
    } else {
      steer(MAX_STEER * -1);
    }
  }
  if (strategyState == 'B') { // steering back
    if ((t - strategyStateStart) < 2000) { // let time to steer back
      //bluetooth.println("let time to steer back");
      return;
    }
    startStrategyState('C');
    accelerate(-1);
  }
  if (strategyState == 'C') { // steering forward
    if ((t - strategyStateStart) < TIME_TO_BRAKE) { // let time to brake
      //bluetooth.println("let time to brake ");
      return;
    }
    startStrategyState('D');
    forward();
    steer(cur_steering * -1);
    accelerate(50);
  }

  if (strategyState == 'D') { // steering forward
    if (t - strategyStateStart > 2000) { // let time to steer forward
      strategyLock = false; // release the lock and auto define a new strategy
      bluetooth.println("strategyLock = false");
      goAhead();
    }
  }

}

void reverse() {
  dir_A = LOW;
  accelerate(-1); // brake before reverse
  digitalWrite(A_motor_dir, dir_A);
}

void forward() {
  dir_A = HIGH;
  accelerate(-1); // brake before reverse
  digitalWrite(A_motor_dir, dir_A);
}

void steerHalf() {
  bool start = startStrategy('H');
  if (start) {
    accelerate(100);
    int newdir = random(0, 2);
    if (newdir == 1) {
      steer(MAX_STEER / 2);
    } else {
      steer(MAX_STEER / 2 * -1);
    }
  }
}


void steerFull() {
  bool start = startStrategy('F');
  if (start) {
    accelerate(75);
    int newdir = random(0, 2);
    if (newdir == 1) {
      steer(MAX_STEER);
    } else {
      steer(MAX_STEER * -1);
    }
  }
}

void steer(int val) { // steer angle from -90 to +90
  if (cur_steering == val) {
    return;
  }
  steering_servo.write(val + 90 + STEER_TRIM); // servo need angle from 0 to 180
  cur_steering = val;
  //bluetooth.print("Steering: ");
  //bluetooth.println(val);
}

void accelerate(int val) {
  target_speed_A = val; //actually do nothing, just set target speed, motorLoop() will do the work
  //bluetooth.print("New target motor speed: ");
  //bluetooth.println(val);
}

void motorLoop() {
  if (speed_A == target_speed_A) {
    return;
  }
  if (target_speed_A == -1) { // brake
    analogWrite(A_motor_pwm, 0);
    digitalWrite(A_motor_brake, 1);
    speed_A = 0;
    brake_A = true;
    //bluetooth.println("brake");
  } else if (brake_A) { // if not braking, release the brake
    brake_A = false;
    digitalWrite(A_motor_brake, 0);
    //bluetooth.println("brake release");
  }
  if (target_speed_A < MIN_SPEED) {
    target_speed_A = 0;
  }

  if (target_speed_A < speed_A) { // decelerate
    analogWrite(A_motor_pwm, map(target_speed_A, 0, 100, 0, 255));
    speed_A = target_speed_A;
  }

  if (target_speed_A > speed_A) { // accelerate
    unsigned long t = millis();
    if ((t - last_speed_change) > MILLIS_SPEED_CHANGE) {
      speed_A += STEP_SPEED_CHANGE;
      if (speed_A < MIN_SPEED) {
        speed_A = MIN_SPEED;
      }
      analogWrite(A_motor_pwm, map(speed_A, 0, 100, 0, 255));
      last_speed_change = millis();
    }
  }
  //bluetooth.print("motor speed: ");
  //bluetooth.println(speed_A);
  //bluetooth.print("motor speed map: ");
  //bluetooth.println(map(speed_A, 0, 100, 0, 255));


}

long testDistance() {
  //porta bassa l'uscita del trigger
  digitalWrite(Dist_trig, LOW);

  //invia un impulso di 10microsec su trigger
  digitalWrite(Dist_trig, HIGH);
  delayMicroseconds( 10 );
  digitalWrite(Dist_trig, LOW);
  long r;
  long duration = pulseIn(Dist_echo, HIGH, 38001);
  delay(10);
  r = 0.034 * duration / 2;
  return r;
}

long checkDistance() {
  long r1 = testDistance();
  long r2 = testDistance();
  long r3 = testDistance();
  long m = (r1 + r2 + r3) / 3;
  if (abs(r1 - m) + abs(r2 - m) + abs(r3 - m) > 60) {
    bluetooth.print("anomalia distanza - media: ");
    bluetooth.print(m);
    bluetooth.print(" - ");
    bluetooth.print(r1);
    bluetooth.print(", ");
    bluetooth.print(r2);
    bluetooth.print(", ");
    bluetooth.println(r3);
  }
  return m;
}

long checkDistanceNew() {
  return sonar.convert_cm(sonar.ping_median(5));
}


void test() {
  int direzione;
  // put your main code here, to run repeatedly:
  int brakebtn = digitalRead(7);
  if (brakebtn != last_brake) {
    last_brake = brakebtn;
    if (brakebtn) {
      analogWrite(3, 0);
      digitalWrite(9, 1);
      bluetooth.println("freno");
      return;
    } else {
      digitalWrite(9, 0);
    }
  }


  int dirbtn = digitalRead(6);
  //if (last_dirbtn == 0 && dirbtn == 1){
  //  last_dirbtn = dirbtn;
  direzione = !direzione;
  digitalWrite(12, direzione);
  bluetooth.print("direzione: ");
  bluetooth.println(direzione);
  //}
  //last_dirbtn = dirbtn;

  int btn_speed_A = analogRead(A2);
  if (btn_speed_A != speed_A) {
    digitalWrite(12, direzione);
    analogWrite(3, map(0, 255, 0, 1023, btn_speed_A));
    bluetooth.print("velocita: ");
    bluetooth.print(direzione);
    bluetooth.println(btn_speed_A);
    speed_A = btn_speed_A;
  }
  int last_consumo;
  int consumo = analogRead(A0);
  if (consumo != last_consumo) {
    bluetooth.print("consumo: ");
    bluetooth.println(consumo);
    last_consumo = consumo;
  }
}
