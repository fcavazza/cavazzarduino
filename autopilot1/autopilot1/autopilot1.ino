#include <Servo.h>
#include <NewPing.h>
#include <SoftwareSerial.h>
#include <RunningMedian.h>

#define DEBUG_NO_MOTOR false

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
#define Dist_trig 2
#define MAX_DISTANCE_CM 500

#define Pot_input A2
#define Btn_1 7
#define Btn_2 8

#define MILLIS_SPEED_CHANGE 100 // accelerate every x millis, to let last acceleration have effect
#define STEP_SPEED_CHANGE 10 // how much acceleration for each step
#define MIN_SPEED 40 // minimum speed for motor to move
#define TIME_TO_BRAKE 1000 // time to stop from top speed

#define CURRENT_TO_MA 2.96 // 2A at 3.3 volt: convert analog read to mA 
#define MAX_TORQUE_MA 330 // lego medium motor max torque 300

#define MAX_STEER 55 // maximum steering angle for servo
#define STEER_TRIM 24// adjust 0 angle

#define BT_RX A5 // bluetooth
#define BT_TX A4
#define TIME_UNIQUE_MANEUVER 5000 //if less than 5 seconds from last maneuver, it is an unique maneuver

#define BT_COMM_DELAY 50 // bluetooth communication frequency

#define DIST_SENSOR_DELAY 20 // read frequency
#define DIST_SENSOR_NUM 10 // number of observation retained
#define DIST_SENSOR_CENTRAL 8 // number of observation retained

#define CURR_SENSOR_DELAY 200 // read frequency
#define CURR_SENSOR_NUM 20 // number of observation retained

#define DIST_HALF_STEER 100
#define DIST_FULL_STEER 50
#define DIST_BRAKE 20

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
float last_dist;
long last_dist_check = millis();
long last_curr_check = millis();

long last_brake_and_back_time;
int brake_and_back_direction;

bool strategyLock = false;
char strategy = 'Z'; // current strategy
char strategyStep = 'Z'; // current strategy state
unsigned long strategyStart; // time since strategy start
unsigned long strategyStepStart; // time since strategy state start

unsigned long lastCommCicles = 0; // cicles since last communication
unsigned long lastCommTime = 0;

bool communicateNow = false;

SoftwareSerial bluetooth(BT_RX, BT_TX);

Servo steering_servo;
NewPing sonar(Dist_trig, Dist_echo, MAX_DISTANCE_CM);

String manualCommand;
String btLog;

RunningMedian distMem(DIST_SENSOR_NUM);
RunningMedian sensorclock(DIST_SENSOR_NUM);
RunningMedian current(CURR_SENSOR_NUM);


void setup() {
  Serial.begin( 250000 );
  bluetooth.begin( 57600); //set baud rate

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
  lastCommCicles = 0;
  lastCommTime = 0;
  forward();
  stopEngine();
}

void loop() {
  lastCommCicles += 1;
  float dist;
  unsigned long ms = millis();
  unsigned long ms_tot = ms;
  if ((millis() - last_dist_check) > DIST_SENSOR_DELAY) {
    dist = checkDistanceAvg();
    last_dist_check = millis();
  } else  {
    dist = last_dist;
  }
  float delta_dist = dist - last_dist;
  last_dist = dist;

  if ((millis() - last_curr_check) > CURR_SENSOR_DELAY) {
    current.add(analogRead(A0) * CURRENT_TO_MA);
    last_curr = current.getAverage();
    last_curr_check = millis();
  }

  //ms = millis();

  checkManualStrategy();

  //ms = millis();
  if (strategyLock == false) {
    if (last_curr > MAX_TORQUE_MA) {
      btLog = btLog + "# high current\n";
      brakeAndBack();
    } else if (dist >= DIST_HALF_STEER) { // direction is good
      goAhead();
    }
    else if (dist < DIST_BRAKE) { // emergency brake
      btLog = btLog + "# emergency brake\n";
      brakeAndBack();
    } else if (dist < DIST_FULL_STEER) { // full steer
      steerFull();
    }
    else if (dist < DIST_HALF_STEER) { // half steer
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
      case 'M':
        // manual command from bluetooth: do nothing, commands are given in checkManualStrategy
        break;
    }
  }

  //ms = millis();
  motorLoop();

  ms = millis();
  manualCommand = communicate();
  if (manualCommand.length() > 0) {
    Serial.println(String("manualCommand: ") + manualCommand + "\n");
    Serial.println(String("communicate: ") + (millis() - ms) + "\n");
    Serial.println(String("total: ") + (millis() - ms_tot) + "\n");
  }
}


String communicate() {
  unsigned long timer = millis();
  if (btLog.length() == 0 && ((timer - lastCommTime) < BT_COMM_DELAY) && (!communicateNow)) {
    return (String());
  }
  if (btLog.length() > 0) {
    bluetooth.println(btLog);
    btLog = String();
  }
  String stringOne;
  stringOne = String(timer) + ";" + last_dist + ";" + cur_steering + ";" + dir_A + ";"
              + brake_A + ";" + speed_A + ";" + target_speed_A + ";" + last_curr + ";"
              + strategyLock + ";" + strategy + ";" + strategyStart + ";"
              + strategyStep + ";" + strategyStepStart + ";" + lastCommCicles + ";" + lastCommTime + ";";

  for (byte j = 1; j <= (distMem.getCount() / 2); j++) {
    stringOne += String(int(distMem.getElement(j))) + ":";
    stringOne += String(int(sensorclock.getElement(j))) + ";";
  }
  char XOR = 0;
  for (int i = 0; i < (stringOne.length()); i++) {
    XOR = XOR ^ stringOne.charAt(i);
  }
  stringOne = XOR + stringOne;

  bluetooth.println(stringOne);
  Serial.println(stringOne);
  Serial.print("spedisco BT: ");
  Serial.println(millis() - timer);

  //timer = millis();
  String pool;
  while (bluetooth.available()) {
    pool += char(bluetooth.read());
  }

  if (pool != "") {
    int lastReturn = pool.lastIndexOf('\n', (pool.length() - 2));
    if (lastReturn > -1) {
      pool = pool.substring(lastReturn);
    }
    pool.trim();
    char XORpy = pool[0];

    XOR = 0;
    for (int i = 1; i < pool.length(); i++) {
      XOR = XOR ^ pool.charAt(i);
    }

    if (XOR != XORpy) {
      Serial.println(String("comando sbagliato: ") + XORpy + " - " + XOR + " - " + pool);
      pool = "";
    }
  }
  //Serial.print("leggo BT: ");
  //Serial.println(millis() - timer);
  lastCommTime = millis();
  lastCommCicles = 0;
  communicateNow = false;
  return pool.substring(1);
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

void serialRemote() {
  bool start = startStrategy('M');
  strategyLock = true;

  char dir = manualCommand[1];
  if (dir == '0') {
    reverse();
  } else {
    forward();
  }
  int speed_ = manualCommand.substring(2, 5).toInt();
  Serial.println(String("speed_: ") + speed_ + "\n");
  int steer_ = manualCommand.substring(5, 8).toInt();
  Serial.println(String("steer_: ") + steer_ + "\n");

  int brake = int(manualCommand[9]);
  if (brake == 1) {
    accelerate(-1);
  } else {
    accelerate(speed_);
  }
  steer(steer_);
  Serial.println(String("serialRemote: ") + speed_ + ";" + steer_ + "\n");
}

void checkManualStrategy() {
  switch (manualCommand[0]) {
    case 'A':
      if (strategy == 'M') {
        bluetooth.println("# AUTO");
        strategyLock = false;
        goAhead();
      }
      break;
    case 'M':
      serialRemote();
      break;
  }

  manualCommand = "";
}

//  if (digitalRead(Btn_1) == HIGH && last_btn_1 == LOW && digitalRead(Btn_2) == HIGH && last_btn_2 == LOW) {
//    if (strategy == 'X') {
//      strategyLock = false;
//    } else {
//      stopEngine();
//    }
//  }
//
//  if (digitalRead(Btn_1) == HIGH && last_btn_1 == LOW) {
//    last_btn_1 == HIGH;
//    if (strategy == 'X') {
//      strategyLock = false;
//    } else {
//      manualSteer();
//    }
//  }
//  if (digitalRead(Btn_1) == LOW) {
//    last_btn_1 == LOW;
//    delay(100);
//  }
//
//  if (digitalRead(Btn_2) == HIGH && last_btn_2 == LOW) {
//    last_btn_2 == HIGH;
//    if (strategy == 'Y') {
//      strategyLock = false;
//    } else {
//      manualSpeed();
//    }
//  }
//  if (digitalRead(Btn_1) == LOW) {
//    last_btn_1 == LOW;
//    delay(100);
//  }


bool startStrategy(char st) {
  if (st == strategy) {
    return false; // strategy is in esecution
  } else {
    strategy = st;
    strategyStart = millis();
    strategyStepStart = millis();
    strategyStep = 'Z';
    btLog = btLog + "# New strategy: " + st + " - dist: " + last_dist + "\n";
    communicateNow = true;
    return true;
  }

}

bool startstrategyStep(char st) {
  if (st == strategyStep) {
    return false; // strategy is in esecution
  } else {
    strategyStep = st;
    strategyStepStart = millis();
    btLog = btLog + "# New strategy state: " + st + "\n";
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
    btLog = btLog + "# strategyLock = true\n";
    accelerate(-1);
    steer(0);
    startstrategyStep('A');
    return;
  }
  //bluetooth.println(strategyStep);
  unsigned long t = millis();
  if (strategyStep == 'A') { // braking
    if ((t - strategyStepStart) < TIME_TO_BRAKE) { // let time to brake
      //bluetooth.println("let time to brake ");
      return;
    }
    startstrategyStep('B');
    reverse();
    accelerate(60);

    if ((millis() - last_brake_and_back_time) > TIME_UNIQUE_MANEUVER) {
      brake_and_back_direction = random(0, 2);
    }
    if (brake_and_back_direction == 1) {
      steer(MAX_STEER);
    } else {
      steer(MAX_STEER * -1);
    }
  }
  if (strategyStep == 'B') { // steering back
    if ((t - strategyStepStart) < 2000) { // let time to steer back
      //bluetooth.println("let time to steer back");
      return;
    }
    startstrategyStep('C');
    accelerate(0);
  }
  if (strategyStep == 'C') { // steering forward
    if ((t - strategyStepStart) < (TIME_TO_BRAKE / 2)) { // let time to brake
      //bluetooth.println("let time to brake ");
      return;
    }
    startstrategyStep('D');
    forward();
    steer(cur_steering * -1);
    accelerate(60);
  }

  if (strategyStep == 'D') { // steering forward
    last_brake_and_back_time = millis();
    if (last_dist < (DIST_BRAKE / 2)) { // the reverse run is not enought, some more back
      startstrategyStep('A');
      return;
    } else if (last_dist >= DIST_HALF_STEER) {
      strategyLock = false; // release the lock and auto define a new strategy
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
    int newdir;
    if (cur_steering == 0) { // going straight
      newdir = random(0, 2); // choose a random direction
    } else if (cur_steering > 0) { // yet steering (full)
      newdir == 1;               // steer half in the same direction
    } else {
      newdir == 0;
    }

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
    int newdir;
    accelerate(75);
    if (cur_steering == 0) { // going straight
      newdir = random(0, 2); // choose a random direction
    } else if (cur_steering > 0) { // yet steering (half)
      newdir == 1;               // steer full in the same direction
    } else {
      newdir == 0;
    }

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
  if (DEBUG_NO_MOTOR == false) {
    steering_servo.write(val + 90 + STEER_TRIM); // servo need angle from 0 to 180
  }
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
    if (DEBUG_NO_MOTOR == false) {
      analogWrite(A_motor_pwm, map(target_speed_A, 0, 100, 0, 255));
    }
    speed_A = target_speed_A;
  }

  if (target_speed_A > speed_A) { // accelerate
    unsigned long t = millis();
    if ((t - last_speed_change) > MILLIS_SPEED_CHANGE) {
      speed_A += STEP_SPEED_CHANGE;
      if (speed_A < MIN_SPEED) {
        speed_A = MIN_SPEED;
      }
      if (DEBUG_NO_MOTOR == false) {
        analogWrite(A_motor_pwm, map(speed_A, 0, 100, 0, 255));
      }
      last_speed_change = millis();
    }
  }
  //bluetooth.print("motor speed: ");
  //bluetooth.println(speed_A);
  //bluetooth.print("motor speed map: ");
  //bluetooth.println(map(speed_A, 0, 100, 0, 255));


}

//long testDistance() {
//  //porta bassa l'uscita del trigger
//  digitalWrite(Dist_trig, LOW);
//
//  //invia un impulso di 10microsec su trigger
//  digitalWrite(Dist_trig, HIGH);
//  delayMicroseconds( 10 );
//  digitalWrite(Dist_trig, LOW);
//  long r;
//  long duration = pulseIn(Dist_echo, HIGH, 38001);
//  delay(10);
//  r = 0.034 * duration / 2;
//  return r;
//}

//long checkDistance() {
//  long r1 = testDistance();
//  long r2 = testDistance();
//  long r3 = testDistance();
//  long m = (r1 + r2 + r3) / 3;
//  if (abs(r1 - m) + abs(r2 - m) + abs(r3 - m) > 60) {
//    bluetooth.print("anomalia distanza - media: ");
//    bluetooth.print(m);
//    bluetooth.print(" - ");
//    bluetooth.print(r1);
//    bluetooth.print(", ");
//    bluetooth.print(r2);
//    bluetooth.print(", ");
//    bluetooth.println(r3);
//  }
//  return m;
//}

long checkDistanceNew() {
  //unsigned long timer = millis();
  return sonar.convert_cm(sonar.ping_median(5));
  //Serial.print("leggo sonar: ");
  //Serial.println(millis() - timer);
  //return d;
}

float checkDistanceAvg() {
  float dist =  sonar.ping_cm();
  if(dist==0){
    dist = MAX_DISTANCE_CM;
  }
  distMem.add(dist);
  sensorclock.add(millis());
  return distMem.getAverage(DIST_SENSOR_CENTRAL);
}


//void simpLinReg(float* x, float* y, float* lrCoef, int n){
//  // pass x and y arrays (pointers), lrCoef pointer, and n.  The lrCoef array is comprised of the slope=lrCoef[0] and intercept=lrCoef[1].  n is length of the x and y arrays.
//  // http://en.wikipedia.org/wiki/Simple_linear_regression
//  // http://jwbrooks.blogspot.it/2014/02/arduino-linear-regression-function.html
//  // initialize variables
//  float xbar=0;
//  float ybar=0;
//  float xybar=0;
//  float xsqbar=0;
//
//  // calculations required for linear regression
//  for (int i=0; i<n; i++){
//    xbar=xbar+x[i];
//    ybar=ybar+y[i];
//    xybar=xybar+x[i]*y[i];
//    xsqbar=xsqbar+x[i]*x[i];
//  }
//  xbar=xbar/n;
//  ybar=ybar/n;
//  xybar=xybar/n;
//  xsqbar=xsqbar/n;
//
//  // simple linear regression algorithm
//  lrCoef[0]=(xybar-xbar*ybar)/(xsqbar-xbar*xbar);
//  lrCoef[1]=ybar-lrCoef[0]*xbar;
//}

