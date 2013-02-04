#include <Stepper.h>
#include <Servo.h>


#define STEPPER_SPEED 50
#define MAX_VELOCITY 255
void run_cw (int v);
void run_ccw (int v);
void DCmotor(int v);
int steps = 9;
int dir = 12;

int switchPin = 2;    // switch input
int motor1Pin1 = 10;    // pin 10 on L293D
int motor1Pin2 = 11;    // pin 11 on L293D
int encoderA = 3;    // pin 3 on encoder
int encoderB = 4;    // pin 4 on encoder
int enablePin = 8;    // pin 1 on L293D
char command;
//in this case gear ratio is 1:60					
int gear_ratio = 60;
int desire_angle = 0;
int desire_steps = 0;
int desire_servo = 0;
int smalldegree = 0;
int limit = 0;
//int countAB = 0;
int velocity = 0;
boolean run = false;
boolean spin = false;
boolean servo = false;
boolean stepper = false;
boolean wait4angle = false;
boolean wait4step = false;
boolean wait4servo = false;
int kp = 1;
int ki = 0.1;
int kd = 1;
int sum = 0;
int last = 0;

volatile int encoder0Pos = 0;

// change this to fit the number of steps per revolution for your motor
const int stepsPerRevolution = 200;  

// create servo object to control a servo 
// a maximum of eight servo objects can be created
Servo myservo;

void setup(){
  Stepperinit();
  DCinit();
  Servoinit();
  
  pinMode(encoderA, INPUT); 
  pinMode(encoderB, INPUT); 
  attachInterrupt(0, countA, CHANGE);
  attachInterrupt(1, countB, CHANGE); 
  
  Serial.begin(9600);
  Serial.println("Intruction:");
  Serial.println("C: spin continuously"); 
  Serial.println("S: stop");
  Serial.println("D: DC motor");
  Serial.println("P: stepper motor");
  Serial.println("V: RC servo");
  Serial.println("(either uppercase or lowercase)");
}

void Stepperinit(){
  pinMode(dir, OUTPUT);
  pinMode(steps, OUTPUT); 
  digitalWrite(dir,1);
}

void DCinit(){
  //  Serial.println("Input the desired angle:\n"); 
  // set the switch as an input:
  pinMode(switchPin, INPUT); 
  digitalWrite(switchPin, HIGH);
  
  pinMode(encoderA, INPUT);
  digitalWrite(encoderA, HIGH);
  // set all the other pins you're using as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  // pinMode(enablePin, OUTPUT);
  
  // set enablePin high so that motor can turn on:
  // digitalWrite(enablePin, HIGH);
  // digitalWrite(switchPin, HIGH);
  // attachInterrupt(1, count, FALLING);//numbers 0 (on digital pin 2) and 1 (on digital pin 3)
}

void Servoinit(){
  // attaches the servo on pin 9 to the servo object 
  myservo.attach(6);
  myservo.write(0); //set servo to 0
}

void loop(){
  int error = 0;
  int difference = 0;
  
  //get input from prompt
  Prompt();
  
  if(run){
    //done
    if (encoder0Pos==limit) {
      encoder0Pos = 0;
      run = false;
      last = 0;
      sum = 0;
    }
    //PID
    error = limit-encoder0Pos;
    sum += error;
    difference = last-error;
    velocity = min(kp*error+ki*sum+kd*difference, MAX_VELOCITY);
    last = error;
    Serial.println(limit);
    Serial.println(encoder0Pos);
    DCmotor(velocity);
  }
  else if(spin){
    DCmotor(MAX_VELOCITY);
  }
  else if(servo) {
    myservo.write(desire_servo);
  }
  else if(stepper){
    if(desire_steps!=0){
      desire_steps--;
      digitalWrite(steps, 1);
      delay(500/STEPPER_SPEED);
      digitalWrite(steps, 0);
      delay(500/STEPPER_SPEED);
    }
    else
      stepper=false;
    }
  else {
    analogWrite(enablePin, 0);
  }
  
  if(desire_steps!=0){
    desire_steps--;
    digitalWrite(steps, 1);
    delay(500/STEPPER_SPEED);
    digitalWrite(steps, 0);
    delay(500/STEPPER_SPEED);
  }
}

void Prompt(){
  // if there's any serial available, read it:
  if(Serial.available() > 0) {
    if(wait4angle){
      desire_angle=Serial.parseInt();
      desire_angle = desire_angle%360;
      run=true;
      //countAB=0; //reset the counter
      encoder0Pos = 0;
      Serial.print("The desired angle is:\t");
      Serial.println(desire_angle);
      wait4angle=false;
      //convert the desired agle to the total angle at the rear shaft (multiply gear ratio)						
      smalldegree = desire_angle *gear_ratio;
      //12 count per revolution at the rear shaft
      //meaning each count represent 30 degree
      //to calculate how many counts needed for the desire angle
      //we divide total angle at the rear shaft with 30 degree	
      limit = smalldegree/(360/(3*60));
    }
    else if(wait4step){
      desire_steps=Serial.parseInt();
      Serial.print("The desired number of degrees is:\t");
      Serial.println(desire_steps);
      if(desire_steps>0)
        digitalWrite(dir,1);
      else if(desire_steps<0)
        digitalWrite(dir,0);
      desire_steps=abs(desire_steps);
      desire_steps/=1.8;
      wait4step=false;
    }
    else if(wait4servo){
      desire_servo=Serial.parseInt();
      Serial.print("The desired number of degrees for servo is(0~180):\t");
      Serial.println(desire_servo);
      wait4servo=false;
      servo=true;
    }
    else{
      command=Serial.read();
      if(command=='c'||command=='C'){
        Serial.println("spin continuously");
        spin=true;
        run=false;
        servo=false;
      }
      else if(command=='s'||command=='S'){
        Serial.println("stop");
        run=false;
        spin=false;
        servo=false;
      }
      else if(command=='d'||command=='D'){
        spin=false;
        Serial.println("Please input desired angle for DC motor:\n");
        wait4angle=true;
      }
      else if(command=='p'||command=='P'){
        Serial.println("Please input desired number of degrees for Stepper:\n");
        wait4step=true;
      }
      else if(command=='v'||command=='V'){
        Serial.println("Please input desired number of degrees for Servo:\n");
        wait4servo=true;
      }
    }
  }
}

void DCmotor(int v){
  if (spin) {
    if (digitalRead(switchPin) == HIGH) run_cw(v);
    else run_ccw(v);
  }
  else if (run){
    if (v>=0) run_cw(v);
    else run_ccw(-v);
  }
}
void countA(){
  // look for a low-to-high on channel A
  if (digitalRead(encoderA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderB) == LOW) encoder0Pos++; // CW
    else encoder0Pos--; // CCW
  }
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoderB) == HIGH) encoder0Pos++; // CW
    else encoder0Pos--; // CCW
  }
}
void countB(){
  // look for a low-to-high on channel B
  if (digitalRead(encoderB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoderA) == HIGH) encoder0Pos++; // CW
    else encoder0Pos--; // CCW
  }
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoderA) == LOW) encoder0Pos++; // CW
    else encoder0Pos--; // CCW
  }
}
void run_cw (int v){
  analogWrite(enablePin, v);
  digitalWrite(motor1Pin1, HIGH);  // set pin 2 on L293D high
  digitalWrite(motor1Pin2, LOW);   // set pin 7 on L293D low
}
void run_ccw (int v){
  analogWrite(enablePin, v);
  digitalWrite(motor1Pin1, LOW);  // set pin 2 on L293D high
  digitalWrite(motor1Pin2, HIGH);   // set pin 7 on L293D low
}
