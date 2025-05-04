//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=//
//                    CODE WRITTEN BY:
//
//                    TEAM GOLF (Electronics Technology Lab, Spring 2025)
//
//                    MICHAEL GAYDOS
//                    JONATHAN BOLZ
//                    AVERY PETERLIN
//                    WE DOYLE
//
//                    UNIVERSITY OF COLORADO - BOULDER
//                    ECEN 2270-010, LAB SECTION 11
//                       
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=// Tilt-Pan Servos
#include <Servo.h>
///*
#define TX_PIN  1
#define RX_PIN  0

#define PAN_PWM 3  // x servo
#define TILT_PWM 5 // y servo

#define PAN_SPEED 25 //in ms, goes in delay function

Servo x_servo;
Servo y_servo;

int x_pos;  //variables to store the servo positions
int y_pos;

int x_read;
int y_read;

//*/
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=// Serial
String receivedData = "wait";

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=// LED Lamp state
int Lamp_state = 0;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=// Mic state
int Mic_state = 0;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=// Bot Motor Drive

volatile int leftEncoder = 0;
volatile int rightEncoder = 0;

const int pinON = 6;
const int pinRightForward = 7;
const int pinRightBackward = 8;
const int pinRightPWM = 9;
const int pinLeftPWM = 10;
const int pinLeftForward = 11;
const int pinLeftBackward = 12;

const int two_feet = 2650;
const int clock_wise = 2000;
const int counter_clock = 2000;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=//

void setup() {
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=// Serial SETUP
  Serial.begin(115200); //for debugging
  delay(100);
  Serial1.begin(115200);
  delay(100);
  receivedData = "wait";
  Serial.println("Serial setup complete!");
  
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=// Bot Motor Drive SETUP
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), rightCount, RISING);
  delay(100);
  pinMode(4, INPUT);
  attachInterrupt(digitalPinToInterrupt(4), leftCount, RISING);
  delay(100);
  pinMode(pinON, INPUT_PULLUP);
  pinMode(pinRightPWM, OUTPUT);
  pinMode(pinRightForward, OUTPUT);
  pinMode(pinRightBackward, OUTPUT);
  pinMode(pinLeftPWM, OUTPUT);
  pinMode(pinLeftForward, OUTPUT);
  pinMode(pinLeftBackward, OUTPUT);
  pinMode(13, OUTPUT);                  //LED
  digitalWrite(pinRightForward, LOW);   //Initialize Low
  digitalWrite(pinRightBackward, LOW);  //Initialize Low
  digitalWrite(pinLeftForward, LOW);    //Initialize Low
  digitalWrite(pinLeftBackward, LOW);   //Initialize Low
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=// LAMP SETUP
  pinMode(A0, OUTPUT);
  digitalWrite(A0, LOW); // turn off LED
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=// MIC control setup
  pinMode(A1, OUTPUT);
  digitalWrite(A1, LOW); // turn off LED
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=// Tilt-Pan SETUP

  x_servo.attach(PAN_PWM);   // set the servo's pwm reading to pin 3 on arduino
  delay(100);
  y_servo.attach(TILT_PWM);  // set the servo's pwm readint to pin 10 on arduino
  delay(100);
  x_pos = 90;
  y_pos = 70;
  x_servo.write(x_pos);  //set the servo positions to "center" between range 0 and 180
  y_servo.write(y_pos);
  delay(100);
  x_servo.detach();
  y_servo.detach();
  //*/
  delay(100);
  Serial.println("Pan/Tilt setup complete!");
}

void loop() {

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=// Tilt-Pan Servos MAIN LOOP
  if (Serial1.available() > 0){
    receivedData = Serial1.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(receivedData);
  }
  delay(5);
  
  if(receivedData == "Tilt_up"){

    if(y_pos > 0){
      Serial.println("Tilting up!");
      y_pos--;
      y_servo.attach(TILT_PWM); 
      y_servo.write(y_pos);
    }
    delay(PAN_SPEED);

  }
  else if(receivedData == "Tilt_down"){
    
    if(y_pos < 180){
      Serial.println("Tilting down!");
      y_pos++;
      y_servo.attach(TILT_PWM); 
      y_servo.write(y_pos);
    }
    delay(PAN_SPEED);
    
  }
  else if(receivedData == "Pan_left"){

    if(x_pos < 180){
      Serial.println("Panning left!");
      x_pos++;
      x_servo.attach(PAN_PWM); 
      x_servo.write(x_pos);
    }
    delay(PAN_SPEED);

  }
  else if(receivedData == "Pan_right"){
    
    if(x_pos > 0){
      Serial.println("Panning right!");
      x_pos--;
      x_servo.attach(PAN_PWM); 
      x_servo.write(x_pos);
    }
    delay(PAN_SPEED);

  }
  else if(receivedData == "Center"){
    
    Serial.println("Centering Cam!");
    x_servo.attach(PAN_PWM);   // set the servo's pwm reading to pin 3 on arduino 
    delay(100);
    y_servo.attach(TILT_PWM);  // set the servo's pwm readint to pin 10 on arduino
    delay(100);
    x_pos = 90;
    y_pos = 70;
    x_servo.write(x_pos);  //set the servo positions to "center" between range 0 and 180
    y_servo.write(y_pos);
    delay(100);
    x_servo.detach();
    y_servo.detach();
    receivedData = "wait";
    delay(PAN_SPEED);
  
  }
  else if(receivedData == "Stop"){
    
    x_servo.detach();
    y_servo.detach();
    receivedData = "wait";
    delay(PAN_SPEED);

  }
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=// Bot Motor Drive MAIN LOOP
  else if(receivedData == "Drive_stop"){

    Serial.println("Motors stop!");
    motors_stop();
    receivedData = "wait";
    delay(100);

  }
  else if(receivedData == "Drive_fwd"){

    Serial.println("Driving forward!");
    int ramp_speed = 0; // initial speed is zero, increases speed gradually // until at 255 pwm
    do{
      moveForward(ramp_speed);
      if(ramp_speed < 5*51) { ramp_speed++; }  // pwm255(100%duty) = n*51, where n = voltage value (5v is max)
    } while(ramp_speed < 5*51);
    receivedData = "wait";
    delay(100);

  }
  else if(receivedData == "Drive_bck"){

    Serial.println("Driving backward!");
    int ramp_speed = 0; // initial speed is zero, increases speed gradually // until at 255 pwm
      do{
        moveBackward(ramp_speed);
        if(ramp_speed < 5*51) { ramp_speed++; }  // pwm255(100%duty) = n*51, where n = voltage value (5v is max)
      } while(ramp_speed < 5*51);
    receivedData = "wait";
    delay(100);

  }
  else if(receivedData == "Turn_left"){
   
    Serial.println("Turning left!"); 
    int ramp_speed = 0; // initial speed is zero, increases speed gradually // until at 255 pwm
    do{
      turnCounterClockwise(ramp_speed);
      if(ramp_speed < 5*51) { ramp_speed++; }  // pwm255(100%duty) = n*51, where n = voltage value (5v is max)
    } while(ramp_speed < 5*51);
    receivedData = "wait";
    delay(100);

  }
  else if(receivedData == "Turn_right"){
    
    Serial.println("Turning right!");
    int ramp_speed = 0; // initial speed is zero, increases speed gradually // until at 255 pwm
    do{
      turnClockwise(ramp_speed);
      if(ramp_speed < 5*51) { ramp_speed++; }  // pwm255(100%duty) = n*51, where n = voltage value (5v is max)
    } while(ramp_speed < 5*51);
    receivedData = "wait";
    delay(100);

  }
  else if(receivedData == "LED"){
    
    if(Lamp_state == 0){
      Serial.println("Lamp on!");
      Lamp_state = 1;
      digitalWrite(A0, HIGH); // turn on LED
    }
    else if(Lamp_state == 1){
      Serial.println("Lamp off!");
      Lamp_state = 0;
      digitalWrite(A0, LOW); // turn off LED
    }
    receivedData = "wait";
    delay(100);
  }
  else if(receivedData == "Mic"){
    
    if(Mic_state == 0){
      Serial.println("Mic on!");
      Mic_state = 1;
      digitalWrite(A1, HIGH); // turn on Mic
    }
    else if(Mic_state == 1){
      Serial.println("Mic off!");
      Mic_state = 0;
      digitalWrite(A1, LOW); // turn off Mic
    }
    receivedData = "wait";
    delay(100);

  }

}// main loop()


void moveForward(int speed){
  analogWrite(pinRightPWM, speed); 
  analogWrite(pinLeftPWM, speed); 
  digitalWrite(pinLeftForward, HIGH);
  digitalWrite(pinRightForward, HIGH);
}

void moveBackward(int speed){
  analogWrite(pinRightPWM, speed); 
  analogWrite(pinLeftPWM, speed); 
  digitalWrite(pinLeftBackward, HIGH);
  digitalWrite(pinRightBackward, HIGH);
}

void motors_stop(){
  digitalWrite(pinLeftForward, LOW);
  digitalWrite(pinRightForward, LOW);
  digitalWrite(pinLeftBackward, LOW);
  digitalWrite(pinRightBackward, LOW);
  delay(1000);
  rightEncoder = 0;
  leftEncoder = 0;
}

void turnClockwise(int speed){
  analogWrite(pinRightPWM, speed);    
  analogWrite(pinLeftPWM, speed);
  digitalWrite(pinLeftForward, HIGH);
  digitalWrite(pinRightBackward, HIGH);
}

void turnCounterClockwise(int speed){
  analogWrite(pinRightPWM, speed);
  analogWrite(pinLeftPWM, speed);
  digitalWrite(pinRightForward, HIGH);
  digitalWrite(pinLeftBackward, HIGH);
}

// Interrupt Service Routines
void rightCount() {
  rightEncoder++;
}

void leftCount() {
  leftEncoder++;
}
