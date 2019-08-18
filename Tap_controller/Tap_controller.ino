// This code test the functionality of the Egg circuitry.
// The components connected to the circuit are:
// - LED 1 (Pin 3)
// - LED 2 (Pin 4)
// - LED 3 (Pin 5)
// - Vibratrional Motor (Pin 3)
// - Velostat Sensor (Pin A4)
#include <SD.h>
#include <SPI.h>
#include <arduino.h>
#include <MusicPlayer.h>
#include "Adafruit_NeoPixel.h"

#define BLUE 600
#define YELLOW 700
#define PURPLE 880

#define TapMotorPin 6
#define SqueezeMotorPin 7
#define SpinMotorPin 

#define TapLEDPin 9//4
#define SqueezeLEDPin 5
#define SpinLEDPin 3

// Pin Interrupts
#define TapIntrptPin 2
#define SqueezeIntrptPin 3
#define SpinIntrptPin 21

int TapSensorPin = A2; //analog pin 2
int SqueezeSensorPin = A1; //analog pin 1
int SpinSensorEgg = A4; //analog pin 4

// Stores reading from velostat sensor
int SpinReading = 0;
volatile int TapReading = 0;
int SqueezeReading = 0;

// Variables for music
int currentP = 100;
int nextPTap = 3;

// Variables for motor
int tap_motor_flag = 0;
long previousMillisSpin = 0;   // will store last time motor was updated FOR SPIN
long previousMillisTap = 0;
long previousMillisSqueeze = 0;
long interval = 500;           // time that motor will buzz (milliseconds)

int i = 0;
int TapThreshold=150;
int SqueezeThreshold=100;

int state = 0;

volatile int interruptedflag =0;

//Initialize LED Strip
#define N_LEDS 30 // 5 meter reel @ 30 LEDs/m
#define SIZE 10
Adafruit_NeoPixel TapStrip = Adafruit_NeoPixel(N_LEDS, TapLEDPin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel SqueezeStrip = Adafruit_NeoPixel(N_LEDS, SqueezeLEDPin, NEO_GRB + NEO_KHZ800);

int Timer = 0;
bool stop = true;

void setup() {
  Serial.begin(9600); //baudrate
  //Initializing the LEDs
  pinMode(TapLEDPin, OUTPUT);
  pinMode(SqueezeLEDPin, OUTPUT);
  pinMode(SpinLEDPin, OUTPUT);
  pinMode(TapMotorPin, OUTPUT); //initialized motor for tap

  TapStrip.begin();
  SqueezeStrip.begin();
  
  player.begin();  //will initialize the hardware and set default mode to be normal.
  player.keyDisable();
  player.setPlayMode(PM_REPEAT_ONE); //set mode to repeat playing a song

  attachInterrupt(digitalPinToInterrupt(TapIntrptPin), Tap_ISR, HIGH); //initialized Tap Interrupt ISR
}

void loop(){
  TapReading = analogRead(TapSensorPin);    //Reading from tap sensor
  unsigned long currentMillisTap = millis();   //get time for this loop
//  Serial.print("\n hai");
  Serial.print(TapReading);
  Serial.print("\n");

  // ------------------- STATE TRANSITIONS -----------------------
  if((state == 0) && (TapReading < PURPLE)){
    state = 1;
    tap_motor_flag = 0;
  }
  else if((state == 1) && (TapReading < YELLOW)){
    state = 2;
    tap_motor_flag = 0;
  }
  else if((state == 2) && (TapReading < BLUE)){
    state = 3;
    tap_motor_flag = 0;
  }
  else if((state == 3) && (TapReading > BLUE)){
    state = 4;
    tap_motor_flag = 0;
  }
  else if((state == 4) && (TapReading > YELLOW)){
    state = 5;
    tap_motor_flag = 0;
  }
  else if((state == 5) && (TapReading > PURPLE)){
    state = 0;
    tap_motor_flag = 0;
  }
  else if((state == 1) && (TapReading > PURPLE)){
    state = 0;
    tap_motor_flag = 0;
  }
  else if((state == 2) && (TapReading > YELLOW)){
    state = 5;
    tap_motor_flag = 0;
  }
  else if((state == 4) && (TapReading < BLUE)){
    state = 3;
    tap_motor_flag = 0;
  }
  else if((state == 5) && (TapReading < YELLOW)){
    state = 2;
    tap_motor_flag = 0;
  }
  // we should be having a default state here but what/how?
  
  // ------------------- STATE TRANSITIONS --------------------

  // ------------------- OUTPUT LOGIC ----------------------------
  if(state == 0){
    //reset everything 
    stop = true; //stops the timer

    analogWrite(TapMotorPin, 0);
    Serial.print("tap sensor not triggered \n");  
    
    for(int i = 0;i < N_LEDS; i++) {
      TapStrip.setPixelColor(i, TapStrip.Color(0, 0, 0)); //Off
    }
    TapStrip.show(); //Sends updated pixel color to hardware
    
    nextPTap = 100; //do not play any note. 100 was arbitrary choice
    digitalWrite(TapIntrptPin, LOW);

  }
  else if(state == 1){
    stop = false; //starts the timer
    for(int i=0;i <N_LEDS;i++) {
      TapStrip.setPixelColor(i, TapStrip.Color(255,0,255)); //purple
    }
    TapStrip.show(); //Sends updated pixel color to hardware
    nextPTap = 2;
    // --------------------- MOTOR LOGIC -----------------------
    if(tap_motor_flag == 1){ //check if sensor was also triggered last loop
        // sensor was pressed last loop, so check motor time. do nothing if motor time is less
        if((currentMillisTap - previousMillisTap) > interval){ 
          // motor interval has passed, so need to turn off motor
          analogWrite(TapMotorPin, 0);
        }
    } 
    else{ //tap_motor_flag == 0 
      //sensor was not pressed last loop, which means this is the first "tap"
      //so motor must be turned on and motor timer reset
      previousMillisTap = currentMillisTap;   
      analogWrite(TapMotorPin, 255); // PWM
      tap_motor_flag = 1; //store tap_flag status for next loop
    }
    // --------------------- MOTOR LOGIC -----------------------
  }
  else if(state == 2){
    stop = false; //starts the timer
    for(int i=0;i <N_LEDS;i++) {
      TapStrip.setPixelColor(i, TapStrip.Color(255,255,0)); //yellow
    }
    TapStrip.show(); //Sends updated pixel color to hardware
    nextPTap = 1; 
    // --------------------- MOTOR LOGIC -----------------------
    if(tap_motor_flag == 1){ //check if sensor was also triggered last loop
        // sensor was pressed last loop, so check motor time. do nothing if motor time is less
        if((currentMillisTap - previousMillisTap) > interval){ 
          // motor interval has passed, so need to turn off motor
          analogWrite(TapMotorPin, 0);
        }
    } 
    else{ //tap_motor_flag == 0 
      //sensor was not pressed last loop, which means this is the first "tap"
      //so motor must be turned on and motor timer reset
      previousMillisTap = currentMillisTap;   
      analogWrite(TapMotorPin, 255); // PWM
      tap_motor_flag = 1; //store tap_flag status for next loop
    }
    // --------------------- MOTOR LOGIC ----------------------- 
  }
  else if(state == 3){
    stop = false; //starts the timer
    for(int i=0;i <N_LEDS;i++) {
      TapStrip.setPixelColor(i, TapStrip.Color(0,255,255)); //blue
    }
    TapStrip.show(); //Sends updated pixel color to hardware
    nextPTap = 0; 
    // --------------------- MOTOR LOGIC -----------------------
    if(tap_motor_flag == 1){ //check if sensor was also triggered last loop
        // sensor was pressed last loop, so check motor time. do nothing if motor time is less
        if((currentMillisTap - previousMillisTap) > interval){ 
          // motor interval has passed, so need to turn off motor
          analogWrite(TapMotorPin, 0);
        }
    } 
    else{ //tap_motor_flag == 0 
      //sensor was not pressed last loop, which means this is the first "tap"
      //so motor must be turned on and motor timer reset
      previousMillisTap = currentMillisTap;   
      analogWrite(TapMotorPin, 255); // PWM
      tap_motor_flag = 1; //store tap_flag status for next loop
    }
    // --------------------- MOTOR LOGIC -----------------------
  }
  else if(state == 4){
    for(int i=0;i <N_LEDS;i++) {
      TapStrip.setPixelColor(i, TapStrip.Color(255,255,0)); //yellow
    }
    TapStrip.show(); //Sends updated pixel color to hardware
    nextPTap = 100; 
    analogWrite(TapMotorPin, 0);
      
  }
  else if(state == 5){
    for(int i=0;i <N_LEDS;i++) {
      TapStrip.setPixelColor(i, TapStrip.Color(255,0,255)); //purple
    }
    TapStrip.show(); //Sends updated pixel color to hardware
    nextPTap = 100; 
    analogWrite(TapMotorPin, 0);
  }
  // ------------------- OUTPUT LOGIC ----------------------------
 
  // ------------------- MUSIC LOGIC ----------------------------
  if(nextPTap != currentP && nextPTap - currentP < 0) {
      playNote(nextPTap);
    }
    currentP = nextPTap;

    stop = true;
  // ------------------- MUSIC LOGIC -------------------
  // ------------------- Interrupt LOGIC -------------------

  if(TapReading < PURPLE){  //Checks if there is any pressure detected by sensor
    digitalWrite(TapIntrptPin, HIGH);  // If it's within the thresholds, set the TapInterruptPin to logic HIGH  
  }
  // ------------------- Interrupt LOGIC -------------------
}


//------------------- TAP ISR ---------------------------------  
void Tap_ISR(){   //The program has been interrupted
  digitalWrite(TapIntrptPin, LOW); //Reset interrupt pin to zero
  interruptedflag=1;  //Turn interrupt flag to enable the state machine
}

//------------------- TAP ISR ---------------------------------


/*
 * Input:  
 * 
 */
void playNote(int mode) {
  //player.opStop();
  switch(mode) {
    case 0:
      player.playOne("Piano_B.wav");
      player.play();
      break;
    case 1:
      player.playOne("Piano_C.wav");
      player.play();
      break;
    case 2:
      player.playOne("Piano_D.wav");
      player.play();
      break;
    case 3:
      player.playOne("Piano_E.wav");
      player.play();
      break;  
    case 4:
      player.playOne("Piano_Eb.wav");
      player.play();
      break;
    case 5:
      player.playOne("Piano_Bb.wav");
      player.play();
      break;
    default:
      break;
    }
    delay(100);
    player.opStop();
}

void off() {
  analogWrite(3, 0);
  analogWrite(4, 0);
  analogWrite(5, 0);
}
