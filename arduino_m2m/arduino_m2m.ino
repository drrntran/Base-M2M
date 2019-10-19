
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

//#define BLUE 800//600
//#define YELLOW 900//700
//#define PURPLE 1000//880

/* TAP PINS
 *  MOTOR - 6
 *  LED - 9
 *  SENSOR - A2
*/

/* SQUEEZE PINS
 *  MOTOR - 7
 *  LED - 4
 *  SENSOR - A0
*/


/* SPIN PINS
 *  MOTOR - N/A
 *  LED - 8
 *  SENSOR - A3
*/

// POT Sensor - A9

#define TapMotorPin 6//8
#define SqueezeMotorPin 7
#define SpinMotorPin //8

#define TapLEDPin 9//4
#define SqueezeLEDPin 4 // DOESNT WORK???
#define SpinLEDPin 8

// Pin Interrupts
#define TapIntrptPin 2
#define SqueezeIntrptPin 0//3
#define SpinIntrptPin 1//2
//A1 730, A4 160-190, A5 160-210
int TapSensorPin = A2;//A2; //analog pin 0
int SqueezeSensorPin = A0; //analog pin 1
int SpinSensorEgg = A3; //analog pin 3
int PotPin = A9;

// Stores reading from velostat sensor
int SpinReading = 0;
volatile int TapReading = 0;
volatile int SqueezeReading = 0;
volatile int PotReading = 0;
volatile int PrevPotReading = 0;

// Variables for music
int currentPTap = 100;
int nextPTap = 3;
int currentPSqueeze = 100;
int nextPSqueeze = 3;

// Variables for motor
int tap_motor_flag = 0;
int squeeze_motor_flag = 0;
unsigned long currentMillis = 0;
long previousMillisSpin = 0;   // will store last time motor was updated FOR SPIN
long previousMillisTap = 0;
long previousMillisSqueeze = 0;
long interval = 500;           // time that motor will buzz (milliseconds)

int i = 0;
int TapThreshold=150;
int SqueezeThreshold=100;

int TapState = 0;
int old_TapState = 0;
int SqueezeState = 0;
int old_SqueezeState = 0;

int pin = 0;

float BLUE = 850;
float YELLOW = 875;
float PURPLE = 900;

float sqBLUE = 700;
float sqYELLOW = 800;
float sqPURPLE = 900;

volatile int interruptedflag =0;

char* button1 = "button-1.wav";
char* button2 = "button-2.wav";
char* button3 = "button-3.wav";

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
  //Initializing the Motors
  pinMode(TapMotorPin, OUTPUT); 
  pinMode(SqueezeMotorPin, OUTPUT);
  //Initializing Inputs (???)
  pinMode(TapSensorPin, INPUT);
  pinMode(SqueezeSensorPin, INPUT);
  pinMode(SpinSensorEgg, INPUT);
  pinMode(PotPin, INPUT);

 
  TapStrip.begin();
  SqueezeStrip.begin();

  player.begin();  //will initialize the hardware and set default mode to be normal.
  player.keyDisable();
  //player.setPlayMode(PM_REPEAT_ONE); //set mode to repeat playing a song
  //player.setVolume(0xfe);

  player.addToPlaylist("Piano_B.wav");
  player.addToPlaylist("Piano_C.wav");
  player.addToPlaylist("Piano_D.wav");
  
  
  player1.begin();  //will initialize the hardware and set default mode to be normal.
  player1.keyDisable();
 // player1.setPlayMode(PM_REPEAT_ONE);
  //player1.setVolume(0xfe);

  player1.addToPlaylist(button1);
  player1.addToPlaylist(button2);
  player1.addToPlaylist(button3);

  //attachInterrupt(digitalPinToInterrupt(TapIntrptPin), Tap_ISR, HIGH); //initialized Tap Interrupt ISR
  //attachInterrupt(digitalPinToInterrupt(SqueezeIntrptPin), Squeeze_ISR, HIGH); //initialized Squeeze Interrupt ISR
}

void loop(){ //havent implemented interrupt for squeeze
  //-------------------------POT LEVEL------------------------
  PotReading = analogRead(PotPin);

  if(PrevPotReading != PotReading){       
  PrevPotReading = PotReading;
  
  PURPLE = 800.0 - (PotReading/1023.0)*50.0;  //750
  YELLOW = 750.0 - (PotReading/1023.0)*50.0;  //700
  BLUE = 700.0 - (PotReading/1023.0)*75.0;    //625
  
  sqPURPLE = 900.0 - (PotReading/1023.0)*100.0; //800
  sqYELLOW = 800.0 - (PotReading/1023.0)*250.0; //550
  sqBLUE = 700.0 - (PotReading/1023.0)*350.0;   //350
  }
  
  //Serial.print(PotReading);
  //Serial.print(",");
  //Serial.print(BLUE);
  //Serial.print("\n");
  
  TapReading = analogRead(TapSensorPin);    //Reading from tap sensor
  SqueezeReading = analogRead(SqueezeSensorPin);    //Reading from squeeze sensor
  /*Serial.print(TapReading);
  Serial.print("\n");*/
  //Serial.print(SqueezeReading);
  //Serial.print("\n");
  Serial.print(TapReading);
  Serial.print(",");
  Serial.println(SqueezeReading);
  
  currentMillis = millis();   //get time for this loop

  //---------------------------CHANGES------------------------
  TapState = updateState(TapReading, TapState);
  SqueezeState = updateSqueezeState(SqueezeReading, SqueezeState);

  if (TapState != old_TapState) { //updates flag to allow motor to run only when state has changed
    tap_motor_flag = 0;
  }
  if (SqueezeState != old_SqueezeState) { //updates flag to allow motor to run only when state has changed
    squeeze_motor_flag = 0;
  }
  //Tap(true);
  if (TapReading <= SqueezeReading) { //output
    Tap(true);//function for output with only tap make noise
    Squeeze(false);
  }
  else {
    Tap(false);//function for output with only squeeze make noise
    Squeeze(true);
  }

/*  if (old_TapState != TapState && TapState <= 3 && TapState != 0) {
    Tap(true);//function for output with only tap make noise
    Squeeze(false);
  }
  else {
    Tap(false);//function for output with only squeeze make noise
    Squeeze(true);
  }
*/  
  old_TapState = TapState;
  old_SqueezeState = SqueezeState;
  //---------------------CHANGES-----------------------
}


//------------------- TAP ISR ---------------------------------
void Tap_ISR(){   //The program has been interrupted
  digitalWrite(TapIntrptPin, LOW); //Reset interrupt pin to zero
  interruptedflag=1;  //Turn interrupt flag to enable the state machine
}

//------------------- TAP ISR ---------------------------------

//-------------------CHANGES-----------------------------------
//------------------- SQUEEZE ISR ---------------------------------
void Squeeze_ISR(){   //The program has been interrupted
  digitalWrite(SqueezeIntrptPin, LOW); //Reset interrupt pin to zero
  interruptedflag=1;  //Turn interrupt flag to enable the state machine
}

//------------------- SQUEEZE ISR ---------------------------------
//-------------------CHANGES-----------------------------------
/*
 * Input:
 *
 */
void TapSound(int mode) {
  switch(mode) {
    case 0:
      Serial.print("Tap Case 0 Start \n");
      player1.playOne(button1);
      player1.play();
      Serial.print("Tap Case 0 End \n");
      break;
    case 1:
      Serial.print("Tap Case 1 Start \n");
      player1.playOne(button2);
      player1.play();
      Serial.print("Tap Case 1 End \n");
      break;
    case 2:
      Serial.print("Tap Case 2 Start \n");
      player1.playOne(button3);
      player1.play();
      Serial.print("Tap Case 2 End \n");
      break;
    default:
      break;
    }
    delay(100);
    player1.opStop();
}
void playNote(int mode) {
  //player.opStop();
  switch(mode) {
    case 0:
      Serial.print("Squueze Case 0 Start \n");
      player.playOne("Piano_B.wav");
      player.play();
      Serial.print("Squeeze Case 0 End \n");
      break;
    case 1:
      Serial.print("Squueze Case 1 Start \n");
      player.playOne("Piano_C.wav");
      player.play();
      Serial.print("Squeeze Case 1 End \n");
      break;
    case 2:
      Serial.print("Squeeze Case 2 Start \n");
      player.playOne("Piano_D.wav");
      player.play();
      Serial.print("Squeeze Case 2 End \n");
      break;
    case 3:
      Serial.print("Squeeze Case 3 Start \n");
      player.playOne("Piano_E.wav");
      player.play();
      Serial.print("Squeeze Case 3 End \n");
      break;
    case 4:
      Serial.print("Squeeze Case 4 Start \n");
      player.playOne("Piano_Eb.wav");
      player.play();
      Serial.print("Squeeze Case 4 End \n");
      break;
    case 5:
      Serial.print("Squeeze Case 5 Start \n");
      player.playOne("Piano_Bb.wav");
      player.play();
      Serial.print("Squeeze Case 5 End \n");
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

int updateState(int reading, int state) {
  // ------------------- STATE TRANSITIONS -----------------------
  if((state == 0) && (reading < PURPLE)){
    state = 1;
  }
  else if((state == 1) && (reading < YELLOW)){
    state = 2;
  }
  else if((state == 2) && (reading < BLUE)){
    state = 3;
  }
  else if((state == 3) && (reading > BLUE)){
    state = 4;
  }
  else if((state == 4) && (reading > YELLOW)){
    state = 5;
  }
  else if((state == 5) && (reading > PURPLE)){
    state = 0;
  }
  else if((state == 1) && (reading > PURPLE)){
    state = 0;
  }
  else if((state == 2) && (reading > YELLOW)){
    state = 5;
  }
  else if((state == 4) && (reading < BLUE)){
    state = 3;
  }
  else if((state == 5) && (reading < YELLOW)){
    state = 2;
  }
  // we should be having a default state here but what/how?

  // ------------------- STATE TRANSITIONS --------------------
  return state;
}

int updateSqueezeState(int reading, int state) {
  // ------------------- STATE TRANSITIONS -----------------------
  if((state == 0) && (reading < sqPURPLE)){
    state = 1;
  }
  else if((state == 1) && (reading < sqYELLOW)){
    state = 2;
  }
  else if((state == 2) && (reading < sqBLUE)){
    state = 3;
  }
  else if((state == 3) && (reading > sqBLUE)){
    state = 4;
  }
  else if((state == 4) && (reading > sqYELLOW)){
    state = 5;
  }
  else if((state == 5) && (reading > sqPURPLE)){
    state = 0;
  }
  else if((state == 1) && (reading > sqPURPLE)){
    state = 0;
  }
  else if((state == 2) && (reading > sqYELLOW)){
    state = 5;
  }
  else if((state == 4) && (reading < sqBLUE)){
    state = 3;
  }
  else if((state == 5) && (reading < sqYELLOW)){
    state = 2;
  }
  // we should be having a default state here but what/how?

  // ------------------- STATE TRANSITIONS --------------------
  return state;
}

void Tap(int noise) {
  // ------------------- OUTPUT LOGIC ----------------------------
  if(TapState == 0){
    //reset everything
    stop = true; //stops the timer

    analogWrite(TapMotorPin, 0);
    // Serial.print("tap sensor not triggered \n");

    for(int i = 0;i < N_LEDS; i++) {
      TapStrip.setPixelColor(i, TapStrip.Color(0, 0, 0)); //Off
    }
    TapStrip.show(); //Sends updated pixel color to hardware

    nextPTap = 100; //do not play any note. 100 was arbitrary choice
    digitalWrite(TapIntrptPin, LOW);

  }
  else if(TapState == 1){
    stop = false; //starts the timer
    for(int i=0;i <N_LEDS;i++) {
      TapStrip.setPixelColor(i, TapStrip.Color(255,0,255)); //purple
    }
    TapStrip.show(); //Sends updated pixel color to hardware
    nextPTap = 2;
    // --------------------- MOTOR LOGIC -----------------------
    if(tap_motor_flag == 1){ //check if sensor was also triggered last loop
        // sensor was pressed last loop, so check motor time. do nothing if motor time is less
        if((currentMillis - previousMillisTap) > interval){
          // motor interval has passed, so need to turn off motor
          analogWrite(TapMotorPin, 0);
        }
    }
    else{ //tap_motor_flag == 0
      //sensor was not pressed last loop, which means this is the first "tap"
      //so motor must be turned on and motor timer reset
      previousMillisTap = currentMillis;
      analogWrite(TapMotorPin, 255); // PWM
      tap_motor_flag = 1; //store tap_flag status for next loop
    }
    // --------------------- MOTOR LOGIC -----------------------
  }
  else if(TapState == 2){
    stop = false; //starts the timer
    for(int i=0;i <N_LEDS;i++) {
      TapStrip.setPixelColor(i, TapStrip.Color(255,255,0)); //yellow
    }
    TapStrip.show(); //Sends updated pixel color to hardware
    nextPTap = 1;
    // --------------------- MOTOR LOGIC -----------------------
    if(tap_motor_flag == 1){ //check if sensor was also triggered last loop
        // sensor was pressed last loop, so check motor time. do nothing if motor time is less
        if((currentMillis - previousMillisTap) > interval){
          // motor interval has passed, so need to turn off motor
          analogWrite(TapMotorPin, 0);
        }
    }
    else{ //tap_motor_flag == 0
      //sensor was not pressed last loop, which means this is the first "tap"
      //so motor must be turned on and motor timer reset
      previousMillisTap = currentMillis;
      analogWrite(TapMotorPin, 255); // PWM
      tap_motor_flag = 1; //store tap_flag status for next loop
    }
    // --------------------- MOTOR LOGIC -----------------------
  }
  else if(TapState == 3){
    stop = false; //starts the timer
    for(int i=0;i <N_LEDS;i++) {
      TapStrip.setPixelColor(i, TapStrip.Color(0,255,255)); //blue
    }
    TapStrip.show(); //Sends updated pixel color to hardware
    nextPTap = 0;
    // --------------------- MOTOR LOGIC -----------------------
    if(tap_motor_flag == 1){ //check if sensor was also triggered last loop
        // sensor was pressed last loop, so check motor time. do nothing if motor time is less
        if((currentMillis - previousMillisTap) > interval){
          // motor interval has passed, so need to turn off motor
          analogWrite(TapMotorPin, 0);
        }
    }
    else{ //tap_motor_flag == 0
      //sensor was not pressed last loop, which means this is the first "tap"
      //so motor must be turned on and motor timer reset
      previousMillisTap = currentMillis;
      analogWrite(TapMotorPin, 255); // PWM
      tap_motor_flag = 1; //store tap_flag status for next loop
    }
    // --------------------- MOTOR LOGIC -----------------------
  }
  else if(TapState == 4){
    for(int i=0;i <N_LEDS;i++) {
      TapStrip.setPixelColor(i, TapStrip.Color(255,255,0)); //yellow
    }
    TapStrip.show(); //Sends updated pixel color to hardware
    nextPTap = 100;
    analogWrite(TapMotorPin, 0);

  }
  else if(TapState == 5){
    for(int i=0;i <N_LEDS;i++) {
      TapStrip.setPixelColor(i, TapStrip.Color(255,0,255)); //purple
    }
    TapStrip.show(); //Sends updated pixel color to hardware
    nextPTap = 100;
    analogWrite(TapMotorPin, 0);
  }
  // ------------------- OUTPUT LOGIC ----------------------------
  // ------------------- MUSIC LOGIC ----------------------------
  if(nextPTap != currentPTap && nextPTap - currentPTap < 0 && noise == true) {
      TapSound(nextPTap);
    }
    currentPTap = nextPTap;

    stop = true;  
  // ------------------- MUSIC LOGIC -------------------
  // ------------------- Interrupt LOGIC -------------------

  /*if(TapReading < PURPLE){  //Checks if there is any pressure detected by sensor
    digitalWrite(TapIntrptPin, HIGH);  // If it's within the thresholds, set the TapInterruptPin to logic HIGH
  }*/
}

void Squeeze(int noise) {
  // ------------------- OUTPUT LOGIC ----------------------------
  if(SqueezeState == 0){
    //reset everything
    stop = true; //stops the timer

    analogWrite(SqueezeMotorPin, 0);
    //Serial.print("Squeeze sensor not triggered \n");

    for(int i = 0;i < N_LEDS; i++) {
      SqueezeStrip.setPixelColor(i, SqueezeStrip.Color(0, 0, 0)); //Off
    }
    SqueezeStrip.show(); //Sends updated pixel color to hardware

    nextPSqueeze = 100; //do not play any note. 100 was arbitrary choice
    digitalWrite(SqueezeIntrptPin, LOW);

  }
  else if(SqueezeState == 1){
    stop = false; //starts the timer
    for(int i=0;i <N_LEDS;i++) {
      SqueezeStrip.setPixelColor(i, SqueezeStrip.Color(255,0,255)); //purple
    }
    SqueezeStrip.show(); //Sends updated pixel color to hardware
    nextPSqueeze = 2;
    // --------------------- MOTOR LOGIC -----------------------
    if(squeeze_motor_flag == 1){ //check if sensor was also triggered last loop
        // sensor was pressed last loop, so check motor time. do nothing if motor time is less
        if((currentMillis - previousMillisSqueeze) > interval){
          // motor interval has passed, so need to turn off motor
          analogWrite(SqueezeMotorPin, 0);
        }
    }
    else{ //Squeeze_motor_flag == 0
      //sensor was not pressed last loop, which means this is the first "Squeeze"
      //so motor must be turned on and motor timer reset
      previousMillisSqueeze = currentMillis;
      analogWrite(SqueezeMotorPin, 255); // PWM
      squeeze_motor_flag = 1; //store squeeze_flag status for next loop
    }
    // --------------------- MOTOR LOGIC -----------------------
  }
  else if(SqueezeState == 2){
    stop = false; //starts the timer
    for(int i=0;i <N_LEDS;i++) {
      SqueezeStrip.setPixelColor(i, SqueezeStrip.Color(255,255,0)); //yellow
    }
    SqueezeStrip.show(); //Sends updated pixel color to hardware
    nextPSqueeze = 1;
    // --------------------- MOTOR LOGIC -----------------------
    if(squeeze_motor_flag == 1){ //check if sensor was also triggered last loop
        // sensor was pressed last loop, so check motor time. do nothing if motor time is less
        if((currentMillis - previousMillisSqueeze) > interval){
          // motor interval has passed, so need to turn off motor
          analogWrite(SqueezeMotorPin, 0);
        }
    }
    else{ //squeeze_motor_flag == 0
      //sensor was not pressed last loop, which means this is the first "squeeze"
      //so motor must be turned on and motor timer reset
      previousMillisSqueeze = currentMillis;
      analogWrite(SqueezeMotorPin, 255); // PWM
      squeeze_motor_flag = 1; //store squeeze_flag status for next loop
    }
    // --------------------- MOTOR LOGIC -----------------------
  }
  else if(SqueezeState == 3){
    stop = false; //starts the timer
    for(int i=0;i <N_LEDS;i++) {
      SqueezeStrip.setPixelColor(i, SqueezeStrip.Color(0,255,255)); //blue
    }
    SqueezeStrip.show(); //Sends updated pixel color to hardware
    nextPSqueeze = 0;
    // --------------------- MOTOR LOGIC -----------------------
    if(squeeze_motor_flag == 1){ //check if sensor was also triggered last loop
        // sensor was pressed last loop, so check motor time. do nothing if motor time is less
        if((currentMillis - previousMillisSqueeze) > interval){
          // motor interval has passed, so need to turn off motor
          analogWrite(SqueezeMotorPin, 0);
        }
    }
    else{ //squeeze_motor_flag == 0
      //sensor was not pressed last loop, which means this is the first "squeeze"
      //so motor must be turned on and motor timer reset
      previousMillisSqueeze = currentMillis;
      analogWrite(SqueezeMotorPin, 255); // PWM
      squeeze_motor_flag = 1; //store squeeze_flag status for next loop
    }
    // --------------------- MOTOR LOGIC -----------------------
  }
  else if(SqueezeState == 4){
    for(int i=0;i <N_LEDS;i++) {
      SqueezeStrip.setPixelColor(i, SqueezeStrip.Color(255,255,0)); //yellow
    }
    SqueezeStrip.show(); //Sends updated pixel color to hardware
    nextPSqueeze = 100;
    analogWrite(SqueezeMotorPin, 0);

  }
  else if(SqueezeState == 5){
    for(int i=0;i <N_LEDS;i++) {
      SqueezeStrip.setPixelColor(i, SqueezeStrip.Color(255,0,255)); //purple
    }
    SqueezeStrip.show(); //Sends updated pixel color to hardware
    nextPSqueeze = 100;
    analogWrite(SqueezeMotorPin, 0);
  }
  // ------------------- OUTPUT LOGIC ----------------------------
  // ------------------- MUSIC LOGIC ----------------------------
    if(nextPSqueeze != currentPSqueeze && nextPSqueeze - currentPSqueeze < 0 && noise == true) {
      playNote(nextPSqueeze);
    }
    currentPSqueeze = nextPSqueeze;

    stop = true; 
  // ------------------- MUSIC LOGIC -------------------
  // ------------------- Interrupt LOGIC -------------------
  if(SqueezeReading < PURPLE){  //Checks if there is any pressure detected by sensor
    digitalWrite(SqueezeIntrptPin, HIGH);  // If it's within the thresholds, set the SqueezeInterruptPin to logic HIGH
  }
}

/*void Squeeze(int noise) {
  // ------------------- OUTPUT LOGIC ----------------------------
  if(SqueezeState == 0){
    //reset everything
    stop = true; //stops the timer

    analogWrite(SqueezeMotorPin, 0);
    Serial.print("squeeze sensor not triggered \n");

    for(int i = 0;i < N_LEDS; i++) {
      SqueezeStrip.setPixelColor(i, SqueezeStrip.Color(0, 0, 0)); //Off
    }
    SqueezeStrip.show(); //Sends updated pixel color to hardware

    nextPSqueeze = 100; //do not play any note. 100 was arbitrary choice
    digitalWrite(SqueezeIntrptPin, LOW);

  }
  else if(SqueezeState == 1){
    stop = false; //starts the timer
    for(int i=0;i <N_LEDS;i++) {
      SqueezeStrip.setPixelColor(i, SqueezeStrip.Color(255,0,255)); //purple
    }
    SqueezeStrip.show(); //Sends updated pixel color to hardware
    nextPSqueeze = 2;
    // --------------------- MOTOR LOGIC -----------------------
    if(squeeze_motor_flag == 1){ //check if sensor was also triggered last loop
        // sensor was pressed last loop, so check motor time. do nothing if motor time is less
        if((currentMillis - previousMillisSqueeze) > interval){
          // motor interval has passed, so need to turn off motor
          analogWrite(SqueezeMotorPin, 0);
        }
    }
    else{ //tap_motor_flag == 0
      //sensor was not pressed last loop, which means this is the first "tap"
      //so motor must be turned on and motor timer reset
      previousMillisSqueeze = currentMillis;
      analogWrite(SqueezeMotorPin, 255); // PWM
      squeeze_motor_flag = 1; //store tap_flag status for next loop
    }
    // --------------------- MOTOR LOGIC -----------------------
  }
  else if(SqueezeState == 2){
    stop = false; //starts the timer
    for(int i=0;i <N_LEDS;i++) {
      SqueezeStrip.setPixelColor(i, SqueezeStrip.Color(255,255,0)); //yellow
    }
    SqueezeStrip.show(); //Sends updated pixel color to hardware
    nextPSqueeze = 1;
    // --------------------- MOTOR LOGIC -----------------------
    if(squeeze_motor_flag == 1){ //check if sensor was also triggered last loop
        // sensor was pressed last loop, so check motor time. do nothing if motor time is less
        if((currentMillis - previousMillisSqueeze) > interval){
          // motor interval has passed, so need to turn off motor
          analogWrite(SqueezeMotorPin, 0);
        }
    }
    else{ //tap_motor_flag == 0
      //sensor was not pressed last loop, which means this is the first "tap"
      //so motor must be turned on and motor timer reset
      previousMillisSqueeze = currentMillis;
      analogWrite(SqueezeMotorPin, 255); // PWM
      squeeze_motor_flag = 1; //store tap_flag status for next loop
    }
    // --------------------- MOTOR LOGIC -----------------------
  }
  else if(SqueezeState == 3){
    stop = false; //starts the timer
    for(int i=0;i <N_LEDS;i++) {
      SqueezeStrip.setPixelColor(i, SqueezeStrip.Color(0,255,255)); //blue
    }
    SqueezeStrip.show(); //Sends updated pixel color to hardware
    nextPSqueeze = 0;
    // --------------------- MOTOR LOGIC -----------------------
    if(squeeze_motor_flag == 1){ //check if sensor was also triggered last loop
        // sensor was pressed last loop, so check motor time. do nothing if motor time is less
        if((currentMillis - previousMillisSqueeze) > interval){
          // motor interval has passed, so need to turn off motor
          analogWrite(SqueezeMotorPin, 0);
        }
    }
    else{ //tap_motor_flag == 0
      //sensor was not pressed last loop, which means this is the first "tap"
      //so motor must be turned on and motor timer reset
      previousMillisSqueeze = currentMillis;
      analogWrite(SqueezeMotorPin, 255); // PWM
      squeeze_motor_flag = 1; //store tap_flag status for next loop
    }
    // --------------------- MOTOR LOGIC -----------------------
  }
  else if(SqueezeState == 4){
    for(int i=0;i <N_LEDS;i++) {
      SqueezeStrip.setPixelColor(i, SqueezeStrip.Color(255,255,0)); //yellow
    }
    SqueezeStrip.show(); //Sends updated pixel color to hardware
    nextPSqueeze = 100;
    analogWrite(SqueezeMotorPin, 0);

  }
  else if(SqueezeState == 5){
    for(int i=0;i <N_LEDS;i++) {
      SqueezeStrip.setPixelColor(i, SqueezeStrip.Color(255,0,255)); //purple
    }
    SqueezeStrip.show(); //Sends updated pixel color to hardware
    nextPSqueeze = 100;
    analogWrite(SqueezeMotorPin, 0);
  }
  // ------------------- OUTPUT LOGIC ----------------------------

  // ------------------- MUSIC LOGIC ----------------------------
  if(nextPSqueeze != currentPSqueeze && nextPSqueeze - currentPSqueeze < 0 && noise == true) {
      playNote(nextPSqueeze);
    }
    currentPSqueeze = nextPSqueeze;

    stop = true;
  // ------------------- MUSIC LOGIC -------------------
  // ------------------- Interrupt LOGIC -------------------

  if(SqueezeReading < PURPLE){  //Checks if there is any pressure detected by sensor
    digitalWrite(SqueezeIntrptPin, HIGH);  // If it's within the thresholds, set the SqueezeInterruptPin to logic HIGH
  }
}
//yellow - PWR(5V), black - ground, led - blue, purple - motor, grey - velo (sensor analog)*/
