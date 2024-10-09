/**
 * Written by     : 	Anoof Chappangathil
 * Email          :   anoofdoc@gmail.com
 * Phone          :   +971 50 762 1347
 * Company        :   Interactive Technical Service LLC
 * GitHub         :   https://github.com/anoofc/
 * LinkedIn       :   https://www.linkedin.com/in/anoofc/
 * Date           : 	09/10/2024
 * Description    :   Staircase Lighting System using Arduino Mega 2560, 
 */


#define DEBUG             1

#define SENSOR1           30
#define SENSOR2           31

#define LEDPIN1           48
#define LEDPIN2           49

#define DEBOUNCE_DELAY    500
#define STRIP_CLEAR_DELAY 5000
#define STEP_UPDATE_DELAY 1000

#define NUM_OF_STEPS      16
#define STEP_LENGTH       1 // 28

#define RED               255, 0, 0
#define WHITE             255, 0, 0

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(NUM_OF_STEPS*STEP_LENGTH, LEDPIN1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(NUM_OF_STEPS*STEP_LENGTH, LEDPIN2, NEO_GRB + NEO_KHZ800);

uint32_t sensorUpdateMillis = 0;
uint32_t stepUpdateMillis = 0;
uint32_t stripClearMillis = 0;

uint8_t step_count = 1;
bool sequence_active = 0;

void io_Setup() {
  Serial.println("Setting up IO");
  pinMode(SENSOR1, INPUT_PULLUP);
  pinMode(SENSOR2, INPUT_PULLUP);

  strip1.begin(); strip1.clear(); strip1.setBrightness(255); strip1.show();
  strip2.begin(); strip2.clear(); strip2.setBrightness(255); strip2.show();
}

void showStep(int step){
  for (int i = ((step*STEP_LENGTH)-STEP_LENGTH); i < step*STEP_LENGTH; i++) {
    strip1.setPixelColor(i, WHITE);
    strip2.setPixelColor(i, WHITE);
  }
  strip1.show();
  strip2.show();
  if (DEBUG) {Serial.print("Showing Step: "); Serial.println(step);}
}

void clearStep(int step){
  for (int i = ((step*STEP_LENGTH)-STEP_LENGTH); i < step*STEP_LENGTH; i++) {
    strip1.setPixelColor(i, 0);
    strip2.setPixelColor(i, 0);
  }
  strip1.show();
  strip2.show();
  if (DEBUG) {Serial.print("Clearing Step: "); Serial.println(step);}
}

void stepUpSequence(){
  if (step_count <= NUM_OF_STEPS  && sequence_active){
    if (millis() - stepUpdateMillis < STEP_UPDATE_DELAY){ return; }
    if (DEBUG) {Serial.print("Step Count: "); Serial.println(step_count);}
    showStep(step_count);
    stepUpdateMillis = millis();
    step_count++;
    if (step_count > NUM_OF_STEPS){
      if (DEBUG) {Serial.println("UP Sequence Completed");}
      stripClearMillis = millis();
    }
  }
}

void clearSequence(){
  if (sequence_active == 1 && step_count > NUM_OF_STEPS){
    if (millis() - stripClearMillis < STRIP_CLEAR_DELAY){ return; }
    sequence_active = 0;
    if (DEBUG) {Serial.println("Clearing Strips");}
    strip1.clear(); strip1.show();
    strip2.clear(); strip2.show();
    step_count = 1;
  }
}

void readSensors(){
  if (digitalRead(SENSOR1) == LOW){
    if (millis() - sensorUpdateMillis < DEBOUNCE_DELAY){ return; }
      sensorUpdateMillis = millis();
      if (DEBUG) {Serial.println("Sensor 1 Triggered");}
      stripClearMillis = millis();
      sequence_active = 1;
  }
  if (digitalRead(SENSOR2) == LOW){
    if (millis() - sensorUpdateMillis < DEBOUNCE_DELAY){ return; }
      sensorUpdateMillis = millis();
      if (DEBUG) {Serial.println("Sensor 2 Triggered");}
      // showStep(1);
  }
}

void readSerial(){
  if (Serial.available() > 0) {
    char incoming = Serial.read();
    if (incoming == 'A'){
      
      sequence_active = 1; if (DEBUG) {Serial.println("Sequence Status : ACTIVE");}
      // TODO: Call Function to start the sequence.
    }
    if (incoming == 'B'){
      sequence_active = 0;
      // TODO: Call Function to Clear the sequence.
    }
  }
}

void debugPins(){
  Serial.println("S1: " + String(digitalRead(SENSOR1)) + " \t S2: " + String(digitalRead(SENSOR2)));
}

void setup() {
  Serial.begin(9600);
  io_Setup();
}

void loop() {
  readSerial();
  readSensors();
  stepUpSequence();
  clearSequence();
  
}