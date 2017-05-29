/*
  Reading a serial ASCII-encoded string.

 This sketch demonstrates the Serial parseInt() function.
 It looks for an ASCII string of comma-separated values.
 It parses them into ints, and uses those to fade an RGB LED.

 Circuit: Common-Cathode RGB LED wired like so:
 * Red anode: digital pin 3
 * Green anode: digital pin 5
 * Blue anode: digital pin 6
 * Cathode : GND

 created 13 Apr 2012
 by Tom Igoe
 
 modified 14 Mar 2016
 by Arturo Guadalupi

 This example code is in the public domain.
 */

int enable_pin_hip = 2; 

int hipPot;
int hipPot_pin = 20;

int hipPWM1 = 0;
int hipPWM2 = 0;

const int hipPWM1_forward = 9; // zlow forward -- right front leg
const int hipPWM2_reverse = 10; //

unsigned long now_ms = 0;
unsigned long lastMessage = 0;
const int serialPing = 1000;

void setup() {
  Serial.begin(9600);
  analogWriteResolution(12);
  analogReadResolution(12);
  analogWriteFrequency(3, 17000);
  analogWriteFrequency(5, 17000);
  pinMode(enable_pin_hip, OUTPUT);
  digitalWrite(enable_pin_hip, LOW);
  

}

void loop() {
  
  now_ms = millis();
  if (now_ms - lastMessage > serialPing) {
    Serial.println(analogRead(hipPot_pin));
    lastMessage = now_ms;
  }
  
  // if there's any serial available, read it:
  while (Serial.available() > 0) {
    hipPWM1 = Serial.parseInt();
    hipPWM2 = Serial.parseInt();
  
    if (Serial.read() == '\n') {
      analogWrite(hipPWM1_forward, hipPWM1);
      analogWrite(hipPWM2_reverse, hipPWM2);
      digitalWrite(enable_pin_hip, HIGH);
 
      Serial.print("pwm1: ");
      Serial.print(hipPWM1);
      Serial.print(" pwm2: ");
      Serial.println(hipPWM2);    
    }
  }
}








