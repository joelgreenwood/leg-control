/*
PID control of single joint
attempting to get joystick and directional control (forwards and reverse) working with PID
*/

#include <PID_v1.h>
#include <Bounce2.h>
#include <ADC.h>

//#define DEBUG 1

#define deadman_pin 25 // 5 on test, 25 on bot - Stokes board
Bounce deadmanBounce = Bounce(deadman_pin, 20);

//*********************************
// Major pin changes for joint and test changes
#define enable_pin 2 // for the motor driver board
#define joystick_pin 31 // A2 for test - 31 for bot --joystick input output
#define hip_pot_pin 20 // A3 for test, 20 for bot --the photo sensitive resistor input - pretending to be the string pot 
#define forward_hip_pin 9 // output pin for forward direction - in this case an LED but next it will be the valve
#define reverse_hip_pin 10 // output pin for reverse 
#define max_hip_sensor_value 2896 // This is the maximum read of the string pot - in this case the photo resistor.  I will use the same numbers for forward and reverse to make it easier to convert to the string pot later.  
#define min_hip_sensor_value 376 // This is the minimum read of the string pot - in this case the photo resistor. 
//*********************************
#define setpoint_deadband 0 //This is the difference acceptable between the setpoint (joystick) position and the input (sensor readout).
#define PWM_deadband_percent 30 //percent of PWM range needed to crack the valve
#define pwm_govenor 100 //percent of maximum output

//setup for ADC averaging
#define N_AVG 8
#define ADC_RES 12 //bits of resolution for ADC
#define DAC_PWM_RES 12 // bits of resolution for PWM output
ADC *adc = new ADC(); // adc object

int ADC_num_of_bytes; // (pow(2, ADC_RES) - 1) --holds the (zero indexed) number of bytes for the ADC input 
int half_bytes;
long PWM_deadband = 0;
int pwm_output = 0;
unsigned long max_PWM_output; // This caps the PWM output so we don't blow things up. Today... 
int DAC_num_of_bytes; // (pow(2, DAC_PWM_RES) - 1)  --holds the (zero indexed) number of bytes for the PWM output
double hip_pot_value; // reading from the photo resistor
double joystick_val; // value of the joystick

//PID variables - Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=1, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
const int sampleRate = 1;
const int serialPing = 100; //interval in ms at which serial is sent and read
unsigned long now = 0;
unsigned long lastMessage = 0;

void setup()
{
  Serial.begin(9600);
  analogWriteResolution(DAC_PWM_RES);
  analogWriteFrequency(9, 11718); // 11718
  ADC_num_of_bytes = pow(2, ADC_RES) - 1;
  DAC_num_of_bytes = pow(2, DAC_PWM_RES) - 1;
  half_bytes = ADC_num_of_bytes/2;

  max_PWM_output = ADC_num_of_bytes * ((float)pwm_govenor/100);
  PWM_deadband = ADC_num_of_bytes * ((float)PWM_deadband_percent/100);

  pinMode(deadman_pin, INPUT);
  pinMode(enable_pin, OUTPUT);
  pinMode(joystick_pin, INPUT);
  pinMode(hip_pot_value, INPUT);

  // setup ADC_0 & ADC_1
    adc->setAveraging(N_AVG);
    adc->setResolution(ADC_RES);
    adc->setConversionSpeed(ADC_VERY_LOW_SPEED);
    adc->setSamplingSpeed(ADC_VERY_LOW_SPEED);

    adc->setAveraging(N_AVG, ADC_1);
    adc->setResolution(ADC_RES, ADC_1);
    adc->setConversionSpeed(ADC_VERY_LOW_SPEED, ADC_1);
    adc->setSamplingSpeed(ADC_VERY_LOW_SPEED, ADC_1);

    delay(500);

  myPID.SetOutputLimits(-ADC_num_of_bytes, ADC_num_of_bytes);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(sampleRate);
  
  delay(2000);
  Serial.println("Begin");
  lastMessage = millis();
}

void loop()
{
  deadmanBounce.update(); //update the deadman

  Setpoint = adc->analogRead(joystick_pin); //read in setoint from joystick 

  hip_pot_value = adc->analogRead(hip_pot_pin);  //read the position of the joint sensor

  //The input needs to map to the setpoint range for the PID output to make sense.  So I scale the known range of the sensor to the known range of the joystick
  Input = map(hip_pot_value, min_hip_sensor_value, max_hip_sensor_value, 0, ADC_num_of_bytes);   

  myPID.Compute(); //update the PID calculation based on the setpoint and input

  if (deadmanBounce.read() == 1) {  //if the deadman is hot - then write the scaled output to the valves.
    digitalWrite(enable_pin, HIGH); //enable the driver board
    if (Output > setpoint_deadband) { //If the output is positive then drive the joint forward or out
      pwm_output = map(Output, 0, ADC_num_of_bytes, PWM_deadband, max_PWM_output); // Map the output to the range of the PWM output - deadband to max (either desired or set)
      analogWrite(forward_hip_pin, pwm_output); //write the command 
      analogWrite(reverse_hip_pin, 0); //make sure the other solenoid is off
    }
    else if (Output < -setpoint_deadband) { //If the output is negative then drive the joint in reverse or in.
      pwm_output = map(-Output, 0, ADC_num_of_bytes, PWM_deadband, max_PWM_output); //Invert the output to get rid of the negative and map it to the PWM range.
      analogWrite(reverse_hip_pin, pwm_output);  //write it
      analogWrite(forward_hip_pin, 0); //turn off the other valve
    }
    else {
      analogWrite(forward_hip_pin, 0); //If the output is within the setpoint deadband then write zeros to the valves to prevent jitter at a hold position
      analogWrite(reverse_hip_pin, 0);
      pwm_output = 0;
    }
  }
  else { // If the deadman is low then disable the driver board and write zeros to the PWM output.
    digitalWrite(enable_pin, LOW); 
    analogWrite(forward_hip_pin, 0);
    analogWrite(reverse_hip_pin, 0);
  }

  now = millis();
  if(now - lastMessage > serialPing) {
        //Serial.print("hipPot: ");
        //Serial.print((adc->analogRead(hip_pot_pin)));
        Serial.print("Setpoint:");
        Serial.print("\t");
        Serial.print(Setpoint);
        Serial.print("\t");
        Serial.print("Input:");
        Serial.print("\t");
        Serial.print(Input);
        Serial.print("\t");
        Serial.print("Output:");
        Serial.print("\t");
        Serial.print(Output);
        Serial.print("\t");
        Serial.print("PWM:");
        Serial.print("\t");
        Serial.print(pwm_output);
        Serial.print("\n");
        if (Serial.available() > 0) { // Read in new PID values.  Then need to be sent as floats - e.g. 0.1 instead of .1 
                for (int x = 0; x < 4; x++) {
                       switch (x) {
                           case 0:
                             Kp = Serial.parseFloat();
                             break;
                            case 1:
                             Ki = Serial.parseFloat();
                             break;
                            case 2:
                             Kd = Serial.parseFloat();
                             break;
                            case 3:
                             for (int y = Serial.available(); y == 0; y--) {
                                Serial.read();
                             }
                             break;
                       }
                }
                Serial.print(" Kp,Ki,Kd = ");
                Serial.print(Kp);
                Serial.print(",");
                Serial.print(Ki);
                Serial.print(",");
                Serial.println(Kd);
                myPID.SetTunings(Kp, Ki, Kd);        
        }
        lastMessage = now;
  }
}


