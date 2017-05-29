/*
PID control of single joint
attempting to get joystick and directional control (forwards and reverse) working with PID
*/

#include <PID_v1.h>
#include <Bounce2.h>
#include <ADC.h>

//#define DEBUG 1

//**********  Major pin changes for joint and test changes
#define deadman_pin 25 // 5 on test, 25 on bot - Stokes board
#define enable_pin_thigh_knee 8 // for thigh and knee
#define enable_pin_hip 2 // for hip the motor driver board
#define joystick_pin_X 31 // A2 for test - 31 for bot --joystick input output
#define joystick_pin_Y 14
#define joystick_pin_Z 10
#define hip_pot_pin 20 // A3 for test, 20 for bot --the photo sensitive resistor input - pretending to be the string pot 
#define thigh_pot_pin 17 
#define knee_pot_pin 15
#define calf_pot_pin 16
#define forward_hip_pin 9 // output pin for forward direction - in this case an LED but next it will be the valve
#define reverse_hip_pin 10 // output pin for reverse 
#define down_thigh_pin 3
#define up_thigh_pin 4
#define out_knee_pin 6
#define in_knee_pin 5
//********** Sensor min and max values *****************
#define max_hip_sensor_value 2896 // This is the maximum read of the string pot - in this case the photo resistor.  I will use the same numbers for forward and reverse to make it easier to convert to the string pot later.  
#define min_hip_sensor_value 376 // This is the minimum read of the string pot - in this case the photo resistor. 
#define max_thigh_sensor_value 3668 //this is all the way down or extended
#define min_thigh_sensor_value 136 //this is fully reacted/up or in deadbug
#define max_knee_sensor_value 3736 //this is fully retracted 
#define min_knee_sensor_value 592 //this is fully extended
//********** Deadband and Max PWM output
#define setpoint_deadband 0 //This is the difference acceptable between the setpoint (joystick) position and the input (sensor readout).
#define PWM_deadband_percent 35 //percent of PWM range needed to crack the valve -- for now this is one number but I may break this out into individual joints later
#define pwm_govenor 100 //percent of maximum output

//setup for ADC averaging
#define N_AVG 8
#define ADC_RES 12 //bits of resolution for ADC
#define DAC_PWM_RES 12 // bits of resolution for PWM output
ADC *adc = new ADC(); // adc object

long ADC_num_of_bytes; // (pow(2, ADC_RES) - 1) --holds the (zero indexed) number of bytes for the ADC input 
long DAC_num_of_bytes; // (pow(2, DAC_PWM_RES) - 1)  --holds the (zero indexed) number of bytes for the PWM output
long PWM_deadband;
int pwm_output = 0;
unsigned long max_PWM_output; // This caps the PWM output so we don't blow things up. Today... 
double hip_pot_value; // reading from the photo resistor
double thigh_pot_value;
double knee_pot_value;

double joystick_val; // value of the joystick

Bounce deadmanBounce = Bounce(deadman_pin, 20); //debounce for the deadman pin

//PID variables - Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=7, Ki=1, Kd=0;  //for now I'm going with all the same values but I'm sure I'll need to break it out in the future
PID hipPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID thighPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID kneePID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int sampleRate = 1;
const int serialPing = 100; //interval in ms at which serial is sent and read
unsigned long now = 0;
unsigned long lastMessage = 0;

void setup()
{
  Serial.begin(9600);
  analogWriteResolution(DAC_PWM_RES);
  analogWriteFrequency(9, 11718); // 11718
  analogWriteFrequency(3, 11718); // 11718

  analogWrite(forward_hip_pin, 0);
  analogWrite(reverse_hip_pin, 0);
  analogWrite(down_thigh_pin, 0);
  analogWrite(up_thigh_pin, 0);
  analogWrite(in_knee_pin, 0);
  analogWrite(out_knee_pin, 0);

  ADC_num_of_bytes = pow(2, ADC_RES) - 1;
  DAC_num_of_bytes = pow(2, DAC_PWM_RES) - 1;

  max_PWM_output = ADC_num_of_bytes * ((float)pwm_govenor/100);
  PWM_deadband = ADC_num_of_bytes * ((float)PWM_deadband_percent/100);

  pinMode(deadman_pin, INPUT);
  pinMode(enable_pin_hip, OUTPUT);
  pinMode(enable_pin_thigh_knee, OUTPUT);
  pinMode(joystick_pin_X, INPUT);
  pinMode(hip_pot_value, INPUT);
  pinMode(thigh_pot_value, INPUT);
  pinMode(knee_pot_value, INPUT);
  digitalWrite(enable_pin_hip, LOW);
  digitalWrite(enable_pin_thigh_knee, LOW);

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

  hipPID.SetOutputLimits(-ADC_num_of_bytes, ADC_num_of_bytes);
  hipPID.SetMode(AUTOMATIC);
  hipPID.SetSampleTime(sampleRate);
  // kneePID.SetOutputLimits(-ADC_num_of_bytes, ADC_num_of_bytes);
  // kneePID.SetMode(AUTOMATIC);
  // kneePID.SetSampleTime(sampleRate);
  
  delay(2000);
  Serial.println("Begin");
  lastMessage = millis();
}

void loop()
{
  deadmanBounce.update(); //update the deadman

  //reverse mapping for knee
  Setpoint = map(adc->analogRead(knee_pot_pin),  0, ADC_num_of_bytes, ADC_num_of_bytes, 0); //read in setoint from joystick 

  hip_pot_value = adc->analogRead(hip_pot_pin);  //read the position of the joint sensor
  //thigh_pot_value = adc->analogRead(thigh_pot_pin); 
  //knee_pot_value = adc->analogRead(knee_pot_pin); 

  //Serial.println(knee_pot_value);

  //The input needs to map to the setpoint range for the PID output to make sense.  So I scale the known range of the sensor to the known range of the joystick
  Input = map(hip_pot_value, min_hip_sensor_value, max_hip_sensor_value, 0, ADC_num_of_bytes);   
  //Input = map(thigh_pot_value, min_thigh_sensor_value, max_thigh_sensor_value, 0, ADC_num_of_bytes);   
  //Input = map(knee_pot_value, min_knee_sensor_value, max_knee_sensor_value, 0, ADC_num_of_bytes);   

  hipPID.Compute(); //update the PID calculation based on the setpoint and input
  //thighPID.Compute();
  //kneePID.Compute();

  if (deadmanBounce.read() == 0) {  //if the deadman is hot - then write the scaled output to the valves.
    //Serial.println("deadman hot");
    digitalWrite(enable_pin_hip, HIGH); //enable the driver board
    //digitalWrite(enable_pin_thigh_knee, HIGH); //enable the driver board
    if (Output > setpoint_deadband) { //If the output is positive then drive the joint forward or out
      pwm_output = map(Output, 0, ADC_num_of_bytes, PWM_deadband, max_PWM_output); // Map the output to the range of the PWM output - deadband to max (either desired or set)
       analogWrite(forward_hip_pin, pwm_output); //write the command 
       analogWrite(reverse_hip_pin, 0); //make sure the other solenoid is off
      //Serial.println("extending knee");
      // analogWrite(in_knee_pin, pwm_output); //write the command 
      // analogWrite(out_knee_pin, 0); //make sure the other solenoid is off
    }
    else if (Output < -setpoint_deadband) { //If the output is negative then drive the joint in reverse or in.
      pwm_output = map(-Output, 0, ADC_num_of_bytes, PWM_deadband, max_PWM_output); //Invert the output to get rid of the negative and map it to the PWM range.
       analogWrite(reverse_hip_pin, pwm_output);  //write it
       analogWrite(forward_hip_pin, 0); //turn off the other valve
      //Serial.println("retracting knee");
      // analogWrite(out_knee_pin, pwm_output);  //write it
      // analogWrite(in_knee_pin, 0); //turn off the other valve
    }
    else {
      analogWrite(forward_hip_pin, 0); //If the output is within the setpoint deadband then write zeros to the valves to prevent jitter at a hold position
      analogWrite(reverse_hip_pin, 0);
      // analogWrite(in_knee_pin, 0); //If the output is within the setpoint deadband then write zeros to the valves to prevent jitter at a hold position
      // analogWrite(out_knee_pin, 0);
      pwm_output = 0;
    }
  }
  else { // If the deadman is low then disable the driver board and write zeros to the PWM output.
    digitalWrite(enable_pin_thigh_knee, LOW); 
    digitalWrite(enable_pin_hip, LOW); 
     analogWrite(forward_hip_pin, 0);
     analogWrite(reverse_hip_pin, 0);
    // analogWrite(in_knee_pin, 0);
    // analogWrite(out_knee_pin, 0);
    pwm_output = 0;
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
    Serial.print("Kp:");
    Serial.print("\t");
    Serial.print(Kp);
    Serial.print("\t");
    Serial.print("Ki:");
    Serial.print("\t");
    Serial.print(Ki);
    Serial.print("\t");
    Serial.print("PWM_deadband_percent:");
    Serial.print(PWM_deadband_percent);
    //Serial.print("\t");

    // Serial.print("Output:");
    // Serial.print("\t");
    // Serial.print(Output);
    // Serial.print("\t");
    // Serial.print("PWM:");
    // Serial.print("\t");
    // Serial.print(pwm_output);
    Serial.print("\n");
    if (Serial.available() > 0) { // Read in new PID values.  Then need to be sent as floats - e.g. 0.1 instead of .1 
      for (int x = 0; x < 4; x++) {
        switch (x) {
          // case 0:
          //   Setpoint = map(Serial.parseFloat(), 0, 100, 0, ADC_num_of_bytes); 
          //   break;
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
      //Serial.print("Setpoint= ");
      Serial.print("Kp,Ki,Kd = ");
      //Serial.println(Setpoint);
      //Serial.print(",");
      Serial.print(Kp);
      Serial.print(",");
      Serial.print(Ki);
      Serial.print(",");
      Serial.println(Kd);
      kneePID.SetTunings(Kp, Ki, Kd);        
    }
    lastMessage = now;
  }
}


