/*
PID control of single joint
attempting to get joystick and direcional control (forwards and reverse) working with PID
*/

#include <PID_v1.h>
#include <Bounce2.h>

//#define DEBUG 1

#define deadman_pin 5 // 25 on new board
Bounce deadmanBounce = Bounce(deadman_pin, 20);

#define enable_pin 2 // for the motor driver board

#define joystick_pin A2 // 31 --joystick input 
#define hip_pot_pin A3 // 20 --the photo senstive resistor input - pretending to be the string pot 
//#define photores_reverse_pin A4 // For this test I need a second sensor for the second LED.  This will change when both forward and reverse are acting on the the same sensor (string pot).
#define forward_hip_pin 9 // output pin for forward direction - in this case an LED but next it will be the valve
#define reverse_hip_pin 10 // output pin for reverse 

//setup for ADC averaging
#include <ADC.h>
#define N_AVG 8
#define ADC_RES 12 //bits of resolution for ADC
int ADC_num_of_bytes; // (pow(2, ADC_RES) - 1) --holds the (zero indexed) number of bytes for the ADC input 
#define max_hip_sensor_value 4000 // This is the maximum read of the string pot - in this case the photo resistor.  I will use the same numbers for forward and reverse to make it easier to convert to the string pot later.  
#define min_hip_sensor_value 0 // This is the minimum read of the string pot - in this case the photo resistor. 


//#define max_sensor_value_reverse 3800 // This is the maximum read of the string pot - in this case the photo resistor.  I will use the same numbers for forward and reverse to make it easier to convert to the string pot later.  
//#define min_sensor_value_reverse 500// This is the minimum read of the string pot - in this case the photo resistor. 
int half_bytes;
#define setpoint_deadband 1 //This is the difference accaptable between the setpoint (joystick) position and the input (sensor readout).
#define PWM_deadband 0 //number of bytes required before there is a positive response from the valve.  This will be relative to the bit rate.
#define max_PWM_output 4000 // this scales the PWM output (in bytes) so the PID output doesn't wirte a value higher than we want
int pwm_output = 0;

#define DAC_PWM_RES 12 // bits of resolution for PWM output
int DAC_num_of_bytes; // (pow(2, DAC_PWM_RES) - 1)  --holds the (zero indexed) number of bytes for the PWM output

double hip_pot_value; // reading from the photo resistor
double joystick_val; // value of the joystick

ADC *adc = new ADC(); // adc object

//PID variables
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=0.5, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int sampleRate = 1;
const int serialPing = 100; //interval in ms at which serial is sent and read
unsigned long now = 0;
unsigned long lastMessage = 0;

void setup()
{
  analogWriteResolution(DAC_PWM_RES);
  analogWriteFrequency(9, 11718);
  ADC_num_of_bytes = pow(2, ADC_RES) - 1;
  DAC_num_of_bytes = pow(2, DAC_PWM_RES) - 1;
  half_bytes = ADC_num_of_bytes/2;

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

  Serial.begin(9600);
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
  
  Setpoint = adc->analogRead(joystick_pin);

  hip_pot_value = adc->analogRead(hip_pot_pin);


  Input = map(hip_pot_value, min_hip_sensor_value, max_hip_sensor_value, 0, ADC_num_of_bytes);  

  myPID.Compute();

  if (deadmanBounce.read() == 1) { 
    digitalWrite(enable_pin, HIGH);
    if (Output > setpoint_deadband) {
      pwm_output = map(Output, 0, ADC_num_of_bytes, PWM_deadband, max_PWM_output);
      analogWrite(forward_hip_pin, pwm_output);
      analogWrite(reverse_hip_pin, 0);
    }
    else if (Output < -setpoint_deadband) {
      pwm_output = map(-Output, 0, ADC_num_of_bytes, PWM_deadband, max_PWM_output);
      analogWrite(reverse_hip_pin, pwm_output);
      analogWrite(forward_hip_pin, 0);
    }
    else {
      analogWrite(forward_hip_pin, 0);
      analogWrite(reverse_hip_pin, 0);
    }
  }
  else {
    digitalWrite(enable_pin, LOW);
    analogWrite(forward_hip_pin, 0);
    analogWrite(reverse_hip_pin, 0);
  }

  now = millis();
  if(now - lastMessage > serialPing) {
        //Serial.print("hipPot: ");
        //Serial.print((adc->analogRead(hip_pot_pin)));
        Serial.print("Setpoint: ");
        Serial.print(Setpoint);
        Serial.print(" Input: ");
        Serial.print(Input);
        Serial.print(" Output: ");
        Serial.print(Output);
        Serial.print("PWM: ");
        Serial.print(pwm_output);
        Serial.print("\n");
        if (Serial.available() > 0) {
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


