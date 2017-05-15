/*
PID control of single joint
attempting to get joystick and direcional control (forwards and reverse) working with PID
*/

#include <PID_v1.h>

#define DEBUG 1

#define joystick_pin A2 // joystick input 
#define photores_forward_pin A3 // the photo senstive resistor input - pretending to be the string pot 
#define photores_reverse_pin A4 // For this test I need a second sensor for the second LED.  This will change when both forward and reverse are acting on the the same sensor (string pot).
#define forward_LED_pin 9 // output pin for forward direction - in this case an LED but next it will be the valve
#define reverse_LED_pin 10 // output pin for reverse 

//setup for ADC averaging
#include <ADC.h>
#define N_AVG 8
#define ADC_RES 12 //bits of resolution for ADC
int ADC_num_of_bytes; // (pow(2, ADC_RES) - 1) --holds the (zero indexed) number of bytes for the ADC input 
#define max_sensor_value_forward 3910 // This is the maximum read of the string pot - in this case the photo resistor.  I will use the same numbers for forward and reverse to make it easier to convert to the string pot later.  
#define min_sensor_value_forward 2360 // This is the minimum read of the string pot - in this case the photo resistor. 
#define max_sensor_value_reverse 3920 // This is the maximum read of the string pot - in this case the photo resistor.  I will use the same numbers for forward and reverse to make it easier to convert to the string pot later.  
#define min_sensor_value_reverse 2220 // This is the minimum read of the string pot - in this case the photo resistor. 
int half_bytes;


#define DAC_PWM_RES 12 // bits of resolution for PWM output
int DAC_num_of_bytes; // (pow(2, DAC_PWM_RES) - 1)  --holds the (zero indexed) number of bytes for the PWM output

double lightLevel_forward; // reading from the photo resistor
double lightLevel_reverse; // reading from the second photo resistor
double joystick_val; // value of the joystick

ADC *adc = new ADC(); // adc object

//PID variables
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=0, Ki=10, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int sampleRate = 1;
const int serialPing = 100; //interval in ms at which serial is sent and read
unsigned long now = 0;
unsigned long lastMessage = 0;

void setup()
{
  analogWriteResolution(DAC_PWM_RES);
  ADC_num_of_bytes = pow(2, ADC_RES) - 1;
  DAC_num_of_bytes = pow(2, DAC_PWM_RES) - 1;
  half_bytes = ADC_num_of_bytes/2;

  pinMode(joystick_pin, INPUT);
  pinMode(photores_forward_pin, INPUT);
  pinMode(photores_reverse_pin, INPUT);

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

  Setpoint = map(adc->analogRead(joystick_pin), 0, ADC_num_of_bytes, 0, ADC_num_of_bytes); // this seems like a silly mappng for now but the first max will change

  if (Setpoint > half_bytes) {
    lightLevel_forward = adc->analogRead(photores_forward_pin);
    Input = map(lightLevel_forward, min_sensor_value_forward, max_sensor_value_forward, half_bytes, ADC_num_of_bytes);
  }
  else {
    lightLevel_reverse = adc->analogRead(photores_reverse_pin);
    Input = map(lightLevel_reverse, min_sensor_value_reverse, max_sensor_value_reverse, half_bytes, 0);
  }

  

  Serial.begin(9600);
  //myPID.SetOutputLimits() // This might be helpful depeding on what I do with scaling
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(sampleRate);
  delay(2000);
  Serial.println("Begin");
  lastMessage = millis();
}

void loop()
{
  // The joystick is the setpoint. For now it will be forward or reverse velocity.  Later it could be position.
  // The joystick will be split (with a deadband) into forward and reverse velocity.
  Setpoint = map(adc->analogRead(joystick_pin), 0, ADC_num_of_bytes, 0, ADC_num_of_bytes);

  if (Setpoint > half_bytes) {
    lightLevel_forward = adc->analogRead(photores_forward_pin);
    Input = map(lightLevel_forward, min_sensor_value_forward, max_sensor_value_forward, half_bytes, ADC_num_of_bytes);
  }
  else {
    lightLevel_reverse = adc->analogRead(photores_reverse_pin);
    Input = map(lightLevel_reverse, min_sensor_value_reverse, max_sensor_value_reverse, half_bytes, 0);
  }

  myPID.Compute();

  #ifdef DEBUG
    Serial.print("Setpoint: ");
    Serial.println(Setpoint);
    Serial.print("Input: ");
    Serial.print(Input);
  #endif

  

  if (Setpoint > half_bytes) {
    analogWrite(forward_LED_pin, Output);
  }
  else {
    analogWrite(reverse_LED_pin, Output);
  }

  now = millis();
  if(now - lastMessage > serialPing) {
        Serial.print("Setpoint = ");
        Serial.print(Setpoint);
        Serial.print(" Input = ");
        Serial.print(Input);
        Serial.print(" Output = ");
        Serial.print(Output);
        //Serial.print(" LIGHTLEVEL - raw: ");
        //Serial.print(lightLevel_forward);
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


