/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>

#define pot_pin A2
#define photores_pin A3
#define led_pin A14

#define N_AVG 8
#define ADC_RES 12
int bits; // pow(2, ADC_RES)
#define max_sensor_value 3600
#define min_sensor_value 200

#include <ADC.h>

double lightLevel;
double pot_val;

ADC *adc = new ADC(); // adc object

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=0, Ki=10, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

const int sampleRate = 1;
const int serialPing = 100;
unsigned long now = 0;
unsigned long lastMessage = 0;

void setup()
{
  bits = pow(2, ADC_RES);

  pinMode(pot_pin, INPUT);
  pinMode(photores_pin, INPUT);

  // setup ADC_1
  adc->setAveraging(N_AVG);
  adc->setResolution(ADC_RES);
  adc->setConversionSpeed(ADC_VERY_LOW_SPEED);
  adc->setSamplingSpeed(ADC_VERY_LOW_SPEED);

  adc->setAveraging(N_AVG, ADC_1);
  adc->setResolution(ADC_RES, ADC_1);
  adc->setConversionSpeed(ADC_VERY_LOW_SPEED, ADC_1);
  adc->setSamplingSpeed(ADC_VERY_LOW_SPEED, ADC_1);

  delay(500);

  lightLevel = adc->analogRead(photores_pin, ADC_1);
  Input = map(lightLevel, min_sensor_value, max_sensor_value, 0, bits);

  Setpoint = map(adc->analogRead(pot_pin, ADC_1), 0, bits, 0, bits);

  Serial.begin(9600);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(sampleRate);
  delay(2000);
  Serial.println("Begin");
  lastMessage = millis();
}

void loop()
{
  Setpoint = map(adc->analogRead(pot_pin, ADC_1), 0, bits, 0, bits);
  lightLevel = adc->analogRead(photores_pin, ADC_1);
  //Serial.print("LIGHTLEVEL - raw: ");
  //Serial.println(lightLevel);
  Input = map(lightLevel, min_sensor_value, max_sensor_value, 0, bits);
  myPID.Compute();
  analogWrite(led_pin, Output);
  now = millis();
  if(now - lastMessage > serialPing) {
        Serial.print("Setpoint = ");
        Serial.print(Setpoint);
        Serial.print(" Input = ");
        Serial.print(Input);
        Serial.print(" Output = ");
        Serial.print(Output);
        //Serial.print(" LIGHTLEVEL - raw: ");
        //Serial.print(lightLevel);
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


