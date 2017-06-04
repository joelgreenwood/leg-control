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
#define max_hip_sensor_value 2990 // This is the maximum read of the string pot - in this case the photo resistor.  I will use the same numbers for forward and reverse to make it easier to convert to the string pot later.  
#define min_hip_sensor_value 250 // This is the minimum read of the string pot - in this case the photo resistor. 
#define max_thigh_sensor_value 3688 // these are from the robot: 3668 //this is all the way down or extended
#define min_thigh_sensor_value 136 // these are from the robot: 136 //this is fully reacted/up or in deadbug
#define max_knee_sensor_value 3736 // these are from the robot: 3736 //this is fully retracted 
#define min_knee_sensor_value 592 // these are from the robot: 592 //this is fully extended
//********** This are used for triangle wave limits
#define upper_bound_hip 3000
#define lower_bound_hip 1000
#define upper_bound_thigh 1200
#define lower_bound_thigh 400
#define upper_bound_knee 4000
#define lower_bound_knee 3000
//********** Deadband and Max PWM output
#define setpoint_deadband 0 //This is the difference acceptable between the setpoint (joystick) position and the input (sensor readout).
#define PWM_deadband_percent_hipFwd 30 //This is cylinder extend on the robot -- currently on the electric rig this is lowering the weight
#define PWM_deadband_percent_hipRev 30 //percent of PWM range needed to crack the valve -- for now this is one number but I may break this out into individual joints later
#define PWM_deadband_percent_thighUp 30 
#define PWM_deadband_percent_thighDown 30 
#define PWM_deadband_percent_kneeIn 30
#define PWM_deadband_percent_kneeOut 30 

#define pwm_govenor_hip 100 //percent of maximum output
#define pwm_govenor_thigh 100
#define pwm_govenor_knee 80

#define beadcrumb_time_interval 25 //in ms
elapsedMillis since_crumb_update;
#define breadcrub_spacing_hip 5 //this is in sensor units so for 12 bit 1 up to 4096
#define breadcrub_spacing_thigh 1
#define breadcrub_spacing_knee 3

int last_Setpoint_hip = lower_bound_hip;
int last_Setpoint_thigh = lower_bound_thigh;
int last_Setpoint_knee = lower_bound_knee;
int increasing_hip = 1;
int increasing_thigh = 1;
int increasing_knee = 1;

//setup for ADC averaging
#define N_AVG 8
#define ADC_RES 12 //bits of resolution for ADC
#define DAC_PWM_RES 12 // bits of resolution for PWM output
ADC *adc = new ADC(); // adc object

long ADC_num_of_bytes; // (pow(2, ADC_RES) - 1) --holds the (zero indexed) number of bytes for the ADC input 
//long DAC_num_of_bytes; // (pow(2, DAC_PWM_RES) - 1)  --holds the (zero indexed) number of bytes for the PWM output
long PWM_deadband_hipFwd;
long PWM_deadband_hipRev;
long PWM_deadband_thighUp;
long PWM_deadband_thighDown;
long PWM_deadband_kneeIn;
long PWM_deadband_kneeOut;
int pwm_output_hip = 0;
int pwm_output_thigh = 0;
int pwm_output_knee = 0;
unsigned long max_PWM_output_hip; // This caps the PWM output so we don't blow things up. Today... 
unsigned long max_PWM_output_thigh;
unsigned long max_PWM_output_knee;
double hip_pot_value; //sensor reading
double thigh_pot_value;
double knee_pot_value;

double joystick_val; // value of the joystick

Bounce deadmanBounce = Bounce(deadman_pin, 20); //debounce for the deadman pin

//PID variables - Define Variables we'll be connecting to
double Setpoint_hip, Input_hip, Output_hip;
double Setpoint_thigh, Input_thigh, Output_thigh;
double Setpoint_knee, Input_knee, Output_knee;

//Specify the links and initial tuning parameters
double Kp_h=10, Ki_h=1, Kd_h=0;  //for now I'm going with all the same values but I'm sure I'll need to break it out in the future
double Kp_t=10, Ki_t=1, Kd_t=0;
double Kp_k=6, Ki_k=1, Kd_k=0;
PID hipPID(&Input_hip, &Output_hip, &Setpoint_hip, Kp_h, Ki_h, Kd_h, DIRECT);
PID thighPID(&Input_thigh, &Output_thigh, &Setpoint_thigh, Kp_t, Ki_t, Kd_t, DIRECT);
PID kneePID(&Input_knee, &Output_knee, &Setpoint_knee, Kp_k, Ki_k, Kd_k, DIRECT);

const int sampleRate = 1;
const int serialPing = 100; //interval in ms at which serial is sent and read
unsigned long now = 0;
unsigned long lastMessage = 0;

//***sine block
  elapsedMillis sinceStart;
  float sensor_range_hip;
  float sensor_range_thigh;
  float sensor_range_knee;
  float percent_of_travel_hip = 100;
  float percent_of_travel_thigh = 100;
  float percent_of_travel_knee = 100;
  float desired_percent_of_range_hip;
  float desired_percent_of_range_thigh;
  float desired_percent_of_range_knee;
  float mid_sensor_hip;
  float mid_sensor_thigh;
  float mid_sensor_knee;
  float roundTrip_Sec_hip = 100;  //This is the time in seconds for one round trip (2*pi*r) or 360 deg.
  float roundTrip_Sec_thigh = 100;
  float roundTrip_Sec_knee = 100;
  float frequencyHz_hip; //frequency in Hz for how long it takes to go through one cycle
  float frequencyHz_thigh;
  float frequencyHz_knee;
  float circularFrequency_hip; // This is omega - 2*pi
  float circularFrequency_thigh; 
  float circularFrequency_knee; 
  const float pi = 3.14159265359;
//***End sine block

int getSensorAtTime_hip(int _time) { //time is in ms
  return mid_sensor_hip + (mid_sensor_hip*sin(circularFrequency_hip*((float)_time/1000)));
}
int getSensorAtTime_thigh(int _time) { //time is in ms
  return mid_sensor_thigh + (mid_sensor_thigh*sin(circularFrequency_hip*((float)_time/1000)));
}
int getSensorAtTime_knee(int _time) { //time is in ms
  return mid_sensor_knee + (mid_sensor_knee*sin(circularFrequency_hip*((float)_time/1000)));
}

void setup()
{
  Serial.begin(9600);

  Setpoint_hip = adc->analogRead(hip_pot_pin);
  Setpoint_thigh = adc->analogRead(thigh_pot_pin);
  Setpoint_knee = adc->analogRead(knee_pot_pin);
  //***block for sine wave
    sensor_range_hip = max_hip_sensor_value - min_hip_sensor_value;
    sensor_range_thigh = max_thigh_sensor_value - min_thigh_sensor_value;
    sensor_range_knee = max_knee_sensor_value - min_knee_sensor_value;
    frequencyHz_hip = 1/roundTrip_Sec_hip;
    frequencyHz_thigh = 1/roundTrip_Sec_thigh;
    frequencyHz_knee = 1/roundTrip_Sec_knee;
    circularFrequency_hip = 2*pi*frequencyHz_hip;
    circularFrequency_thigh = 2*pi*frequencyHz_thigh;
    circularFrequency_knee = 2*pi*frequencyHz_knee;
    //desired_percent_of_range = (float)sensor_range * (percent_of_travel/100);
    mid_sensor_hip = sensor_range_hip/2;
    mid_sensor_thigh = sensor_range_thigh/2;
    mid_sensor_knee = sensor_range_knee/2;
  //***end sine wave block


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
  //DAC_num_of_bytes = pow(2, DAC_PWM_RES) - 1;

  max_PWM_output_hip = ADC_num_of_bytes * ((float)pwm_govenor_hip/100);
  max_PWM_output_thigh = ADC_num_of_bytes * ((float)pwm_govenor_thigh/100);
  max_PWM_output_knee = ADC_num_of_bytes * ((float)pwm_govenor_knee/100);

  PWM_deadband_hipFwd = ADC_num_of_bytes * ((float)PWM_deadband_percent_hipFwd/100);
  PWM_deadband_hipRev = ADC_num_of_bytes * ((float)PWM_deadband_percent_hipRev/100);
  PWM_deadband_thighUp = ADC_num_of_bytes * ((float)PWM_deadband_percent_thighUp/100);
  PWM_deadband_thighDown = ADC_num_of_bytes * ((float)PWM_deadband_percent_thighDown/100);
  PWM_deadband_kneeIn = ADC_num_of_bytes * ((float)PWM_deadband_percent_kneeIn/100);
  PWM_deadband_kneeOut = ADC_num_of_bytes * ((float)PWM_deadband_percent_kneeOut/100);

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
  thighPID.SetOutputLimits(-ADC_num_of_bytes, ADC_num_of_bytes);
  thighPID.SetMode(AUTOMATIC);
  thighPID.SetSampleTime(sampleRate);
  kneePID.SetOutputLimits(-ADC_num_of_bytes, ADC_num_of_bytes);
  kneePID.SetMode(AUTOMATIC);
  kneePID.SetSampleTime(sampleRate);
  
  delay(2000);
  Serial.println("Begin");
  lastMessage = millis();
  last_Setpoint_hip = adc->analogRead(hip_pot_pin);  //This will be a bit off (not scaled right) but it'll start the increment loop close to where it is now
  last_Setpoint_thigh = adc->analogRead(thigh_pot_pin);
  last_Setpoint_knee = adc->analogRead(knee_pot_pin);
}

void loop()
{
  deadmanBounce.update(); //update the deadman

  //******************  Setpoint input section
  // Choose one method for selecting a setpoint

  //***slider of joystick input
    //Setpoint = map(adc->analogRead(knee_pot_pin),  0, ADC_num_of_bytes, ADC_num_of_bytes, 0); //read in setoint from joystick 
  //***End slider/joystick input

  //***incrementing a fixed interval in a fixed time
    if (since_crumb_update >= beadcrumb_time_interval) {
      
      last_Setpoint_hip = Setpoint_hip;
      last_Setpoint_thigh = Setpoint_thigh;
      last_Setpoint_knee = Setpoint_knee;
      since_crumb_update = 0;
      //***hip
      if (increasing_hip == 1) {
        Setpoint_hip = last_Setpoint_hip + breadcrub_spacing_hip;
        if (Setpoint_hip >= upper_bound_hip) {
          Setpoint_hip = last_Setpoint_hip;
          increasing_hip = 0;
        }
      }
      if (increasing_hip == 0) {
        Setpoint_hip = last_Setpoint_hip - breadcrub_spacing_hip;
        if (Setpoint_hip <= lower_bound_hip) {
          Setpoint_hip = last_Setpoint_hip + breadcrub_spacing_hip;
          increasing_hip = 1;
        }
      }
      //***thigh
      if (increasing_thigh == 1) {
        Setpoint_thigh = last_Setpoint_thigh + breadcrub_spacing_thigh;
        if (Setpoint_thigh >= upper_bound_thigh) {
          Setpoint_thigh = last_Setpoint_thigh;
          increasing_thigh = 0;
        }
      }
      if (increasing_thigh == 0) {
        Setpoint_thigh = last_Setpoint_thigh - breadcrub_spacing_thigh;
        if (Setpoint_thigh <= lower_bound_thigh) {
          Setpoint_thigh = last_Setpoint_thigh + breadcrub_spacing_thigh;
          increasing_thigh = 1;
        }
      }
      //***knee
      if (increasing_knee == 1) {
        Setpoint_knee = last_Setpoint_knee + breadcrub_spacing_knee;
        if (Setpoint_knee >= upper_bound_knee) {
          Setpoint_knee = last_Setpoint_knee;
          increasing_knee = 0;
        }
      }
      if (increasing_knee == 0) {
        Setpoint_knee = last_Setpoint_knee - breadcrub_spacing_knee;
        if (Setpoint_knee <= lower_bound_knee) {
          Setpoint_knee = last_Setpoint_knee + breadcrub_spacing_knee;
          increasing_knee = 1;
        }
      }
    }


  //***End incrementing along a line
  
  //***Sine wave along percent of travel
    // if (since_crumb_update >= beadcrumb_time_interval) {
    //   since_crumb_update = 0;
    //   Setpoint = map(getSensorAtTime(sinceStart), 0,  sensor_range, 0, ADC_num_of_bytes) * (percent_of_travel/100);
    // }
  //***End Sine wave  

  //******************  End setpoint

  hip_pot_value = adc->analogRead(hip_pot_pin);  //read the position of the joint sensor
  thigh_pot_value = adc->analogRead(thigh_pot_pin); 
  knee_pot_value = adc->analogRead(knee_pot_pin); 

  //Serial.println(knee_pot_value);

  //The input needs to map to the setpoint range for the PID output to make sense.  So I scale the known range of the sensor to the known range of the joystick
  Input_hip = map(hip_pot_value, min_hip_sensor_value, max_hip_sensor_value, 0, ADC_num_of_bytes);   
  Input_thigh = map(thigh_pot_value, min_thigh_sensor_value, max_thigh_sensor_value, 0, ADC_num_of_bytes);   
  Input_knee = map(knee_pot_value, max_knee_sensor_value, min_knee_sensor_value, 0, ADC_num_of_bytes);   

  

  if (deadmanBounce.read() == 1) {  //if the deadman is hot - then write the scaled output to the valves.
    //Serial.println("deadman hot");
    hipPID.Compute(); //update the PID calculation based on the setpoint and input
    thighPID.Compute();
    kneePID.Compute();

    digitalWrite(enable_pin_hip, HIGH); //enable the driver board
    digitalWrite(enable_pin_thigh_knee, HIGH); //enable the driver board
    //***hip
    if (Output_hip > setpoint_deadband) {
      pwm_output_hip = map(Output_hip, 0, ADC_num_of_bytes, PWM_deadband_hipFwd, max_PWM_output_hip);
      analogWrite(forward_hip_pin, pwm_output_hip); //write the command 
      analogWrite(reverse_hip_pin, 0); //make sure the other solenoid is off
    }
    else if (Output_hip < -setpoint_deadband) {
      pwm_output_hip = map(-Output_hip, 0, ADC_num_of_bytes, PWM_deadband_hipRev, max_PWM_output_hip);
      analogWrite(forward_hip_pin, 0); //make sure the other solenoid is off
      analogWrite(reverse_hip_pin, pwm_output_hip); 
    }
    else {
      analogWrite(forward_hip_pin, 0); //If the output is within the setpoint deadband then write zeros to the valves to prevent jitter at a hold position
      analogWrite(reverse_hip_pin, 0);
      pwm_output_hip = 0;
    }
    //***thigh
    if (Output_thigh > setpoint_deadband) {
      pwm_output_thigh = map(Output_thigh, 0, ADC_num_of_bytes, PWM_deadband_thighDown, max_PWM_output_thigh);
      analogWrite(down_thigh_pin, pwm_output_thigh); //write the command 
      analogWrite(up_thigh_pin, 0); //make sure the other solenoid is off
    }
    else if (Output_thigh < -setpoint_deadband) {
      pwm_output_thigh = map(-Output_thigh, 0, ADC_num_of_bytes, PWM_deadband_thighUp, max_PWM_output_thigh);
      analogWrite(down_thigh_pin, 0); 
      analogWrite(up_thigh_pin, pwm_output_thigh); 
    }
    else {
      analogWrite(up_thigh_pin, 0); //If the output is within the setpoint deadband then write zeros to the valves to prevent jitter at a hold position
      analogWrite(down_thigh_pin, 0);
      pwm_output_thigh = 0;
    }
    //***knee
    if (Output_knee > setpoint_deadband) {
      pwm_output_knee = map(Output_knee, 0, ADC_num_of_bytes, PWM_deadband_kneeOut, max_PWM_output_knee);
      analogWrite(out_knee_pin, pwm_output_knee); //write the command 
      analogWrite(in_knee_pin, 0); //make sure the other solenoid is off
    }
    else if (Output_knee < -setpoint_deadband) {
      pwm_output_knee = map(-Output_knee, 0, ADC_num_of_bytes, PWM_deadband_kneeIn, max_PWM_output_knee);
      analogWrite(out_knee_pin, 0);
      analogWrite(in_knee_pin, pwm_output_knee);
    }
    else {
      analogWrite(out_knee_pin, 0); //If the output is within the setpoint deadband then write zeros to the valves to prevent jitter at a hold position
      analogWrite(in_knee_pin, 0);
      pwm_output_knee = 0;
    }
  }


  //   if (Output > setpoint_deadband) { //If the output is positive then drive the joint forward or out
  //     pwm_output_hip = map(Output_hip, 0, ADC_num_of_bytes, PWM_deadband_hipFwd, max_PWM_output_hip); // Map the output to the range of the PWM output - deadband to max (either desired or set)
  //     pwm_output_thigh = map(Output_thigh, 0, ADC_num_of_bytes, PWM_deadband_thighDown, max_PWM_output_thigh);
  //     pwm_output_knee = map(Output_knee, 0, ADC_num_of_bytes, PWM_deadband_kneeOut, max_PWM_output_knee);

  //     analogWrite(forward_hip_pin, pwm_output_hip); //write the command 
  //     analogWrite(reverse_hip_pin, 0); //make sure the other solenoid is off
  //     analogWrite(down_thigh_pin, pwm_output_thigh); //write the command 
  //     analogWrite(up_thigh_pin, 0); //make sure the other solenoid is off
  //     analogWrite(out_knee_pin, pwm_output_knee); //write the command 
  //     analogWrite(in_knee_pin, 0); //make sure the other solenoid is off
      
  //   }
  //   else if (Output < -setpoint_deadband) { //If the output is negative then drive the joint in reverse or in.
  //     pwm_output_hip = map(-Output_hip, 0, ADC_num_of_bytes, PWM_deadband_hipRev, max_PWM_output_hip); //Invert the output to get rid of the negative and map it to the PWM range.
  //     pwm_output_thigh = map(-Output_thigh, 0, ADC_num_of_bytes, PWM_deadband_thighUp, max_PWM_output_thigh);
  //     pwm_output_knee = map(-Output_knee, 0, ADC_num_of_bytes, PWM_deadband_kneeIn, max_PWM_output_knee);

  //     analogWrite(forward_hip_pin, 0); //make sure the other solenoid is off
  //     analogWrite(reverse_hip_pin, pwm_output_hip); 
  //     analogWrite(down_thigh_pin, 0); 
  //     analogWrite(up_thigh_pin, pwm_output_thigh); 
  //     analogWrite(out_knee_pin, 0);
  //     analogWrite(in_knee_pin, pwm_output_knee);

  //   }
  //   else {
  //     analogWrite(forward_hip_pin, 0); //If the output is within the setpoint deadband then write zeros to the valves to prevent jitter at a hold position
  //     analogWrite(reverse_hip_pin, 0);
  //     analogWrite(up_thigh_pin, 0); //If the output is within the setpoint deadband then write zeros to the valves to prevent jitter at a hold position
  //     analogWrite(down_thigh_pin, 0);
  //     analogWrite(out_knee_pin, 0); //If the output is within the setpoint deadband then write zeros to the valves to prevent jitter at a hold position
  //     analogWrite(in_knee_pin, 0);
  //     pwm_output_hip = 0;
  //     pwm_output_thigh = 0;
  //     pwm_output_knee = 0;
  //   }
  // }
  else { // If the deadman is low then disable the driver board and write zeros to the PWM output.
    digitalWrite(enable_pin_thigh_knee, LOW); 
    digitalWrite(enable_pin_hip, LOW); 
    analogWrite(forward_hip_pin, 0); //If the output is within the setpoint deadband then write zeros to the valves to prevent jitter at a hold position
    analogWrite(reverse_hip_pin, 0);
    analogWrite(up_thigh_pin, 0); //If the output is within the setpoint deadband then write zeros to the valves to prevent jitter at a hold position
    analogWrite(down_thigh_pin, 0);
    analogWrite(out_knee_pin, 0); //If the output is within the setpoint deadband then write zeros to the valves to prevent jitter at a hold position
    analogWrite(in_knee_pin, 0);
    pwm_output_hip = 0;
    pwm_output_thigh = 0;
    pwm_output_knee = 0;
  }

  now = millis();
  if(now - lastMessage > serialPing) {
    // Serial.print("hipPot: ");
    // Serial.print((adc->analogRead(hip_pot_pin)));
    // Serial.print("SetHip:");
    // Serial.print("\t");
    Serial.print(Setpoint_hip);
    // Serial.print("\t");
    // Serial.print("InputHip:");
    // Serial.print("\t");
    // Serial.print(Input_hip);
    // Serial.print("\t");

    // Serial.print("SetThigh:");
    // Serial.print("\t");
    // Serial.print(Setpoint_thigh);
    // Serial.print("\t");
    // Serial.print("InputThigh:");
    // Serial.print("\t");
    // Serial.print(Input_thigh);
    // Serial.print("\t");

    // Serial.print("SetKnee:");
    // Serial.print("\t");
    // Serial.print(Setpoint_knee);
    // Serial.print("\t");
    // Serial.print("InputKnee:");
    // Serial.print("\t");
    // Serial.print(Input_knee);

    // Serial.print("\t");
    // Serial.print("Kp:");
    // Serial.print("\t");
    // Serial.print(Kp);
    // Serial.print("\t");
    // Serial.print("Ki:");
    // Serial.print("\t");
    // Serial.print(Ki);
    // Serial.print("\t");
    // Serial.print("deadband-FWD,REV:");
    // Serial.print(PWM_deadband_percent_hipFwd);
    // Serial.print("\t");
    // Serial.print(PWM_deadband_percent_hipRev);

    // Serial.print("Output:");
    // Serial.print("\t");
    // Serial.print(Output);
    // Serial.print("\t");
    // Serial.print("PWM:");
    // Serial.print("\t");
    // Serial.print(pwm_output);
    Serial.print("\n");
    // if (Serial.available() > 0) { // Read in new PID values.  Then need to be sent as floats - e.g. 0.1 instead of .1 
    //   for (int x = 0; x < 4; x++) {
    //     switch (x) {
    //       // case 0:
    //       //   Setpoint = map(Serial.parseFloat(), 0, 100, 0, ADC_num_of_bytes); 
    //       //   break;
    //       case 0:
    //         Kp = Serial.parseFloat();
    //         break;
    //       case 1:
    //         Ki = Serial.parseFloat();
    //         break;
    //       case 2:
    //         Kd = Serial.parseFloat();
    //         break;
    //       case 3:
    //         for (int y = Serial.available(); y == 0; y--) {
    //           Serial.read();
    //        }
    //        break;
    //          }
    //   }
    //   //Serial.print("Setpoint= ");
    //   Serial.print("Kp,Ki,Kd = ");
    //   //Serial.println(Setpoint);
    //   //Serial.print(",");
    //   Serial.print(Kp);
    //   Serial.print(",");
    //   Serial.print(Ki);
    //   Serial.print(",");
    //   Serial.println(Kd);
    //   kneePID.SetTunings(Kp, Ki, Kd);        
    // }
    lastMessage = now;
  }
}


