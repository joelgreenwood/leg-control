/*
PID control of single joint
attempting to get joystick and directional control (forwards and reverse) working with PID
*/

#include <PID_v1.h>
#include <Bounce2.h>
#include <ADC.h>
#include <legControl.h>

//#define FAKE_SENSORS 1
//#define PRINT_SENSORS 1
#define PRINT_SETPOINTS 1
#define PRINT_INPUTS 1


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
#define max_hip_sensor_value 2888.00 // This is the maximum read of the string pot - in this case the photo resistor.  I will use the same numbers for forward and reverse to make it easier to convert to the string pot later.  
#define min_hip_sensor_value 372.00 // This is the minimum read of the string pot - in this case the photo resistor. 
#define max_thigh_sensor_value 3668.00 // these are from the robot: 3668 //this is all the way down or extended
#define min_thigh_sensor_value 136.00 // these are from the robot: 136 //this is fully reacted/up or in deadbug
#define max_knee_sensor_value 3736.00 // these are from the robot: 3736 //this is fully retracted 
#define min_knee_sensor_value 592.00 // these are from the robot: 592 //this is fully extended
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

#define pwm_govenor_hip 50 //percent of maximum output
#define pwm_govenor_thigh 100
#define pwm_govenor_knee 100

#define beadcrumb_time_interval 25 //in ms
elapsedMillis since_crumb_update;
#define breadcrub_spacing_hip 5 //this is in sensor units so for 12 bit 1 up to 4096
#define breadcrub_spacing_thigh 1
#define breadcrub_spacing_knee 3

legControl leg1;

int last_Setpoint_hip = lower_bound_hip;
int last_Setpoint_thigh = lower_bound_thigh;
int last_Setpoint_knee = lower_bound_knee;
int increasing_hip = 1;
int increasing_thigh = 1;
int increasing_knee = 1;

float A[3] = {1, 3, 2}; //first point
float B[3] = {-5, 0, 4}; //second point
float C[3]; //This holds a new point along the vector
float r[3]; // vector between the two points
float distance; // distance between the two points
float startingAngles[3] = {0, 80, 101};
float goalAngles[3] = {0, 80, 100};
float startingXYZ[3];
float goalXYZ[3];
float sensorGoals[3];

float xyz[3];
float sensors[3];

float inches_to_travel = 20; //in inches
float time_for_travel = 3; //in seconds
float inches_increment;
int number_of_steps;
int loopCount = 0;
int goingUp = 1;
int goingHot = 0;


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
double Kp_h=2, Ki_h=0, Kd_h=0;  //for now I'm going with all the same values but I'm sure I'll need to break it out in the future
double Kp_t=24, Ki_t=1, Kd_t=0;
double Kp_k=17, Ki_k=1, Kd_k=0;
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

        while(!Serial);

        // hip_pot_value = adc->analogRead(hip_pot_pin);  //read the position of the joint sensor
        // thigh_pot_value = adc->analogRead(thigh_pot_pin); 
        // knee_pot_value = adc->analogRead(knee_pot_pin); 

        // last_Setpoint_hip = map(hip_pot_value, min_hip_sensor_value, max_hip_sensor_value, 0, ADC_num_of_bytes);   
        // last_Setpoint_thigh = map(thigh_pot_value, min_thigh_sensor_value, max_thigh_sensor_value, 0, ADC_num_of_bytes);   
        // last_Setpoint_knee = map(knee_pot_value, max_knee_sensor_value, min_knee_sensor_value, 0, ADC_num_of_bytes);   

        // Setpoint_hip = last_Setpoint_hip;
        // Setpoint_thigh = last_Setpoint_thigh;
        // Setpoint_knee = last_Setpoint_knee;

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

        number_of_steps = (time_for_travel * 1000) / beadcrumb_time_interval;
        Serial.print("number of steps: ");
        Serial.println(number_of_steps);
        inches_increment = inches_to_travel / (float)number_of_steps;
        Serial.print("inches for each step: ");
        Serial.println(inches_increment, 6);

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


  hipPID.SetOutputLimits(-ADC_num_of_bytes, ADC_num_of_bytes);
  hipPID.SetMode(AUTOMATIC);
  hipPID.SetSampleTime(sampleRate);
  thighPID.SetOutputLimits(-ADC_num_of_bytes, ADC_num_of_bytes);
  thighPID.SetMode(AUTOMATIC);
  thighPID.SetSampleTime(sampleRate);
  kneePID.SetOutputLimits(-ADC_num_of_bytes, ADC_num_of_bytes);
  kneePID.SetMode(AUTOMATIC);
  kneePID.SetSampleTime(sampleRate);
  
  //delay(2000);
  Serial.println("Begin");
  lastMessage = millis();
  
  read_pot_to_both_input_setpoint();

  sensors[0] = hip_pot_value;  //read the position of the joint sensor
  sensors[1] = thigh_pot_value;
  sensors[2] = knee_pot_value; 
  printArray(sensors);

        #ifdef FAKE_SENSORS
                //*******This is for testing
                sensors[0] = 1000;
                sensors[1] = 1000;
                sensors[2] = 1000;
                //*******testing
        #endif

        leg1.sensors_to_xyz(sensors, xyz);
        cpArray(xyz, goalXYZ, 3);

  // last_Setpoint_hip = adc->analogRead(hip_pot_pin);
  // last_Setpoint_thigh = adc->analogRead(thigh_pot_pin);
  // last_Setpoint_knee = adc->analogRead(knee_pot_pin);
  // hip_pot_value = adc->analogRead(hip_pot_pin);  //read the position of the joint sensor
  // thigh_pot_value = adc->analogRead(thigh_pot_pin); 
  // knee_pot_value = adc->analogRead(knee_pot_pin); 

  // last_Setpoint_hip = map(hip_pot_value, min_hip_sensor_value, max_hip_sensor_value, 0, ADC_num_of_bytes);   
  // last_Setpoint_thigh = map(thigh_pot_value, min_thigh_sensor_value, max_thigh_sensor_value, 0, ADC_num_of_bytes);   
  // last_Setpoint_knee = map(knee_pot_value, max_knee_sensor_value, min_knee_sensor_value, 0, ADC_num_of_bytes);   
  // Setpoint_hip = last_Setpoint_hip;
  // Setpoint_thigh = last_Setpoint_thigh;
  // Setpoint_knee = last_Setpoint_knee;
  // sensors[0] = Setpoint_hip;
  // sensors[1] = Setpoint_thigh;
  // sensors[2] = Setpoint_knee;
  // leg1.sensors_to_xyz(sensors, xyz);
  // cpArray(xyz, goalXYZ, 3);


}

void loop()
{
  deadmanBounce.update(); //update the deadman

  //******************  Setpoint input section
  // Choose one method for selecting a setpoint

  //Z increasing and decreasing with a fixed distance and time
        if (goingHot == 1) {
                if (goingUp == 1) {
                        if (loopCount <= number_of_steps) {
                                if (since_crumb_update >= beadcrumb_time_interval) {
                                        since_crumb_update = 0;
                                        goalXYZ[2] = xyz[2] + inches_increment;
                                        increment_along_vector(1, xyz, goalXYZ);
                                        leg1.xyzToSensors(goalXYZ, sensors);

                                        sensorValues_to_setpoint();

                                        // sensors[0] = map(sensors[0], min_hip_sensor_value, max_hip_sensor_value, 0, ADC_num_of_bytes);   
                                        // sensors[1] = map(sensors[1], min_thigh_sensor_value, max_thigh_sensor_value, 0, ADC_num_of_bytes);   
                                        // sensors[2] = map(sensors[2], max_knee_sensor_value, min_knee_sensor_value, 0, ADC_num_of_bytes);   

                                        // Setpoint_hip = sensors[0];
                                        // Setpoint_thigh = sensors[1];
                                        // Setpoint_knee = sensors[2];
                                        //printArray(sensors);
                                        cpArray(goalXYZ, xyz, 3);
                                        loopCount++;
                                        if (loopCount >= number_of_steps) {
                                                loopCount = 0;
                                                goingUp = 0;
                                        }       
                                }
                        }
                }
                if (goingUp == 0) {
                        if (loopCount <= number_of_steps) {
                                if (since_crumb_update >= beadcrumb_time_interval) {
                                        since_crumb_update = 0;
                                        goalXYZ[2] = xyz[2] - inches_increment;
                                        increment_along_vector(1, xyz, goalXYZ);
                                        leg1.xyzToSensors(goalXYZ, sensors);

                                        // sensors[0] = map(sensors[0], min_hip_sensor_value, max_hip_sensor_value, 0, ADC_num_of_bytes);   
                                        // sensors[1] = map(sensors[1], min_thigh_sensor_value, max_thigh_sensor_value, 0, ADC_num_of_bytes);   
                                        // sensors[2] = map(sensors[2], max_knee_sensor_value, min_knee_sensor_value, 0, ADC_num_of_bytes);   

                                        // Setpoint_hip = sensors[0];
                                        // Setpoint_thigh = sensors[1];
                                        // Setpoint_knee = sensors[2];

                                        sensorValues_to_setpoint();
                                        //printArray(sensors);
                                        cpArray(goalXYZ, xyz, 3);
                                        loopCount++;
                                        if (loopCount >= number_of_steps) {
                                                loopCount = 0;
                                                goingUp = 1;
                                        }       
                                }
                        }
                }
        }


  //End Z

  //***slider of joystick input
    //Setpoint = map(adc->analogRead(knee_pot_pin),  0, ADC_num_of_bytes, ADC_num_of_bytes, 0); //read in setoint from joystick 
  //***End slider/joystick input

        //***incrementing a fixed interval in a fixed time
                // if (since_crumb_update >= beadcrumb_time_interval) {

                //   last_Setpoint_hip = Setpoint_hip;
                //   last_Setpoint_thigh = Setpoint_thigh;
                //   last_Setpoint_knee = Setpoint_knee;
                //   since_crumb_update = 0;
                //   //***hip
                //   if (increasing_hip == 1) {
                //     Setpoint_hip = last_Setpoint_hip + breadcrub_spacing_hip;
                //     if (Setpoint_hip >= upper_bound_hip) {
                //       Setpoint_hip = last_Setpoint_hip;
                //       increasing_hip = 0;
                //     }
                //   }
                //   if (increasing_hip == 0) {
                //     Setpoint_hip = last_Setpoint_hip - breadcrub_spacing_hip;
                //     if (Setpoint_hip <= lower_bound_hip) {
                //       Setpoint_hip = last_Setpoint_hip + breadcrub_spacing_hip;
                //       increasing_hip = 1;
                //     }
                //   }
                //   //***thigh
                //   if (increasing_thigh == 1) {
                //     Setpoint_thigh = last_Setpoint_thigh + breadcrub_spacing_thigh;
                //     if (Setpoint_thigh >= upper_bound_thigh) {
                //       Setpoint_thigh = last_Setpoint_thigh;
                //       increasing_thigh = 0;
                //     }
                //   }
                //   if (increasing_thigh == 0) {
                //     Setpoint_thigh = last_Setpoint_thigh - breadcrub_spacing_thigh;
                //     if (Setpoint_thigh <= lower_bound_thigh) {
                //       Setpoint_thigh = last_Setpoint_thigh + breadcrub_spacing_thigh;
                //       increasing_thigh = 1;
                //     }
                //   }
                //   //***knee
                //   if (increasing_knee == 1) {
                //     Setpoint_knee = last_Setpoint_knee + breadcrub_spacing_knee;
                //     if (Setpoint_knee >= upper_bound_knee) {
                //       Setpoint_knee = last_Setpoint_knee;
                //       increasing_knee = 0;
                //     }
                //   }
                //   if (increasing_knee == 0) {
                //     Setpoint_knee = last_Setpoint_knee - breadcrub_spacing_knee;
                //     if (Setpoint_knee <= lower_bound_knee) {
                //       Setpoint_knee = last_Setpoint_knee + breadcrub_spacing_knee;
                //       increasing_knee = 1;
                //     }
                //   }
                // }


        //***End incrementing along a line
  
  //***Sine wave along percent of travel
    // if (since_crumb_update >= beadcrumb_time_interval) {
    //   since_crumb_update = 0;
    //   Setpoint = map(getSensorAtTime(sinceStart), 0,  sensor_range, 0, ADC_num_of_bytes) * (percent_of_travel/100);
    // }
  //***End Sine wave  

  //******************  End setpoint
        #ifdef FAKE_SENSORS
                hip_pot_value = sensors[0];
                thigh_pot_value = sensors[1];
                knee_pot_value = sensors[2];
        #endif

        // #ifndef FAKE_SENSORS
        //         hip_pot_value = adc->analogRead(hip_pot_pin);  //read the position of the joint sensor
        //         thigh_pot_value = adc->analogRead(thigh_pot_pin); 
        //         knee_pot_value = adc->analogRead(knee_pot_pin); 
        // #endif
  //Serial.println(knee_pot_value);

  // //The input needs to map to the setpoint range for the PID output to make sense.  So I scale the known range of the sensor to the known range of the joystick
  // Input_hip = map(hip_pot_value, min_hip_sensor_value, max_hip_sensor_value, 0, ADC_num_of_bytes);   
  // Input_thigh = map(thigh_pot_value, min_thigh_sensor_value, max_thigh_sensor_value, 0, ADC_num_of_bytes);   
  // Input_knee = map(knee_pot_value, max_knee_sensor_value, min_knee_sensor_value, 0, ADC_num_of_bytes);   
  read_pot_to_input();


  

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
        #ifdef PRINT_SENSORS
                Serial.print("hipPot ThighPot KneePot:  ");
                #ifdef FAKE_SENSORS
                        Serial.print(sensors[0]);
                        Serial.print("\t");
                        Serial.print(sensors[0]);
                        Serial.print("\t");
                        Serial.print(sensors[0]);
                #endif

                #ifndef FAKE_SENSORS      
                        Serial.print((adc->analogRead(hip_pot_pin)));
                        Serial.print("\t");
                        Serial.print((adc->analogRead(thigh_pot_pin)));
                        Serial.print("\t");
                        Serial.print((adc->analogRead(knee_pot_pin)));
                #endif
                Serial.println();
        #endif

        #ifdef PRINT_SETPOINTS
                Serial.print("setpoints: ");
                Serial.print(Setpoint_hip);
                Serial.print("\t");
                Serial.print(Setpoint_thigh);
                Serial.print("\t");
                Serial.print(Setpoint_knee);
                Serial.print("\t");
        #endif

        #ifdef PRINT_INPUTS
                Serial.print("inputs: ");
                Serial.print(Input_hip);
                Serial.print("\t");
                Serial.print(Input_thigh);
                Serial.print("\t");
                Serial.print(Input_knee);
                Serial.print("\t");
        #endif
    // Serial.print("hipPot: ");
    // Serial.print((adc->analogRead(hip_pot_pin)));
    // Serial.print("SetHip:");
    // Serial.print("\t");
    // Serial.print(Setpoint_hip);
    //  Serial.print("\t");
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
    
    while (Serial.available() > 0) {
    goingHot = Serial.parseInt();
    goingUp = Serial.parseInt();
    inches_to_travel = Serial.parseFloat();
    time_for_travel = Serial.parseFloat();

      if (Serial.read() == '\n') {
        Serial.print("goingUp inches time: ");
        Serial.print(goingUp);
        Serial.print('\t');
        Serial.print(inches_to_travel);
        Serial.print('\t');
        Serial.print(time_for_travel);
        Serial.print('\t');

        number_of_steps = (time_for_travel * 1000) / beadcrumb_time_interval;
        Serial.print("number of steps: ");
        Serial.println(number_of_steps);
        inches_increment = inches_to_travel / (float)number_of_steps;
        Serial.print("inches for each step: ");
        Serial.println(inches_increment, 6);

        Serial.println();

        // hip_pot_value = adc->analogRead(hip_pot_pin);  //read the position of the joint sensor
        // thigh_pot_value = adc->analogRead(thigh_pot_pin); 
        // knee_pot_value = adc->analogRead(knee_pot_pin); 

        // last_Setpoint_hip = map(hip_pot_value, min_hip_sensor_value, max_hip_sensor_value, 0, ADC_num_of_bytes);   
        // last_Setpoint_thigh = map(thigh_pot_value, min_thigh_sensor_value, max_thigh_sensor_value, 0, ADC_num_of_bytes);   
        // last_Setpoint_knee = map(knee_pot_value, max_knee_sensor_value, min_knee_sensor_value, 0, ADC_num_of_bytes);   

        // Setpoint_hip = last_Setpoint_hip;
        // Setpoint_thigh = last_Setpoint_thigh;
        // Setpoint_knee = last_Setpoint_knee;
       
        read_pot_to_both_input_setpoint();

        // sensors[0] = Setpoint_hip;
        // sensors[1] = Setpoint_thigh;
        // sensors[2] = Setpoint_knee;

        sensors[0] = adc->analogRead(hip_pot_pin);  //read the position of the joint sensor
        sensors[1] = adc->analogRead(thigh_pot_pin); 
        sensors[2] = adc->analogRead(knee_pot_pin); 

        
        #ifdef FAKE_SENSORS
                //*******This is for testing
                sensors[0] = 1000;
                sensors[1] = 1000;
                sensors[2] = 1000;
                //*******testing
        #endif

        leg1.sensors_to_xyz(sensors, xyz);
        cpArray(xyz, goalXYZ, 3);

        //delay(5000);


      }
    }


    lastMessage = now;
  }
}


void increment_along_vector(int inc_num, float A[3], float B[3]) {
        float startPoint[3];
        float endPoint[3];
        cpArray(A, startPoint, 3);
        cpArray(B, endPoint, 3);
        float inc = get_distance(A, B) / inc_num;
        for (int i = 0; i < inc_num; i ++) {
                get_point_on_vector(inc, startPoint, endPoint);
                float tmp[3];
                for (int g = 0; g < 3; g++) {
                        // Serial.print(C[g]);
                        // Serial.print("\t");
                        tmp[g] = C[g];
                }
                leg1.xyzToSensors(tmp, sensorGoals);
                // Serial.print("\t");
                cpArray(C, startPoint, 3);
        }
}

float get_distance(float A[3], float B[3]) {
        return sqrt(sq(B[0] - A[0]) + sq(B[1] - A[1]) + sq(B[2] - A[2]));
}

void get_unit_vector(float A[3], float B[3]) {
        float d = sqrt(sq(B[0] - A[0]) + sq(B[1] - A[1]) + sq(B[2] - A[2])); //get the distance between the two points
        for (int i = 0; i < 3; i++) {
                r[i] = (B[i] - A[i]) / d;   
        }
}

void get_point_on_vector(float dist_on_vector, float A[3], float B[3]) {
        distance = sqrt(sq(B[0] - A[0]) + sq(B[1] - A[1]) + sq(B[2] - A[2])); //get the distance between the two points
        for (int i = 0; i < 3; i++) {
                r[i] = (B[i] - A[i]) / distance; //calculate the unit vector
                C[i] = A[i] + (dist_on_vector * r[i]);  //multiply the unit vector by the distance along the vector
        }
}

void cpArray(float *arrayOriginal, float *arrayCopy, int arraySize) {
        memcpy(arrayCopy, arrayOriginal, sizeof(arrayOriginal[0]) *arraySize);
}

void read_pot_to_input() {
  hip_pot_value = adc->analogRead(hip_pot_pin);  //read the position of the joint sensor
  thigh_pot_value = adc->analogRead(thigh_pot_pin); 
  knee_pot_value = adc->analogRead(knee_pot_pin); 

  Input_hip = map(hip_pot_value, min_hip_sensor_value, max_hip_sensor_value, 0, ADC_num_of_bytes);   
  Input_thigh = map(thigh_pot_value, min_thigh_sensor_value, max_thigh_sensor_value, 0, ADC_num_of_bytes);   
  Input_knee = map(knee_pot_value, max_knee_sensor_value, min_knee_sensor_value, 0, ADC_num_of_bytes);

}

void read_pot_to_setpoint() {
  hip_pot_value = adc->analogRead(hip_pot_pin);  //read the position of the joint sensor
  thigh_pot_value = adc->analogRead(thigh_pot_pin); 
  knee_pot_value = adc->analogRead(knee_pot_pin); 

  Setpoint_hip = map(hip_pot_value, min_hip_sensor_value, max_hip_sensor_value, 0, ADC_num_of_bytes);   
  Setpoint_thigh = map(thigh_pot_value, min_thigh_sensor_value, max_thigh_sensor_value, 0, ADC_num_of_bytes);   
  Setpoint_knee = map(knee_pot_value, max_knee_sensor_value, min_knee_sensor_value, 0, ADC_num_of_bytes);

}

void read_pot_to_both_input_setpoint() {
  hip_pot_value = adc->analogRead(hip_pot_pin);  //read the position of the joint sensor
  thigh_pot_value = adc->analogRead(thigh_pot_pin); 
  knee_pot_value = adc->analogRead(knee_pot_pin); 
  Input_hip = map(hip_pot_value, min_hip_sensor_value, max_hip_sensor_value, 0, ADC_num_of_bytes);   
  Input_thigh = map(thigh_pot_value, min_thigh_sensor_value, max_thigh_sensor_value, 0, ADC_num_of_bytes);   
  Input_knee = map(knee_pot_value, max_knee_sensor_value, min_knee_sensor_value, 0, ADC_num_of_bytes);
  Setpoint_hip = Input_hip;
  Setpoint_thigh = Input_thigh;
  Setpoint_knee = Input_knee;

}

void sensorValues_to_setpoint() {
  Setpoint_hip = map(sensors[0], min_hip_sensor_value, max_hip_sensor_value, 0, ADC_num_of_bytes);   
  Setpoint_thigh = map(sensors[1], min_thigh_sensor_value, max_thigh_sensor_value, 0, ADC_num_of_bytes);   
  Setpoint_knee = map(sensors[2], max_knee_sensor_value, min_knee_sensor_value, 0, ADC_num_of_bytes);
}


void printArray(float x[]) {
  for (int i = 0; i < 3; i ++) {
    Serial.print(x[i]);
    Serial.print('\t');
  }
  Serial.println();
}

void printArray(int x[]) {
  for (int i = 0; i < 3; i ++) {
    Serial.print(x[i]);
    Serial.print('\t');
  }
  Serial.println();
}



