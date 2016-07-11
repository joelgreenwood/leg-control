/* Code for Joel's leg test box for Stompy 
This is written for a teensy driving Chris's stepper drivers (half h-bridge)
First order:
 take in sensors
 read joystick
 set current to valves
Second order:
 record sensor/joystick values 
Third order:
 recieve valve current commands independant of joystick via usb 
*/

//Joystick values - analog read at 10 bit resolution 0-1023
int potCenterValue = 1023/2;
int x = potCenterValue;
int y = potCenterValue;
int z = potCenterValue;
int x_pin = 14; //A0
int y_pin = 15; //A1
int z_pin = 16; //A2

//Deadman button -- with the joystick I'll be using a momentary switch on the panel.  It will need to be held down or jumpered 
//to set the joystick as "hot"
int deadMan_pin = 0;
int deadMan = 0; //JOYSTICK is OFF if pin is low (pull high to enable joystick).

//enable pins for motor divers
int enable_pin = 1;
int enable_pin_hip = 2;  //I've decided to enable both driver boards with pin one - makes more sense in the code but I'm leaving this in for now.

//error signals and pins in from motor driver board- M1 SF, M2 SF, M1 (hip) SF
//These are currently unused
int M1SF;
int M2SF;
int M1SF_hip;
int M1SF_pin = 7;
int M2SF_pin = 8;
int M1SF_hip_pin = 11;

//Sensor values from string pots and compliant link
int kneePot;
int thighPot;
int compliantPot;
int hipPot;
int kneePot_pin = 17; //A3
int thighPot_pin = 18;
int compliantPot_pin = 19;
int hipPot_pin = 20;

//M1 FB, M2 FB vlaues - this reports the current output of the driver - 525mv/amp.  We're expecting to only drive ~1.1 amps max so we'll have ~0-1V for 0-1amp.
int M1FB;
int M2FB;
int M1FB_hip;
int M1FB_pin = 21;
int M2FB_pin = 22;
int M1FB_hip_pin = 23;



//I'm creating arrays to make the code easier to deal with in the main loop
int sensor_values[] = {x, y, z, kneePot, thighPot, compliantPot, hipPot, M1FB, M2FB, M1FB_hip};
int sensor_pins[] = {x_pin, y_pin, z_pin, kneePot_pin, thighPot_pin, compliantPot_pin, hipPot_pin, M1FB_pin, M2FB_pin, M1FB_hip_pin};

char* sensor_pin_labels[] = {"x", "y", "z", "kneePot", "thighPot", "compPot", "hipPot", "thighCurFB", "kneeCurFB", "hipCurFB"};
  
//set up deadband and response percentages to joystick
//deadband will be defined as a percentage of range -- e.g. for 0-1023 (10 bit analog read)   
float deadband_percent_of_range = 15; //20% of total range
float deadband = (1023 * (deadband_percent_of_range/100));
int low_threshold = ((1023-deadband)/2);
int high_threshold = (1023/2)+deadband;

//set governor for max duty
//cycle -- e.g. 40 would mean 40% of max valve open at full joystick movement
float govP_thigh = 50;
float govP_knee  =  50;
float govP_hip   =  50;
float governor_percent[] = {govP_knee, govP_thigh, govP_hip}; //percent of max duty cycle for PWM
float governor[] = {410, 410, 410}; //these will be calculated below but I'm giving a 40% default value to them to start

//PWM output values
int kneePWM1 = 0; // e.g. M1 INI 1
int kneePWM2 = 0; // e.g. M1 INI 2
int thighPWM1 = 0; // e.g. M2 INI 1
int thighPWM2 = 0; // e.g. M2 INI 2
int hipPWM1 = 0;
int hipPWM2 = 0;

//These are currently mapped to work on the right front leg.  
//joystick positive x (to the right) == knee out == knee cylindar retract
//neg x == knee in == knee cylindar extend
//pos y == thigh down == thigh cylindar extend
//neg y (joystick pulled down) == thigh up == thigh cylindar retract
//pos z (joystick rotated right) == hip back (swing leg towards rear of robot) == hip cylindar reverse
//neg z (joystick rotated to the left) == hip forward == hip cylindar forward

//teensy pin PWM map to motor diver
//PWM pin 3 == M1 OUT1 (thigh solenoid #1 (S1), which is the outermost solenoid) --- P9 21 on bone cape
//PWM pin 4 == M1 OUT2 (thigh solenoid #2) --- P9 22 on bone cape
//PWM pin 5 == M2 OUT1 (knee solenoid #1) --- P8 13 on bone cape
//PWM pin 6 == M2 OUT2 (knee solenoid #2) --- P8 19 on bone cape
//PWM pin 9 == M1_hip OUT1 -- hip -- this is the "loose" half bridge -- (hip forward) --- these wires are tails coming off my board - not on bone cape
//PWM pin 10 == M1_hip OUT2 -- hip reverse --- this is loose - not on bone cape

const int kneePWM1_retract = 5;                                                          ;
const int kneePWM2_extend = 6;
const int thighPWM1_up = 4;
const int thighPWM2_down = 3; //high (down)
const int hipPWM1_forward = 9; // zlow forward -- right front leg
const int hipPWM2_reverse = 10; //

//These arrays I'm making just so the code is easier to deal with in the main block
int pwm_values[] = {kneePWM1, kneePWM2, thighPWM1, thighPWM2, hipPWM1, hipPWM2};
int high_pwm_values[] = {kneePWM2, thighPWM2, hipPWM2};
int low_pwm_values[] = {kneePWM1, thighPWM1, hipPWM1};
const int pwm_pins[] = {kneePWM1_retract, kneePWM2_extend, thighPWM1_up, thighPWM2_down, hipPWM1_forward, hipPWM2_reverse};
const int high_pwm_pins[] = {kneePWM2_extend, thighPWM2_down, hipPWM2_reverse};
const int low_pwm_pins[] = {kneePWM1_retract, thighPWM1_up, hipPWM1_forward};

//Dither block - reccomended dither from manufacuture is 100Hz @ 150mA
float dither_percent = 0; //percent of dither e.g. 13.6 = 13.6% dither of max, which is ~150mA (1100 mA max current)
float dither_freq = 100; // in Hz
elapsedMicros since_dither_period;
int bit_resolution = pow(2,10)-1;
float dither_val;
float dither_period;

int tmp_dither_value;
int tmp_pwm_value;
int tmp_pwm_readout= 500;

//Sensor reading to angle of joint block
//These sensor values are for the right front leg
int kneePotMax = 934;
int kneePotMin = 148;
float kneeAngleMin = 13;
float kneeAngleMax = 123;
float kneeSensorUnitsPerDeg;

int thighPotMax = 917;
int thighPotMin = 34;
float thighAngleMin = -6;
float thighAngleMax = 84;
float thighSensorUnitsPerDeg;

int hipPotMax = 722;
int hipPotMin = 93;
float hipAngleMin = -40.46;
float hipAngleMax = 40.46;
float hipSensorUnitsPerDeg;

float currentKneeAngle;
float currentThighAngle;
float currentHipAngle;

// kinematics block
/*  Forward kinematics for stompy
x = cos(theta1) * [L1 + L2*cos(theta2) + L3*cos(theta2 + theta3 -180deg)]
y = x * tan(theta1)
z = [L2 * sin(theta2)] + [L3 * sin(theta2 + theta3 - 180deg)] 
 */
const float pi = 3.141593;

//leg link lengths hip, thigh, and knee
const int L1 = 11;
const int L2 = 54;
const int L3 = 72; //72 inches is from knee joint to ankle joint

//angles in degrees - hip is zero when straight out from the robot body
//hip angle is zero when the thigh is parallel to the ground
//knee angle is between thigh and calf (13 degrees fully retracted) 
float theta1;
float theta2;
float theta3;

float currentX;
float currentY;
float currentZ;


void setup()
{   
  Serial.begin(9600);  
  //setup analog wite resolution and PWM frequency
  analogWriteResolution(10);
  //max PWM freqency of the motor driver board is 20kHz
  analogWriteFrequency(3, 20000);
  analogWriteFrequency(5, 20000);
  
  //setup deadman, enable, and error digital I/O pins 
  pinMode(deadMan_pin, INPUT);
  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, LOW);
  pinMode(enable_pin_hip, OUTPUT);
  digitalWrite(enable_pin_hip, LOW);
  pinMode(M1SF_pin, INPUT);
  pinMode(M2SF_pin, INPUT);
  pinMode(M1SF_hip_pin, INPUT);
  
  //calculate govenors for each joint
  for (int i=0; i<3; i++) {
   governor[i] = 1023*(governor_percent[i]/100); 
  }
  
  //dither
  dither_val = (dither_percent/100)*(bit_resolution/2);
  dither_period = 1000000/dither_freq; //number of microseconds for a dither period  

  //sensor units per deg
  kneeSensorUnitsPerDeg = (kneePotMax - kneePotMin) / (kneeAngleMax - kneeAngleMin);
  thighSensorUnitsPerDeg = (thighPotMax - thighPotMin) / (thighAngleMax - thighAngleMin);
  hipSensorUnitsPerDeg = (hipPotMax - hipPotMin) / (hipAngleMax - hipAngleMin);
  
}

//Main loop
void loop() 
{
 //Read deadman first
   deadMan = digitalRead(deadMan_pin);
   //Serial.print("deadman value: ");
   //Serial.println(deadMan); 
   if (deadMan == 0) { // if deadman is off -- joystick is deactivated, then disable driver boards and write zero to PWM lines
     all_off();
   }
  else {
    digitalWrite(enable_pin, HIGH);
    digitalWrite(enable_pin_hip, HIGH);
    //Serial.println("Joystick is enabled!");
  } 
 
// Serial.print(":");
 
 //read all sensors and stuff the values into an array
   for (int i = 0; i < 10; i++) {
     sensor_values[i] = analogRead(sensor_pins[i]);
     
     if (i==1) { //This flips the y axis to match the joystick
       sensor_values[i] = map(sensor_values[i], 0, 1023, 1023, 0); 
     }

     //this is to convert sensor reading into angles
     if (i==3) { //knee pot
      currentKneeAngle = ((sensor_values[i] - kneePotMin) / kneeSensorUnitsPerDeg) + kneeAngleMin;
//      Serial.print("Current Knee Angle: ");
//      Serial.print(currentKneeAngle);
//      Serial.print("\t");
     }

     if (i==4) { //thigh pot
      currentThighAngle = ((sensor_values[i] - thighPotMin) / thighSensorUnitsPerDeg) + thighAngleMin;
//      Serial.print("Current Thigh Angle: ");
//      Serial.print(currentThighAngle);
//      Serial.print("\t");
     }

     if (i==6) { //knee hip
      currentHipAngle = ((sensor_values[i] - hipPotMin) / hipSensorUnitsPerDeg) + hipAngleMin;
//      Serial.print("Current Hip Angle: ");
//      Serial.print(currentHipAngle);
//      Serial.print("\t");
     }     

     //block for printing labels
//     Serial.print(sensor_pin_labels[i]);
//     
//     if (i==7) {
//       Serial.print("@");
//       Serial.print(govP_thigh);
//     }
//     if (i==8) {
//       Serial.print("@");
//       Serial.print(govP_knee);
//     }
//     if (i==9) {
//       Serial.print("@");
//       Serial.print(govP_hip);
//     }
          
     //Serial.print('\t');
     Serial.print(sensor_values[i]);
     Serial.print("\t");
   }
   
//Calculate current (x,y,z) from angles
//convert angles into radians
  
  float theta1R = (currentHipAngle * 71) / 4068; //this takes the current hip angle calculated above from the sensor and converts it into radians
  float theta2R = (currentThighAngle * 71) / 4068;
  float theta3R = (currentKneeAngle * 71) / 4068;
  
  currentX = cos(theta1R) * (L1 + L2*cos(theta2R) + L3*cos(theta2R + theta3R - pi));
  currentY = currentX * tan(theta1R);
  currentZ = (L2 * sin(theta2R)) + (L3 * sin(theta2R + theta3R - pi));
 // Serial.println();
  Serial.print(" Current (x,y,z): (");
  Serial.print(currentX);
  Serial.print(", ");
  Serial.print(currentY);
  Serial.print(", ");
  Serial.print(currentZ);
  Serial.print(")  ");

 //publish current angles 
  Serial.print("current angles (hip, thigh, knee): (");
  Serial.print(currentHipAngle);
  Serial.print(", ");
  Serial.print(currentThighAngle);
  Serial.print(", ");
  Serial.print(currentKneeAngle);
  Serial.print(")   ");
    
//set PWM outputs
  
//  Serial.print("deadband: ");
//  Serial.println(deadband);
//  Serial.print("threashold values:  ");
//  Serial.print(low_threshold);
//  Serial.print("   :  ");
//  Serial.println(high_threshold);

  
  //set dither add or subtract from pwm depending on elapsed time
  float half_dither_period = dither_period/2;
  
  dither_val = (dither_percent/100)*(bit_resolution/2);
//  Serial.print("dither val: ");
//  Serial.println(dither_val);
  
  int tmp_pwm_readout_value;

//  Serial.print("tmp pwm readout: ");
//  Serial.println(tmp_pwm_readout);
  
//  float top = (bit_resolution - );
//  Serial.print("bitresolution - half_dither_peroiod");
//  Serial.println(top);
  
   if (tmp_pwm_readout <= dither_val) {
     dither_val = tmp_pwm_readout; 
   }
   else if (dither_val >= (bit_resolution - tmp_pwm_readout)) {
     dither_val = (bit_resolution - tmp_pwm_readout);
   }
  
//  Serial.print("new dither value");
//  Serial.println(dither_val);
  
   //This checks to see if the dither value should be positive or negative depending on how much time has elapsed
   if (since_dither_period < half_dither_period) {
    tmp_dither_value = dither_val;
   }
   else if (since_dither_period >= half_dither_period && since_dither_period <= dither_period) {
    tmp_dither_value = -dither_val;
   }
   else { 
    since_dither_period = 0;
  } 
 
//  Serial.print("dither value: ");
//  Serial.println(tmp_dither_value);
  
  
  for (int i = 0; i < 3; i++) {
//     Serial.print("value: ");
//     Serial.println(sensor_values[i]);
     
     if (deadMan == 1) {
       if (sensor_values[i] >= high_threshold) {
//         Serial.print("governor: ");
//         Serial.println(governor[i]);
         high_pwm_values[i] = map(sensor_values[i], high_threshold, 1023, 0, governor[i]);
//         Serial.print("mapped pwm value: ");
//         Serial.println(high_pwm_values[i]);
         high_pwm_values[i] = high_pwm_values[i] + tmp_dither_value;
         analogWrite(high_pwm_pins[i], high_pwm_values[i]);
//         Serial.print(tmp_pwm_value);
//         Serial.println("crossed high threshold");
       } 
       else if (sensor_values[i] <= low_threshold) {
         low_pwm_values[i] = map(sensor_values[i], 0, low_threshold, governor[i], 0);
//         Serial.print("mapped pwm value: ");
//         Serial.println(low_pwm_values[i]);
         low_pwm_values[i] = low_pwm_values[i] + tmp_dither_value;
         analogWrite(low_pwm_pins[i], low_pwm_values[i]);
//         Serial.println("crossed low threashold"); 
       }
       else {
         analogWrite(low_pwm_pins[i], 0);
         analogWrite(high_pwm_pins[i], 0);
         high_pwm_values[i] = 0;
         low_pwm_values[i] = 0;
         
//         Serial.println("in deadband - turning pwm high and low off");
       }
    }
    
    Serial.print(high_pwm_values[i]);
    Serial.print('\t');
    Serial.print(low_pwm_values[i]);
    Serial.print('\t');
    
    
    
  }
  
  
   Serial.println();
 
 delay(2); //I have a delay here becuase I've found polling the analog pins at ~1kHz tends to mess things up.  Here we're polling at 500Hz wich is way fast enough for human reaction (joystick) times.
}

//This is a simple program to make sure the driver board is not enabled if the joystick is turned off (deadMan is low)
void all_off() {
  digitalWrite(enable_pin, LOW);
  digitalWrite(enable_pin_hip, LOW);
  for (int i = 0; i < 6; i++) {
    analogWrite(pwm_pins[i], 0);
  }
  //Serial.println("Joystick is deactivated -- all off!");
}





