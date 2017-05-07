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
int x_pin = 31; //(old A0, 14) - now 31 A20  
int y_pin = 30; //(old A1, 15) - now 30 A19 
int z_pin = 29; //(old A2, 16) - now 29 A18

//Deadman button -- with the joystick I'll be using a momentary switch on the panel.  It will need to be held down or jumpered 
//to set the joystick as "hot"
int deadMan_pin = 25; // was 0 - now 25
int deadMan = 0; //JOYSTICK is OFF if pin is low (pull high to enable joystick).

//enable pins for motor divers
int enable_pin = 8; //(joel's test board was 1 -- now Pin 8 for Chris's board)
int enable_pin_hip = 2;  //I've decided to enable both driver boards with pin one - makes more sense in the code but I'm leaving this in for now.

//error signals and pins in from motor driver board- M1 SF, M2 SF, M1 (hip) SF
//These are currently unused
int M1SF;
int M2SF;
int M1SF_hip;
int M1SF_pin = 7;
//int M2SF_pin = 8;
int M1SF_hip_pin = 11;

//Sensor values from string pots and compliant link
int kneePot;
int thighPot;
int compliantPot;
int hipPot;
int kneePot_pin = 17; //A3
int thighPot_pin = 15;
int compliantPot_pin = 16;
int hipPot_pin = 20;

 //Pressor Sensors
int pressureSensorPin_1 = 28; //knee extend
int pressureSensorPin_2 = 27; //thigh extend
int pressureSensorPin_3 = 26;  //thigh retract
int pressureSensorPin_4 = A12;  //knee retract
int pressureSensor_1, pressureSensor_2, pressureSensor_3, pressureSensor_4;
int pressure_values[] = {pressureSensor_1, pressureSensor_2, pressureSensor_3, pressureSensor_4};
int pressure_pins[] = {pressureSensorPin_1, pressureSensorPin_2, pressureSensorPin_3, pressureSensorPin_4};


//M1 FB, M2 FB vlaues - this reports the current output of the driver - 525mv/amp.  We're expecting to only drive ~1.1 amps max so we'll have ~0-1V for 0-1amp.
int M1FB;
int M2FB;
int M1FB_hip;
int M1FB_pin = 21;
int M2FB_pin = 22;
int M1FB_hip_pin = 23;


//This is the array to select which values to print
int sensorPWMpressure[17];
elapsedMillis since_print;
int print_period = 100; //in ms - so 100 is priting at 10Hz

/* This is the list of values:
0   Pressure 1  //knee extend
1   Pressure 2  //thigh extend
2   Pressure 3  //thigh retract
3   Pressure 4  //knee retract
4   X joystick  
5   Y joystick  
6   Z joystick  
7   Thigh Pot 
8   Knee Pot  
9   Compliant Pot 
10  Hip Pot 
11  knee PWM-1  
12  thigh PWM-1 
13  hip PWM-1 
14  knee PWM-2  
15  thigh PWM-2 
16  hip PWM-2
*/

char* print_lables[] = {"PreKneeExt", "PreThighExt", "PreThighRet", "PreKneeRet", "Xjoy", "Yjoy", "Zjoy", "ThighPot", "KneePot", "CompliantPot", "HipPot", "KneePWM1", "ThighPWM1", "HipPWM1", "KneePWM2", "ThighPWM2", "HipPWM2"};
 
int include[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
//int include[] = {0,2,5,7,9,12,15};
//for printing only a subset of values
int size = sizeof(include)/sizeof(int);

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
float govP_knee  =  70;
float govP_hip   =  70;
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

// string pots are S (increase as they extend)
// knee out = low
// thigh dead bug = low
// hip back = low

const int high_pwm_pins[] = {kneePWM2_extend, thighPWM2_down, hipPWM2_reverse};
const int low_pwm_pins[] = {kneePWM1_retract, thighPWM1_up, hipPWM1_forward};

//Dither block - reccomended dither from manufacuture is 100Hz @ 150mA
float dither_percent = 10; //percent of dither e.g. 13.6 = 13.6% dither of max, which is ~150mA (1100 mA max current)
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


//range limits {knee, thigh, hip}
//int joint_pot_low_limits[] = {160, 44, 103};
//int joint_pot_high_limits[] = {925, 907, 712};
#define KNEE_LIMIT 0
#define THIGH_LIMIT 1
#define HIP_LIMIT 2
int potLowLimits[] = {kneePotMin + 10, thighPotMin + 10, hipPotMin + 10};
int potHighLimits[] = {kneePotMax - 10, thighPotMax - 10, hipPotMax - 10};
// knee extend decreases string pot, when at low limit, don't allow high_pwm_pin
// thigh up decreases string pot, when at low limit, don't allow low_pwm_pin
// hip back decreases string pot, when at low limit, don't allow high_pwm_pin
bool potAtLowLimit[] = {false, false, false};
bool potAtHighLimit[] = {false, false, false};

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
 // pinMode(M2SF_pin, INPUT);
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

  //print lables
  delay(3000);
  for (int i; i < size; i++) {
                Serial.print(print_lables[include[i]]);
                Serial.print("\t"); 
        }
  Serial.println();
  
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

for (int i = 0; i < 4; i ++) { //read pressure pins
  pressure_values[i] = analogRead(pressure_pins[i]);
 }

for (int i = 0; i < 4; i++) {
  sensorPWMpressure[i]  = pressure_values[i];
}
 
 //read all sensors and stuff the values into an array
   for (int i = 0; i < 10; i++) {
     sensor_values[i] = analogRead(sensor_pins[i]);
     
     if (i==1) { //This flips the y axis to match the joystick
       sensor_values[i] = map(sensor_values[i], 0, 1023, 1023, 0); 
     }

    //print sensors
    if (i < 7) {
      sensorPWMpressure[i+4] = sensor_values[i];
    }

     //this is to convert sensor reading into angles
     if (i==3) { //knee pot
      potAtLowLimit[KNEE_LIMIT] = (sensor_values[i] < potLowLimits[KNEE_LIMIT]);
      potAtHighLimit[KNEE_LIMIT] = (sensor_values[i] > potHighLimits[KNEE_LIMIT]);
      currentKneeAngle = ((sensor_values[i] - kneePotMin) / kneeSensorUnitsPerDeg) + kneeAngleMin;
      //Serial.print("Current Knee Angle: ");
      //Serial.print(currentKneeAngle);
      //Serial.print("\t");
     }

     if (i==4) { //thigh pot
      potAtLowLimit[THIGH_LIMIT] = (sensor_values[i] < potLowLimits[THIGH_LIMIT]);
      potAtHighLimit[THIGH_LIMIT] = (sensor_values[i] > potHighLimits[THIGH_LIMIT]);
      currentThighAngle = ((sensor_values[i] - thighPotMin) / thighSensorUnitsPerDeg) + thighAngleMin;
      //Serial.print("Current Thigh Angle: ");
      //Serial.print(currentThighAngle);
      //Serial.print("\t");
     }

     if (i==6) { //knee hip
      potAtLowLimit[HIP_LIMIT] = (sensor_values[i] < potLowLimits[HIP_LIMIT]);
      potAtHighLimit[HIP_LIMIT] = (sensor_values[i] > potHighLimits[HIP_LIMIT]);
      currentHipAngle = ((sensor_values[i] - hipPotMin) / hipSensorUnitsPerDeg) + hipAngleMin;
    //Serial.print("Current Hip Angle: ");
    //Serial.print(currentHipAngle);
    //Serial.print("\t");
     }     
   }
   
//Calculate current (x,y,z) from angles
  //convert angles into radians
  /*  
    float theta1R = (currentHipAngle * 71) / 4068; //this takes the current hip angle calculated above from the sensor and converts it into radians
    float theta2R = (currentThighAngle * 71) / 4068;
    float theta3R = (currentKneeAngle * 71) / 4068;
    
    currentX = cos(theta1R) * (L1 + L2*cos(theta2R) + L3*cos(theta2R + theta3R - pi));
    currentY = currentX * tan(theta1R);
    currentZ = (L2 * sin(theta2R)) + (L3 * sin(theta2R + theta3R - pi));
   // Serial.println();
    Serial.print("(x,y,z)");
    Serial.print("\t");
    Serial.print(currentX);
    Serial.print("\t");
    Serial.print(currentY);
    Serial.print("\t");
    Serial.print(currentZ);
    Serial.print("\t");

   //publish current angles 
    Serial.print("angles (hip, thigh, knee)\t");
    Serial.print(currentHipAngle);
    Serial.print("\t");
    Serial.print(currentThighAngle);
    Serial.print("\t");
    Serial.print(currentKneeAngle);
    Serial.print("\t");
  */

  //set dither add or subtract from pwm depending on elapsed time
    float half_dither_period = dither_period/2;
    
    dither_val = (dither_percent/100)*(bit_resolution/2);
    int tmp_pwm_readout_value;

     if (tmp_pwm_readout <= dither_val) {
       dither_val = tmp_pwm_readout; 
     }
     else if (dither_val >= (bit_resolution - tmp_pwm_readout)) {
       dither_val = (bit_resolution - tmp_pwm_readout);
     }
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
 
  for (int i = 0; i < 3; i++) {
     
     if (deadMan == 1) {
       // knee extend decreases string pot, when at low limit, don't allow high_pwm_pin
       // thigh up decreases string pot, when at low limit, don't allow low_pwm_pin
       // hip back decreases string pot, when at low limit, don't allow high_pwm_pin
       if (sensor_values[i] >= high_threshold) {
          //Serial.print("governor: ");
          //Serial.println(governor[i]);
         high_pwm_values[i] = map(sensor_values[i], high_threshold, 1023, 0, governor[i]);
          //Serial.print("mapped pwm value: ");
          //Serial.println(high_pwm_values[i]);
         high_pwm_values[i] = high_pwm_values[i] + tmp_dither_value;
         // check if joint is at limit
         if ((i == 0) && (potAtLowLimit[0])) high_pwm_values[i] = 0;
         if ((i == 1) && (potAtHighLimit[0])) high_pwm_values[i] = 0;
         if ((i == 2) && (potAtLowLimit[0])) high_pwm_values[i] = 0;
         analogWrite(high_pwm_pins[i], high_pwm_values[i]);
          //Serial.print(tmp_pwm_value);
          //Serial.println("crossed high threshold");
       } 
       else if (sensor_values[i] <= low_threshold) {
         low_pwm_values[i] = map(sensor_values[i], 0, low_threshold, governor[i], 0);
          //Serial.print("mapped pwm value: ");
          //Serial.println(low_pwm_values[i]);
         low_pwm_values[i] = low_pwm_values[i] + tmp_dither_value;
         // check if joint is at limit
         if ((i == 0) && (potAtHighLimit[0])) low_pwm_values[i] = 0;
         if ((i == 1) && (potAtLowLimit[0])) low_pwm_values[i] = 0;
         if ((i == 2) && (potAtHighLimit[0])) low_pwm_values[i] = 0;
         analogWrite(low_pwm_pins[i], low_pwm_values[i]);
          //Serial.println("crossed low threashold"); 
       }
       else {
         analogWrite(low_pwm_pins[i], 0);
         analogWrite(high_pwm_pins[i], 0);
         high_pwm_values[i] = 0;
         low_pwm_values[i] = 0;
         
       }
    }
  }

  if (deadMan == 1) {
    for (int i = 0; i < 3; i ++) {
      sensorPWMpressure[i + 11] = low_pwm_values[i];
    }
    for (int i = 0; i < 3; i ++) {
      sensorPWMpressure[i + 14] = high_pwm_values[i];
    }
  }

if (since_print >= print_period) {
        for (int i = 0; i < size; i++) {
                Serial.print(sensorPWMpressure[include[i]]);
                Serial.print("\t"); 
        }
        since_print = 0;
        Serial.println();
}


while (Serial.available() > 0) {
        // do stuff with serial here if needed

        //look for a carriage return - to print lables
        if (Serial.read() == '\r') {
                Serial.println("**************");
                for (int i; i < size; i++) {
                        Serial.print(print_lables[include[i]]);
                        Serial.print("\t"); 
                }
                Serial.println();      
        }
}
 
delay(1); //I have a delay here becuase I've found polling the analog pins at ~1kHz tends to mess things up.  Here we're polling at 500Hz wich is way fast enough for human reaction (joystick) times.
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






