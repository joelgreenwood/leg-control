
//Dither block - reccomended dither from manufacuture is 100Hz @ 150mA
float dither_percent = 10; //percent of dither e.g. 13.6 = 13.6% dither of max, which is ~150mA (1100 mA max current)
float dither_freq = 100; // in Hz -- recomended to be 100 Hz
elapsedMicros since_dither_period;
//int bit_resolution = pow(2,10)-1;
float dither_val;
float dither_period;
float half_dither_period;

int tmp_dither_value;
int tmp_pwm_value;
int tmp_pwm_readout= 500;


// Desired (x,y,z) position
float x;
float y;
float z;


/*  Inverse kinematics for stompy
 *   theta1 = arctan(y/x)
 *   theta2 = arctan(z/x1) + arccos([L2^2 + r^2 - L3^2] / [2*L2*r])
 *        where: x1 = [y/sin(theta1)] - L1
 *               r = z/[sin(beta)]
 *               beta = arctan(z/x1)
 *               alpha = arccos[(L2^2 + r^2 - L3^2) / (2*L2*r)]
 *   theta3 = arccos[(L3^2 + L2^2  - r^2) / [2*L3*L2]
 *   
 *   There are three cases that must be caught for this math to work.
 *   1) If theta1 (hip angle) equals zero then x1 = (x - L1).
 *   2) If z equals zero then r = x1.
 *   3) If x = 0 then the hip angle is degenerate so I add .0001" to x when x = 0 to avoid these degenerate cases.
 */

/* to convert deg to rad and rad to deg
 *  radians = (degrees * 71) / 4068
 *  degrees = (radians * 4068) / 71
*/


// Block for reading sensors and setting the PWM to drive the solenoids
float bit_resolution = pow(2,10)-1;

float PWM_percent[] = {50, 50, 40}; //PWM percentage to drive the solenoids at - this will translate into speed
float PWM_value[] = {0, 0, 0};
float tmp_PWM_value[] = {0, 0, 0};

int hipGoal;
int thighGoal;
int kneeGoal;

//int sensorGoal[] = {kneeGoal, thighGoal, hipGoal}; //old order - fixing on next lin
int sensorGoal[] = {hipGoal, thighGoal, kneeGoal};
int goingHot[] = {0,0,0};
int kneeSensorReading;
int thighSensorReading;
int hipSensorReading;
//int sensorReading[] = {kneeSensorReading, thighSensorReading, hipSensorReading}; // old order - fixing on next line
int sensorReading[] = {hipSensorReading, thighSensorReading, kneeSensorReading};
int kneeSensorPin = 17;
int thighSensorPin = 18;
int hipSensorPin = 20;
//int sensorPin[] = {kneeSensorPin, thighSensorPin, hipSensorPin}; // old order - fixing on next line
int sensorPin[] = {hipSensorPin, thighSensorPin, kneeSensorPin};
// this is the distance in sensor reading that is close enough for directed movement
// I am putting this here so we can avoid chasing our tails early in positional control
int closeEnough = 1; 


//These are mapped to the right front leg
int kneePWM2_extend = 6;
int kneePWM1_retract = 5;
int thighPWM2_down = 3;
int thighPWM1_up = 4;
int hipPWM1_forward = 9;
int hipPWM2_reverse = 10;

const int pwm_pins[] = {kneePWM1_retract, kneePWM2_extend, thighPWM1_up, thighPWM2_down, hipPWM1_forward, hipPWM2_reverse};
//int moving_joint[] = {0,0,0,0,0,0}; //same mapping as pwm_pins above - used to keep vlaves that are on and will need to be turned off at goal

//Deadman button -- with the joystick I'll be using a momentary switch on the panel.  It will need to be held down or jumpered 
//to set the joystick as "hot"
int deadMan_pin = 0;
int deadMan = 0; //JOYSTICK is OFF if pin is low (pull high to enable joystick).

//enable pins for motor divers
int enable_pin = 1;
int enable_pin_hip = 2;

int M1FB_pin = 21;
int M2FB_pin = 22;
int M1FB_hip_pin = 23;
int current_reading_pin[] = {M2FB_pin, M1FB_pin, M1FB_hip_pin};

//values for home position of leg
//x home is knee fully retracted (cylindar fully extended) i.e. large pot value 
//y home is thigh fully up (cylindar fully retracted) i.e. small pot value
//z home is hip in the ~middle i.e. mid pot value
int homePosition[] = {900, 200, 350}; 

//Sensor reading to angle of joint block
//These sensor values are for the right front leg

int hipPotMax = 722;
int hipPotMin = 93;
float hipAngleMin = -40.46;
float hipAngleMax = 40.46;
float hipSensorUnitsPerDeg;

int thighPotMax = 917;
int thighPotMin = 34;
float thighAngleMin = -6;
float thighAngleMax = 84;
float thighSensorUnitsPerDeg;

int kneePotMax = 934;
int kneePotMin = 148;
float kneeAngleMin = 13;
float kneeAngleMax = 123;
float kneeSensorUnitsPerDeg;





//float currentKneeAngle;
//float currentThighAngle;
//float currentHipAngle;

float currentAngles[] = {0.00, 0.00, 0.00};
float currentAnglesR[] = {0.00, 0.00, 0.00};

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
//float theta1;
//float theta2;
float theta3;
//float theta1R; 
//float theta2R; 
float theta3R; 

float currentX;
float currentY;
float currentZ;

float currentPos[] = {0.00, 0.00, 0.00};

void setup() {
  Serial.begin(9600); 
  Serial.setTimeout(10); 

  for (int i = 0; i < 3; i++) {
   PWM_value[i] = (PWM_percent[i]/100) * bit_resolution;
  }
 //setup analog wite resolution and PWM frequency

///*disable when testing on < teensy 3.0
 
  analogWriteResolution(10);
  //max PWM freqency of the motor driver board is 20kHz
  analogWriteFrequency(3, 20000);
  analogWriteFrequency(5, 20000);
 
//*/ 
 
  //setup deadman, enable, and error digital I/O pins 
  pinMode(deadMan_pin, INPUT);
  pinMode(enable_pin, OUTPUT);
  digitalWrite(enable_pin, LOW);
  pinMode(enable_pin_hip, OUTPUT);
  digitalWrite(enable_pin_hip, LOW);

  //dither
  dither_val = (dither_percent/100)*(bit_resolution/2);
  dither_period = 1000000/dither_freq; //number of microseconds for a dither period  
  half_dither_period = dither_period/2;

  //sensor units per deg
  hipSensorUnitsPerDeg = (hipPotMax - hipPotMin) / (hipAngleMax - hipAngleMin);
  thighSensorUnitsPerDeg = (thighPotMax - thighPotMin) / (thighAngleMax - thighAngleMin);
  kneeSensorUnitsPerDeg = (kneePotMax - kneePotMin) / (kneeAngleMax - kneeAngleMin);
}

void loop() {
   //Read deadman first

  

   
   deadMan = digitalRead(deadMan_pin);
   //deadMan = 1;  //This is here for debugging but can be seriously dangious on the real robot if not commented out
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

  //take sensor readings
  Serial.print("(sensor H,T,K)\t");
  for (int i = 0; i < 3; i++) {
    sensorReading[i] = analogRead(sensorPin[i]);
    Serial.print(sensorReading[i]);
    Serial.print("\t");

    currentAngles[i] = sensorToAngle(i, sensorReading[i] );
    currentAnglesR[i] = ((currentAngles[i] * 71)/4068);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> commented out to test dither  
   // Serial.print("current angles (hip, thigh, knee): (");
//    Serial.print(currentAngles[0]);
//    if (i == 2) {
//      Serial.print(")   ");
//    }
//    else {
//      Serial.print(", ");
//    }
    
//    if (i==0) { //this is the hip pot
//      currentHipAngle = ((sensorReading[i] - hipPotMin) / hipSensorUnitsPerDeg) + hipAngleMin;
//      theta1R = (currentHipAngle * 71) / 4068; 
//    }
//    if (i==1) { //this is the thigh pot
//      currentThighAngle = thighAngleMax - ((sensorReading[i] - thighPotMin) / thighSensorUnitsPerDeg);
//      theta2R = (currentThighAngle * 71) / 4068;
//    }
//    if (i==2) { //this is the knee pot
//      currentKneeAngle = kneeAngleMax - ((sensorReading[i] - kneePotMin) / kneeSensorUnitsPerDeg);
//      theta3R = (currentKneeAngle * 71) / 4068; //this takes the current knee angle calculated above from the sensor and converts it into radians
//    }
    
  }

//      //publish current angles 
  Serial.print("angle (H, T, K)\t");
 // Serial.print("\t");
  Serial.print(currentAngles[0]);
  Serial.print("\t");
  Serial.print(currentAngles[1]);
  Serial.print("\t");
  Serial.print(currentAngles[2]);
  Serial.print("\t");
  
  //publish current (x,y,z);
  currentX = cos(currentAnglesR[0]) * (L1 + L2*cos(currentAnglesR[1]) + L3*cos(currentAnglesR[1] + currentAnglesR[2] - pi));
  currentY = currentX * tan(currentAnglesR[0]);
  currentZ = (L2 * sin(currentAnglesR[1])) + (L3 * sin(currentAnglesR[1] + currentAnglesR[2] - pi));

  //short version of angles and x,y,z
  //format:
  //x y z hipangle  thighangle  kneeangle
  Serial.print("(x y z)");
  Serial.print("\t");
  Serial.print(currentX);
  Serial.print("\t");
  Serial.print(currentY);
  Serial.print("\t");
  Serial.print(currentZ);
  Serial.print("\t");
//  Serial.print(currentHipAngle);
//  Serial.print('\t');
//  Serial.print(currentThighAngle);
//  Serial.print('\t');
//  Serial.println(currentKneeAngle);


  //dither block
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

  //Serial.print(millis());
  //Serial.print("\t");
  //Serial.print("dither value: ");
  //Serial.println(tmp_dither_value);
  
  while (Serial.available() > 0) {
    // look for first valid integar to be x
    x = Serial.parseInt();
    if (x == 0) {
      x = x + .0001;
      //Serial.println("x = 0 so .0001 was added to avoid degenerate case");
    }
    // next position
    y = Serial.parseInt();
    z = Serial.parseInt();
    PWM_percent[2] = Serial.parseInt();
    
    Serial.print("goal (x,y,z, PWM percent): ");
    Serial.print("(");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.print(", ");
    Serial.print(PWM_percent[2]);
    Serial.println(")");
    

    //look for newline or carriage return 
    if (Serial.read() == '\n') {
      //now calculate theta1, theta2, and theta3
       
     //theta1
      float theta1R = atan(y/x);
      //convert to degrees
      float theta1 = (theta1R * 4068) / 71;
      Serial.print("theta1 = ");
      Serial.println(theta1);
      //angle goal to pot reading
      hipGoal = ((theta1 - hipAngleMin) * hipSensorUnitsPerDeg) + hipPotMin;
      if (hipGoal < hipPotMin || hipGoal > hipPotMax) {
        Serial.println("HIP GOAL OUT OF RANGE!!");
        hipGoal = constrain(hipGoal, hipPotMin, hipPotMax);
        Serial.print("constrained goal: ");
        Serial.println(hipGoal);
      }
      sensorGoal[0] = hipGoal;
      Serial.print("  hip sensor goal: ");
      Serial.println(hipGoal); 
      goingHot[0] = 1; 
      
     //theta2
      float r;
      float x1;
      if (theta1R == 0) {
        x1 = (x - L1);
      }
      else {
        x1 = (y/sin(theta1R)) - L1;
      } 
      x1 = abs(x1);    
      float beta = atan(z/x1);
      if (x == L1) {
        beta = -(pi/2);
      }
      else if (x < L1) {
        if (z == 0) {
        r = x1;
        }
        else {
        r = z/sin(beta);
        }
        r = abs(r);
        float gama = asin(x1/r);
        beta = -(gama + pi/2);
      }
      else {
        beta = atan(z/x1);

      }
      if (z == 0) {
        r = x1;
      }
      else {
        r = z/sin(beta);
      }
      r = abs(r); 
      float theta2R = beta + acos((sq(L2) + sq(r) - sq(L3))/(2*L2*r));
      float theta2 = (theta2R * 4068) / 71;
      Serial.print("theta2 = ");
      Serial.println(theta2);
      //thighGoal is sensor reading at goal angle
      thighGoal = thighPotMin + ((thighAngleMax - theta2) * thighSensorUnitsPerDeg);
      if (thighGoal < thighPotMin || thighGoal > thighPotMax) {
        Serial.println("THIGH GOAL OUT OF RANGE!!");
        thighGoal = constrain(thighGoal, thighPotMin, thighPotMax);
        Serial.print("constrained goal: ");
        Serial.println(thighGoal);
      }
      sensorGoal[1] = thighGoal;
      Serial.print("  thigh sensor goal: ");
      Serial.println(thighGoal);
      goingHot[1] = 1;

      //theta3
      theta3R = acos((sq(L3) + sq(L2) - sq(r)) / (2*L3*L2));
      theta3 = (theta3R * 4068) / 71;
      Serial.print("theta3 = ");
      Serial.print(theta3);
      kneeGoal= ((kneeAngleMax - theta3) * kneeSensorUnitsPerDeg) + kneePotMin;
      //kneeGoal = ((theta3 - kneeAngleMin) * kneeSensorUnitsPerDeg) + kneePotMin;
      if (kneeGoal < kneePotMin || kneeGoal > kneePotMax) {
        Serial.println("KNEE GOAL OUT OF RANGE!!");
       // kneeGoal = constrain(kneeGoal, kneePotMin, kneePotMax);
       // Serial.print("constrained goal: ");
        Serial.println(kneeGoal);
      }
      sensorGoal[2] = kneeGoal;
      Serial.print("  knee sensor goal: ");
      Serial.println(kneeGoal);
      goingHot[2] = 1;
    }
  }
  
   //dither block
   
     
   //Now check sensors and compare the readings to sensor goals set above
   for (int i = 0; i < 3; i++) {
    if (goingHot[i] == 1 && deadMan == 1) {
      //dither block
      tmp_PWM_value[i] = PWM_value[i] + tmp_dither_value;
      
        Serial.print("PWM value\t");
        Serial.println(tmp_PWM_value[i]);

      
    //compare current sensor reading to goal.
    //if sensor reading is within a % of goal then stop and set goingHot to zero
    //read sensor
    sensorReading[i] = analogRead(sensorPin[i]);
    //compare sensor reading to goal and only move if not close enough
    if (abs(sensorReading[i] - sensorGoal[i]) >= closeEnough) {
      
      if (sensorReading[i] > sensorGoal[i]) {
        if (i == 2) { // i == 2 is the knee
          analogWrite(kneePWM2_extend, tmp_PWM_value[i]);
    //      moving_joint[1] = 1; 
          print_reading(i, sensorReading[i], sensorGoal[i], "extending knee");
        }
        else if (i == 1) { // i == 1 is the thigh
          analogWrite(thighPWM1_up, tmp_PWM_value[i]);
   //       moving_joint[2] = 1;
          print_reading(i, sensorReading[i], sensorGoal[i], "bringing thigh up");
        }      
        else if (i == 0) { // hip
          analogWrite(hipPWM2_reverse, tmp_PWM_value[i]);
   //       moving_joint[5] = 1;
          print_reading(i, sensorReading[i], sensorGoal[i], "bringing hip back - reverse");
        }
      }
      else if (sensorReading[i] < sensorGoal[i]) {
        if (i == 2) { // i == 0 is the knee
          analogWrite(kneePWM1_retract, tmp_PWM_value[i]); 
  //        moving_joint[0] = 1;
          print_reading(i, sensorReading[i], sensorGoal[i], "retracting knee");
        }
        else if (i == 1) { // i == 1 is the thigh
          analogWrite(thighPWM2_down, tmp_PWM_value[i]);
 //         moving_joint[3] = 1;
          print_reading(i, sensorReading[i], sensorGoal[i], "bringing thigh down");
        }      
        else if (i == 0) { // hip
          analogWrite(hipPWM1_forward, tmp_PWM_value[i]);
 //         moving_joint[4] = 1;
          print_reading(i, sensorReading[i], sensorGoal[i], "bringing hip forward");
        }
      }
    }
    else {
      goingHot[i] = 0;  //if joint was in motion and the goal was reached - turn motion flag and solenoids off
      if (i == 2) { //if knee was moving - turn both solenoids off
        analogWrite(kneePWM2_extend, 0);
        analogWrite(kneePWM1_retract, 0);
      }
      if (i == 1) {
        analogWrite(thighPWM2_down, 0);
        analogWrite(thighPWM1_up, 0);
      } 
      if (i == 0) {
        analogWrite(hipPWM1_forward, 0);
        analogWrite(hipPWM2_reverse, 0);
      }
      Serial.println(">>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<");
      Serial.println("reached goal");
    }
   } 
  }
  
  Serial.println();
  //this is a short delay so I can read the serial while programing
  delay(2);
}


//This is a simple program to make sure the driver board is not enabled if the joystick is turned off (deadMan is low)
void all_off() {
  digitalWrite(enable_pin, LOW);
  digitalWrite(enable_pin_hip, LOW);
  for (int i = 0; i < 6; i++) {
    analogWrite(pwm_pins[i], 0);
  }
  for (int i = 0; i < 3; i++) {
  goingHot[i] = 0;
  }
  //Serial.println("Joystick is deactivated -- all off!");
}


//int i, int sensor, int goal, String 'joint'

void print_reading(int i, int sensor, int goal, String joint) {
  Serial.print(i);
  Serial.print('\t');
  Serial.print(sensor);
  Serial.print('\t');
  Serial.print(goal);
  Serial.print('\t');
  Serial.println(joint);
}

float sensorToAngle(int joint, int sensorReading) {
  static int cylinderMinLength; 
  static int cylinderMaxLength;
  static float cylinderTravel;
  float currentCylinderLength;
  static float C1; //This is the length of one side of the triangle - not the cylinder
  static float C2; ; //This is the other side
  static float beta; //This is a constant angle between the changing angle calculated here (alpha) and the desired angle - e.g. theta1 for hip
  float alpha; //This is the angle that is changing when the cylinder length changes
  static float deadBugTheta; //This is the desired angle (e.g. theta2 plus beta and alpha) at deadbug (fully retracted and centered)
  float theta; //This is the final ouput angle 
  static int sensorMax;
  static int sensorMin;

  switch (joint) {
        case 0:
          //Serial.println("HIP");
          sensorMax = 722;
          sensorMin = 93;
          cylinderMinLength = 16;
          cylinderTravel = 8;
          C1 = 6.83905;
          C2 = 19.62051;
          beta = 77.92503;
          deadBugTheta = -7.8707;
          break;
        case 1:
          //Serial.println("THIGH");
          sensorMax = 917;
          sensorMin = 34;
          cylinderMinLength = 24;
          cylinderTravel = 14;
          C1 = 10.21631;
          C2 = 33.43093;
          beta = 26.6594;
          deadBugTheta = 129.6249;
          break;
        case 2:
          //Serial.println("KNEE");
          sensorMax = 934;
          sensorMin = 148;
          cylinderMinLength = 20;
          cylinderTravel = 12;
          C1 = 25.6021;
          C2 = 7.4386;
          beta = 35.8658;
          deadBugTheta = 194.1805;
          break;
      }
        float sensorUnitsPerInch = (sensorMax - sensorMin) / (cylinderTravel);
//      Serial.print("sensor units per inch: ");
//      Serial.println(sensorUnitsPerInch);
      int sensorReading_sensorMin = sensorReading - sensorMin;
//      Serial.print("sensorReading_sensorMin: ");
//      Serial.println(sensorReading_sensorMin);
      
      currentCylinderLength = ((sensorReading - sensorMin) / sensorUnitsPerInch) + cylinderMinLength;
//      Serial.print("currentCylindarLength: ");
//      Serial.println(currentCylinderLength);
      
      float alphaR = acos((sq(C1) + sq(C2) - sq(currentCylinderLength))/(2*C1*C2));
      alpha = (alphaR * 4068) / 71;
      if (joint == 0) { 
        //The hip is inverted logic because the sensor is at minimum when the joint is at minimum angle 
        //The other joints are at their maximum angle whne the sensor is fully retracted                
        theta = deadBugTheta - beta + alpha;
      }
      else {
        theta = deadBugTheta - beta - alpha;
      }
//      Serial.print("alpha: ");
//      Serial.println(alpha);
//      Serial.print("theta in void: ");
//      Serial.println(theta);
      return theta;
}


