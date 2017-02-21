

float angleToSensor(int joint, float angle) {
  static int cylinderMinLength; 
  static int cylinderMaxLength;
  static float cylinderTravel;
  float currentCylinderLength;
  static float C1; //This is the length of one side of the triangle - not the cylinder
  static float C2; ; //This is the other side
  static float beta; //This is a constant angle between the changing angle calculated here (alpha) and the desired angle - e.g. theta1 for hip
  float alpha; //This is the angle that is changing when the cylinder length changes
  static float deadBugTheta; //This is the desired angle (e.g. theta2 plus beta and alpha) at deadbug (fully retracted and centered)
  static float sensorMax;
  static float sensorMin;
  float sensorGoal; //This is the final output from joint and angle input

  switch (joint) {
        case 0:
          //Serial.println("HIP");
          sensorMax = 722.00;
          sensorMin = 93.00;
          cylinderMinLength = 16;
          cylinderTravel = 8;
        //  cylinderMaxLength = cylinderMinLength + cylinderTravel;
          C1 = 6.83905;
          C2 = 19.62051;
          beta = 77.92503;
          deadBugTheta = -7.8707;
          break;
        case 1:
          //Serial.println("THIGH");
          sensorMax = 917.00;
          sensorMin = 34.00;
          cylinderMinLength = 24;
          cylinderTravel = 14;
        //  cylinderMaxLength = cylinderMinLength + cylinderTravel;
          C1 = 10.21631;
          C2 = 33.43093;
          beta = 26.6594;
          deadBugTheta = 129.6249;
          break;
        case 2:
          //Serial.println("KNEE");
          sensorMax = 934.00;
          sensorMin = 148.00;
          cylinderMinLength = 20;
          cylinderTravel = 12;
        //  cylinderMaxLength = cylinderMinLength + cylinderTravel;
          C1 = 25.6021;
          C2 = 7.4386;
          beta = 35.8658;
          deadBugTheta = 194.1805;
          break;
      }
      
      //calculate alpha (the internal angle opposite the cylinder in the cylinder triangle
      if (joint == 0) { 
        //The hip is inverted logic because the sensor is at minimum when the joint is at minimum angle 
        //The other joints are at their maximum angle whne the sensor is fully retracted                
        alpha = angle - deadBugTheta + beta;
      }
      else {
        alpha = deadBugTheta - beta - angle;
      }
      float alphaR = (alpha * 71) / 4068;
//      Serial.print("alpha: ");
//      Serial.println(alpha);
      
      //calculate cylinder length from alpha
      float cylinderGoalLength = sqrt((sq(C2) + sq(C1) - (2*C1*C2*cos(alphaR))));
      float pistonGoal = cylinderGoalLength - cylinderMinLength;
      float sensorUnitsPerInch = (sensorMax - sensorMin) / (cylinderTravel);
//      Serial.print("sensor units per inch: ");
//      Serial.println(sensorUnitsPerInch);
      sensorGoal = (pistonGoal * sensorUnitsPerInch) + sensorMin;
   
      return sensorGoal;           
}
