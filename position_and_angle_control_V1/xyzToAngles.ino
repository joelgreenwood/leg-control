//need global array:
// float theta1;
// float theta2;
// float theta3;
// commandAngle[] = {theta1, theta2, theta3};


void xyzToAngles(int x, int y, int z) {
      //now calculate theta1, theta2, and theta3
     if (x == 0) {
      x = x + .0001;
      //Serial.println("x = 0 so .0001 was added to avoid degenerate case");
     }   
     //theta1
      float theta1R = atan(y/x);
      //convert to degrees
      float theta1 = (theta1R * 4068) / 71;
      commandAngle[0] = theta1;
      Serial.print("theta1 = ");
      Serial.println(theta1);
      //angle goal to pot reading
      hipGoal = angleToSensor(0, theta1);
      //hipGoal = ((theta1 - hipAngleMin) * hipSensorUnitsPerDeg) + hipPotMin;
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
      commandAngle[1] = theta2;
      Serial.print("theta2 = ");
      Serial.println(theta2);
      //thighGoal is sensor reading at goal angle
      thighGoal = angleToSensor(0, theta2);
      //thighGoal = thighPotMin + ((thighAngleMax - theta2) * thighSensorUnitsPerDeg);
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
      commandAngle[2] = theta3;
      Serial.print("theta3 = ");
      Serial.print(theta3);
      kneeGoal = angleToSensor(0, theta3);
      //kneeGoal= ((kneeAngleMax - theta3) * kneeSensorUnitsPerDeg) + kneePotMin;
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
    

