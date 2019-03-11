#include "encoders.h"
#include "pid.h"
#include "kinematics.h"
#include "line_sensors.h"

#define LOOP_DELAY 10
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15
#define BAUD_RATE = 115200;
#define LINE_LEFT_PIN A2 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN A4 //Pin for the right line sensor
#define BUZZER_PIN 6

float target_pose = 0; //home encoder count
float target_angle = 0; //home angle
unsigned long time_now = millis();
unsigned long time_elapsed = 0;
unsigned long time_read = millis();
float homeAngle = 0;
float leftData = 0;
float centreData = 0;
float rightData = 0;
int d = 0; //difference between left sensor and right sensor
int dc = 0;
float bias = 30; //initial speed
float bias2 = 20;
bool flag = true;
bool flagTurn = true;
bool flagStraight = false;
int stage = 0;
float last_turn_speed_left = 0;
float last_turn_speed_right = 0;
float last_straight_speed_left = 0;
float last_straight_speed_right = 0;
int i = 0;
float theta = 0;
float x = 0;
float y = 0;
float distance = 0;
float a = 0;
float headingFix = 0;
float headingNow = 0;
float ave = 0;

float initialX = 0; //mm
float initialY = 0; //mm
float initialTheta = 0;
Kinematics coordinates(initialX, initialY, initialTheta); //kinematics object

Line_Sensor lineLeft(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
Line_Sensor lineCentre(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor
Line_Sensor lineRight(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor

float Kp_pose = 0.1; 
float Kd_pose = -0.05; 
float Ki_pose = 0.000004; 
PID leftPose(Kp_pose, Kd_pose, Ki_pose); //go home Position controller 
PID rightPose(Kp_pose, Kd_pose, Ki_pose);

float Kp_goHomeSpeed = 1; 
float Kd_goHomeSpeed = 0; 
float Ki_goHomeSpeed = 0; 
PID leftGoHomeSpeed(Kp_goHomeSpeed, Kd_goHomeSpeed, Ki_goHomeSpeed); //go home Position controller 
PID rightGoHomeSpeed(Kp_goHomeSpeed, Kd_goHomeSpeed, Ki_goHomeSpeed);

float Kp_go = 0.8; 
float Kd_go = 0; 
float Ki_go = 0; 
PID GoHomeSpeed(Kp_goHomeSpeed, Kd_goHomeSpeed, Ki_goHomeSpeed);
 
float Kp_angle = 0.1;
float Kd_angle = -0.05;
float Ki_angle = 0.00015;
PID leftAngle(Kp_angle, Kd_angle, Ki_angle); //go home Rotation controller
PID rightAngle(Kp_angle, Kd_angle, Ki_angle);

float Kp = 0.2;
float Kd = 0;
float Ki = 0;
PID leftWeel(Kp, Kd, Ki); //line following wheel speed controller
PID rightWeel(Kp, Kd, Ki);

float Kp_count = 0.1;
float Kd_count = -0.05;
float Ki_count = 0;
PID leftCount(Kp_count, Kd_count, Ki_count); //first case go straight controller
PID rightCount(Kp_count, Kd_count, Ki_count);

void setupMotorPins()
{
  pinMode( L_PWM_PIN, OUTPUT );
  pinMode( L_DIR_PIN, OUTPUT );
  pinMode( R_PWM_PIN, OUTPUT );
  pinMode( R_DIR_PIN, OUTPUT );
  pinMode( BUZZER_PIN, OUTPUT );
}

void setup()
{

  setupEncoder0();
  setupEncoder1();
  setupMotorPins();

  //set maximum speed
  leftAngle.setMax(30);
  rightAngle.setMax(30);
  leftPose.setMax(40);
  rightPose.setMax(40);
//  leftWeel.setMax(50);
//  rightWeel.setMax(50);
  GoHomeSpeed.setMax(100);

  //calibrate line sensor
  analogWrite(BUZZER_PIN, 120);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
  while (millis() < 7000)
  {
    lineLeft.calibrate();
    lineCentre.calibrate();
    lineRight.calibrate();
  }
  analogWrite(BUZZER_PIN, 120);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);

  digitalWrite( L_DIR_PIN, LOW  );
  digitalWrite( R_DIR_PIN, LOW  );

  Serial.begin( 9600 );
  time_read = millis();
}


void loop()
{

  time_now = millis();
  time_elapsed = time_now - time_read;
  
  if (time_elapsed > 4000) { //waite for 3 seconds

    leftData = lineLeft.read_calibrated();
    rightData = lineRight.read_calibrated();
    centreData = lineCentre.read_calibrated();

    

    Serial.print(leftData);
          Serial.print(", ");
          Serial.print(centreData);
          Serial.print(", ");
          Serial.println(rightData);

    switch (stage)
    {
      case 0:
      if (leftData + rightData + centreData < 700){
      
      
         //go straight
         dc = (e0_speed + e1_speed) / 2;
         float countSpeedLeft = leftCount.update(dc, e0_speed);
         float countSpeedRight = rightCount.update(dc, e1_speed);
         coordinates.Update(count_e0, count_e1);
         analogWrite( L_PWM_PIN, bias + countSpeedLeft);
         analogWrite( R_PWM_PIN, bias + countSpeedRight);

//          Serial.print(bias + countSpeedLeft);
//          Serial.print(", ");
//          Serial.println(bias + countSpeedLeft);
    
    //      Serial.print(dc);
    //      Serial.print(", ");
    //      Serial.print(e0_speed);
    //      Serial.print(", ");
    //      Serial.println(e1_speed);

      }else{ 
          stage ++;
      }
       break;
       
        
      case 1:
      
        if (leftData + rightData + centreData < 700){
          
          stage++;
          
        }else{
          
         d = (leftData + rightData) / 2;
         float leftSpeed = leftWeel.update(d, leftData);
         float rightSpeed = rightWeel.update(d, rightData);
        
         float finalSpeedLeft = bias2 + leftSpeed;
         float finalSpeedRight = bias2 + rightSpeed;
    
        if (finalSpeedLeft < 0)
        {
          digitalWrite( L_DIR_PIN, HIGH );
        }
        if (finalSpeedLeft >= 0)
        {
          digitalWrite( L_DIR_PIN, LOW );
        }
        if (finalSpeedRight < 0)
        {
          digitalWrite( R_DIR_PIN, HIGH );
        }
        if (finalSpeedRight >= 0)
        {
          digitalWrite( R_DIR_PIN, LOW );
        }
        
        analogWrite( L_PWM_PIN, abs(finalSpeedLeft));
        analogWrite( R_PWM_PIN, abs(finalSpeedRight));
  
        coordinates.Update(count_e0, count_e1);
    
//        Serial.print(finalSpeedLeft);
//        Serial.print(", ");
//        Serial.println(finalSpeedRight);

      
      }

      break;
      
      case 2:
        if(flag){
          analogWrite( L_PWM_PIN, 0);
          analogWrite( R_PWM_PIN, 0);
          
          analogWrite(BUZZER_PIN, 120);
          delay(500);
          digitalWrite(BUZZER_PIN, LOW);
          delay(1000);
          
          flag = false;
          
          resetEncoder();
          coordinates.reset();
          
          theta = coordinates.returnTheta();
          x = coordinates.returnX();
          x = x * COUNTS_PER_MM;
          y = coordinates.returnY();
          y = y * COUNTS_PER_MM;
          distance = sqrt(x*x + y*y);
          a = atan2(y,x);
          if (a >= 0)
          {
            headingFix = PI - a;
            homeAngle = (theta + headingFix) * 180 / PI;
          }
          else
          {
            headingFix = PI + a;
            homeAngle = (theta - headingFix) * 180 / PI;
          }
        }
        
        goHome();

        break;
    }
    
    delay(LOOP_DELAY);
  }
}




//******************go home************************

void goHome()
{
  coordinates.Update(count_e0, count_e1);
   
  //turn romi
  theta = coordinates.returnTheta();
  
  x = coordinates.returnX();
  x = x * COUNTS_PER_MM;
  y = coordinates.returnY();
  y = y * COUNTS_PER_MM;

//  distance = sqrt(x*x + y*y);
  
  a = atan2(y,x);
  if (a >= 0)
  {
    headingNow = PI - a;
  }
  else
  {
    headingNow = PI + a;
  }

//        Serial.print(count_e0);
//        Serial.print(",");
//        Serial.print(count_e1);
//        Serial.print("|||");
//        Serial.print(x);
//        Serial.print(",");
//        Serial.print(y);
//        Serial.print(",");
//        Serial.print(distance);
//        Serial.print("|||");
//        Serial.print(theta);
//        Serial.print(",");
//        Serial.print(a);
//        Serial.print(",");
//        Serial.println(homeAngle);
  
  if (flagTurn){
    turnRomi(homeAngle);
  }

  if (flagStraight){
    straightRomi(distance);
  }

  
  
}






//****************position controller*********************

void straightRomi(float target_pose){
  
    float outputL = leftPose.update(target_pose, count_e0);
    float outputR = rightPose.update(target_pose, count_e1);

    
    
//    float speedCompensate = GoHomeSpeed.update(headingFix*10000,headingNow*10000);

//    ave = (e0_speed + e1_speed) / 2;
//    
//    float leftCountSpeedCom = leftGoHomeSpeed.update(ave, e0_speed);
//    float rightCountSpeedCom = rightGoHomeSpeed.update(ave, e1_speed);
    
    float finalSL = outputL  ;
    float finalSR = outputR  ;

    if (finalSL >= 0 ) {
      digitalWrite( L_DIR_PIN, HIGH  );
    } else if (finalSL < 0) {
      digitalWrite( L_DIR_PIN, LOW  );
    }

    if (finalSR >= 0 ) {
      digitalWrite( R_DIR_PIN, HIGH  );
    } else if (finalSR < 0 ) {
      digitalWrite( R_DIR_PIN, LOW  );
    }

//        Serial.print(count_e0);
//        Serial.print(",");
//        Serial.print(count_e1);
//        Serial.print("|||");
//        Serial.print(x);
//        Serial.print(",");
//        Serial.print(y);
//        Serial.print(",");
//        Serial.print(distance);
//        Serial.print("|||");
//        Serial.print(headingNow);
//        Serial.print(",");
//        Serial.print(a);
//        Serial.print(",");
//        Serial.print(headingFix);
//        Serial.print(",");
//        Serial.println(homeAngle);

    analogWrite( L_PWM_PIN, abs(finalSL) );
    analogWrite( R_PWM_PIN, abs(finalSR) );

    
}






// ************angle controller***********

void turnRomi(float target_angle){

    float target = 140 * PI * abs(target_angle) / 360 / 0.152; //transform degrees to counts

    float left_angle = leftAngle.update(-target, count_e0);
    float right_angle = rightAngle.update(target, count_e1);

      Serial.println(target);

    //turn right
    if (target_angle >= 0){
      
      if (left_angle >= 0 ) {
        digitalWrite( L_DIR_PIN, HIGH  );
      } else {
        digitalWrite( L_DIR_PIN, LOW  );
      }
  
      if (right_angle >= 0 ) {
        digitalWrite( R_DIR_PIN, HIGH  );
      } else {
        digitalWrite( R_DIR_PIN, LOW  );
      }
    }
    
    //turn left
    else{
      
      if (left_angle >= 0 ) {
        digitalWrite( L_DIR_PIN, HIGH  );
      } else{
        digitalWrite( L_DIR_PIN, LOW  );
      }
  
      if (right_angle >= 0 ) {
        digitalWrite( R_DIR_PIN, LOW  );
      } else {
        digitalWrite( R_DIR_PIN, HIGH  );
      }
    }
    analogWrite( L_PWM_PIN, abs(left_angle));
    analogWrite( R_PWM_PIN, abs(right_angle));
    

    if (leftAngle.getError() < 5 && rightAngle.getError() < 5){
      flagTurn = false;
      flagStraight = true;
      analogWrite(BUZZER_PIN, 120);
      analogWrite( L_PWM_PIN, 0);
      analogWrite( R_PWM_PIN, 0);
      delay(500);
      digitalWrite(BUZZER_PIN, LOW);
      delay(1000);
      resetEncoder();
//      coordinates.reset();
    }
    
} 
