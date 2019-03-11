#ifndef _Kinematics_h
#define _Kinematics_h
#include <stdint.h>

const int WHEEL_DIAMETER = 70; //mm
const int WHEEL_DISTANCE = 140; //mm
const int GEAR_RATIO = 120;
const int COUNTS_PER_SHAFT_REVOLUTION = 12;
const int COUNTS_PER_WHEEL_REVOLUTION =  1440;
const float COUNTS_PER_MM = 6.548;
const float MM_PER_COUNT = 0.152;

class Kinematics
{
  public:
    Kinematics(float X, float Y, float T);
    void Update(float leftCounts_new, float rightCounts_new);
    float returnX();
    float returnY();
    float returnTheta();
    void printCoordinates();
    void reset();
    
  private:
    float x = 0;
    float y = 0;
    float theta = 0;
    float leftCounts_last = 0;
    float rightCounts_last = 0;
};

//constructor
Kinematics::Kinematics(float X, float Y, float T)
{
  x = X;
  y = Y;
  theta = T;
}

void Kinematics::Update(float leftCounts_new, float rightCounts_new)
{
  float dl = leftCounts_new - leftCounts_last;
  float dr = rightCounts_new - rightCounts_last;
  float d = ( dl * MM_PER_COUNT + dr * MM_PER_COUNT ) / 2; //transform counts to mm, increased x and y
  x += d * cos(theta);
  y += d * sin(theta);
  d = atan2(( dl * MM_PER_COUNT - dr * MM_PER_COUNT ), WHEEL_DISTANCE); //transform counts to mm, increased degrees
  theta += d;

  //to keep theta in the range of -180 to 180
  if (theta >= 180)
  {
    theta = theta - 360;
  }
  if (theta <= -180)
  {
    theta += 360;
  }
  
  leftCounts_last = leftCounts_new;
  rightCounts_last = rightCounts_new;
}


float Kinematics::returnX()
{
  return x;
}

float Kinematics::returnY()
{
  return y;
}

float Kinematics::returnTheta()
{
  return theta;
}

void Kinematics::printCoordinates()
{
    Serial.print(x);
    Serial.print(",");
    Serial.print(y); 
    Serial.print(",");
    Serial.println(theta);
}

void Kinematics::reset()
{
  leftCounts_last = 0;
  rightCounts_last = 0;
}

#endif
