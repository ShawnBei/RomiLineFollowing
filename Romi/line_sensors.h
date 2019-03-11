#ifndef _Line_follow_h
#define _Line_follow_h

//Number of readings to take for calibration
//const int NUM_CALIBRATIONS = 200;


class Line_Sensor
{
  public:
    //Constructor
    Line_Sensor(int pin);
    void calibrate();
    //Return the uncalibrated value from the sensor
    float read_raw();
    //Return the calibrated value from the sensor
    float read_calibrated();
    
  private:
  
    int pin;
    float valueMax = 0;
    float valueMin = 1023;
    float rawValue = 0;
};

Line_Sensor::Line_Sensor(int Line_pin)
{
  pin = Line_pin;
  pinMode(pin, INPUT);
}

float Line_Sensor::read_raw()
{
  return analogRead(pin);
}

void Line_Sensor::calibrate()
{
   rawValue = analogRead(pin);
    
   if (rawValue > valueMax)
   {
     valueMax = rawValue;
   }
   if (rawValue < valueMin)
   {
     valueMin = rawValue;
   }
}

float Line_Sensor::read_calibrated()
{
  rawValue = analogRead(pin);
  return (rawValue - valueMin) * 1023 / (valueMax - valueMin);
}

 

#endif
