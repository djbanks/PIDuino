/*
  RTDSensor.h - Library for openenergymonitor
  Created by Trystan Lea, May 16 2010
  Licenced under GPL
*/

#ifndef RTDModule_h
#define RTDModule_h

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class RTDModule
{
  public:

    void setPins(int _analogInPin);
    void calibration(double _calA,double _calB);
    double getTemperature();
    int getADC();
  private:

    int analogInPin;
    double scale;
    double offset;


};

#endif
