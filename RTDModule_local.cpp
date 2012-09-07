/*
  RTDSensor.cpp - Library for openenergymonitor
  Created by Trystan Lea, April 27 2010
  Licenced under GPL
*/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#include "RTDModule_local.h"

//-----------------------------------------------------------------------
// Set module pins
//-----------------------------------------------------------------------
void RTDModule::setPins(int _analogInPin)
{
   analogInPin = _analogInPin;
}

//-----------------------------------------------------------------------
// Set calibration values
//
// Calibration needs to be set for every RTD input on the multiplexor
//-----------------------------------------------------------------------
void RTDModule::calibration(double _calA,double _calB)
{
   scale = _calA;
   offset = _calB;
}

//-----------------------------------------------------------------------
// Read temperature
//-----------------------------------------------------------------------
double RTDModule::getTemperature()
{
   double temperature = scale * analogRead(analogInPin) + offset; 
   return temperature;
}
//-----------------------------------------------------------------------

//-----------------------------------------------------------------------
// RAW analogread
//-----------------------------------------------------------------------
int RTDModule::getADC() { return analogRead(analogInPin); }
//-----------------------------------------------------------------------
