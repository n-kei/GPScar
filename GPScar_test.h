#ifndef GPSCAR_TEST_H_INCLUDED
#define GPSCAR_TEST_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "run.h"

class GPSCAR_test : public RunGPScar
{
  public :
    
};

#endif
