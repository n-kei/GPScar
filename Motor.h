#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//////////ステータス値/////////////////
enum StatusMotor {
  NO_ERR_MOTOR = 0,
  STATUS_NUM_MOTOR,
};
////////////////////////////////////

class Motor
{
  public:
    Motor(int LFPin, int LBPin, int RFPin, int RBPin);
    int control(int motorL, int motorR);
    void disp_errorMsg(char *message);
    void disp_errorMsg(void);
    void record_errorMsg(char *message);
    
  private:
    static int LFPin;
    static int LBPin;
    static int RFPin;
    static int RBPin;

  private:
    static enum StatusMotor status;
    static char *status_msg[STATUS_NUM_MOTOR];
    static char *errorFunctionName;
};

#endif

