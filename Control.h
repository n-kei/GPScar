#ifndef CONTROL_H_INCLUDED
#define CONTROL_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//////////ステータス値/////////////////
enum StatusControl {
  NO_ERR_CONTROL = 0,
  NOT_SET_PIDGAIN,
  NOT_SET_VALUERANGE,
  STATUS_NUM_CONTROL,
};
////////////////////////////////////

class Control
{
  public :
    void setPIDgain(float PG, float IG, float DG);
    void setControlValueRange(int minval, int maxval);
    int getPID(int currentVal, int targetVal);
    void disp_errorMsg(char *message);
    void disp_errorMsg(void);
    void record_errorMsg(char *message);

  private:
    static float p_gain;
    static float i_gain;
    static float d_gain;
    static int min;
    static int max;

  private:
    static float i_error;
    static float lastValue;

  private:
    static enum StatusControl status;
    static char *status_msg[STATUS_NUM_CONTROL];
    static char *errorFunctionName;
};

extern Control control;
#endif
