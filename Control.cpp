#include <math.h>
#include "Control.h"

float Control::p_gain;
float Control::i_gain;
float Control::d_gain;
int Control::min;
int Control::max;

float Control::i_error;
float Control::lastValue;

enum StatusControl Control::status = NO_ERR_CONTROL;
char* Control::status_msg[STATUS_NUM_CONTROL] = {
  "You don't mistake. Don't worry",
  "Can't set pid gain value. You must to use 'setPIDgain' to set gain value",
  "Can't set control value range. You must to use 'setControlValueRange' to set control value range",
};
char* Control::errorFunctionName = "Don't exist any error on everything function.";

void Control::setPIDgain(float PG, float IG, float DG)
{
  p_gain = PG;
  i_gain = IG;
  d_gain = DG;
}

void Control::setControlValueRange(int minval, int maxval)
{
  min = minval;
  max = maxval;
}

int Control::getPID(int currentValue, int targetValue)
{
  float controlValue;
  float error, d_error;

  if (p_gain == 0 && i_gain == 0 && d_gain) {
    status = NOT_SET_PIDGAIN;
    errorFunctionName = (char*)__FUNCTION__;
    return (-1);
  }
  if (min == 0 && max == 0) {
    status = NOT_SET_VALUERANGE;
    errorFunctionName = (char*)__FUNCTION__;
    return (-1);
  }

  error = targetValue - currentValue;
  d_error = currentValue - lastValue;

  controlValue = p_gain * error + i_gain * i_error + d_gain * d_error;
  lastValue = currentValue;
  i_error += error;
  controlValue = (controlValue < 0 ? -1 : 1)
                 * constrain(abs(controlValue), min, max);

  return (controlValue);
}

void Control::disp_errorMsg(char *message)
{
  Serial.print(__FILE__);
  Serial.print(": ");
  Serial.print(errorFunctionName);
  Serial.print(": ");
  Serial.print(message);
  Serial.print(": ");
  Serial.println(status_msg[status]);
}

void Control::disp_errorMsg(void)
{
  Serial.print(__FILE__);
  Serial.print(": ");
  Serial.print(errorFunctionName);
  Serial.print(": ");
  Serial.println(status_msg[status]);
}

void Control::record_errorMsg(char *message)
{

}
