#include "Motor.h"

int Motor::LFPin;
int Motor::LBPin;
int Motor::RFPin;
int Motor::RBPin;

enum StatusMotor Motor::status = NO_ERR_MOTOR;
char* Motor::status_msg[STATUS_NUM_MOTOR] = {
  "You don't mistake. Don't worry",
};
char* Motor::errorFunctionName = "Don't exist any error on everything function.";

Motor::Motor(int in1Pin, int in2Pin, int in3Pin, int in4Pin)
{
  LFPin = in1Pin;
  LBPin = in2Pin;
  RFPin = in3Pin;
  RBPin = in4Pin;
}

int Motor::control(int motorL, int motorR)
{

  if (motorL >= 0 && motorR <= 0) {
    analogWrite(LFPin, motorL);
    digitalWrite(LBPin, LOW);
    digitalWrite(RFPin, LOW);
    digitalWrite(RBPin, -motorR);
  }

  else if (motorL <= 0 && motorR >= 0) {
    digitalWrite(LFPin, LOW);
    analogWrite(LBPin, -motorL);
    analogWrite(RFPin, motorR);
    digitalWrite(RBPin, LOW);
  }

  else if (motorR <= 0 && motorL <= 0) {
    digitalWrite(LFPin, LOW);
    analogWrite(LBPin, -motorL);
    digitalWrite(RFPin, LOW);
    analogWrite(RBPin, -motorR);
  }

  else {
    analogWrite(LFPin, motorL);
    digitalWrite(LBPin, LOW);
    analogWrite(RFPin, motorR);
    digitalWrite(RBPin, LOW);
  }

  return (0);
}

void Motor::disp_errorMsg(char *message)
{
  Serial.print(__FILE__);
  Serial.print(": ");
  Serial.print(errorFunctionName);
  Serial.print(": ");
  Serial.print(message);
  Serial.print(": ");
  Serial.println(status_msg[status]);
}

void Motor::disp_errorMsg(void)
{
  Serial.print(__FILE__);
  Serial.print(": ");
  Serial.print(errorFunctionName);
  Serial.print(": ");
  Serial.println(status_msg[status]);
}

void Motor::record_errorMsg(char *message)
{

}
