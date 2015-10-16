#include "Run.h"
#include <TinyGPS++.h>

float wpRange_m;
int wp_cur;
int wp_num;

struct Coordinate origin_p;
struct Coordinate current_p;
struct Coordinate goal_p;
struct Coordinate wp_buf[WP_NUM];

enum StatusRun status = NO_ERR_RUN;
char *status_msg[STATUS_NUM_RUN] = {
  "You don't mistake. Don't worry",
  "Too few way point. You must set getting way point value much more.",
  "Too Many way point. You must set getting way point value much less.",
  "Way point number error",
};
char *errorFunctionName = "Don't exist any error on everything function.";

int setOriginPoint(float originFlat_deg, float originFlon_deg)
{
  origin_p.flat_deg = originFlat_deg;
  origin_p.flon_deg = originFlon_deg;

  return (0);
}

int setGoalPoint(float goalFlat_deg, float goalFlon_deg)
{
  goal_p.flat_deg = goalFlat_deg;
  goal_p.flon_deg = goalFlon_deg;

  return (0);
}

int setWPrange(float wpr)
{
  wpRange_m = wpr;

  return (0);
}

int setWayPoint(float originFlat_deg, float originFlon_deg,
                float goal_flat_deg, float goal_flon_deg)
{
  wp_buf[0].flat_deg = goal_flat_deg;
  wp_buf[0].flon_deg = goal_flon_deg;

  wp_num = 1;
  wp_cur = wp_num - 1;

  if (wp_num < 1 || wp_num < 0) {
    status = WPNUM_ERR;
    errorFunctionName = (char*)__FUNCTION__;
    return (-1);
  }

  return (0);
}

float getDirect(float originFlat_deg, float originFlon_deg,
                float destFlat_deg, float destFlon_deg, int mode)
{
  float originAngle_deg, destAngle_deg;
  float dir_deg;

  if (mode == DIRMODIFY_NO) {
    originAngle_deg = TinyGPSPlus::courseTo(origin_p.flat_deg, origin_p.flon_deg,
                                            originFlat_deg, originFlon_deg);
    destAngle_deg = TinyGPSPlus::courseTo(origin_p.flat_deg, origin_p.flon_deg,
                                          destFlat_deg, destFlon_deg);
    dir_deg = destAngle_deg - originAngle_deg;
    
    return (dir_deg);

  } else if (mode == DIRMODIFY_YES) {
    return (-1);

  } else {
    return (-1);
  }

}

float getDirect2Goal(float originFlat_deg, float originFlon_deg)
{
  float dir_deg;

  dir_deg = getDirect(originFlat_deg, originFlon_deg, goal_p.flat_deg, goal_p.flon_deg, DIRMODIFY_NO);

  return (dir_deg);
}

unsigned long int getDistance2Goal(float currentFlat_deg, float currentFlon_deg)
{
  unsigned long int dis_m;

  dis_m = (unsigned long)TinyGPSPlus::distanceBetween(
          currentFlat_deg, currentFlon_deg,
          goal_p.flat_deg, goal_p.flon_deg);

  return (dis_m);
}

int isGoalPoint(float curDis_m)
{
  if (curDis_m < wpRange_m) {
    wp_num--;
    wp_cur++;
    if (wp_num == 0)
      return (CUR_IS_GOAL);
    else if (wp_num > 0)
      return (CUR_IS_WP);
    else
      return (-1);

  } else
    return (CUR_IS_WAY);

}

float changeRange_deg(float angle_deg)
{
  float tmp_angle_deg;

  if (angle_deg > 180) tmp_angle_deg = angle_deg - 360.0;
  else if(angle_deg < -180) tmp_angle_deg = angle_deg + 360;
  else tmp_angle_deg = angle_deg;

  return (tmp_angle_deg);
}

void disp_errorMsg_run(char *message)
{
  Serial.print(__FILE__);
  Serial.print(": ");
  Serial.print(errorFunctionName);
  Serial.print(": ");
  Serial.print(message);
  Serial.print(": ");
  Serial.println(status_msg[status]);
}

void record_errorMsg_run(char *message)
{

}
