#ifndef RUN_H_INCLUDED
#define RUN_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//////////ステータス値/////////////////
enum StatusRun {
  NO_ERR_RUN = 0,
  TOO_FEW_WP,
  TOO_MANY_WP,
  WPNUM_ERR,
  STATUS_NUM_RUN,
};
////////////////////////////////////

///////////座標関係/////////////
#define DIRMODIFY_YES 1
#define DIRMODIFY_NO  0
#define WP_NUM      16 //ウェイポイントの数
#define CUR_IS_WP   0 //ウェイポイントに到着
#define CUR_IS_GOAL 1 //ゴールに到着
#define CUR_IS_WAY  2 //まだついてない
///////////////////////////////////

////////////通常座標型///////////
struct Coordinate {
  float flat_deg;
  float flon_deg;
};
/////////////////////////////////

extern float wpRange_m;
extern int wp_num;
extern int wp_cur;

extern struct Coordinate origin_p;
extern struct Coordinate current_p;
extern struct Coordinate goal_p;
extern struct Coordinate wp_buf[WP_NUM];

extern enum StatusRun status;
extern char *status_msg[STATUS_NUM_RUN];
extern char *errorFunctionName;

int setOriginPoint(float originFlat_deg, float originFlon_deg);
int setGoalPoint(float goalFlat_deg, float goalFlon_deg);
int setWPrange(float wpr);
int setWayPoint(float originFlat_deg, float originFlon_deg,
                float goal_flat_deg, float goal_flon_deg);
float getDirect(float originFlat_deg, float originFlon_deg,
                float destFlat_deg, float destFlon_deg, int mode);
float getDirect2Goal(float currentFlat_deg, float currentFlon_deg);
unsigned long int getDistance2Goal(float currentFlat_deg, float currentFlon_deg);
int isGoalPoint(float curDis_m);
float changeRange_deg(float angle);
void disp_errorMsg_run(char *message);
void record_errorMsg_run(char *message);

#endif
