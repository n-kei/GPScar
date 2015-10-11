#ifndef RUN_H_INCLUDED
#define RUN_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

///////////座標関係/////////////
#define DIRMODIFY_YES 1
#define DIRMODIFY_NO 0
#define GPS_GET_NUM  10    //GPS取得数
#define EARTH_R_KM      6378.137 //地球の半径[km]
#define DEG2RAD (M_PI/180.0)  //度数法表示から弧度法表示へ座標を変換
#define RAD2DEG (180.0/M_PI)  //弧度法表示から度数法表示へ座標変換
#define WP_NUM   16 //ウェイポイントの数
#define CUR_IS_WP 0 //ウェイポイントに到着
#define CUR_IS_GOAL 1 //ゴールに到着
#define CUR_IS_WAY 2 //まだついてない
///////////////////////////////////

//////////ステータス値/////////////////
enum Status {
  NO_ERR = 0,
  TOO_FEW_WP,
  TOO_MANY_WP,
  OVERFLOW_GPSBUF,
  WPNUM_ERR,
  NO_SERIALPERIPHERAL,
  STATUS_NUM,
};
////////////////////////////////////

enum SerialPeripheralName {
  HARDWARESERIAL0 = 0,
  HARDWARESERIAL1,
  HARDWARESERIAL2,
  HARDWARESERIAL3,
  SOFTWARESERIAL,
  UNKNOWNPERI,
};

struct SerialPeripheral {
  enum SerialPeripheralName name;
  long baudrate;
  int rx;
  int tx;
};

////////////極座標型///////////////
struct PolarCoordinate {
  float dis_m;
  float dir_deg;
};
/////////////////////////////////

////////////通常座標型///////////
struct Coordinate {
  float flat_deg;
  float flon_deg;
};
/////////////////////////////////

class RunGPScar
{
  public:
    RunGPScar();
    RunGPScar(int in1Pin, int in2Pin, int in3Pin, int in4Pin,
              float goalLat, float goalLon,
              float wp_r);
    RunGPScar(int in1Pin, int in2Pin, int in3Pin, int in4Pin,
              float goalLat, float goalLon,
              float wp_r,
              struct SerialPeripheral sp);
    int recvStr(char *buf);
    int recvGPS(float *flat_deg, float *flon_deg, float *speed_mps, float *direct_deg);
    int getGPS(float *flat_deg, float *flon_deg, float *speed_mps, float *direct_deg);
    int init_run(void);
    int control_motor(int MotorL, int MotorR);
    int into_GPSbuf(float flat_deg, float flon_deg);
    int setWayPoint(float origin_flat_deg, float origin_flon_deg,
                    float goal_flat_deg, float goal_flon_deg);
    float getDistance(float origin_flat_deg, float origin_flon_deg,
                      float dest_flat_deg, float dest_flon_deg);
    float getDistance(float origin_flat_deg, float origin_flon_deg);
    float getAngle(float origin_flat_deg, float origin_flon_deg,
                   float dest_flat_deg, float dest_flon_deg);
    float getDirect(float originFlat_deg, float originFlon_deg, int mode);
    float getDirect(float originFlat_deg, float originFlon_deg,
                    float destFlat_deg, float destFlon_deg, int mode);
    int isGoalPoint(float curDis_m);

  private:
    void disp_errorMsg(char *message);
    void record_errorMsg(char *message);

  private:
    static enum Status status;
    static char *status_msg[STATUS_NUM];

  private:
    static float wpRange_m;
    static struct Coordinate origin_p;
    static struct Coordinate current_p;
    static struct Coordinate goal_p;
    static struct PolarCoordinate current_pp;
    static struct PolarCoordinate goal_pp;
    static struct Coordinate gps_buf[GPS_GET_NUM];
    static int curGB_num;
    static struct Coordinate wp_buf[WP_NUM];
    static int wp_num;
    static int wp_cur;
    static struct SerialPeripheral serial_peri;

  private:
    static int LFPin;
    static int LBPin;
    static int RFPin;
    static int RBPin;
};

#endif
