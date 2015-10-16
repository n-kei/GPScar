#ifndef RECVGPS_H_INCLUDED
#define RECVGPS_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <TinyGPS++.h>
#include "run.h"

#define GPS_GET_NUM  10    //GPS取得数
#define EARTH_R_KM      6378.137 //地球の半径[km]
#define DEG2RAD (M_PI/180.0)  //度数法表示から弧度法表示へ座標を変換
#define RAD2DEG (180.0/M_PI)  //弧度法表示から度数法表示へ座標変換

//////////ステータス値/////////////////
enum StatusReceiveGPS {
  NO_ERR_RECEIVEGPS = 0,
  OVERFLOW_GPSBUF,
  NO_SERIALPERIPHERAL,
  CANT_RECVGPS,
  STATUS_NUM_RECEIVEGPS,
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

class ReceiveGPS
{
  public:
    ReceiveGPS(TinyGPSPlus *gps_tmp);
    ReceiveGPS(TinyGPSPlus *gps_tmp, struct SerialPeripheral serial_peri);
    int recvGPS(void);
    int recvGPS(unsigned long int timeout);
    int gelay(unsigned long int dtime);
    int into_GPSbuf(float flat_deg, float flon_deg);
    void disp_errorMsg(char *message);
    void disp_errorMsg(void);
    void record_errorMsg(char *message);
    
  private:
    static TinyGPSPlus *gps;
    static struct SerialPeripheral serial_peri;
    static struct Coordinate gps_buf[GPS_GET_NUM];
    static int curGB_num;

  private:
    static enum StatusReceiveGPS status;
    static char *status_msg[STATUS_NUM_RECEIVEGPS];
    static char *errorFunctionName;
};

#endif
