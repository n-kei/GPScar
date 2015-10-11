#include "run.h"
#include <SoftwareSerial.h>

#define MODE_NOMODIFY //GPSで方位が得られなかったときニュートラルに戻す
//#define MODE_RUN   //自律走行実行モード
#define MODE_TEST   //テストモード

////////////モータ制御ピン/////////////
#define LFPin 6
#define LBPin 7
#define RFPin 5
#define RBPin 4
////////////////////////////////////////

///////////各種定数///////////////
#define DEF_WR_M 2.0   //ウェイポイントの半径[m]
#define DEF_KS   0.1    //ステアリングキレ角比例係数
#define DEF_STRMIN 30     //ステアリング最小キレ角[duty]
#define DEF_STRMAX 255    //ステアリング最大キレ角[duty]
#define DEF_RUNSP  125   //ステアリングニュートラルの時のモータ[duty]
//////////////////////////////////

/////////座標関係/////////////////
#define GOAL_LAT 134.1445
#define GOAL_LON 34.5333
/////////////////////////////////

///////////ソフトウェアシリアル/////////////
#define SSRX 10
#define SSTX 9
////////////////////////////////////////////


#ifdef MODE_RUN
void setup()
{
  Serial.begin(9600);

}

void loop()
{
  float flat_deg, flon_deg, speed_knot, dir_deg;
  struct PolarCoordinate current_pp;
  int posSteer = 0;
  struct SerialPeripheral software_serial = {SOFTWARESERIAL, SSRX, SSTX, 9600};
  RunGPScar run(LFPin, LBPin, RFPin, RBPin,
                GOAL_LAT, GOAL_LON,
                DEF_WR_M,
                software_serial);

  while (run.getGPS(&flat_deg, &flon_deg, &speed_knot, &dir_deg) < 0);
  current_pp.dis_m = run.getDistance(flat_deg, flon_deg);
  current_pp.dir_deg = run.getDirect(flat_deg, flon_deg, DIRMODIFY_NO);
  posSteer = current_pp.dir_deg * DEF_KS;
  posSteer = constrain(posSteer, DEF_STRMIN, DEF_STRMAX);

#ifdef MODE_NOMODIFY
  if (speed_knot == 0) posSteer = 0;
#endif

  run.control_motor(DEF_RUNSP + posSteer, DEF_RUNSP - posSteer);

  if (run.isGoalPoint(current_pp.dis_m) == CUR_IS_GOAL) {
    run.control_motor(0, 0);
    while (1);
  }

  //delay(1000);
}
#endif

#ifdef MODE_TEST

SoftwareSerial ss1(10, 9);
RunGPScar run;

void setup()
{
  Serial.begin(9600);
  ss1.begin(9600);

  run.init_run();
}

void loop()
{
  char str[100];

  int i = 0;
  char c;

  while (1) {
    if (ss1.available()) {
      c = ss1.read();
      str[i] = c;
      if (c == '\n') break;
      i++;
    }
  }
  str[i] = '\0';  // \0: end of string

  Serial.println(str);
}
#endif

