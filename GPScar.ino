#include "ReceiveGPS.h"
#include "Control.h"
#include "Motor.h"
#include "run.h"

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

//#define MODE_NOMODIFY //GPSで方位が得られなかったときニュートラルに戻す
#define MODE_RUN      //自律走行実行モード
//#define MODE_TEST   //テストモード

#define STACKCOUNTER_NUM 10

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
TinyGPSPlus gps;
struct SerialPeripheral ss = {SOFTWARESERIAL, 9600, SSRX, SSTX};
ReceiveGPS recvGPS(&gps, ss);
Control control;
Motor motor(LFPin, LBPin, RFPin, RBPin);

void setup()
{
  Serial.begin(9600);
  control.setPIDgain(DEF_KS, 0, 0);
  control.setControlValueRange(DEF_STRMIN, DEF_STRMAX);
  setGoalPoint(GOAL_LAT, GOAL_LON);
  setWPrange(DEF_WR_M);

  recvGPS.gelay(1000);
  setOriginPoint(gps.location.lat(), gps.location.lng());

  motor.control(255, 255);
  delay(10000);
  motor.control(0, 0);
}

void loop()
{
  float currentDir_deg;
  unsigned long int currentDis_m;
  int posSteer;

  recvGPS.gelay(1000);
  currentDir_deg = getDirect2Goal(gps.location.lat(), gps.location.lng());
  currentDis_m = getDistance2Goal(gps.location.lat(), gps.location.lng());
  currentDir_deg = changeRange_deg(currentDir_deg);

  if (isGoalPoint(currentDis_m) == CUR_IS_GOAL) {
    motor.control(0, 0);
    return;
  }
  
  posSteer = control.getPID(currentDir_deg, 0);

  motor.control(DEF_RUNSP - posSteer, DEF_RUNSP + posSteer);

#ifdef MODE_NOSTRMODIFY
  float lastFlat_deg = gps.location.lat();
  float lastFlon_deg = gps.location.lng();
  unsigned long int stackCounter = 0;

  if (TinyGPSPlus::courseTo(lastFlat_deg, lastFlon_deg,
                            gps.location.lat(), gps.location.lng()) == 0)
    stackCounter++;

  if (stackCounter > STACKCOUNTER_NUM)
    motor.control(255, 255);
#endif

}
#endif

#ifdef MODE_TEST

TinyGPSPlus gps;
struct SerialPeripheral ss = {SOFTWARESERIAL, 9600, 10, 9};
ReceiveGPS recvGPS(&gps, ss);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  double angle;

  angle = TinyGPSPlus::courseTo(35.6544, 139.74477, 21.4225, 39.8261);
  Serial.print("angle = ");  Serial.println(changeRange_deg(angle));
  delay(1000);

}

#endif

