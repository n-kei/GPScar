#include "ReceiveGPS.h"
#include "Control.h"
#include "Motor.h"
#include "run.h"
#include "ADXL345.h"
#include "skLPSxxSPI.h"
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

//#define MODE_NOMODIFY //GPSで方位が得られなかったときニュートラルに戻す
//#define MODE_RUN      //自律走行実行モード
#define MODE_TEST   //テストモード

#define STACKCOUNTER_NUM 10

////////////モータ制御ピン/////////////
#define LFPin 6
#define LBPin 7
#define RFPin 5
#define RBPin 4
////////////////////////////////////////

///////////各種定数///////////////
#define DEF_WR_M    2.0      //ウェイポイントの半径[m]
#define DEF_KS      0.1      //ステアリングキレ角比例係数
#define DEF_STRMIN  30     //ステアリング最小キレ角[duty]
#define DEF_STRMAX  255    //ステアリング最大キレ角[duty]
#define DEF_RUNSP   125    //ステアリングニュートラルの時のモータ[duty]
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
  delay(30000);
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

skLPSxxx LPS(LPS331AP, SS) ;    // デバイスはLPS25Hを指定、SS=10

float pressure_origin;

void setup()
{
  int ans ;

  // シリアルモニターの設定
  Serial.begin(9600) ;
  // ＳＰＩの初期化
  SPI.begin() ;                           // ＳＰＩを行う為の初期化
  SPI.setBitOrder(MSBFIRST) ;             // ビットオーダー
  SPI.setClockDivider(SPI_CLOCK_DIV4) ;   // クロック(CLK)をシステムクロックの1/4で使用(16MHz/4)
  SPI.setDataMode(SPI_MODE3) ;            // クロック極性 1(HIGH)　クロック位相 1(HIGH)

  delay(3000) ;                           // 3Sしたら開始
  // 気圧センサの初期化を行う(更新速度は1Hz)
  ans = LPS.PressureInit() ;
  if (ans == 0) Serial.println("Initialization normal") ;
  else {
    Serial.print("Initialization abnormal ans=") ;
    Serial.println(ans) ;
  }
  delay(1000) ;
  LPS.PressureRead();
  pressure_origin = LPS.getPressure();
  Serial.println(pressure_origin);
}

void loop()
{
  LPS.PressureRead() ;               // 圧力と温度を読み出す
  Serial.print("[LPS331AP]") ;
  Serial.print(LPS.getPressure()) ;              // 気圧値の表示を行う
  Serial.print(" hPa  ") ;
  Serial.print(LPS.getTempreture()) ;               // 温度の表示を行う
  Serial.print(" 'C ") ;
  Serial.print(LPS.AltitudeCalc(pressure_origin, LPS.getPressure()));
  Serial.println(" m");
  delay(1000) ;                      // １秒後に繰り返す
}
#endif

