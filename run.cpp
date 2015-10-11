#include "run.h"
#include <math.h>
#include <string.h>
#include <SoftwareSerial.h>

enum Status RunGPScar::status = NO_ERR;
char* RunGPScar::status_msg[STATUS_NUM] = {
  "You don't mistake. Don't worry",
  "Too few way point. You must set getting way point value much more.",
  "Too Many way point. You must set getting way point value much less.",
  "Overflow gps buffer. Please call Mr.Nishimura.. 080-3358-1274",
  "Way point number error",
  "You can't available the serial peripheral",
};
float RunGPScar::wpRange_m;
struct Coordinate RunGPScar::origin_p;
struct Coordinate RunGPScar::current_p;
struct Coordinate RunGPScar::goal_p;
struct PolarCoordinate RunGPScar::current_pp;
struct PolarCoordinate RunGPScar::goal_pp;
struct Coordinate RunGPScar::gps_buf[GPS_GET_NUM];
int RunGPScar::curGB_num;
struct Coordinate RunGPScar::wp_buf[WP_NUM];
int RunGPScar::wp_num;
int RunGPScar::wp_cur;
int RunGPScar::LFPin;
int RunGPScar::LBPin;
int RunGPScar::RFPin;
int RunGPScar::RBPin;
struct SerialPeripheral RunGPScar::serial_peri;

RunGPScar::RunGPScar()
{
  struct SerialPeripheral sp = {SOFTWARESERIAL, 10, 9, (long)9600};
  serial_peri = sp;
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  LFPin = 6;
  LBPin = 7;
  RFPin = 5;
  RBPin = 4;
}

RunGPScar::RunGPScar(int in1Pin, int in2Pin, int in3Pin, int in4Pin,
                     float goalLat, float goalLon,
                     float wp_r)
{
  LFPin = in1Pin;
  LBPin = in2Pin;
  RFPin = in3Pin;
  RBPin = in4Pin;

  pinMode(LFPin, OUTPUT);
  pinMode(LBPin, OUTPUT);
  pinMode(RFPin, OUTPUT);
  pinMode(RBPin, OUTPUT);

  goal_p.flat_deg = goalLat;
  goal_p.flon_deg = goalLon;

  wpRange_m = wp_r;

  serial_peri.name = HARDWARESERIAL0;
}

RunGPScar::RunGPScar(int in1Pin, int in2Pin, int in3Pin, int in4Pin,
                     float goalLat, float goalLon,
                     float wp_r,
                     struct SerialPeripheral sp)
{
  LFPin = in1Pin;
  LBPin = in2Pin;
  RFPin = in3Pin;
  RBPin = in4Pin;

  pinMode(LFPin, OUTPUT);
  pinMode(LBPin, OUTPUT);
  pinMode(RFPin, OUTPUT);
  pinMode(RBPin, OUTPUT);

  goal_p.flat_deg = goalLat;
  goal_p.flon_deg = goalLon;

  wpRange_m = wp_r;

  serial_peri = sp;
}

int RunGPScar::recvStr(char *buf)
{
  int i = 0;
  char c;

  switch (serial_peri.name) {
    case SOFTWARESERIAL:
      {
        SoftwareSerial ss(serial_peri.rx, serial_peri.tx);
        ss.begin(serial_peri.baudrate);
        while (1) {
          if (ss.available()) {
            c = ss.read();
            buf[i] = c;
            if (c == '\n') break;
            i++;
          } else {
            return (0);
          }
        }
        buf[i] = '\0';  // \0: end of string
        return (1);
        break;
      }

    case HARDWARESERIAL0 :
      {
        Serial.begin(serial_peri.baudrate);

        while (1) {
          if (Serial.available()) {
            c = Serial.read();
            buf[i] = c;
            if (c == '\n') break;
            i++;
          }

          else return (0);
        }
        buf[i] = '\0';  // \0: end of string
        return (1);
        break;
      }

#ifdef HAVE_HWSERIAL1
    case HARDWARESERIAL1 :
      {
        Serial1.begin(serial_peri.baudrate);
        while (1) {
          if (Serial1.available()) {
            c = Serial1.read();
            buf[i] = c;
            if (c == '\n') break;
            i++;
          }

          else return (0);
        }
        buf[i] = '\0';  // \0: end of string
        return (1);
        break;
      }
#endif

#ifdef HAVE_HWSERIAL2
    case HARDWARESERIAL2 :
      {
        Serial2.begin(serial_peri.buadrate);
        while (1) {
          if (Serial2.available()) {
            c = Serial2.read();
            buf[i] = c;
            if (c == '\n') break;
            i++;
          }

          else return (0);
        }
        buf[i] = '\0';  // \0: end of string
        return (1);
        break;
      }
#endif

#ifdef HAVE_HWSERIAL3
    case HARDWARESERIAL3 :
      {
        Serial3.begin(serial_peri.baudrate);
        while (1) {
          if (Serial3.available()) {
            c = Serial3.read();
            buf[i] = c;
            if (c == '\n') break;
            i++;
          }

          else return (0);
        }
        buf[i] = '\0';  // \0: end of string
        return (1);
        break;
      }
#endif

    default :
      {
        status = NO_SERIALPERIPHERAL;
        return (-1);
        break;
      }
  }

}

int RunGPScar::recvGPS(float *flat_deg, float *flon_deg, float *speed_mps, float *dir_deg)
{
  char str[100];
  char *str_lat, *str_lon, *str_speed, *str_dir;

  if (!recvStr(str)) return (0);

  if (strcmp(strtok(str, ","), "$GPRMC") == 0) { //if RMC line
    strtok(NULL, ",");
    strtok(NULL, ",");
    str_lat = strtok(NULL, ","); //get latitude
    strtok(NULL, ",");
    str_lon = strtok(NULL, ","); //get longtude
    strtok(NULL, ",");         // E読み飛ばし
    str_speed = strtok(NULL, ",");  // 速度 get
    str_dir = strtok(NULL, ","); // 進行方向 get

    *flat_deg = atof(str_lat + 2);
    *flon_deg = atof(str_lon + 3);
    *speed_mps = atof(str_speed) * 0.51;
    *dir_deg = atof(str_dir);
    return (1);
  }
  return (0);
}

int RunGPScar::getGPS(float *flat_deg, float *flon_deg, float *speed_mps, float *dir_deg)
{
  *flat_deg = *flon_deg = *speed_mps = *dir_deg = 0;
  into_GPSbuf(*flat_deg, *flon_deg);
  return (0);
}

int RunGPScar::init_run(void)
{
  float flat_deg, flon_deg, speed, angle;

  //while (recvGPS(&flat_deg, &flon_deg, &speed, &angle) == 0);

  /*origin_p.flat_deg = flat_deg;
  origin_p.flon_deg = flon_deg;
  goal_pp.dis_m = getDistance(origin_p.flat_deg, origin_p.flon_deg,
                              goal_p.flat_deg, goal_p.flon_deg);
  goal_pp.dir_deg = getAngle(origin_p.flat_deg, origin_p.flon_deg,
                             goal_p.flat_deg, goal_p.flon_deg);
*/
  control_motor(255, 255);
  delay(10000);
  control_motor(0, 0);
  delay(1000);

  return (0);
}

int RunGPScar::control_motor(int motorL, int motorR)
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

int RunGPScar::into_GPSbuf(float flat_deg, float flon_deg)
{
  int i;

  if (curGB_num > GPS_GET_NUM) {
    status = OVERFLOW_GPSBUF;
    curGB_num = GPS_GET_NUM - 1;
    return (-1);
  }

  if (curGB_num < GPS_GET_NUM) {
    gps_buf[curGB_num].flat_deg = flat_deg;
    gps_buf[curGB_num].flon_deg = flon_deg;
    curGB_num++;
  } else {
    for (i = 0; i < GPS_GET_NUM - 1; i++)
      gps_buf[i] = gps_buf[i + 1];
    gps_buf[GPS_GET_NUM - 1].flat_deg = flat_deg;
    gps_buf[GPS_GET_NUM - 1].flon_deg = flon_deg;
  }

  return (0);
}

int RunGPScar::setWayPoint(float originFlat_deg, float originFlon_deg,
                           float goal_flat_deg, float goal_flon_deg)
{
  wp_buf[0].flat_deg = goal_flat_deg;
  wp_buf[0].flon_deg = goal_flon_deg;

  wp_num = 1;
  wp_cur = wp_num - 1;

  if (wp_num < 1 || wp_num < 0) {
    status = WPNUM_ERR;
    return (-1);
  }

  return (0);
}

float RunGPScar::getDistance(float originFlat_deg, float originFlon_deg,
                             float destFlat_deg, float destFlon_deg)
{
  float dis_m;
  float sin4dis1, sin4dis2;
  float cos4dis1, cos4dis2, cos4dis3;

  sin4dis1 = sin(originFlat_deg * DEG2RAD);
  sin4dis2 = sin(destFlat_deg * DEG2RAD);
  cos4dis1 = cos(originFlat_deg * DEG2RAD);
  cos4dis2 = cos(destFlat_deg * DEG2RAD);
  cos4dis3 = cos((destFlon_deg - originFlon_deg) * DEG2RAD);

  dis_m = (EARTH_R_KM * acos(sin4dis1 * sin4dis2 + cos4dis1 * cos4dis2 * cos4dis3)) * 1000.0;

  return (dis_m);
}

float RunGPScar::getDistance(float originFlat_deg, float originFlon_deg)
{
  float dis_m;
  float sin4dis1, sin4dis2;
  float cos4dis1, cos4dis2, cos4dis3;

  sin4dis1 = sin(originFlat_deg * DEG2RAD);
  sin4dis2 = sin(goal_p.flat_deg * DEG2RAD);
  cos4dis1 = cos(originFlat_deg * DEG2RAD);
  cos4dis2 = cos(goal_p.flat_deg * DEG2RAD);
  cos4dis3 = cos((goal_p.flon_deg - originFlon_deg) * DEG2RAD);

  dis_m = (EARTH_R_KM * acos(sin4dis1 * sin4dis2 + cos4dis1 * cos4dis2 * cos4dis3)) * 1000.0;

  return (dis_m);
}

float RunGPScar::getAngle(float originFlat_deg, float originFlon_deg,
                          float destFlat_deg, float destFlon_deg)
{
  float angle_deg;
  float sin4angle1, sin4angle2;
  float cos4angle1, cos4angle2;
  float tan4angle;


  sin4angle1 = sin((destFlon_deg - originFlon_deg) * DEG2RAD);
  sin4angle2 = sin(originFlat_deg * DEG2RAD);
  cos4angle1 = cos(originFlat_deg * DEG2RAD);
  cos4angle2 = cos((destFlon_deg - originFlon_deg) * DEG2RAD);
  tan4angle = tan(destFlat_deg * DEG2RAD);

  angle_deg = RAD2DEG * atan2(sin4angle1, cos4angle1 * tan4angle
                              - sin4angle2 * cos4angle2);

  return (angle_deg);
}

float RunGPScar::getDirect(float originFlat_deg, float originFlon_deg,
                           float destFlat_deg, float destFlon_deg, int mode)
{
  float originAngle_deg, destAngle_deg;
  float dir_deg;

  if (mode == DIRMODIFY_NO) {
    originAngle_deg = getAngle(origin_p.flat_deg, origin_p.flon_deg,
                               originFlat_deg, originFlon_deg);
    destAngle_deg = getAngle(origin_p.flat_deg, origin_p.flon_deg,
                             destFlat_deg, destFlon_deg);
    dir_deg = destAngle_deg - originAngle_deg;
    return (dir_deg);

  } else if (mode == DIRMODIFY_YES) {
    return (-1);

  } else {
    return (-1);
  }

}

float RunGPScar::getDirect(float originFlat_deg, float originFlon_deg, int mode)
{
  float originAngle_deg, destAngle_deg;
  float dir_deg;

  if (mode == DIRMODIFY_NO) {
    originAngle_deg = getAngle(origin_p.flat_deg, origin_p.flon_deg,
                               originFlat_deg, originFlon_deg);
    destAngle_deg = getAngle(origin_p.flat_deg, origin_p.flon_deg,
                             goal_p.flat_deg, goal_p.flon_deg);
    dir_deg = destAngle_deg - originAngle_deg;
    return (dir_deg);

  } else if (mode == DIRMODIFY_YES) {
    return (-1);

  } else {
    return (-1);

  }
}

int RunGPScar::isGoalPoint(float curDis_m)
{
  if (curDis_m < wpRange_m) {
    wp_num--;
    if (wp_num == 0)
      return (CUR_IS_GOAL);
    else if (wp_num < 0)
      return (CUR_IS_WP);
    else
      return (-1);

  } else
    return (CUR_IS_WAY);

}

void RunGPScar::disp_errorMsg(char *message)
{
  Serial.print(message);
  Serial.print(": ");
  Serial.println(status_msg[status]);
}

void RunGPScar::record_errorMsg(char *message)
{

}
