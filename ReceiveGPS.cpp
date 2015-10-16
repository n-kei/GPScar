#include "ReceiveGPS.h"
#include <SoftwareSerial.h>

TinyGPSPlus* ReceiveGPS::gps;
struct SerialPeripheral ReceiveGPS::serial_peri;
struct Coordinate ReceiveGPS::gps_buf[GPS_GET_NUM];
int ReceiveGPS::curGB_num;

enum StatusReceiveGPS ReceiveGPS::status = NO_ERR_RECEIVEGPS;
char* ReceiveGPS::status_msg[STATUS_NUM_RECEIVEGPS] = {
  "You don't mistake. Don't worry",
  "Overflow gps buffer. Please call Mr.Nishimura.. 080-3358-1274",
  "You can't available the serial peripheral",
  "Can't receive GPS data",
};
char* ReceiveGPS::errorFunctionName = "Don't exist any error on everything function.";

ReceiveGPS::ReceiveGPS(TinyGPSPlus *gps_tmp)
{
  struct SerialPeripheral sp = {HARDWARESERIAL0, 9600, 0, 0};
  serial_peri = sp;
  gps = gps_tmp;
}

ReceiveGPS::ReceiveGPS(TinyGPSPlus *gps_tmp, struct SerialPeripheral sp)
{
  serial_peri = sp;
  gps = gps_tmp;
}

/*GPSデータ受信用関数
 *    引数：なし
 *  返り値：int データ読み込み成功 0
 *             データ読み込み失敗 -1
 *  ※モータ駆動時に使用するとデータが壊れるので使用しないこと。（要デバッグ）
 */
int ReceiveGPS::recvGPS(void)
{
  switch (serial_peri.name) {
    case SOFTWARESERIAL:
      {
        SoftwareSerial ss(serial_peri.rx, serial_peri.tx);
        ss.begin(serial_peri.baudrate);
        if (!ss.available()) {
          status = CANT_RECVGPS;
          return (-1);
        }
        while (ss.available())
          gps->encode(ss.read());
        return (0);
        break;
      }

    case HARDWARESERIAL0 :
      {
        Serial.begin(serial_peri.baudrate);

        if (!Serial.available()) {
          status = CANT_RECVGPS;
          return (-1);
        }
        while (Serial.available())
          gps->encode(Serial.read());
        return (0);
        break;
      }

#ifdef HAVE_HWSERIAL1
    case HARDWARESERIAL1 :
      {
        Serial1.begin(serial_peri.baudrate);

        if (!Serial1.available()) {
          status = CANT_RECVGPS;
          return (-1);
        }
        while (Serial1.available())
          gps->encode(Serial1.read());
        return (0);
        break;
      }
#endif

#ifdef HAVE_HWSERIAL2
    case HARDWARESERIAL2 :
      {
        Serial2.begin(serial_peri.buadrate);

        if (!Serial2.available()) {
          status = CANT_RECVGPS;
          return (-1);
        }
        while (Serial2.available())
          gps->encode(Serial2.read());
        return (0);
        break;
      }
#endif

#ifdef HAVE_HWSERIAL3
    case HARDWARESERIAL3 :
      {
        Serial3.begin(serial_peri.baudrate);

        if (!Serial3.available()) {
          status = CANT_RECVGPS;
          return (-1);
        }
        while (Serial3.available())
          gps->encode(Serial3.read());
        return (0);
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


int ReceiveGPS::recvGPS(unsigned long int timeout)
{
  unsigned long start = millis();

  switch (serial_peri.name) {
    case SOFTWARESERIAL:
      {
        SoftwareSerial ss(serial_peri.rx, serial_peri.tx);
        ss.begin(serial_peri.baudrate);

        while (ss.available()) {
          if (millis() - start >= timeout) {
            status = CANT_RECVGPS;
            return (-1);
          }
          gps->encode(ss.read());
        }

        return (0);
        break;
      }

    case HARDWARESERIAL0 :
      {
        Serial.begin(serial_peri.baudrate);

        while (Serial.available()) {
          if (millis() - start >= timeout) {
            status = CANT_RECVGPS;
            return (-1);
          }
          gps->encode(Serial.read());
        }

        return (0);
        break;
      }

#ifdef HAVE_HWSERIAL1
    case HARDWARESERIAL1 :
      {
        Serial1.begin(serial_peri.baudrate);

        while (Serial1.available()) {
          if (millis() - start >= timeout) {
            status = CANT_RECVGPS;
            return (-1);
          }
          gps->encode(Serial1.read());
        }

        return (0);
        break;
      }
#endif

#ifdef HAVE_HWSERIAL2
    case HARDWARESERIAL2 :
      {
        Serial2.begin(serial_peri.buadrate);

        while (Serial2.available()) {
          if (millis() - start >= timeout) {
            status = CANT_RECVGPS;
            return (-1);
          }
          gps->encode(Serial2.read());
        }

        return (0);
        break;
      }
#endif

#ifdef HAVE_HWSERIAL3
    case HARDWARESERIAL3 :
      {
        Serial3.begin(serial_peri.baudrate);
        while (Serial3.available()) {
          if (millis() - start >= timeout) {
            status = CANT_RECVGPS;
            return (-1);
          }
          gps->encode(Serial3.read());
        }

        return (0);
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

int ReceiveGPS::gelay(unsigned long int dtime)
{
  unsigned long start = millis();

  switch (serial_peri.name) {
    case SOFTWARESERIAL:
      {
        SoftwareSerial ss(serial_peri.rx, serial_peri.tx);
        ss.begin(serial_peri.baudrate);
        do {
          while (ss.available())
            gps->encode(ss.read());
        } while (millis() - start < dtime);

        return (0);
        break;
      }

    case HARDWARESERIAL0 :
      {
        Serial.begin(serial_peri.baudrate);
        do {
          while (Serial.available())
            gps->encode(Serial.read());
        } while (millis() - start < dtime);

        return (0);
        break;
      }

#ifdef HAVE_HWSERIAL1
    case HARDWARESERIAL1 :
      {
        Serial1.begin(serial_peri.baudrate);
        do {
          while (Serial1.available())
            gps->encode(Serial1.read());
        } while (millis() - start < ms);

        return (0);
        break;
      }
#endif

#ifdef HAVE_HWSERIAL2
    case HARDWARESERIAL2 :
      {
        Serial2.begin(serial_peri.buadrate);
        do {
          while (Serial2.available())
            gps->encode(Serial2.read());
        } while (millis() - start < dtime);

        return (0);
        break;
      }
#endif

#ifdef HAVE_HWSERIAL3
    case HARDWARESERIAL3 :
      {
        Serial3.begin(serial_peri.baudrate);
        do {
          while (Serial3.available())
            gps->encode(Serial3.read());
        } while (millis() - start < dtime);

        return (0);
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

int ReceiveGPS::into_GPSbuf(float flat_deg, float flon_deg)
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

void ReceiveGPS::disp_errorMsg(char *message)
{
  Serial.print(__FILE__);
  Serial.print(": ");
  Serial.print(errorFunctionName);
  Serial.print(": ");
  Serial.print(message);
  Serial.print(": ");
  Serial.println(status_msg[status]);
}

void ReceiveGPS::disp_errorMsg(void)
{
  Serial.print(__FILE__);
  Serial.print(": ");
  Serial.print(errorFunctionName);
  Serial.print(": ");
  Serial.println(status_msg[status]);
}

void ReceiveGPS::record_errorMsg(char *message)
{

}
