#include <Arduino.h>
#include <M5Unified.h>
#include "TCA9548.h"
#include "ntp.h"
#include <Ticker.h>
#include <SD.h>

// for StickCPlus2
// https://gist.github.com/mongonta0716/d4a8948572ea3f723fc92c568c8d2578

SPIClass SPI_EXT;

// ToDo:
// v NTP WiFi接続情報をSDから
// v SD保存
// v M5.Imuデータ取得
// - PaHubでの複数取得@250Hz

TCA9548 PaHub2(0x70); // PaHub2 with address=0x70

// IMU
// https://www.switch-science.com/products/6623
// PaHUB2
// https://shop.m5stack.com/products/i2c-hub-1-to-6-expansion-unit-pca9548apw?_pos=1&_sid=9ea0dfe51&_ss=r&variant=40724626833580
// https://github.com/m5stack/M5Stack/blob/master/examples/Unit/PaHUB_TCA9548A/PaHUB_TCA9548A.ino?fbclid=IwAR35i5FBEDKPZBe5ovpPSaczyekhg507NC6amWv-EwJnkGQSw4HOd5b0Qbc

#define SAMPLE_FREQ 250
#define LOG_FILENAME "/log.csv"

Ticker ticker;
uint8_t w = 0;
uint8_t fRun = 0;
File fp;

void IRAM_ATTR onTicker(){
  float x0, y0, z0, x1, y1, z1;
  if (fRun == 1){
    PaHub2.selectChannel(0);
    M5.Imu.getAccel(&x0, &y0, &z0);
    PaHub2.selectChannel(1);
    M5.Imu.getAccel(&x1, &y1, &z1);
    // x0 = (float)random(1000) / 1000.0; y0 = (float)random(1000) / 1000.0; z0 = (float)random(1000) / 1000.0;
    // x1 = (float)random(1000) / 1000.0; y1 = (float)random(1000) / 1000.0; z1 = (float)random(1000) / 1000.0;
    auto dt = M5.Rtc.getDateTime();
    fp.printf("%02d%02d%02d,%02d%02d%02d,", dt.date.year % 100, dt.date.month, dt.date.date, dt.time.hours, dt.time.minutes, dt.time.seconds);
    fp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", x0, y0, z0, x1, y1, z1);
    w++;

    if (w == SAMPLE_FREQ){ // display every 1 sec
      w = 0;
      M5.Display.clear();
      M5.Display.setCursor(0, 0);
      M5.Display.printf("%02d%02d%02d %02d%02d%02d ", dt.date.year % 100, dt.date.month, dt.date.date, dt.time.hours, dt.time.minutes, dt.time.seconds);
      M5.Display.setCursor(10, 30); M5.Display.printf("%.2f %.2f %.2f", x0, y0, z0);
      M5.Display.setCursor(10, 50); M5.Display.printf("%.2f %.2f %.2f", x1, y1, z1);
  }
  }
}

void setup() {
  auto cfg = M5.config();
  cfg.external_imu = true;
  cfg.internal_imu = false;
  M5.begin(cfg);

//  PaHub2.enableChannel(0);
//  PaHub2.enableChannel(1);

  M5.Display.setRotation(1);
  M5.Lcd.setFont(&fonts::DejaVu24);
  pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, 0); // StickC's red LED

  // StickCPlus's SD HAT
  // https://booth.pm/ja/items/2385035
//  SPI_EXT.begin(G0, G36, G26);
  SPI_EXT.begin(G0, G36, G26, -1);
//  SPI_EXT.begin(G0, G36, G26, 2); // dummy CS for G2
  if (!SD.begin(-1, SPI_EXT, 15000000)){
    while(1){
      digitalWrite(PIN_LED, 0); delay(100);
      digitalWrite(PIN_LED, 1); delay(100);
    }
  }
  M5.Display.fillRect(200, 0, 240, 135, RED);
}

void loop() {
  M5.update();
  if (M5.BtnA.wasClicked()){
    if (fRun == 0){
      fRun = 1;
      M5.Display.fillRect(200, 0, 240, 135, BLACK);
      fp = SD.open(LOG_FILENAME, "a");
      if (!fp){
        // fast LED flash if log file open error
        for (uint8_t i = 0; i < 5; i++){
          digitalWrite(PIN_LED, 0); delay(100);
          digitalWrite(PIN_LED, 1); delay(100);
        }
        digitalWrite(PIN_LED, 0); delay(50);
      }
      ticker.attach_ms((int)(1000 / SAMPLE_FREQ), onTicker);
    }
    else{
      fRun = 0;
      M5.Display.fillRect(200, 0, 240, 135, RED);
      fp.close();
      ticker.detach();
    }
  }

  if (M5.BtnB.wasClicked()){
    printf("NTP adjust\n");
    NTPadjust();
  }
}
