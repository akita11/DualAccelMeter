#include <Arduino.h>
#include <M5Unified.h>
#include <Ticker.h>

// IMU Unit
// https://www.switch-science.com/products/6623

// 250Hz, UART, 2byte/axis=6byte/IMU=12byte/2IMU+4byte(header)=16byte -> 4000byte/s
// 115200bps = 11520byte/s 

#define SAMPLE_FREQ 250

#define I2C_ADDR1 0x68 // IMU#1
#define I2C_ADDR2 0x69 // IMU#2

Ticker ticker;
uint8_t w = 0;
uint8_t fRun = 0;

void IRAM_ATTR onTicker()
{
  if (fRun == 1)
  {
    w++;
    if (w == SAMPLE_FREQ)
    { // display every 1 sec
      w = 0;
    }
  }
}

void drawStatus(uint8_t f)
{
  if (f == 1)
    M5.Display.fillRect(230, 0, 240, 240, RED);
  else
    M5.Display.fillRect(230, 0, 240, 240, GREEN);
}

void setup()
{
  auto cfg = M5.config();
  cfg.external_imu = false; // for local IMU instalces
  cfg.internal_imu = false;
  cfg.internal_spk = false;
  cfg.internal_mic = false;

  // I2C clock is defined in Unified/src/utility/imu/IMU_Base.hpp
  M5.begin(cfg);
  M5.Ex_I2C.begin();
  M5.Lcd.setFont(&fonts::DejaVu24);

  M5.Lcd.println("I2C device is ready.");

  M5.Display.setCursor(10, 30);
	M5.Display.printf("IMU test");
}

void loop()
{
  printf("%02x\n", M5.Ex_I2C.readRegister8(I2C_ADDR1, 0x00, I2C_CLK_FREQ_MAX));
	delay(1000);
//  ticker.attach_ms((int)(1000 / SAMPLE_FREQ), onTicker);
//  ticker.detach();
}

