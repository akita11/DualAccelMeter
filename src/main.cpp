#include <Arduino.h>
#include <M5Unified.h>
#include <Ticker.h>
#include "bmi270_config.h"

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

//	static constexpr const std::uint8_t INIT_ADDR_0             = 0x5B;
//	static constexpr const std::uint8_t INIT_ADDR_1             = 0x5C;
//	static constexpr const std::uint8_t INIT_DATA_ADDR          = 0x5E;

	uint8_t index = 0;
  uint8_t addr_array[2] = {
      (uint8_t)((index >> 1) & 0x0F),
      (uint8_t)(index >> 5)
    };

//	M5.Ex_I2C.writeRegister( I2C_ADDR1, 0x5b, addr_array, 2, I2C_CLK_FREQ_MAX);
//  M5.Ex_I2C.writeRegister( I2C_ADDR1, 0x5e, (uint8_t *)bmi270_config_file, sizeof(bmi270_config_file), I2C_CLK_FREQ_MAX);

	// IMU init sequence
	// write_reg(0x7c, 0x00); // disable adv.power save
	// wait 450us
	// write_reg(0x59, 0x00); // prepare init
	// prepare init_array[] & burst_write from 0x5e
	// write_reg(0x59, 0x01); // complete init

	// read_reg(0x21) == 1 -> OK
	// write_reg(0x7d, 0x04); enable acc
	// write_reg(0x40, 0xaa); // ODR=400Hz, bwp=normal, filer=performance opt. 
	// write_reg(0x7c, 0x02); // disable adv. power save, enable fifo wakeup
	// read acc

}

void loop()
{
	// CHIP_ID(0x00) = 0x24
  printf("%02x\n", M5.Ex_I2C.readRegister8(I2C_ADDR1, 0x00, I2C_CLK_FREQ_MAX));
	delay(1000);
//  ticker.attach_ms((int)(1000 / SAMPLE_FREQ), onTicker);
//  ticker.detach();
}

