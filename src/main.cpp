#include <Arduino.h>
#include <M5Unified.h>
#include <Ticker.h>
#include "bmi270_config.h"

// IMU Unit
// https://www.switch-science.com/products/6623

// 250Hz, UART, 2byte/axis=6byte/IMU=12byte/2IMU+4byte(header)=16byte -> 4000byte/s
// 115200bps = 11520byte/s 

// note: I2C_CLK_FREQ_MAX is defined -1 in include/hal/i2c_types.h

#define SAMPLE_FREQ 250

#define I2C_ADDR_IMU0 0x68 // IMU#1
#define I2C_ADDR_IMU1 0x69 // IMU#2
#define I2C_CLK_FREQ 400000 // 400kHz

float ax[2], ay[2], az[2];
float conv_acc(uint8_t dh, uint8_t dl){
	int16_t d = (dh << 8) | dl;
	if (d & 0x8000) d = -(~d & 0x7fff); else d = d & 0x7fff;
	return (float)d / 16384.0f;
}
Ticker ticker;
uint8_t fRun = 0;
uint32_t w = 0;
uint8_t buf[10];

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

#define writeRegB(i2c_addr, reg_addr, data) M5.Ex_I2C.writeRegister8(i2c_addr, reg_addr, data, I2C_CLK_FREQ)
#define writeReg(i2c_addr, reg_addr, data, len) M5.Ex_I2C.writeRegister(i2c_addr, reg_addr, data, len, I2C_CLK_FREQ)
#define readRegB(i2c_addr, reg_addr) M5.Ex_I2C.readRegister8(i2c_addr, reg_addr, I2C_CLK_FREQ)
#define readReg(i2c_addr, reg_addr, data, len) M5.Ex_I2C.readRegister(i2c_addr, reg_addr, data, len, I2C_CLK_FREQ)

// for BMI270
void IMUinit(uint8_t i2c_addr)
{
	uint8_t index = 0;
  uint8_t addr_array[2] = { (uint8_t)((index >> 1) & 0x0F), (uint8_t)(index >> 5) };

	// IMU init sequence
//	printf("CHIP_ID(%02x) : %02x\n", readRegB(I2C_ADDR_IMU1, 0x00)); // CHIP_ID(0x00) = 0x24
	writeRegB(i2c_addr, 0x7c, 0x00); // disable adv.power save
	delayMicroseconds(450);
	writeRegB(i2c_addr, 0x59, 0x00); // prepare init

	writeReg(i2c_addr, 0x5b, addr_array, 2);
  writeReg(i2c_addr, 0x5e, (uint8_t *)bmi270_config_file, sizeof(bmi270_config_file));
	writeRegB(i2c_addr, 0x59, 0x01);
	while(readRegB(i2c_addr, 0x21) != 0x01); // INTERNAL_STATUS(0x21), 0x01=init ok

	writeRegB(i2c_addr, 0x7d, 0x04); // enable acc
	writeRegB(i2c_addr, 0x40, 0xaa); // ODR=400Hz, bwp=normal, filer=performance opt.
	writeRegB(i2c_addr, 0x7c, 0x02); // disable adv. power save, enable fifo wakeup
	writeRegB(i2c_addr, 0x41, 0x00); // range : +-2g
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

  M5.Display.setCursor(0, 0); M5.Display.printf("IMU init #0...");
	IMUinit(I2C_ADDR_IMU0);
  M5.Display.setCursor(0, 0); M5.Display.printf("IMU init #1...");
	IMUinit(I2C_ADDR_IMU1);

	M5.Display.clear();
  M5.Display.setCursor(0, 0);
	M5.Display.printf("IMU test");

//  ticker.attach_ms((int)(1000 / SAMPLE_FREQ), onTicker);
//  ticker.detach();
}

void loop()
{
	readReg(I2C_ADDR_IMU0, 0x0c, buf, 6); // accxL, accxH, accyL, accyH, acczL, acczH
	ax[0] = conv_acc(buf[1], buf[0]);
	ay[0] = conv_acc(buf[3], buf[2]);
	az[0] = conv_acc(buf[5], buf[4]);
	readReg(I2C_ADDR_IMU1, 0x0c, buf, 6); // accxL, accxH, accyL, accyH, acczL, acczH
	ax[1] = conv_acc(buf[1], buf[0]);
	ay[1] = conv_acc(buf[3], buf[2]);
	az[1] = conv_acc(buf[5], buf[4]);
	printf("%f %f %f / %f %f %f\n", ax[0], ay[0], az[0], ax[1], ay[1], az[1]);
}

