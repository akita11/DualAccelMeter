#include <Arduino.h>
#include <M5Unified.h>
#include <Ticker.h>
#include <FastLED.h>
#include "bmi270_config.h"

// IMU Pro Unit
// https://www.switch-science.com/products/9426

// 250Hz, UART, 2byte/axis=6byte/IMU=12byte/2IMU+4byte(header)=16byte -> 4000byte/s
// 115200bps = 11520byte/s

// note: I2C_CLK_FREQ_MAX is defined -1 in include/hal/i2c_types.h

#define NUM_LEDS 1
#define LED_DATA_PIN 35
static CRGB leds[NUM_LEDS];

#define SAMPLE_FREQ 250

#define I2C_ADDR_IMU0 0x68	// IMU#1
#define I2C_ADDR_IMU1 0x69	// IMU#2
#define I2C_CLK_FREQ 400000 // 400kHz

float ax[2], ay[2], az[2];
float gx[2], gy[2], gz[2];
float mx[2], my[2], mz[2];
volatile uint8_t fReady = 0;

int conv_value(uint8_t dh, uint8_t dl)
{
	uint16_t d = dh << 8 | dl;
	int ret;
	if (d & 0x8000) ret = -(~d & 0x7fff);
	else ret = d & 0x7fff;
	return (ret);
}
Ticker ticker;
uint8_t fRun = 0;
;
uint8_t buf0[20], buf1[20];

#define writeRegB(i2c_addr, reg_addr, data) M5.Ex_I2C.writeRegister8(i2c_addr, reg_addr, data, I2C_CLK_FREQ)
#define writeReg(i2c_addr, reg_addr, data, len) M5.Ex_I2C.writeRegister(i2c_addr, reg_addr, data, len, I2C_CLK_FREQ)
#define readRegB(i2c_addr, reg_addr) M5.Ex_I2C.readRegister8(i2c_addr, reg_addr, I2C_CLK_FREQ)
#define readReg(i2c_addr, reg_addr, data, len) M5.Ex_I2C.readRegister(i2c_addr, reg_addr, data, len, I2C_CLK_FREQ)

bool auxWriteRegB(uint8_t i2c_addr, uint8_t reg, uint8_t data)
{
	writeRegB(i2c_addr, 0x4f, data); // data to write to AUX
	writeRegB(i2c_addr, 0x4e, reg);	 // address to write to AUX
	int retry = 3;
	while ((readRegB(i2c_addr, 0x03) & 0b100) && --retry)
	{
		vTaskDelay(1);
	}
	return retry;
}

uint8_t auxReadRegB(uint8_t i2c_addr, uint8_t reg)
{
	writeRegB(i2c_addr, 0x4c, 0x80); // enable read write. Burst length 1
	writeRegB(i2c_addr, 0x4d, reg);	 // addr to read from AUX
	int retry = 3;
	while ((readRegB(i2c_addr, 0x03) & 0b100) && --retry)
	{
		vTaskDelay(1);
	}
	return readRegB(i2c_addr, 0x04); // AUX X's LSB
}

// for BMI270
void IMUinit(uint8_t i2c_addr)
{
	uint8_t index = 0;
	uint8_t addr_array[2] = {(uint8_t)((index >> 1) & 0x0F), (uint8_t)(index >> 5)};

	// IMU init sequence
	//	printf("CHIP_ID(%02x) : %02x\n", readRegB(I2C_ADDR_IMU1, 0x00)); // CHIP_ID(0x00) = 0x24
	writeRegB(i2c_addr, 0x7c, 0x00); // disable adv.power save
	delayMicroseconds(450);
	writeRegB(i2c_addr, 0x59, 0x00); // prepare init
	writeReg(i2c_addr, 0x5b, addr_array, 2);
	writeReg(i2c_addr, 0x5e, (uint8_t *)bmi270_config_file, sizeof(bmi270_config_file));
	writeRegB(i2c_addr, 0x59, 0x01);
	while (readRegB(i2c_addr, 0x21) != 0x01)
		; // INTERNAL_STATUS(0x21), 0x01=init ok

	writeRegB(i2c_addr, 0x7d, 0x07); // enable acc/gyro/aux
	writeRegB(i2c_addr, 0x40, 0xaa); // Acc ODR=400Hz, bwp=normal, filer=performance opt.
	writeRegB(i2c_addr, 0x7c, 0x02); // disable adv. power save, enable fifo wakeup
	writeRegB(i2c_addr, 0x41, 0x00); // Acc range : +-2g
	writeRegB(i2c_addr, 0x42, 0xea); // Gyro bwp=2(normal), perf=perform opt., ODR=400Hz
	writeRegB(i2c_addr, 0x43, 0x00); // Gyro range : +-2000dps(+-15bit)

	// AUX init
	writeRegB(i2c_addr, 0x6b, 0x20); // AUX I2C enable.
	writeRegB(i2c_addr, 0x7c, 0x00); // Power save disabled.
	writeRegB(i2c_addr, 0x7d, 0x0e); // AUX sensor disable.
	writeRegB(i2c_addr, 0x4c, 0x80); // enable manual AUX
	writeRegB(i2c_addr, 0x4b, 0x10 << 1); // BMM150's I2C addr

	auxWriteRegB(i2c_addr, 0x4b, 0x83);					 // software reset + power on
	auto who_am_i = auxReadRegB(i2c_addr, 0x40); // 0x40 = WhoAmI
	if (who_am_i == 0x32)
	{
		auxWriteRegB(i2c_addr, 0x4C, 0x38); // normal mode / ODR 30Hz
		writeRegB(i2c_addr, 0x4c, 0x4f);		// FCU_WRITE_EN + Manual BurstLength 8 + BurstLength 8
		writeRegB(i2c_addr, 0x4d, 0x42);		// 0x42 = BMM150 I2C Data X LSB reg
		writeRegB(i2c_addr, 0x7d, 0x0f);		// temp en | ACC en | GYR en | AUX en
	}
}

uint32_t t0, tm;

void IRAM_ATTR onTicker()
{
	if (fRun == 1)
	{
		readReg(I2C_ADDR_IMU0, 0x04, buf0, 20);
		readReg(I2C_ADDR_IMU1, 0x04, buf1, 20);
		mx[0] = (float)(conv_value(buf0[1], buf0[0]) >> 3); // 13bit
		my[0] = (float)(conv_value(buf0[3], buf0[2]) >> 3); // 13bit
		mz[0] = (float)(conv_value(buf0[5], buf0[4]) >> 1); // 15bit
		ax[0] = (float)conv_value(buf0[ 9], buf0[ 8]) / 16384.0f;  // [x1g]
		ay[0] = (float)conv_value(buf0[11], buf0[10]) / 16384.0f;
		az[0] = (float)conv_value(buf0[13], buf0[12]) / 16384.0f;
		gx[0] = (float)conv_value(buf0[15], buf0[14]) / (32768.0f * 2000.0f); // [dps(deg/sec)]
		gy[0] = (float)conv_value(buf0[17], buf0[16]) / (32768.0f * 2000.0f);
		gz[0] = (float)conv_value(buf0[19], buf0[18]) / (32768.0f * 2000.0f);
		mx[1] = (float)(conv_value(buf1[1], buf0[0]) >> 3); // 13bit
		my[1] = (float)(conv_value(buf1[3], buf0[2]) >> 3); // 13bit
		mz[1] = (float)(conv_value(buf1[5], buf0[4]) >> 1); // 15bit
		ax[1] = (float)conv_value(buf1[ 9], buf0[ 8]) / 16384.0f;  // [x1g]
		ay[1] = (float)conv_value(buf1[11], buf0[10]) / 16384.0f;
		az[1] = (float)conv_value(buf1[13], buf0[12]) / 16384.0f;
		gx[1] = (float)conv_value(buf1[15], buf0[14]) / (32768.0f * 2000.0f); // [dps(deg/sec)]
		gy[1] = (float)conv_value(buf1[17], buf0[16]) / (32768.0f * 2000.0f);
		gz[1] = (float)conv_value(buf1[19], buf0[18]) / (32768.0f * 2000.0f);
		/*
		void BMI270_Class::getConvertParam(imu_convert_param_t* param) const
		{
			param->mag_res = 10.0f * 4912.0f / 32760.0f;
			param->temp_offset = 23.0f;
			param->temp_res = 1.0f / 512.0f;
		}
		*/
		fReady = 1;
	}
}

void setMeasure(uint8_t f)
{
	if (f == 1)
	{
		leds[0] = CRGB(30, 30, 0);
		FastLED.show();
		ticker.attach_ms((int)(1000 / SAMPLE_FREQ), onTicker);
	}
	else
	{
		ticker.detach();
		leds[0] = CRGB(0, 30, 0);
		FastLED.show();
	}
}

void setup()
{
	auto cfg = M5.config();
	cfg.external_imu = false;
	cfg.internal_imu = false;
	cfg.internal_spk = false;
	cfg.internal_mic = false;

	// I2C clock is defined in Unified/src/utility/imu/IMU_Base.hpp
	M5.begin(cfg);
	M5.Ex_I2C.begin();

	FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
	FastLED.setBrightness(128);
	FastLED.clear();

	leds[0] = CRGB(30, 0, 0);
	FastLED.show();
	IMUinit(I2C_ADDR_IMU0);
	IMUinit(I2C_ADDR_IMU1);

	leds[0] = CRGB(0, 30, 0);
	FastLED.show();

	fRun = 0;
	setMeasure(fRun);
}

void loop()
{
	M5.update();
	if (M5.BtnA.wasPressed()){
		fRun = 1 - fRun;
		setMeasure(fRun);
		delay(500);
	}
	if (fRun == 1){
		fReady = 0; while (fReady == 0);
		uint32_t t1 = micros();
		tm = t1 - t0;
		t0 = t1;
//		printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", tm, ax[0], ay[0], az[0], ax[1], ay[1], az[1]);
//		printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", tm, gx[0], gy[0], gz[0], gx[1], gy[1], gz[1]);
		printf("%d,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n",tm, mx[0], my[0], mz[0], mx[1], my[1], mz[1]);
	}
}
