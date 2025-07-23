#include <Arduino.h>
#include <M5Unified.h>
#include <Ticker.h>
#include <FastLED.h>
#include "bmi270_config.h"
#include <EEPROM.h>

#define SAMPLE_FREQ 250
//#define SAMPLE_FREQ 100
//#define SAMPLE_FREQ 25

#define I2C_ADDR_IMU0 0x68	// IMU#0
#define I2C_ADDR_IMU1 0x69	// IMU#1
uint8_t i2c_addr[2] = {I2C_ADDR_IMU0, I2C_ADDR_IMU1};

// Error codes
#define BMI270_OK 0
#define BMI270_ERR_WRITE_FAILED 1
#define BMI270_ERR_READ_FAILED 2
#define BMI270_ERR_TIMEOUT 3
#define BMI270_ERR_WRONG_CHIP_ID 4
#define BMI270_ERR_AUX_WRITE_FAILED 5

// Physical constants
#define GRAVITY 9.80665f // Standard gravity in m/s^2

// Sensor data validity checks
#define ACC_MAX_MS2 2.0f     // Maximum acceleration in [g]
#define GYRO_MAX_DPS 2000.0f // Maximum angular rate in degrees/s
#define MAG_MAX_UT 1000.0f	 // Maximum magnetic field in μT

// Retry settings
#define MAX_RETRY_COUNT 10
#define RETRY_DELAY_MS 1

// BMI270 Register Addresses
#define BMI270_REG_CHIP_ID 0x00
#define BMI270_REG_STATUS 0x03
#define BMI270_REG_AUX_DATA 0x04
#define BMI270_REG_INTERNAL_STATUS 0x21
#define BMI270_REG_GYRO_RANGE 0x43
#define BMI270_REG_GYRO_CONFIG 0x42
#define BMI270_REG_ACC_CONFIG 0x40
#define BMI270_REG_ACC_RANGE 0x41
#define BMI270_REG_AUX_DEV_ID 0x4B
#define BMI270_REG_AUX_IF_CONF 0x4C
#define BMI270_REG_AUX_RD_ADDR 0x4D
#define BMI270_REG_AUX_WR_ADDR 0x4E
#define BMI270_REG_AUX_WR_DATA 0x4F
#define BMI270_REG_CMD 0x7E
#define BMI270_REG_PWR_CONF 0x7C
#define BMI270_REG_PWR_CTRL 0x7D
#define BMI270_REG_INIT_CTRL 0x59
#define BMI270_REG_INIT_ADDR_0 0x5B
#define BMI270_REG_INIT_ADDR_1 0x5C
#define BMI270_REG_INIT_DATA 0x5E

// BMI270 Configuration Values
#define BMI270_PWR_CONF_ADV_OFF 0x00
#define BMI270_PWR_CONF_FIFO_WU 0x02
#define BMI270_PWR_CTRL_ALL_ON 0x07
#define BMI270_INIT_START 0x00
#define BMI270_INIT_COMPLETE 0x01
#define BMI270_ACC_ODR_400HZ 0xAA
#define BMI270_ACC_RANGE_2G 0x00
#define BMI270_GYRO_ODR_400HZ 0xEA
#define BMI270_GYRO_ODR_100HZ 0xE8
#define BMI270_GYRO_ODR_25HZ 0xE6

// AUX (BMM150) related
#define BMM150_AUX_RD_BURST_LEN1 0x80

// MadgwickAHRS: https://qiita.com/Ninagawa123/items/9520bad3c78ee40194fc
// 9axis: https://misosoup4258.hatenablog.com/entry/2021/12/29/185801#Madgwick%E3%83%95%E3%82%A3%E3%83%AB%E3%82%BF%E3%83%BC

#include "MadgwickAHRS.h"
Madgwick mf[2];
float roll[2], pitch[2], yaw[2];
std::vector<int> magX[2], magY[2], magZ[2];

// IMU Pro Unit
// https://www.switch-science.com/products/9426

// 250Hz, UART, 2byte/axis=6byte/IMU=12byte/2IMU+4byte(header)=16byte -> 4000byte/s
// 115200bps = 11520byte/s

// note: I2C_CLK_FREQ_MAX is defined -1 in include/hal/i2c_types.h

#define NUM_LEDS 1
#define LED_DATA_PIN 35
static CRGB leds[NUM_LEDS];

#define I2C_CLK_FREQ 400000 // 400kHz

float ax[2], ay[2], az[2];
float gx[2], gy[2], gz[2];
float mx[2], my[2], mz[2];
volatile uint8_t fReady = 0;

// High precision timer handle
esp_timer_handle_t timer_handle;

// Function prototypes
bool is_acc_valid(float ax, float ay, float az);
bool is_gyro_valid(float gx, float gy, float gz);
bool is_mag_valid(float mx, float my, float mz);

// Sensor data validation functions
bool is_acc_valid(float ax, float ay, float az)
{
	return (fabs(ax) <= ACC_MAX_MS2 &&
					fabs(ay) <= ACC_MAX_MS2 &&
					fabs(az) <= ACC_MAX_MS2);
}
bool is_gyro_valid(float gx, float gy, float gz)
{
	return (fabs(gx) <= GYRO_MAX_DPS &&
					fabs(gy) <= GYRO_MAX_DPS &&
					fabs(gz) <= GYRO_MAX_DPS);
}
bool is_mag_valid(float mx, float my, float mz)
{
	return (fabs(mx) <= MAG_MAX_UT &&
					fabs(my) <= MAG_MAX_UT &&
					fabs(mz) <= MAG_MAX_UT);
}

int conv_value(uint8_t dh, uint8_t dl)
{
	uint16_t d = dh << 8 | dl;
	int ret;
	if (d & 0x8000)
		ret = -(~d & 0x7fff);
	else
		ret = d & 0x7fff;
	return (ret);
}
Ticker ticker;
uint8_t fRun = 0;

uint8_t buf[20];

#define writeRegB(i2c_addr, reg_addr, data) M5.Ex_I2C.writeRegister8(i2c_addr, reg_addr, data, I2C_CLK_FREQ)
#define writeReg(i2c_addr, reg_addr, data, len) M5.Ex_I2C.writeRegister(i2c_addr, reg_addr, data, len, I2C_CLK_FREQ)
#define readRegB(i2c_addr, reg_addr) M5.Ex_I2C.readRegister8(i2c_addr, reg_addr, I2C_CLK_FREQ)
#define readReg(i2c_addr, reg_addr, data, len) M5.Ex_I2C.readRegister(i2c_addr, reg_addr, data, len, I2C_CLK_FREQ)

bool auxWriteRegB(uint8_t i2c_addr, uint8_t reg, uint8_t data)
{
	// AUXにアドレスを書き込み
	if (!writeRegB(i2c_addr, BMI270_REG_AUX_WR_ADDR, reg))
	{
		return false;
	}

	// AUXにデータを書き込み
	if (!writeRegB(i2c_addr, BMI270_REG_AUX_WR_DATA, data))
	{
		return false;
	}

	// 操作完了を待機（タイムアウト付き）
	int retry = MAX_RETRY_COUNT;
	int status;
	do
	{
		status = readRegB(i2c_addr, BMI270_REG_STATUS);
		if (status < 0)
		{ // 読み取りエラー
			return false;
		}
		if (--retry <= 0)
		{ // タイムアウト
			return false;
		}
		vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
	} while (status & 0b100);

	return true;
}
int auxReadRegB(uint8_t i2c_addr, uint8_t reg)
{
	// Enable read with burst length 1
	if (!writeRegB(i2c_addr, BMI270_REG_AUX_IF_CONF, BMM150_AUX_RD_BURST_LEN1))
	{
		return BMI270_ERR_WRITE_FAILED;
	}

	// Set address to read from AUX
	if (!writeRegB(i2c_addr, BMI270_REG_AUX_RD_ADDR, reg))
	{
		return BMI270_ERR_WRITE_FAILED;
	}

	// Wait for operation to complete with timeout
	int retry = MAX_RETRY_COUNT;
	while ((readRegB(i2c_addr, BMI270_REG_STATUS) & 0b100) && --retry)
	{
		vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
	}

	if (retry <= 0)
	{
		return BMI270_ERR_TIMEOUT;
	}

	// Read the data
/*
	int result = readRegB(i2c_addr, BMI270_REG_AUX_DATA);
	if (result < 0)
	{
		return BMI270_ERR_READ_FAILED;
	}

	return result;
*/
	return(readRegB(i2c_addr, BMI270_REG_AUX_DATA));
}

// for BMI270&BMM150
int IMUinit(uint8_t i2c_addr)
{
	uint8_t index = 0;
	uint8_t addr_array[2] = {(uint8_t)((index >> 1) & 0x0F), (uint8_t)(index >> 5)};

	// IMU init sequence
	//printf("CHIP_ID : %02x\n", readRegB(i2c_addr, 0x00)); // CHIP_ID = 0x24
	if (!writeRegB(i2c_addr, BMI270_REG_PWR_CONF, BMI270_PWR_CONF_ADV_OFF))
	{ // disable adv.power save
		return BMI270_ERR_WRITE_FAILED;
	}
	delayMicroseconds(450);

	if (!writeRegB(i2c_addr, BMI270_REG_INIT_CTRL, BMI270_INIT_START))
	{ // prepare init
		return BMI270_ERR_WRITE_FAILED;
	}
	if (!writeReg(i2c_addr, BMI270_REG_INIT_ADDR_0, addr_array, 2))
	{
		return BMI270_ERR_WRITE_FAILED;
	}
	if (!writeReg(i2c_addr, BMI270_REG_INIT_DATA, (uint8_t *)bmi270_config_file, sizeof(bmi270_config_file)))
	{
		return BMI270_ERR_WRITE_FAILED;
	}
	if (!writeRegB(i2c_addr, BMI270_REG_INIT_CTRL, BMI270_INIT_COMPLETE))
	{
		return BMI270_ERR_WRITE_FAILED;
	}

	// 初期化完了待ち
	uint8_t status;
	uint8_t retry = 0;
	do
	{
		status = readRegB(i2c_addr, BMI270_REG_INTERNAL_STATUS);
		if (status == 0xFF)
		{ // 読み取りエラー
			return BMI270_ERR_READ_FAILED;
		}
		if (++retry > 100)
		{ // タイムアウト
			return BMI270_ERR_TIMEOUT;
		}
		delay(1);
	} while (status != BMI270_INIT_COMPLETE);

	// センサーの設定
	writeRegB(i2c_addr, BMI270_REG_PWR_CTRL, BMI270_PWR_CTRL_ALL_ON);	 // enable acc/gyro/aux
	writeRegB(i2c_addr, BMI270_REG_ACC_CONFIG, BMI270_ACC_ODR_400HZ);	 // Acc ODR=400Hz
	writeRegB(i2c_addr, BMI270_REG_PWR_CONF, BMI270_PWR_CONF_FIFO_WU);	 // disable adv. power save
	writeRegB(i2c_addr, BMI270_REG_ACC_RANGE, BMI270_ACC_RANGE_2G);		 // Acc range : +-2g
	writeRegB(i2c_addr, BMI270_REG_GYRO_CONFIG, BMI270_GYRO_ODR_400HZ); // Gyro config
	writeRegB(i2c_addr, BMI270_REG_GYRO_RANGE,  0x00); // Gyro range = +-2000dps

	// AUX (BMM150) の初期化
	writeRegB(i2c_addr, 0x6b, 0x20); // AUX I2C enable
	writeRegB(i2c_addr, 0x7c, 0x00); // Power save disabled
	writeRegB(i2c_addr, 0x7d, 0x0e); // AUX sensor disable
	writeRegB(i2c_addr, 0x4c, 0x80); // enable manual AUX
	writeRegB(i2c_addr, 0x4b, 0x10 << 1);


	if (!auxWriteRegB(i2c_addr, 0x4b, 0x83))
	{ // software reset + power on
		return BMI270_ERR_AUX_WRITE_FAILED;
	}

	auto who_am_i = auxReadRegB(i2c_addr, 0x40); // 0x40 = WhoAmI
	if (who_am_i != 0x32)
	{
		return BMI270_ERR_WRONG_CHIP_ID;
	}

	auxWriteRegB(i2c_addr, 0x4C, 0x38);  // normal mode / ODR 30Hz
	writeRegB(i2c_addr, 0x4c, 0x4f); 	 // FCU_WRITE_EN + Manual BurstLength 8
	writeRegB(i2c_addr, 0x4d, 0x42); 	 // 0x42 = BMM150 I2C Data X LSB reg
	writeRegB(i2c_addr, 0x7d, 0x0f);  // temp en | ACC en | GYR en | AUX en

	return BMI270_OK;
}

uint32_t t0, tm;

//void IRAM_ATTR onTimer(void *arg)
void IRAM_ATTR onTimer()
{
	if (!fRun) return;
	bool imu_valid[2];
	// Read and validate IMU data
	for (uint8_t i = 0; i < 2; i++){
		imu_valid[i] = true;
		if (!readReg(i2c_addr[i], BMI270_REG_AUX_DATA, buf, 20)) imu_valid[i] = false;
		else {
			// Process IMU data
			// BMNM150 : +-1300uT(x/y), +-2500uT(z) (typ)
			mx[i] = (float)(conv_value(buf[1], buf[0]) >> 3); // 13bit (+-4096)
			my[i] = (float)(conv_value(buf[3], buf[2]) >> 3); // 13bit (+-4096)
			mz[i] = (float)(conv_value(buf[5], buf[4]) >> 1); // 15bit (+-16384)
			ax[i] = (float)conv_value(buf[ 9], buf[ 8]) / 16384.0f; // [g]
			ay[i] = (float)conv_value(buf[11], buf[10]) / 16384.0f;
			az[i] = (float)conv_value(buf[13], buf[12]) / 16384.0f;
			gx[i] = (float)conv_value(buf[15], buf[14]) / 32768.0f * 2000.0f; // [dps]
			gy[i] = (float)conv_value(buf[17], buf[16]) / 32768.0f * 2000.0f;
			gz[i] = (float)conv_value(buf[19], buf[18]) / 32768.0f * 2000.0f;
/*
			// Validate sensor data
			if (!is_acc_valid(ax[i], ay[i], az[i]) ||
				!is_gyro_valid(gx[i], gy[i], gz[i]) ||
				!is_mag_valid(mx[i], my[i], mz[i])) imu_valid[i] = false;
*/
		}
	}
	fReady = 1;
}

void setMeasure(uint8_t f)
{
	if (f == 1 || f == 3)
	{
		if (f == 1) leds[0] = CRGB(30, 30, 0); // yellow, running
		else if (f == 3) leds[0] = CRGB(30, 0, 30); // purple, calibration
		ticker.attach_ms((int)(1000 / SAMPLE_FREQ), onTimer);
	}
	else if (f == 0 || f == 2)
	{
		ticker.detach();
		if (f == 0) leds[0] = CRGB(0, 30, 0); // green, idle
		else if (f == 2) leds[0] = CRGB(0, 30, 30); // cyan, calibration idle
	}
	FastLED.show();
}

float mx0[2], my0[2], mz0[2], ma[2], mb[2], mc[2], fmx[2], fmy[2], fmz[2];
uint16_t nSample = 0;
const int EEPROM_SIZE = 64;
const int EEPROM_ADDR = 0; 
float calib[12];

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

	// LEDの初期化
	FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
	FastLED.setBrightness(128);
	FastLED.clear();

	// IMUの初期化（エラーチェック付き）
	leds[0] = CRGB(30, 0, 0); // 初期化中は赤
	FastLED.show();

	int result;
	while(IMUinit(I2C_ADDR_IMU0) != BMI270_OK) delay(10);
	while(IMUinit(I2C_ADDR_IMU1) != BMI270_OK) delay(10);
/*
	if (result != BMI270_OK)
	{
		while (1)
		{
			leds[0] = CRGB::Red; FastLED.show(); delay(200);
			leds[0] = CRGB::Black; FastLED.show(); delay(200);
		}
	}
	result = IMUinit(I2C_ADDR_IMU1);
	if (result != BMI270_OK)
	{
		while (1)
		{
			leds[0] = CRGB::Yellow; FastLED.show(); delay(200);
			leds[0] = CRGB::Black; FastLED.show(); delay(200);
		}
	}
*/
/*
	// 高精度タイマーの設定（250Hz）
	esp_timer_create_args_t timer_args = {
			.callback = onTimer,
			.arg = NULL,
			.dispatch_method = ESP_TIMER_TASK,
			.name = "imu_timer"};

	ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
	ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, 1000000 / SAMPLE_FREQ)); // マイクロ秒に変換
*/

	// Madgwickフィルタの初期化
	mf[0].begin(SAMPLE_FREQ);
	mf[1].begin(SAMPLE_FREQ);
//	mf[0].setGain(0.1); // beta (default=0.1)
//	mf[1].setGain(0.9); // beta (default=0.1)

	// 初期化成功を表示
	leds[0] = CRGB(0, 30, 0); // 成功時は緑
	FastLED.show();

	M5.update();
	if (M5.BtnA.isPressed())
	{
		fRun = 2; // calibration mode (idle)
	}
	else{
		fRun = 0; // measurement mode (idle)
		EEPROM.begin(EEPROM_SIZE);
		EEPROM.get(EEPROM_ADDR, calib);
		for (uint8_t i = 0; i < 2; i++){
			mx0[i] = calib[i*6+6]; my0[i] = calib[i*6+7];  mz0[i] = calib[i*6+8];
			ma[i]  = calib[i*6+9]; mb[i]  = calib[i*6+10]; mc[i]  = calib[i*6+11];
			printf("calib[%d]: %.3f %.3f %.3f / %.3f %.3f %.3f\n", i, mx0[i], my0[i], mz0[i], ma[i], mb[i], mc[i]);
		}
	}
	setMeasure(fRun);
}

void calc_calib()
{
	int N = magX[0].size();
	printf("cap.N=%d\n", N);
	if (N < 100) printf("Not enough data.\n");
	else
	{
		for (uint8_t p = 0; p < 2; p++){
			// 正規方程式に基づく最小二乗法, Solve Ax = b for [Bx By Bz D]^T
			float Sxx = 0, Sxy = 0, Sxz = 0, Sx1 = 0;
			float Syy = 0, Syz = 0, Sy1 = 0;
			float Szz = 0, Sz1 = 0;
			float S11 = N;
			float Sx_r2 = 0, Sy_r2 = 0, Sz_r2 = 0, S1_r2 = 0;
			for (int i = 0; i < N; ++i){
				float x = magX[p][i], y = magY[p][i], z = magZ[p][i];
				//printf("%d %d %.3f\n", i, magX[i], x);
				float r2 = x * x + y * y + z * z;
				Sxx += x * x;
				Sxy += x * y;
				Sxz += x * z;
				Sx1 += x;
				Syy += y * y;
				Syz += y * z;
				Sy1 += y;
				Szz += z * z;
				Sz1 += z;

				Sx_r2 += x * r2;
				Sy_r2 += y * r2;
				Sz_r2 += z * r2;
				S1_r2 += r2;
			}

			float A[4][4] = {
				{Sxx, Sxy, Sxz, Sx1},
				{Sxy, Syy, Syz, Sy1},
				{Sxz, Syz, Szz, Sz1},
				{Sx1, Sy1, Sz1, S11}};
			float B[4] = {Sx_r2, Sy_r2, Sz_r2, S1_r2};

			// ガウス消去
			for (int i = 0; i < 4; i++){
				float pivot = A[i][i];
				for (int j = 0; j < 4; j++) A[i][j] /= pivot;
				B[i] /= pivot;
				for (int k = i + 1; k < 4; k++){
					float factor = A[k][i];
					for (int j = 0; j < 4; j++) A[k][j] -= factor * A[i][j];
					B[k] -= factor * B[i];
				}
			}
			float X[4];
			for (int i = 3; i >= 0; i--){
				X[i] = B[i];
				for (int j = i + 1; j < 4; j++){
					X[i] -= A[i][j] * X[j];
				}
			}
			mx0[p] = X[0] / 2.0;
			my0[p] = X[1] / 2.0;
			mz0[p] = X[2] / 2.0;

			// スケールは中心補正後の分散で近似
			float sumX2 = 0, sumY2 = 0, sumZ2 = 0;
			for (int i = 0; i < N; ++i){
				float dx = magX[p][i] - mx0[p];
				float dy = magY[p][i] - my0[p];
				float dz = magZ[p][i] - mz0[p];
				sumX2 += dx * dx;
				sumY2 += dy * dy;
				sumZ2 += dz * dz;
			}
			ma[p] = sqrt(sumX2 / N);
			mb[p] = sqrt(sumY2 / N);
			mc[p] = sqrt(sumZ2 / N);
			printf("%d: %.3f %.3f %.3f %.3f %.3f %.3f\n", p, mx0[p], my0[p], mz0[p], ma[p], mb[p], mc[p]);
			// そのまま使用可能なスケール係数（1で正規化する場合）
			EEPROM.begin(EEPROM_SIZE);
			calib[p*6]   = mx0[p]; calib[p*6+1] = my0[p]; calib[p*6+2] = mz0[p];
			calib[p*6+3] = ma[p];  calib[p*6+4] = mb[p];  calib[p*6+5] = mc[p];
		}
		EEPROM.put(EEPROM_ADDR, calib);
		EEPROM.commit();
	}
}

void loop()
{
	M5.update();
	if (M5.BtnA.wasPressed())
	{
		switch(fRun){
			case 0 : fRun = 1; for (uint8_t i = 0; i < 2; i++){ magX[i].clear(); magY[i].clear(); magZ[i].clear();} break;
			case 1 : fRun = 0; calc_calib(); break;
			case 2 : fRun = 3; break;
			case 3 : fRun = 2; break;
		}
/*
		if (fRun == 1){
			// Roller485 cmd:
			// 0x00: 1=on
			// 0x01: 1=speed mode
			// 0x40-0x43: speed (LSB->MSB) / 100
			writeRegB(0x64, 0x01, 1); // speed mode
			writeRegB(0x64, 0x40, 0x00); // speed[0]
			writeRegB(0x64, 0x41, 0x18); // speed[1]
			writeRegB(0x64, 0x42, 0x00); // speed[2]
			writeRegB(0x64, 0x43, 0x00); // speed[3]
			writeRegB(0x64, 0x00, 0x01); // ON
		}
		else{
			writeRegB(0x64, 0x00, 0x00); // OFF
		}
*/
		setMeasure(fRun);
		delay(500);
	}
	if (fRun == 1)
	{
		fReady = 0;
		while (fReady == 0);
		uint32_t t1 = micros();
		tm = t1 - t0;
		t0 = t1;
		// g: [deg/s], a[g]
		for (uint8_t i = 0; i < 2; i++){
			fmx[i] = ((float)mx[i] - mx0[i]) / ma[i];
			fmy[i] = ((float)my[i] - my0[i]) / mb[i];
			fmz[i] = ((float)mz[i] - mz0[i]) / mc[i];
			float norm = sqrt(fmx[i] * fmx[i] + fmy[i] * fmy[i] + fmz[i] * fmz[i]);
			fmx[i] /= norm;
			fmy[i] /= norm;
			fmz[i] /= norm;
			mf[i].update(gx[i], gy[i], gz[i], ax[i], ay[i], az[i], fmx[i], fmy[i], fmz[i]);
			//mf[i].updateIMU(gx[i], gy[i], gz[i], ax[i], ay[i], az[i]);
			roll[i] = mf[i].getRoll();
			pitch[i] = mf[i].getPitch();
			yaw[i] = mf[i].getYaw();
		}

//		printf(">gx0:%f\n>gy0:%f\n>gz0:%f\n", gx[0], gy[0], gz[0]);
//		printf(">ax0:%f\n>ay0:%f\n>az0:%f\n", ax[0], ay[0], az[0]);
//		printf(">gx1:%f\n>gy1:%f\n>gz1:%f\n", gx[1], gy[1], gz[1]);
//		printf(">ax1:%f\n>ay1:%f\n>az1:%f\n", ax[1], ay[1], az[1]);
		printf(">r0:%f\n>y0:%f\n>p0:%f\n", roll[0], yaw[0], pitch[0]);
//		printf(">r1:%f\n>y1:%f\n>p1:%f\n", roll[1], yaw[1], pitch[1]);
//		printf(">r0-r1:%f\n>y0-y1:%f\n>p0-p1:%f\n", roll[0]-roll[1], yaw[0]-yaw[1], pitch[0]-pitch[1]);
		//printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", tm, ax[0], ay[0], az[0], ax[1], ay[1], az[1]);
		//printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", tm, ax[0], ay[0], az[0], ax[1], ay[1], az[1]);
		///printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", tm, gx[0], gy[0], gz[0], gx[1], gy[1], gz[1]);
		//printf("%d,%f,%f,%f\n", tm, gx[0], gy[0], gz[0]);
		//printf("%d,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n",tm, mx[0], my[0], mz[0], mx[1], my[1], mz[1]);
//		printf(">mx0:%f\n>my0:%f\n>mz0:%f\n", mx[0], my[0], mz[0]);
		//printf("%d,%.3f,%.3f,%.3f , %.3f,%.3f,%.3f\n", tm, ax[0], ay[0], az[0], roll[0], pitch[0], yaw[0]);
		//printf("%d,%.3f,%.3f,%.3f , %.3f,%.3f,%.3f\n", tm, roll[0], pitch[0], yaw[0], roll[1], pitch[1], yaw[1]);
		//printf("Orientation: %.3f %.3f %.3f %.3f %.3f %.3f\n", roll[0], pitch[0], yaw[0], roll[1], pitch[1], yaw[1]);
		//printf("Dir: %d %.3f %.3f %.3f %.3f %.3f %.3f\n", tm, roll[0], pitch[0], yaw[0], roll[1], pitch[1], yaw[1]);
//		printf("Dir:,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", tm, roll[0], pitch[0], yaw[0], roll[1], pitch[1], yaw[1]);
//		printf("%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", tm, roll[0], pitch[0], yaw[0], roll[1], pitch[1], yaw[1]);
//		printf("%.3f %.3f %.3f %.3f %.3f %.3f\n", roll[0], pitch[0], yaw[0], roll[1], pitch[1], yaw[1]);
	}

	if (fRun == 3){
		for (uint8_t i = 0; i < 2; i++){
			magX[i].push_back(mx[i]);
			magY[i].push_back(my[i]);
			magZ[i].push_back(mz[i]);
		}
		printf("%d %d %d %d %d %d\n", mx[0], my[0], mz[0], mx[1], my[1], mz[1]);
		delay(10);
	}
}
