#include <Arduino.h>
#include <M5Unified.h>
#include <Ticker.h>
#include <FastLED.h>
#include "bmi270_config.h"
#include <EEPROM.h>

#define SAMPLE_FREQ 250
//#define SAMPLE_FREQ 100
//#define SAMPLE_FREQ 25

bool fUse9Axis = false;

// ToDo: wait for stable of Madgwick filter
//#define WAIT_FOR_STABLE

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
float roll0[2], pitch0[2], yaw0[2];
float roll_[2], pitch_[2], yaw_[2];
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
volatile int axRaw[2], ayRaw[2], azRaw[2];
volatile int gxRaw[2], gyRaw[2], gzRaw[2];
int mx[2], my[2], mz[2];
float gxOffset[2], gyOffset[2], gzOffset[2];

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
	bool imu_valid[2];
	// Read and validate IMU data
	for (uint8_t i = 0; i < 2; i++){
		imu_valid[i] = true;
		if (!readReg(i2c_addr[i], BMI270_REG_AUX_DATA, buf, 20)) imu_valid[i] = false;
		else {
			// Process IMU data
			// BMNM150 : +-1300uT(x/y), +-2500uT(z) (typ)
			axRaw[i] = conv_value(buf[ 9], buf[ 8]);
			ayRaw[i] = conv_value(buf[11], buf[10]);
			azRaw[i] = conv_value(buf[13], buf[12]);
			gxRaw[i] = conv_value(buf[15], buf[14]);
			gyRaw[i] = conv_value(buf[17], buf[16]);
			gzRaw[i] = conv_value(buf[19], buf[18]);
			ax[i] = (float)axRaw[i] / 16384.0f; // [g]
			ay[i] = (float)ayRaw[i] / 16384.0f;
			az[i] = (float)azRaw[i] / 16384.0f;
			gx[i] = (float)gxRaw[i] / 32768.0f * 2000.0f; // [dps]
			gy[i] = (float)gyRaw[i] / 32768.0f * 2000.0f;
			gz[i] = (float)gzRaw[i] / 32768.0f * 2000.0f;
			mx[i] = (conv_value(buf[1], buf[0]) >> 3); // 13bit (+-4096)
			my[i] = (conv_value(buf[3], buf[2]) >> 3); // 13bit (+-4096)
			mz[i] = (conv_value(buf[5], buf[4]) >> 1); // 15bit (+-16384)
		}
/*
			// Validate sensor data
			if (!is_acc_valid(ax[i], ay[i], az[i]) ||
				!is_gyro_valid(gx[i], gy[i], gz[i]) ||
				!is_mag_valid(mx[i], my[i], mz[i])) imu_valid[i] = false;
*/
		gx[i] -= gxOffset[i];
		gy[i] -= gyOffset[i];
		gz[i] -= gzOffset[i];
	}
	fReady = 1;
}

void setMeasure(uint8_t f)
{
//	printf("mode=%d\n", f);
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

	fRun = 0;
/*
	M5.update();
	if (M5.BtnA.isPressed())
	{
		fRun = 2; // mag calibration mode (idle)
	}
	else{
		fRun = 0; // measurement mode (idle)
		leds[0] = CRGB(30, 30, 30); FastLED.show();

		if (fUse9Axis == true){
			EEPROM.begin(EEPROM_SIZE);
			EEPROM.get(EEPROM_ADDR, calib);
			leds[0] = CRGB(0, 0, 30); FastLED.show();
			for (uint8_t i = 0; i < 2; i++){
				mx0[i] = calib[i*6]; my0[i] = calib[i*6+1];  mz0[i] = calib[i*6+2];
				ma[i]  = calib[i*6+3]; mb[i]  = calib[i*6+4]; mc[i]  = calib[i*6+5];
				printf("calib[%d]: %.3f %.3f %.3f / %.3f %.3f %.3f\n", i, mx0[i], my0[i], mz0[i], ma[i], mb[i], mc[i]);
			}
		}
	}
*/
	// IMUの初期化（エラーチェック付き）
	leds[0] = CRGB(30, 0, 0); // 初期化中は赤
	FastLED.show();

	while(IMUinit(I2C_ADDR_IMU0) != BMI270_OK) delay(10);
	while(IMUinit(I2C_ADDR_IMU1) != BMI270_OK) delay(10);

//	M5.update(); while(!M5.BtnA.isPressed()) M5.update();

	leds[0] = CRGB(50, 30, 0); // orange, calibrating before run
	FastLED.show();

	long gxSum[2], gySum[2], gzSum[2];
	#define N_SAMPLE_INIT (SAMPLE_FREQ * 4)
	for (uint8_t i = 0; i < 2; i++){
		gxSum[i] = 0; gySum[i] = 0; gzSum[i] = 0;
		gxOffset[i] = 0; gyOffset[i] = 0; gzOffset[i] = 0;
	}
	ticker.attach_ms((int)(1000 / SAMPLE_FREQ), onTimer);
	for (uint16_t j = 0; j < N_SAMPLE_INIT; j++){
		fReady = 0;
		while(fReady == 0);
		for (uint8_t i = 0; i < 2; i++){
			gxSum[i] += gxRaw[i];
//			printf("# %d %d : %d %d \n", i, j, gxRaw[i], gxSum[i]);
			gySum[i] += gyRaw[i];
			gzSum[i] += gzRaw[i];
		}
	}
	ticker.detach();
	for (uint8_t i = 0; i < 2; i++){
		gxOffset[i] = (float)(gxSum[i] / N_SAMPLE_INIT) / 32768.0f * 2000.0f; // [dps]
		gyOffset[i] = (float)(gySum[i] / N_SAMPLE_INIT) / 32768.0f * 2000.0f; // [dps]
		gzOffset[i] = (float)(gzSum[i] / N_SAMPLE_INIT) / 32768.0f * 2000.0f; // [dps]
//		printf("IMU%d offset: gx=%.3f, gy=%.3f, gz=%.3f\n", i, gxOffset[i], gyOffset[i], gzOffset[i]);
	}
	// Madgwickフィルタの初期化
	mf[0].begin(SAMPLE_FREQ);
	mf[1].begin(SAMPLE_FREQ);
	mf[0].setGain(0.5); // beta (default=0.1)
	mf[1].setGain(0.5); // beta (default=0.1)

	// 初期化成功を表示
	leds[0] = CRGB(0, 30, 0); // 成功時は緑
	FastLED.show();

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
			calib[p*6]   = mx0[p]; calib[p*6+1] = my0[p]; calib[p*6+2] = mz0[p];
			calib[p*6+3] = ma[p];  calib[p*6+4] = mb[p];  calib[p*6+5] = mc[p];
		}
		EEPROM.begin(EEPROM_SIZE);
		EEPROM.put(EEPROM_ADDR, calib);
		EEPROM.commit();
	}
}

#ifdef WAIT_FOR_STABLE
uint16_t cntDataReady = 0;
#define TH_CNT_DATAREADY 100
bool fStabilizationFinished = false;
#endif

void loop()
{
	M5.update();
	if (M5.BtnA.wasPressed())
	{
		switch(fRun){
			case 0 : fRun = 1; 
				if (fUse9Axis == true)
					for (uint8_t i = 0; i < 2; i++)
						printf("calib[%d]: %.3f %.3f %.3f / %.3f %.3f %.3f\n", i, mx0[i], my0[i], mz0[i], ma[i], mb[i], mc[i]);
				break;
			case 1 : fRun = 0; 
//				cntDataReady = 0; fStabilizationFinished = false;
				break;
			case 2 : fRun = 3; for (uint8_t i = 0; i < 2; i++){ magX[i].clear(); magY[i].clear(); magZ[i].clear();} break;
			case 3 : fRun = 2; calc_calib(); break;
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
			if (fUse9Axis == true){
				fmx[i] = ((float)mx[i] - mx0[i]) / ma[i];
				fmy[i] = ((float)my[i] - my0[i]) / mb[i];
				fmz[i] = ((float)mz[i] - mz0[i]) / mc[i];
				float norm = sqrt(fmx[i] * fmx[i] + fmy[i] * fmy[i] + fmz[i] * fmz[i]);
				fmx[i] /= norm;
				fmy[i] /= norm;
				fmz[i] /= norm;
				mf[i].update(gx[i], gy[i], gz[i], ax[i], ay[i], az[i], fmx[i], fmy[i], fmz[i]);
			}
			else{
				mf[i].updateIMU(gx[i], gy[i], gz[i], ax[i], ay[i], az[i]);
			}
			roll[i] = mf[i].getRoll();
			pitch[i] = mf[i].getPitch();
			yaw[i] = mf[i].getYaw();
		}
		printf("%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", tm, roll[0], pitch[0], yaw[0], roll[1], pitch[1], yaw[1]);
//		printf(">gx0:%f\n>gy0:%f\n>gz0:%f\n", gx[0], gy[0], gz[0]); printf(">gx1:%f\n>gy1:%f\n>gz1:%f\n", gx[1], gy[1], gz[1]);
//		printf(">r0:%f\n>y0:%f\n>p0:%f\n", roll[0], yaw[0], pitch[0]); printf(">r1:%f\n>y1:%f\n>p1:%f\n", roll[1], yaw[1], pitch[1]);
//	printf(">ax0:%f\n>ay0:%f\n>az0:%f\n", ax[0], ay[0], az[0]); printf(">ax1:%f\n>ay1:%f\n>az1:%f\n", ax[1], ay[1], az[1]);
	}
/*
	if (cntDataReady < TH_CNT_DATAREADY){
		bool fDataReady = true;
#define TH 3
		for (uint8_t i = 0; i < 2; i++){
			if (abs(roll[i] - roll0[i] > TH)) fDataReady = false;
			if (abs(yaw[i] - yaw0[i] > TH)) fDataReady = false;
			if (abs(pitch[i] - pitch0[i] > TH)) fDataReady = false;
		}
		if (fDataReady == true){
			cntDataReady++;
			if (cntDataReady >= TH_CNT_DATAREADY){
				 fStabilizationFinished = true;
				leds[0] = CRGB(30, 30, 0); FastLED.show();
				for (uint8_t i = 0; i < 2; i++){
					roll_[i] = roll[i];
					pitch_[i] = pitch[i];
					yaw_[i] = yaw[i];
				}
			}
		}
		else{
			cntDataReady = 0;
		}
		for (uint8_t i = 0; i < 2; i++){
			roll0[i] = roll[i];

			pitch0[i] = pitch[i];
			yaw0[i] = yaw[i];
		}
	}
		if (fStabilizationFinished == true){
//		printf(">fmx0:%f\n>fmy0:%f\n>fmz0:%f\n", fmx[0], fmy[0], fmz[0]);
//		printf(">fmx1:%f\n>fmy1:%f\n>fmz1:%f\n", fmx[1], fmy[1], fmz[1]);
//		printf(">gx0:%f\n>gy0:%f\n>gz0:%f\n", gx[0], gy[0], gz[0]); printf(">ax0:%f\n>ay0:%f\n>az0:%f\n", ax[0], ay[0], az[0]);
//		printf(">gx1:%f\n>gy1:%f\n>gz1:%f\n", gx[1], gy[1], gz[1]); printf(">ax1:%f\n>ay1:%f\n>az1:%f\n", ax[1], ay[1], az[1]);
//		printf(">r0:%f\n>y0:%f\n>p0:%f\n", roll[0], yaw[0], pitch[0]);
//		printf(">r1:%f\n>y1:%f\n>p1:%f\n", roll[1], yaw[1], pitch[1]);
//		printf("%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", tm, roll[0], pitch[0], yaw[0], roll[1], pitch[1], yaw[1]);
		}
*/
	if (fRun == 3){
		fReady = 0;
		while (fReady == 0);
		for (uint8_t i = 0; i < 2; i++){
			magX[i].push_back(mx[i]);
			magY[i].push_back(my[i]);
			magZ[i].push_back(mz[i]);
		}
		printf("%d %d %d %d %d %d\n", mx[0], my[0], mz[0], mx[1], my[1], mz[1]);
		delay(100);
	}
}
