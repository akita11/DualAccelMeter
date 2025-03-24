#include <Arduino.h>
#include <M5Unified.h>
#include <Ticker.h>
#include <FastLED.h>
#include "bmi270_config.h"
#include "esp_timer.h"

// Error codes
#define BMI270_OK                    0
#define BMI270_ERR_WRITE_FAILED     1
#define BMI270_ERR_READ_FAILED      2
#define BMI270_ERR_TIMEOUT          3
#define BMI270_ERR_WRONG_CHIP_ID    4
#define BMI270_ERR_AUX_WRITE_FAILED 5

// Physical constants
#define GRAVITY                 9.80665f // Standard gravity in m/s^2

// Sensor data validity checks
#define ACC_MAX_MS2             (2.0f * GRAVITY)  // Maximum acceleration in m/s^2
#define GYRO_MAX_DPS            2000.0f          // Maximum angular rate in degrees/s
#define MAG_MAX_UT              1000.0f          // Maximum magnetic field in μT

// Retry settings
#define MAX_RETRY_COUNT         3
#define RETRY_DELAY_MS          1

// BMI270 Register Addresses
#define BMI270_REG_CHIP_ID          0x00
#define BMI270_REG_STATUS          0x03
#define BMI270_REG_AUX_DATA        0x04
#define BMI270_REG_INTERNAL_STATUS 0x21
#define BMI270_REG_GYRO_CONFIG     0x42
#define BMI270_REG_ACC_CONFIG      0x40
#define BMI270_REG_ACC_RANGE       0x41
#define BMI270_REG_AUX_DEV_ID      0x4B
#define BMI270_REG_AUX_RD          0x4C
#define BMI270_REG_AUX_WR_ADDR     0x4D
#define BMI270_REG_AUX_WR_DATA     0x4F
#define BMI270_REG_CMD             0x7E
#define BMI270_REG_PWR_CONF        0x7C
#define BMI270_REG_PWR_CTRL        0x7D
#define BMI270_REG_INIT_CTRL       0x59
#define BMI270_REG_INIT_ADDR_0     0x5B
#define BMI270_REG_INIT_ADDR_1     0x5C
#define BMI270_REG_INIT_DATA       0x5E

// BMI270 Configuration Values
#define BMI270_PWR_CONF_ADV_OFF    0x00
#define BMI270_PWR_CONF_FIFO_WU    0x02
#define BMI270_PWR_CTRL_ALL_ON     0x07
#define BMI270_INIT_START          0x00
#define BMI270_INIT_COMPLETE       0x01
#define BMI270_ACC_ODR_400HZ       0xAA
#define BMI270_ACC_RANGE_2G        0x00
#define BMI270_GYRO_ODR_400HZ      0xEA

// AUX (BMM150) related
#define BMM150_AUX_RD_BURST_LEN1   0x80

// MadgwickAHRS: https://qiita.com/Ninagawa123/items/9520bad3c78ee40194fc
// 9axis: https://misosoup4258.hatenablog.com/entry/2021/12/29/185801#Madgwick%E3%83%95%E3%82%A3%E3%83%AB%E3%82%BF%E3%83%BC

#include "MadgwickAHRS.h"
Madgwick mf[2];

float roll[2], pitch[2], yaw[2];

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

// High precision timer handle
esp_timer_handle_t timer_handle;

// Function prototypes
bool is_acc_valid(float ax, float ay, float az);
bool is_gyro_valid(float gx, float gy, float gz);
bool is_mag_valid(float mx, float my, float mz);

// Sensor data validation functions
bool is_acc_valid(float ax, float ay, float az) {
    return (fabs(ax) <= ACC_MAX_MS2 && 
            fabs(ay) <= ACC_MAX_MS2 && 
            fabs(az) <= ACC_MAX_MS2);
}

bool is_gyro_valid(float gx, float gy, float gz) {
    return (fabs(gx) <= GYRO_MAX_DPS && 
            fabs(gy) <= GYRO_MAX_DPS && 
            fabs(gz) <= GYRO_MAX_DPS);
}

bool is_mag_valid(float mx, float my, float mz) {
    return (fabs(mx) <= MAG_MAX_UT && 
            fabs(my) <= MAG_MAX_UT && 
            fabs(mz) <= MAG_MAX_UT);
}

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
    // AUXにデータを書き込み
    if (writeRegB(i2c_addr, BMI270_REG_AUX_WR_DATA, data) != BMI270_OK) {
        return false;
    }
    
    // AUXにアドレスを書き込み
    if (writeRegB(i2c_addr, BMI270_REG_AUX_WR_ADDR, reg) != BMI270_OK) {
        return false;
    }

    // 操作完了を待機（タイムアウト付き）
    int retry = MAX_RETRY_COUNT;
    int status;
    do {
        status = readRegB(i2c_addr, BMI270_REG_STATUS);
        if (status < 0) { // 読み取りエラー
            return false;
        }
        if (--retry <= 0) { // タイムアウト
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
    } while (status & 0b100);

    return (retry > 0);
}

int auxReadRegB(uint8_t i2c_addr, uint8_t reg)
{
    // Enable read with burst length 1
    if (writeRegB(i2c_addr, BMI270_REG_AUX_RD, BMM150_AUX_RD_BURST_LEN1) != BMI270_OK) {
        return BMI270_ERR_WRITE_FAILED;
    }

    // Set address to read from AUX
    if (writeRegB(i2c_addr, BMI270_REG_AUX_WR_ADDR, reg) != BMI270_OK) {
        return BMI270_ERR_WRITE_FAILED;
    }

    // Wait for operation to complete with timeout
    int retry = MAX_RETRY_COUNT;
    while ((readRegB(i2c_addr, BMI270_REG_STATUS) & 0b100) && --retry) {
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
    }

    if (retry <= 0) {
        return BMI270_ERR_TIMEOUT;
    }

    // Read the data
    int result = readRegB(i2c_addr, BMI270_REG_AUX_DATA);
    if (result < 0) {
        return BMI270_ERR_READ_FAILED;
    }

    return result;
}

// for BMI270&BMM150
int IMUinit(uint8_t i2c_addr)
{
    uint8_t index = 0;
    uint8_t addr_array[2] = {(uint8_t)((index >> 1) & 0x0F), (uint8_t)(index >> 5)};

    // IMU init sequence
    //  printf("CHIP_ID(%02x) : %02x\n", readRegB(I2C_ADDR_IMU1, 0x00)); // CHIP_ID(0x00) = 0x24
    if (!writeRegB(i2c_addr, BMI270_REG_PWR_CONF, BMI270_PWR_CONF_ADV_OFF)) { // disable adv.power save
        return BMI270_ERR_WRITE_FAILED;
    }
    delayMicroseconds(450);

    if (!writeRegB(i2c_addr, BMI270_REG_INIT_CTRL, BMI270_INIT_START)) { // prepare init
        return BMI270_ERR_WRITE_FAILED;
    }
    if (!writeReg(i2c_addr, BMI270_REG_INIT_ADDR_0, addr_array, 2)) {
        return BMI270_ERR_WRITE_FAILED;
    }
    if (!writeReg(i2c_addr, BMI270_REG_INIT_DATA, (uint8_t *)bmi270_config_file, sizeof(bmi270_config_file))) {
        return BMI270_ERR_WRITE_FAILED;
    }
    if (!writeRegB(i2c_addr, BMI270_REG_INIT_CTRL, BMI270_INIT_COMPLETE)) {
        return BMI270_ERR_WRITE_FAILED;
    }

    // 初期化完了待ち
    uint8_t status;
    uint8_t retry = 0;
    do {
        status = readRegB(i2c_addr, BMI270_REG_INTERNAL_STATUS);
        if (status == 0xFF) { // 読み取りエラー
            return BMI270_ERR_READ_FAILED;
        }
        if (++retry > 100) { // タイムアウト
            return BMI270_ERR_TIMEOUT;
        }
        delay(1);
    } while (status != BMI270_INIT_COMPLETE);

    // センサーの設定
    if (!writeRegB(i2c_addr, BMI270_REG_PWR_CTRL, BMI270_PWR_CTRL_ALL_ON) || // enable acc/gyro/aux
        !writeRegB(i2c_addr, BMI270_REG_ACC_CONFIG, BMI270_ACC_ODR_400HZ) || // Acc ODR=400Hz
        !writeRegB(i2c_addr, BMI270_REG_PWR_CONF, BMI270_PWR_CONF_FIFO_WU) || // disable adv. power save
        !writeRegB(i2c_addr, BMI270_REG_ACC_RANGE, BMI270_ACC_RANGE_2G) || // Acc range : +-2g
        !writeRegB(i2c_addr, BMI270_REG_GYRO_CONFIG, BMI270_GYRO_ODR_400HZ) || // Gyro config
        !writeRegB(i2c_addr, 0x43, 0x00)) { // Gyro range
        return BMI270_ERR_WRITE_FAILED;
    }

    // AUX (BMM150) の初期化
    if (!writeRegB(i2c_addr, 0x6b, 0x20) || // AUX I2C enable
        !writeRegB(i2c_addr, 0x7c, 0x00) || // Power save disabled
        !writeRegB(i2c_addr, 0x7d, 0x0e) || // AUX sensor disable
        !writeRegB(i2c_addr, 0x4c, 0x80) || // enable manual AUX
        !writeRegB(i2c_addr, 0x4b, 0x10 << 1)) { // BMM150's I2C addr
        return BMI270_ERR_WRITE_FAILED;
    }

    if (!auxWriteRegB(i2c_addr, 0x4b, 0x83)) { // software reset + power on
        return BMI270_ERR_AUX_WRITE_FAILED;
    }

    auto who_am_i = auxReadRegB(i2c_addr, 0x40); // 0x40 = WhoAmI
    if (who_am_i != 0x32) {
        return BMI270_ERR_WRONG_CHIP_ID;
    }

    if (!auxWriteRegB(i2c_addr, 0x4C, 0x38) || // normal mode / ODR 30Hz
        !writeRegB(i2c_addr, 0x4c, 0x4f) || // FCU_WRITE_EN + Manual BurstLength 8
        !writeRegB(i2c_addr, 0x4d, 0x42) || // 0x42 = BMM150 I2C Data X LSB reg
        !writeRegB(i2c_addr, 0x7d, 0x0f)) { // temp en | ACC en | GYR en | AUX en
        return BMI270_ERR_WRITE_FAILED;
    }

    return BMI270_OK;
}

uint32_t t0, tm;

void IRAM_ATTR onTimer(void* arg)
{
    if (!fRun) return;
    fReady = 0;

    // Read and validate IMU#1 data
    bool imu1_valid = true;
    if (readReg(I2C_ADDR_IMU0, BMI270_REG_AUX_DATA, buf0, 20) != BMI270_OK) {
        imu1_valid = false;
    } else {
        // Process IMU#1 data
        mx[0] = (float)(conv_value(buf0[1], buf0[0]) >> 3); // 13bit
        my[0] = (float)(conv_value(buf0[3], buf0[2]) >> 3); // 13bit
        mz[0] = (float)(conv_value(buf0[5], buf0[4]) >> 1); // 15bit
        
        ax[0] = (float)conv_value(buf0[9], buf0[8]) * GRAVITY / 16384.0f;  // [m/s^2]
        ay[0] = (float)conv_value(buf0[11], buf0[10]) * GRAVITY / 16384.0f;
        az[0] = (float)conv_value(buf0[13], buf0[12]) * GRAVITY / 16384.0f;
        
        gx[0] = (float)conv_value(buf0[15], buf0[14]) / (32768.0f * 2000.0f); // [dps]
        gy[0] = (float)conv_value(buf0[17], buf0[16]) / (32768.0f * 2000.0f);
        gz[0] = (float)conv_value(buf0[19], buf0[18]) / (32768.0f * 2000.0f);

        // Validate sensor data
        if (!is_acc_valid(ax[0], ay[0], az[0]) || 
            !is_gyro_valid(gx[0], gy[0], gz[0]) || 
            !is_mag_valid(mx[0], my[0], mz[0])) {
            imu1_valid = false;
        }
    }

    // Read and validate IMU#2 data
    bool imu2_valid = true;
    if (readReg(I2C_ADDR_IMU1, BMI270_REG_AUX_DATA, buf1, 20) != BMI270_OK) {
        imu2_valid = false;
    } else {
        // Process IMU#2 data
        mx[1] = (float)(conv_value(buf1[1], buf1[0]) >> 3); // 13bit
        my[1] = (float)(conv_value(buf1[3], buf1[2]) >> 3); // 13bit
        mz[1] = (float)(conv_value(buf1[5], buf1[4]) >> 1); // 15bit
        
        ax[1] = (float)conv_value(buf1[9], buf1[8]) * GRAVITY / 16384.0f;  // [m/s^2]
        ay[1] = (float)conv_value(buf1[11], buf1[10]) * GRAVITY / 16384.0f;
        az[1] = (float)conv_value(buf1[13], buf1[12]) * GRAVITY / 16384.0f;
        
        gx[1] = (float)conv_value(buf1[15], buf1[14]) / (32768.0f * 2000.0f); // [dps]
        gy[1] = (float)conv_value(buf1[17], buf1[16]) / (32768.0f * 2000.0f);
        gz[1] = (float)conv_value(buf1[19], buf1[18]) / (32768.0f * 2000.0f);

        // Validate sensor data
        if (!is_acc_valid(ax[1], ay[1], az[1]) || 
            !is_gyro_valid(gx[1], gy[1], gz[1]) || 
            !is_mag_valid(mx[1], my[1], mz[1])) {
            imu2_valid = false;
        }
    }

    // Update Madgwick filter only if data is valid
    if (imu1_valid) {
        mf[0].updateIMU(gx[0], gy[0], gz[0], ax[0], ay[0], az[0]);
        mf[0].computeAngles();
        roll[0] = mf[0].getRoll();
        pitch[0] = mf[0].getPitch();
        yaw[0] = mf[0].getYaw();
    }

    if (imu2_valid) {
        mf[1].updateIMU(gx[1], gy[1], gz[1], ax[1], ay[1], az[1]);
        mf[1].computeAngles();
        roll[1] = mf[1].getRoll();
        pitch[1] = mf[1].getPitch();
        yaw[1] = mf[1].getYaw();
    }

    fReady = 1;
}

void setMeasure(uint8_t f)
{
	if (f == 1)
	{
		leds[0] = CRGB(30, 30, 0);
		FastLED.show();
		ticker.attach_ms((int)(1000 / SAMPLE_FREQ), []() { onTimer(nullptr); });
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
    // M5Stackの設定
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
    result = IMUinit(I2C_ADDR_IMU0);
    if (result != BMI270_OK) {
        M5.Lcd.println("IMU1の初期化に失敗");
        leds[0] = CRGB::Red;
        FastLED.show();
        while(1) { delay(100); } // エラー時は停止
    }
    
    result = IMUinit(I2C_ADDR_IMU1);
    if (result != BMI270_OK) {
        M5.Lcd.println("IMU2の初期化に失敗");
        leds[0] = CRGB::Red;
        FastLED.show();
        while(1) { delay(100); } // エラー時は停止
    }

    // 高精度タイマーの設定（250Hz）
    esp_timer_create_args_t timer_args = {
        .callback = onTimer,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "imu_timer"
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, 1000000 / SAMPLE_FREQ)); // マイクロ秒に変換

    // Madgwickフィルタの初期化
    mf[0].begin(100);
    mf[1].begin(100);

    // 初期化成功を表示
    leds[0] = CRGB(0, 30, 0); // 成功時は緑
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

		// g: [deg/s]<-[rad/s]?, a[m/s^2]
//		mf[0].updateIMU(gx[0], gy[0], gz[0], ax[0], ay[0], az[0]);
		mf[0].update(gx[0], gy[0], gz[0], ax[0], ay[0], az[0], mx[0], my[0], mz[0]);
		roll[0] = mf[0].getRoll();
		pitch[0] = mf[0].getPitch();
		yaw[0] = mf[0].getYaw();

		//		printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", tm, ax[0], ay[0], az[0], ax[1], ay[1], az[1]);
		//		printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", tm, gx[0], gy[0], gz[0], gx[1], gy[1], gz[1]);
		//		printf("%d,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n",tm, mx[0], my[0], mz[0], mx[1], my[1], mz[1]);
		printf("%d,%.3f,%.3f,%.3f\n", tm, roll[0], pitch[0], yaw[0]);
	}
}
