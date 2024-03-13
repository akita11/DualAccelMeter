#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "mbedtls/md.h"
//#include "SD.h"
#include "SdFat.h"

#include "time.h"
#include <WiFi.h>
// Different versions of the framework have different SNTP header file names and availability.
#if __has_include (<esp_sntp.h>)
 #include <esp_sntp.h>
 #define SNTP_ENABLED 1
#elif __has_include (<sntp.h>)
 #include <sntp.h>
 #define SNTP_ENABLED 1
#endif
#ifndef SNTP_ENABLED
#define SNTP_ENABLED 0
#endif

#define PIN_LED 19 // StickCPlus2's red LED

extern void NTPadjust();

//File fp; // for SD.h
//SdFile32 fp; // for SdFat.h
extern File32 fp; // for SdFat.h
extern SdFat32 sd; // for SdFat.h

