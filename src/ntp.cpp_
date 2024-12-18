#include <Arduino.h>
#include <M5Unified.h>
#include "ntp.h"
#define NTP_TIMEZONE  "JST-9"

void NTPadjust()
{
  // using NTP
  // https://knt60345blog.com/m5stack-ntp/

  char ssid[64];
  char ssid_pwd[64];

  digitalWrite(PIN_LED, 1);
//  File fp = SD.open("/wifi.txt", "r"); // for SD.h
  fp = sd.open("/wifi.txt", O_RDONLY); // for SdFat.h
  if (!fp){
/*

    // fast LED flash if "wifi.txt" doesn't exist
    // for StickC
    for (uint8_t i = 0; i < 5; i++){
      digitalWrite(PIN_LED, 0); delay(100);
      digitalWrite(PIN_LED, 1); delay(100);
    }
    digitalWrite(PIN_LED, 0); delay(50);
*/
    M5.Lcd.printf("wifi.txt not found\n");
  }
  else{
    uint8_t p = 0, tp = 0;
    while(fp.available() && tp < 2) {
      char c = (char)fp.read();
      if (c == 0x0d || c == 0x0a){
        if (tp == 0) ssid[p] = '\0';
        else ssid_pwd[p] = '\0';
        tp++; p = 0;
      }
      if (c != 0x0d && c != 0x0a){
        if (tp == 0) ssid[p++] = c;
        else ssid_pwd[p++] = c;
      }
    }
    fp.close();
    uint8_t f = 0;
    WiFi.mode(WIFI_STA);
    M5.Lcd.printf("Connecting to %s", ssid);
    printf("Connecting to %s", ssid);
    WiFi.begin(ssid, ssid_pwd);
    while (WiFi.status() != WL_CONNECTED) {
//      f = 1 - f; digitalWrite(PIN_LED, f); 
      M5.Lcd.printf(".");  
      printf("."); delay(500);
    }

    printf("connected, adjusting..\n");
    printf("%s\n", WiFi.localIP().toString().c_str());
    M5.Lcd.printf("connected, adjusting..\n");
    M5.Lcd.printf("%s\n", WiFi.localIP().toString().c_str());
    configTzTime(NTP_TIMEZONE, "ntp.nict.jp");
#if SNTP_ENABLED
    while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED)
    {
//      f = 1 - f; digitalWrite(PIN_LED, f);
      M5.Lcd.printf(".");
      printf(".");
      delay(500);
    }
#else
    delay(1600);
    struct tm timeInfo;
    while (!getLocalTime(&timeInfo, 1000)) Serial.print('.');
#endif
//    digitalWrite(PIN_LED, 1);
    time_t t = time(nullptr)+1; // Advance one second.
    while (t > time(nullptr));  /// Synchronization in seconds
    M5.Rtc.setDateTime( gmtime( &t ) );
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
//    digitalWrite(PIN_LED, 0);
    printf("done\n");
    M5.Lcd.printf("done\n");
    M5.Lcd.clear();
  }
}
