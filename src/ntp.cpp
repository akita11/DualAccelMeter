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

  digitalWrite(G10, 0);
  File fp = SD.open("/wifi.txt", "r");
  if (!fp){
    // fast LED flash if "wifi.txt" doesn't exist
    for (uint8_t i = 0; i < 5; i++){
      digitalWrite(G10, 0); delay(100);
      digitalWrite(G10, 1); delay(100);
    }
    digitalWrite(G10, 1); delay(50);
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
  printf("Connecting to %s", ssid);
  WiFi.begin(ssid, ssid_pwd);
  while (WiFi.status() != WL_CONNECTED) {
    f = 1 - f; digitalWrite(G10, f); printf("."); delay(500);
  }

  printf("connected\n");
  printf("%s\n", WiFi.localIP().toString().c_str());
  configTzTime(NTP_TIMEZONE, "ntp.nict.jp");
#if SNTP_ENABLED
  while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED)
  {
    f = 1 - f;
    digitalWrite(G10, f);
    Serial.print('.');
    delay(500);
  }
#else
  delay(1600);
  struct tm timeInfo;
  while (!getLocalTime(&timeInfo, 1000)) Serial.print('.');
#endif
  digitalWrite(G10, 0);
  time_t t = time(nullptr)+1; // Advance one second.
  while (t > time(nullptr));  /// Synchronization in seconds
  M5.Rtc.setDateTime( gmtime( &t ) );
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  digitalWrite(G10, 1);
  printf("done");
  }
}