
// // THIS FILE CAN BE USED AS A TEMPLATE TO CREATE A NEW Ex_file.cpp!!!
// #include "esp32-hal.h"
// #include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.
// #include <Arduino.h>
// #include <WiFi.h>
// #include <time.h>

// #define PIN_LED 32
// #define PIN_BUTTON 35

// #define RETRY_WIFI_EVERY_SECS 20

// uint32_t _LastCheck4Wifi = 0;

// /* Variable holding number of times ESP32 restarted since first boot.
//  * It is placed into RTC memory using RTC_DATA_ATTR and
//  * maintains its value when ESP32 wakes from deep sleep.
//  */
// RTC_DATA_ATTR static int boot_count = 0;

// bool Connect2WiFi()
// {
//     if (WiFi.isConnected()) {
//         return false; // false because was already connected
//     }
//     auto temps = millis() / 1000;

//     if ((temps < RETRY_WIFI_EVERY_SECS * 2) || (temps - _LastCheck4Wifi) >= RETRY_WIFI_EVERY_SECS * 5) {
//         _LastCheck4Wifi = temps;
//         log_d("[%d] Trying WiFi connection to [%s]", millis(), WIFI_SSID);
//         auto err = WiFi.begin(WIFI_SSID, WIFI_PASS); // FROM mykeys.h
//         err = (wl_status_t)WiFi.waitForConnectResult();
//         if (err != wl_status_t::WL_CONNECTED) {
//             log_d("WiFi connection FAILED! Error=[%d]. Will retry later", err);
//             return false;
//         } else {
//             log_d("WiFi CONNECTED!");
//             return true;
//         }
//     }
//     return false; // Too soon to retry, wait.
// }

// void setup()
// {
//     ++boot_count;
//     Serial.begin(115200);
//     // wait for serial monitor to open
//     while (!Serial)
//         ;

//     log_i("Number Of Boots=%d", boot_count);
//     // pinMode(PIN_LED, OUTPUT);
//     // pinMode(PIN_BUTTON, INPUT);
// }

// void printLocalTime()
// {
//     struct tm timeinfo;
//     if (!getLocalTime(&timeinfo)) {
//         log_e("Failed to obtain time");
//         return;
//     }
//     Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
//     // char timeWeekDay[10];
//     // strftime(timeWeekDay, 10, "%A", &timeinfo);
//     // Serial.println(timeWeekDay);
//     // Serial.println();
// }

// void loop()
// {
//     time_t now;
//     struct tm timeinfo;
//     time(&now);
//     localtime_r(&now, &timeinfo);
//     // Is time set? If not, tm_year will be (1970 - 1900).
//     if (timeinfo.tm_year < (2020 - 1900)) {
//         log_i("Time is not set yet. Connecting to WiFi and getting time over NTP.");
//         if (Connect2WiFi()) {
//             configTime(3600, 3600, "pool.ntp.org");
//         }
//         //        obtain_time();
//         // update 'now' variable with current time
//         time(&now);
//     } else {
//         setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
//         tzset();
//     }
//     int iPrints = 10;
//     while(--iPrints>=0) {
//         printLocalTime();
//         sleep(1);
//     }

//     const int deep_sleep_sec = 10;
//     log_i("Entering deep sleep for %d seconds", deep_sleep_sec);
//     esp_deep_sleep(1000000LL * deep_sleep_sec);
// }
