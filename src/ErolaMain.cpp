// // THIS FILE CAN BE USED AS A TEMPLATE TO CREATE A NEW Ex_file.cpp!!!
// #include "esp32-hal.h"
// #include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.
// #include <Arduino.h>
// // #include <WiFi.h>
// // #include <time.h>

// #include "ErolaHelper.h"

// #include "ErolaFakeMain.h"

// #define PIN_LED 32
// #define PIN_BUTTON 35

// // #define RETRY_WIFI_EVERY_SECS 20

// // uint32_t _LastCheck4Wifi = 0;

// // /* Variable holding number of times ESP32 restarted since first boot.
// //  * It is placed into RTC memory using RTC_DATA_ATTR and
// //  * maintains its value when ESP32 wakes from deep sleep.
// //  */
// // RTC_DATA_ATTR static int boot_count = 0;

// // bool Connect2WiFi()
// // {
// //     if (WiFi.isConnected()) {
// //         return false; // false because was already connected
// //     }
// //     auto temps = millis() / 1000;

// //     if ((temps < RETRY_WIFI_EVERY_SECS * 2) || (temps - _LastCheck4Wifi) >= RETRY_WIFI_EVERY_SECS * 5) {
// //         _LastCheck4Wifi = temps;
// //         log_d("[%d] Trying WiFi connection to [%s]", millis(), WIFI_SSID);
// //         auto err = WiFi.begin(WIFI_SSID, WIFI_PASS); // FROM mykeys.h
// //         err = (wl_status_t)WiFi.waitForConnectResult();
// //         if (err != wl_status_t::WL_CONNECTED) {
// //             log_d("WiFi connection FAILED! Error=[%d]. Will retry later", err);
// //             return false;
// //         } else {
// //             log_d("WiFi CONNECTED!");
// //             return true;
// //         }
// //     }
// //     return false; // Too soon to retry, wait.
// // }

// void setup()
// {
//     Serial.begin(115200);
//     // wait for serial monitor to open
//     while (!Serial)
//         ;

//     // pinMode(PIN_LED, OUTPUT);
//     //  pinMode(PIN_BUTTON, INPUT);
//     pinMode(GPIO_NUM_2, OUTPUT);
//     pinMode(GPIO_NUM_9, OUTPUT);
//     pinMode(GPIO_NUM_8, OUTPUT);
// }

// void loop()
// {
//     Programa();
// }
