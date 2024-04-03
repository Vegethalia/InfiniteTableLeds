// THIS FILE CAN BE USED AS A TEMPLATE TO CREATE A NEW Ex_file.cpp!!!
#include "esp32-hal.h"
#include "mykeys.h" //header containing sensitive information. Not included in repo. You will need to define the missing constants.
#include <Arduino.h>
#include <WiFi.h>

#include <FastLED.h>

#include <PubSubClient.h>

#include <IRrecv.h>
#include <IRremoteESP8266.h>

#include <IRac.h>
#include <IRtext.h>
#include <IRutils.h>

#define DECODE_NEC // Includes Apple and Onkyo
// #include <IRremote.hpp>

// How many leds in your strip?
#define NUM_LEDS 36

// For led chips like WS2812, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
// Clock pin only needed for SPI based chipsets when not using hardware SPI
#define DATA_PIN1 GPIO_NUM_9
#define DATA_PIN2 GPIO_NUM_10
#define DATA_PIN3 GPIO_NUM_11
#define DATA_PIN4 GPIO_NUM_12

#define PIN_LED 32
#define PIN_BUTTON 35
#define PIN_RECEIVE_IR GPIO_NUM_1

#define RETRY_WIFI_EVERY_SECS 20

uint32_t _LastCheck4Wifi = 0;

// Define the array of leds
CRGB _TheLeds1[NUM_LEDS];
CRGB _TheLeds2[NUM_LEDS];
CRGB _TheLeds3[NUM_LEDS];
CRGB _TheLeds4[NUM_LEDS];

OtaUpdater _OTA;
PubSubClient _ThePubSub;

//----------------------
// Advanced declarations
//----------------------
// Tries to reconnect to wifi (if disconnected), or nothing if WiFi is already connected or if last try was before RETRY_WIFI_EVERY_SECS
// Returns true if WiFi is NOW connected and before was not.
// bool Connect2WiFi();
/// @brief  Connects to the MQTT broquer if not connected.
void Connect2MQTT();
/// @brief  PubSubClient callback for received messages
void PubSubCallback(char* pTopic, uint8_t* pData, unsigned int dalaLength);

void vTaskWifiReconnect(void* pvParameters)
{
    bool reconnected = false;

    auto lastCheck = millis();
    // WiFiClientFixed httpWifiClient;
    // HTTPClient theHttpClient;
    WiFiClientSecure* __httpWifiClient = new WiFiClientSecure();
    HTTPClient* __theHttpClient = new HTTPClient();
    std::string theUrl;
    time_t now;

    sleep(20); // wait a bit...

    _OTA.Setup("MicFun", "", 3434);
    WiFi.mode(WIFI_STA);

    while (true) {
        _Connected2Wifi = false;
        reconnected = false;
        if (WiFi.isConnected()) {
            _Connected2Wifi = true;
        } else {
            auto temps = millis() / 1000;

            if ((temps - _LastCheck4Wifi) >= RETRY_WIFI_EVERY_SECS) {
                _LastCheck4Wifi = temps;
                log_i("[%d] Trying WiFi connection to [%s]", millis(), WIFI_SSID);
                auto err = WiFi.begin(WIFI_SSID, WIFI_PASS); // FROM mykeys.h
                err = (wl_status_t)WiFi.waitForConnectResult();
                if (err != wl_status_t::WL_CONNECTED) {
                    log_e("WiFi connection FAILED! Error=[%d]. Will retry later", err);
                } else {
                    log_i("WiFi CONNECTED!");
                    // _TheNTPClient.begin();
                    // _TheNTPClient.setTimeOffset(7200);
                    _Connected2Wifi = true;
                    reconnected = true;
                    // ConfigureNTP();
                }
            }
        }
        if (reconnected) {
            _OTA.Begin();
        }
        if (_Connected2Wifi) {
            _ThePubSub.loop(); // allow the pubsubclient to process incoming messages
            _OTA.Process();
            // _TheNTPClient.update();
            if (!_ThePubSub.connected()) {
                _ThePubSub.setBufferSize(1024);
                Connect2MQTT();
                if (_ThePubSub.connected()) {
                    _ThePubSub.publish(TOPIC_DEBUG, Utils::string_format("MicFun connected with IP=[%s]", WiFi.localIP().toString().c_str()).c_str(), false);
                    _ThePubSub.publish(TOPIC_LASTIP, Utils::string_format("%s", WiFi.localIP().toString().c_str()).c_str(), true);
                }
            }
        }
        delay(2000);
    }
}

void Connect2MQTT()
{
    if (!_ThePubSub.connected()) {
        _ThePubSub.setClient(_TheWifi);
        _ThePubSub.setServer(MQTT_BROKER, MQTT_PORT);
        _ThePubSub.setCallback(PubSubCallback);
        //		String s = WiFi.macAddress());
        if (!_ThePubSub.connect((String("ESP32_Espectrometer") + WiFi.macAddress()[0]).c_str())) {
            log_e("ERROR!! PubSubClient was not able to connect to PiRuter!!");
        } else { // Subscribe to the feeds
            log_i("PubSubClient connected to PiRuter MQTT broker!!");
            _ThePubSub.publish(TOPIC_DEBUG, "PubSubClient connected to PiRuter MQTT broker!!", true);

            if (!_ThePubSub.subscribe(TOPIC_INTENSITY)) {
                log_e("ERROR!! PubSubClient was not able to suibscribe to [%s]", TOPIC_INTENSITY);
            }
            // if (!_ThePubSub.subscribe(TOPIC_FPS)) {
            //     log_e("ERROR!! PubSubClient was not able to suibscribe to [%s]", TOPIC_FPS);
            // }
        }
    }
}

bool Connect2WiFi()
{
    if (WiFi.isConnected()) {
        return false; // false because was already connected
    }
    auto temps = millis() / 1000;

    if ((temps < RETRY_WIFI_EVERY_SECS * 2) || (temps - _LastCheck4Wifi) >= RETRY_WIFI_EVERY_SECS * 5) {
        _LastCheck4Wifi = temps;
        log_d("[%d] Trying WiFi connection to [%s]", millis(), WIFI_SSID);
        auto err = WiFi.begin(WIFI_SSID, WIFI_PASS); // FROM mykeys.h
        err = (wl_status_t)WiFi.waitForConnectResult();
        if (err != wl_status_t::WL_CONNECTED) {
            log_d("WiFi connection FAILED! Error=[%d]. Will retry later", err);
            return false;
        } else {
            log_d("WiFi CONNECTED!");
            return true;
        }
    }
    return false; // Too soon to retry, wait.
}

// As this program is a special purpose capture/decoder, let us use a larger
// than normal buffer so we can handle Air Conditioner remote codes.
const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTimeout = 50;
// Set higher if you get lots of random short UNKNOWN messages when nothing
// should be sending a message.
// Set lower if you are sure your setup is working, but it doesn't see messages
// from your device. (e.g. Other IR remotes work.)
// NOTE: Set this value very high to effectively turn off UNKNOWN detection.
const uint16_t kMinUnknownSize = 25; // 12;
// How much percentage lee way do we give to incoming signals in order to match
// it?
// e.g. +/- 25% (default) to an expected value of 500 would mean matching a
//      value between 375 & 625 inclusive.
// Note: Default is 25(%). Going to a value >= 50(%) will cause some protocols
//       to no longer match correctly. In normal situations you probably do not
//       need to adjust this value. Typically that's when the library detects
//       your remote's message some of the time, but not all of the time.
const uint8_t kTolerancePercentage = 40; // kTolerance is normally 25%

// Use turn on the save buffer feature for more complete capture coverage.
IRrecv irrecv(PIN_RECEIVE_IR, kCaptureBufferSize, kTimeout, true);
decode_results results; // Somewhere to store the results

void setup()
{
    Serial.begin(115200);
    // wait for serial monitor to open
    while (!Serial)
        ;

    FastLED.addLeds<WS2811, DATA_PIN1, RGB>(_TheLeds1, NUM_LEDS); // GRB ordering is typical
    FastLED.addLeds<WS2811, DATA_PIN2, RGB>(_TheLeds2, NUM_LEDS); // GRB ordering is typical
    FastLED.addLeds<WS2811, DATA_PIN3, RGB>(_TheLeds3, NUM_LEDS); // GRB ordering is typical
    FastLED.addLeds<WS2811, DATA_PIN4, RGB>(_TheLeds4, NUM_LEDS); // GRB ordering is typical

    pinMode(PIN_RECEIVE_IR, INPUT);

    // printActiveIRProtocols(&Serial);
    //  Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    // IrReceiver.begin(PIN_RECEIVE_IR, ENABLE_LED_FEEDBACK);

    Connect2WiFi();

    // pinMode(PIN_LED, OUTPUT);
    // pinMode(PIN_BUTTON, INPUT);

    irrecv.setUnknownThreshold(kMinUnknownSize);
    irrecv.setTolerance(kTolerancePercentage); // Override the default tolerance.
    irrecv.enableIRIn(); // Start the receiver

    FastLED.setBrightness(15);
}

void receive_ir_data()
{
    if (irrecv.decode(&results)) {
        // Display a crude timestamp.
        uint32_t now = millis();
        // Check if we got an IR message that was to big for our capture buffer.
        if (results.overflow)
            Serial.printf(D_WARN_BUFFERFULL "\n", kCaptureBufferSize);
        // Display the library version the message was captured with.
        Serial.println(D_STR_LIBRARY "   : v" _IRREMOTEESP8266_VERSION_STR "\n");
        // Display the tolerance percentage if it has been change from the default.
        // if (kTolerancePercentage != kTolerance)
        //     Serial.printf(D_STR_TOLERANCE " : %d%%\n", kTolerancePercentage);
        // Display the basic output of what we found.
        Serial.print(resultToHumanReadableBasic(&results));
        // Display any extra A/C info if we have it.
        String description = IRAcUtils::resultAcToString(&results);
        if (description.length())
            Serial.println(D_STR_MESGDESC ": " + description);
        yield(); // Feed the WDT as the text output can take a while to print.
#if LEGACY_TIMING_INFO
        // Output legacy RAW timing info of the result.
        Serial.println(resultToTimingInfo(&results));
        yield(); // Feed the WDT (again)
#endif // LEGACY_TIMING_INFO
       // Output the results as source code
        Serial.println(resultToSourceCode(&results));
        Serial.println(); // Blank line between entries
        yield(); // Feed the WDT (again)
    }
}

uint8_t _intensity = 0;
int8_t _inc = 1;
bool _Pulse = false;
uint16_t _PulsePos = 0;
uint8_t _PulseHue;
uint8_t _initHue = 0;

void loop()
{
    // receive_ir_data();
    //  Turn the LED on, then pause
    fill_rainbow(_TheLeds1, NUM_LEDS, _initHue++, 1);
    fill_rainbow(_TheLeds2, NUM_LEDS, _initHue + 64, 1);
    fill_rainbow(_TheLeds3, NUM_LEDS, _initHue + 128, 1);
    fill_rainbow(_TheLeds4, NUM_LEDS, _initHue + 192, 1);
    // fill_solid(_TheLeds, NUM_LEDS, CHSV(HUE_RED, 255, _intensity));
    //  _TheLeds[0] = CHSV(HUE_AQUA, 255, 255);
    //  _TheLeds[1] = CHSV(HUE_RED, 255, 255);
    //  _TheLeds[2] = CHSV(HUE_GREEN, 255, 255);
    //  _TheLeds[3] = CHSV(HUE_BLUE, 255, 255);
    //  _TheLeds[4] = CHSV(HUE_PINK, 255, 255);
    //  _TheLeds[5] = CHSV(HUE_ORANGE, 255, 255);
    //  _TheLeds[6] = CHSV(HUE_PURPLE, 255, 255);
    //  _TheLeds[7] = CHSV(HUE_YELLOW, 255, 255);

    // if (!_Pulse && random8(255) == 44) {
    //     _Pulse = true;
    //     _PulseHue = random8();
    //     _PulsePos = 0;
    // }
    // if (_Pulse) {
    //     if (_PulsePos > (NUM_LEDS - 3)) {
    //         _Pulse = false;
    //     } else {
    //         _TheLeds[_PulsePos] = CHSV(_PulseHue, 255, 150);
    //         _TheLeds[_PulsePos + 1] = CHSV(_PulseHue, 255, 250);
    //         _TheLeds[_PulsePos + 2] = CHSV(_PulseHue, 255, 150);
    //         _PulsePos++;
    //     }
    // }

    FastLED.show();

    _intensity += _inc;
    if (_inc > 0 && _intensity > 225) {
        _inc = -1;
    } else if (_inc < 0 && _intensity < 5) {
        _inc = 1;
    }
    delay(33);
    //  // Now turn the LED off, then pause
    //  _TheLeds[0] = CRGB::Blue;
    //  FastLED.show();
    //  delay(100);
    //  _TheLeds[0] = CRGB::White;
    //  FastLED.show();
    //  delay(100);
}
