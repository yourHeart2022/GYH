/*
 * 
 * 
 * @author
 * @version
 * @since
 */

// Motor Device     : FM64G         https://akizukidenshi.com/catalog/g/gP-15317/
// HeartRate Device : MAX30100      https://akizukidenshi.com/catalog/g/gM-17212/

#include <Wire.h> 
#include <SPI.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "MAX30100_PulseOximeter.h"
#include "MAX30100.h"   //心拍センサ用のArduinoライブラリ
#include "secrets.h"

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// StdTypes ※後で別ファイルに定義する
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
typedef unsigned char   u1;
typedef unsigned short  u2;
typedef unsigned long   u4;
typedef signed char     s1;
typedef signed short    s2;
typedef signed long     s4;
typedef float           pl;

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Preprocessor
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// 端子の設定
// Motor: Pin 17
#define MOTOR_PIN       17

#define HEARTRATE_SW    1

// webを参考にマクロを定義 ---------------------------------------------------------------------------------
// Sampling is tightly related to the dynamic range of the ADC. refer to the datasheet for further info
#define HEARTRATE_SAMPLING_RATE         MAX30100_SAMPRATE_100HZ

// The LEDs currents must be set to a level that avoids clipping and maximises the dynamic range
#define HEARTRATE_IR_LED_CURRENT        MAX30100_LED_CURR_50MA
#define HEARTRATE_RED_LED_CURRENT       MAX30100_LED_CURR_27_1MA

// The pulse width of the LEDs driving determines the resolution of the ADC (which is a Sigma-Delta).
// set HIGHRES_MODE to true only when setting PULSE_WIDTH to MAX30100_SPC_PW_1600US_16BITS
#define HEARTRATE_PULSE_WIDTH           MAX30100_SPC_PW_1600US_16BITS
#define HEARTRATE_HIGHRES_MODE          true
// ------------------------------------------------------------------------------------------------------

#define HEART_RATE_BEAT_MODE1           0
#define HEART_RATE_BEAT_MODE2           1
#define HEART_RATE_BEAT_MODE3           2
#define HEART_RATE_BEAT_MODE_NUM        3

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Const
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
const u4 cu4_HEART_RATE_BEAT_ARRAY[3][2] = {
                                {0x00000096/* 150ms */, (u4)HIGH},
                                {0x00000032/* 50ms  */, (u4)LOW},
                                {0x00000046/* 70ms  */, (u4)HIGH}};    

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Global variable
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
//timer
hw_timer_t *timer0 = NULL;  
hw_timer_t *timer1 = NULL;

//heart rate (MAX30100)
MAX30100 heartRateSensor;

// PulseOximeter is the higher level interface to the sensor
// it offers:
//  * beat detection reporting
//  * heart rate calculation
//  * SpO2 (oxidation level) calculation
PulseOximeter pox;

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Static variable
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
u4  u4s_counter;
u4  u4s_counterOld;         // 暫定
u4  u4s_heartRateInterval;
u4  u4s_oldTime;

u1  u1s_isInitHeartRateSensor;

u1  u1s_heartRateBeatMode;
u1  u1s_isBeat;

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Prototypes
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
void setup_heartRateSensor();
void setup_wifi();
void IRAM_ATTR timer_callback();
void hearRateSendManager();
void hearRateReceiveManager();
void hearRateManager();
static void changeHeartRateInterval(u1);

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Initialize
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
void setup() 
{
    //■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
    //■ デバイスの設定 
    //■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
    // setting debug console
    Serial.begin(115200);

    // setting timer0
    timer0 = timerBegin(0, 80, true);       // LSB 1us
    timerStart(timer0);

    // setting timer1
    timer1 = timerBegin(1, 80, true);       // LSB 1us
    timerAttachInterrupt(timer1, &timer_callback, true);
    timerAlarmWrite(timer1, 1000000, true); // 1000000us = 1s
    timerAlarmEnable(timer1);    

    // setting GPIO
    pinMode(MOTOR_PIN,OUTPUT);

    // setting heart rate sensor
    setup_heartRateSensor();

    // setting wifi
//    setup_wifi();

    //■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
    //■ aws の設定 
    //■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
//    setup_awsiot();

    //■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
    //■ 変数の初期設定
    //■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
    u4s_heartRateInterval       = (u4)0x0007A120;           // 500ms (LSB 1us) ※ひとまず適当な値を設定
    u1s_heartRateBeatMode       = (u1)HEART_RATE_BEAT_MODE1;
    u1s_isBeat                  = true;
    u4s_oldTime                 = (u4)0x00000000;

    // setting common
    u4s_counter                 = (u4)0x00000000;
    u4s_counterOld              = (u4)0x00000000;

    // start heart rate
    digitalWrite(MOTOR_PIN,LOW);
}

/**
 * heart rate sensor の初期設定
 */
void setup_heartRateSensor()
{
    //
    if (HEARTRATE_SW == 0) {
        
        if (heartRateSensor.begin()) {
            heartRateSensor.setMode(MAX30100_MODE_SPO2_HR);
            heartRateSensor.setLedsCurrent(HEARTRATE_IR_LED_CURRENT, HEARTRATE_RED_LED_CURRENT);
            heartRateSensor.setLedsPulseWidth(HEARTRATE_PULSE_WIDTH);
            heartRateSensor.setSamplingRate(HEARTRATE_SAMPLING_RATE);
            heartRateSensor.setHighresModeEnabled(HEARTRATE_HIGHRES_MODE);
            
            u1s_isInitHeartRateSensor = true;
            Serial.println("Heart rate sensor initialization was successful.");
        } else {
            
            u1s_isInitHeartRateSensor = false;
            Serial.println("Heart rate sensor initialization was failed.");
        }

    //      
    } else {
        if (pox.begin()) {
            u1s_isInitHeartRateSensor = true;
            Serial.println("Heart rate sensor initialization was successful.");
        } else {
            u1s_isInitHeartRateSensor = false;
            Serial.println("Heart rate sensor initialization was failed.");
        }
    }
}


/**
 * wifi の初期設定
 */
void setup_wifi()
{
    Serial.print("Connecting to ");
    Serial.println(SSID);

    // ESP32でWiFiに繋がらなくなるときのための対策
    WiFi.disconnect(true);
    delay(1000);

    WiFi.begin(SSID, PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Main loop
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
void loop()
{
    hearRateSendManager();
    hearRateReceiveManager();
    hearRateManager();
}

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Interrupt method
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
/**
 * 1ms 割り込み
 */
void IRAM_ATTR timer_callback()
{
    // 割り込み処理内で Serial.print 実行すると WDT リセット (esp32 interrupt wdt timeout on cpu1)
    // が発生するので、以下の処理を有効化しないこと
    // Serial.println("1ms 割り込み start");

    u4s_counter++;
    
    // Serial.println("1ms 割り込み end");
}


// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Public method
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■

/**
 * 心拍数の送信を制御
 * 
 */
void hearRateSendManager()
{
    if (u1s_isInitHeartRateSensor) {

        if (HEARTRATE_SW == 0) {
            
            // Make sure to call update as fast as possible
            // (If you don't run at the fast, you will always get "0" output.)
            heartRateSensor.update();
            if (u4s_counter - u4s_counterOld > (u4)0x00000003) {
                u2 u2t_ir, u2t_red;
                heartRateSensor.getRawValues(&u2t_ir, &u2t_red);
                
                Serial.print("心拍数: ");
                Serial.print(u2t_ir);
                Serial.print(", ");
                Serial.println(u2t_red);

                u4s_counterOld = u4s_counter;
            }
        } else {
            
            // Make sure to call update as fast as possible
            // (If you don't run at the fast, you will always get "0" output.)
            pox.update();
            if (u4s_counter - u4s_counterOld > (u4)0x00000003) {
                Serial.print("Heart rate:");
                Serial.print(pox.getHeartRate());
                Serial.print("bpm / SpO2:");
                Serial.print(pox.getSpO2());
                Serial.println("%");

                u4s_counterOld = u4s_counter;
            }
        }
    }
}

/**
 * 心拍数の受信を制御
 */
void hearRateReceiveManager()
{
    if (u1s_isBeat) {
        
        u1 u1t_receiveDataSize = Serial.available();

        // シリアル通信で受信した場合
        if (u1t_receiveDataSize > 0) {

            // 先頭から 3byte 分取得し 4byte 目以降は破棄する
            u4 u4t_incomingByte[3] = {0x00000000, 0x00000000, 0x00000000};
            for (int i = 0; i <= u1t_receiveDataSize; i++) {
                if (i < 3) {
                    u4t_incomingByte[i] = Serial.read();
                } else {
                    Serial.read();  // dummy read
                }
            }
            
            u1 u1t_bpm = (u1)0x00;
            for (int i = 0; i < 3; i++) {

                // 文字列を数値に変換
                u1 u1t_num = (u1)(u4t_incomingByte[i] - '0');

                // 0～9 の場合
                if (u1t_num >= 0x00 && u1t_num < 0x0A) {
                    u1t_bpm = u1t_bpm * (u1)0x0A + u1t_num;
                }
            }
            
            Serial.print("received: ");
            Serial.print(u1t_bpm);
            Serial.println("(bpm)");

            // 0 割り対策
            if (u1t_bpm != 0x00) {
                // bpm から 1回あたりの心拍の時間(ms) を取得
                u4s_heartRateInterval = 600000 / u1t_bpm;
            }

            Serial.print("interval: ");
            Serial.print(u4s_heartRateInterval);
            Serial.println("(ms)");
        }
    }
}

/**
 * 心拍数(モーター)を制御
 * 
 */
void hearRateManager()
{
    u4 u4t_nowTime;
    u4 u4t_interval;
    u1 u1t_port_val;

    // 現在時刻取得
    u4t_nowTime = timerRead(timer0);

    // フリーランのオバーフロー処置
    if (u4t_nowTime > u4s_oldTime) {
        u4t_interval = u4t_nowTime - u4s_oldTime;
    } else {
        u4t_interval = (u4)0xFFFFFFFF - u4s_oldTime + u4t_nowTime + (u4)0x00000001;
    }

    // Beat Mode
    if (u1s_isBeat) {
        if (u4t_interval > cu4_HEART_RATE_BEAT_ARRAY[u1s_heartRateBeatMode][0]) {

            u1s_heartRateBeatMode++;           
            if (u1s_heartRateBeatMode < HEART_RATE_BEAT_MODE_NUM) {
                digitalWrite(MOTOR_PIN, cu4_HEART_RATE_BEAT_ARRAY[u1s_heartRateBeatMode][1]);
            } else {
                u1s_heartRateBeatMode = HEART_RATE_BEAT_MODE1;
                u1s_isBeat = false;
                
//                Serial.println("Beat Mode -> Interval Mode");
            }
            u4s_oldTime = u4t_nowTime;
        }

    // Interval Mode
    } else {
        if (u4t_interval > u4s_heartRateInterval) {
            // u1t_port_val = digitalRead(MOTOR_PIN) == LOW ? HIGH : LOW;
            digitalWrite(MOTOR_PIN, LOW);
            u1s_isBeat = true;
            u4s_oldTime = u4t_nowTime;
            
//            Serial.println("Interval Mode -> Beat Mode");
        }
    }
}

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Private method
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Service Layer ※ここで Application と MCAL の依存関係を排除
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
