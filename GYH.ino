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
#define MOTOR_PIN   17

#define DEBUG_SW    1

#define HEART_RATE_BEAT_MODE1           0
#define HEART_RATE_BEAT_MODE2           1
#define HEART_RATE_BEAT_MODE3           2
#define HEART_RATE_BEAT_MODE_NUM        3

#define HEART_RATE_INTERVAL_MODE1       0
#define HEART_RATE_INTERVAL_MODE2       1
#define HEART_RATE_INTERVAL_MODE3       2               // nomal interval mode
#define HEART_RATE_INTERVAL_MODE4       3
#define HEART_RATE_INTERVAL_MODE5       4
#define HEART_RATE_INTERVAL_MODE_NUM    5
#define HEART_RATE_INTERVAL1            (u4)0x000186A0  //  100ms (LSB 1us)
#define HEART_RATE_INTERVAL2            (u4)0x00030D40  //  200ms (LSB 1us)
#define HEART_RATE_INTERVAL3            (u4)0x0007A120  //  500ms (LSB 1us)
#define HEART_RATE_INTERVAL4            (u4)0x000F4240  // 1000ms (LSB 1us)
#define HEART_RATE_INTERVAL5            (u4)0x001E8480  // 2000ms (LSB 1us)

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Const
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
const u4 cu4_HEART_RATE_BEAT_ARRAY[3][2] = {
                                {0x00000096/* 150ms */, (u4)HIGH},
                                {0x00000032/* 50ms  */, (u4)LOW},
                                {0x00000046/* 70ms  */, (u4)HIGH}};    

const u4 cu4_HEART_RATE_INTERVAL_ARRAY[HEART_RATE_INTERVAL_MODE_NUM] = {
                                HEART_RATE_INTERVAL1, 
                                HEART_RATE_INTERVAL2, 
                                HEART_RATE_INTERVAL3, 
                                HEART_RATE_INTERVAL4, 
                                HEART_RATE_INTERVAL5};

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Global variable
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
//timer
hw_timer_t *timer0 = NULL;  
hw_timer_t *timer1 = NULL;  

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Static variable
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
u4  u4s_counter;
u4  u4s_counterOld;         // 暫定
u4  u4s_heartRateInterval;
u4  u4s_oldTime;

// 後で減らしたい
u1  u1s_heartRateBeatMode;
u1  u1s_heartRateIntervalMode;
u1  u1s_isBeat;

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Prototypes
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
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

    // setting wifi
    setup_wifi();

    //■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
    //■ aws の設定 
    //■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
//    setup_awsiot();

    //■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
    //■ 変数の初期設定
    //■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
    u1s_heartRateIntervalMode   = (u1)HEART_RATE_INTERVAL_MODE3;
    u4s_heartRateInterval       = cu4_HEART_RATE_INTERVAL_ARRAY[HEART_RATE_INTERVAL_MODE3];
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
    
}

/**
 * 心拍数の受信を制御
 */
void hearRateReceiveManager()
{
    // シリアル通信で受信した場合
    if (Serial.available() > 0) {
        
        u4 u4t_incomingByte = Serial.read();
        for (int i = 1; i <= Serial.available(); i++ ) {
            Serial.read();  // dummy read
        }

        // 文字列を数値に変換
        u4 u4t_nervousness = u4t_incomingByte - '0';
        Serial.print("received: ");
        Serial.println(u4t_nervousness);

        // 緊張度(nervousness) から heart rate intarval を産出
        if (u4t_nervousness < HEART_RATE_INTERVAL_MODE_NUM) {
            u1s_heartRateIntervalMode = (u1)u4t_nervousness;
        } else {
            u1s_heartRateIntervalMode = (u1)HEART_RATE_INTERVAL_MODE3;
            Serial.print("heart rate mode is out of range !!!");
        }

        changeHeartRateInterval(u1s_heartRateIntervalMode);
        Serial.print("heartRateIntervalMode=");
        Serial.println(u1s_heartRateIntervalMode);
    }
    
    // 暫定で 10 秒毎に heart rate interval mode を切り替える
//    if (u4s_counter - u4s_counterOld > (u4)0x0000000A) {
//        u1s_heartRateIntervalMode = ++u1s_heartRateIntervalMode < HEART_RATE_INTERVAL_MODE_NUM ? u1s_heartRateIntervalMode : HEART_RATE_INTERVAL_MODE1;
//        changeHeartRateInterval(u1s_heartRateIntervalMode);
//        u4s_counterOld = u4s_counter;

//        Serial.print("heartRateIntervalMode=");
//        Serial.println(u1s_heartRateIntervalMode);
//    }
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
//            Serial.print("★★heartRateBeatMode=");
//            Serial.println(u1s_heartRateBeatMode);

            u1s_heartRateBeatMode++;           
            if (u1s_heartRateBeatMode < HEART_RATE_BEAT_MODE_NUM) {
                digitalWrite(MOTOR_PIN, cu4_HEART_RATE_BEAT_ARRAY[u1s_heartRateBeatMode][1]);
            } else {
                u1s_heartRateBeatMode = HEART_RATE_BEAT_MODE1;
                u1s_isBeat = false;
                
//                Serial.println("★Interval Mode へ遷移");
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
            
//            Serial.println("★Beat Mode へ遷移");
        }
    }
}

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Private method
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
static void changeHeartRateInterval(u1 u1t_heartRateIntervalMode)
{
    u4s_heartRateInterval = cu4_HEART_RATE_INTERVAL_ARRAY[u1t_heartRateIntervalMode];
}

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Service Layer ※ここで Application と MCAL の依存関係を排除
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
