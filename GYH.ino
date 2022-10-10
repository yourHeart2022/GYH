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
#include <M5Stack.h>

#include "efont.h"
#include "efontESP32.h"
#include "efontEnableJa.h"

#include "MAX30100_PulseOximeter.h"
#include "MAX30100.h"   //心拍センサ用のArduinoライブラリ
//#include "secrets.h"

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

// Heart Rate Device の動作モードを切り替えるスイッチ
// 0:device が取得した センサ値(raw value) を取得する
// 1:device で算出した bpm                を取得する
#define HEARTRATE_ACTIVE_MODE_SW    0

// シリアルデータの受信モードを切り替えるスイッチ
// 0:バイナリ値で取得する
// 1:ASCII値で取得する (シリアルモニタからデータを送る場合は "1" を設定する)
#define SERIAL_RECEIVE_MODE_SW   1

// HEARTRATE_ACTIVE_MODE_SW = 0 (センサ値の取得) で動作させるためのマクロを定義 -------------------------------
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
                                {0x000240F0 /* 150ms  */, (u4)HIGH},
                                {0x0000C350 /* 50ms   */, (u4)LOW},
                                {0x00011170 /* 70ms   */, (u4)HIGH}};

const u4 cu4_HEART_RATE_INTERVAL_ARRAY[16] = {
                                0x000F4240, /* 1000ms */  0x000CF850, /* 850ms */   0x000AAE60, /*  700ms */
                                0x0007A120, /*  500ms */  0x00061A80, /*  400ms */  0x000493E0, /*  300ms */
                                0x00030D40, /*  200ms */  0x000186A0, /*  100ms */  0x0000C350, /*   50ms */
                                0xFFFFFFFF, /* ****ms */  0xFFFFFFFF, /* ****ms */  0xFFFFFFFF, /* ****ms */
                                0xFFFFFFFF, /* ****ms */  0xFFFFFFFF, /* ****ms */  0xFFFFFFFF, /* ****ms */
                                0xFFFFFFFF  /* ****ms */};
                                
// TODO 後で日本語に変更する
const String HELP_MESSAGE_ARRAY[100] = {
                                "Good luck with your date today!!!",
                                "You're in good shape!",
                                "No worries!"};

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Global variable
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// timer
hw_timer_t *timer0 = NULL;  
hw_timer_t *timer1 = NULL;

// sprite の インスタンス生成 (LCD チラつき防止)
TFT_eSprite sprite = TFT_eSprite(&M5.Lcd);

// Heart Rate Device で計測したセンサ値を格納する変数
// ※HEARTRATE_ACTIVE_MODE_SW = 0 の時のみ値が設定される
MAX30100        heartRateSensor;

// Heart Rate Device で算出した bpm を格納する変数
// ※HEARTRATE_ACTIVE_MODE_SW = 1 の時のみ値が設定される
PulseOximeter   pulseOximeter;

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Static variable
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// アライメントを考慮し PL → u4/s4 → u2/s2 → u1/s1 の順で定義すること
u4  u4s_counter;                    // カウンタ値(LSB 1ms)
u4  u4s_counterOld;                 // カウンタ値(LSB 1ms) 前回値
u4  u4s_heartRateSendCntOld;        // Hear Rate Send で使うカウンタの前回値(LSB 1ms)
u4  u4s_heartRateSendRollingCnt;    // Hear Rate Send で使う Rolling counter
u4  u4s_heartRateInterval;          // u1s_isBeat = false の時の intarval 値
u4  u4s_heartRateOldTime;           // フリーランカウンタの前回値. beatMode の遷移や intaval の経過判定に使う

u1  u1s_isInitHeartRateSensor;      // Heart Rate Devie の初期化が成功したかどうかを保持する

u1  u1s_isBeat;                     // true:beat mode, false:interval mode
u1  u1s_heartRateBeatMode;          // u1s_isBeat = true の時の内部状態

u1  u1s_messageNumber;              // display に表示するメッセージ番号

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Prototypes
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
void setup_heartRateSensor();
void setup_wifi();
void IRAM_ATTR timer_callback();
void hearRateSendManager();
void serialReceiveManager();
void hearRateManager();
void displayManager();
static void parseReceveData(u1);
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
    timerAlarmWrite(timer1, 1000, true); // 100us = 1ms,  1000000us = 1s
    timerAlarmEnable(timer1);    

    // setting GPIO
    pinMode(MOTOR_PIN,OUTPUT);

    // setting M5Stack
    setup_M5Stack();

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
    u4s_heartRateOldTime        = (u4)0x00000000;
    u4s_heartRateSendCntOld     = (u4)0x00000000;
    u4s_heartRateSendRollingCnt = (u4)0x00000000;

    u1s_messageNumber           = (u1)0x00;

    // setting common
    u4s_counter                 = (u4)0x00000000;
    u4s_counterOld              = (u4)0x00000000;

    // start heart rate
    digitalWrite(MOTOR_PIN,LOW);
}

/**
 * M5Stack の初期設定
 */
void setup_M5Stack()
{
    // M5Stack オブジェクトの初期化
    M5.begin();
    M5.Lcd.setTextSize(3);

    //Power chipがgpio21, gpio22, I2Cにつながれたデバイスに接続される
    //バッテリー動作の場合はこの関数を読んでください（バッテリーの電圧を調べるらしい）
    M5.Power.begin();

    M5.Lcd.setBrightness(200); //バックライトの明るさを0（消灯）～255（点灯）で制御
    //M5.Lcd.loadFont("filename", SD); // フォント読み込み

    // スプライトの初期化
    sprite.setColorDepth(8);    // 1:2色, 8:256色, 16:65536色
    sprite.createSprite(M5.Lcd.width(), M5.Lcd.height());
}

/**
 * heart rate sensor の初期設定
 */
void setup_heartRateSensor()
{
    // Heart Rate Device から センサ値 を取得する場合
    if (HEARTRATE_ACTIVE_MODE_SW == 0) {      
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

    // Heart Rate Device から bpm を取得する場合 
    } else {
        if (pulseOximeter.begin()) {
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
//    Serial.println(SSID);
//
//    // ESP32でWiFiに繋がらなくなるときのための対策
//    WiFi.disconnect(true);
//    delay(1000);
//
//    WiFi.begin(SSID, PASSWORD);
//
//    while (WiFi.status() != WL_CONNECTED) {
//        delay(500);
//        Serial.print(".");
//    }
//
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
//    Serial.println(WiFi.localIP());
}

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Main loop
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
void loop()
{
    // 心拍数の送信
    hearRateSendManager();

    // シリアルデータの受信
    serialReceiveManager();

    // 心拍数からモーターを制御
    hearRateManager();

    // 100 ms スケジューラ
    if (u4s_counter - u4s_counterOld >= (u4)0x00000064) {

        u4s_counterOld = u4s_counter;

        // ディスプレイの制御
        displayManager();
    }
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
    // Heart Rate Device の初期化が成功した場合
    if (u1s_isInitHeartRateSensor) {

        // Heart Rate Device から センサ値 を取得する場合
        if (HEARTRATE_ACTIVE_MODE_SW == 0) {
            
            // Make sure to call update as fast as possible
            // (If you don't run at the fast, you will always get "0" output.)
//            heartRateSensor.update();
            if (u4s_counter - u4s_heartRateSendCntOld >= (u4)0x0000000a) {
                heartRateSensor.update();
                u2 u2t_ir, u2t_red;
                heartRateSensor.getRawValues(&u2t_ir, &u2t_red);
                
//                Serial.print(u2t_ir);
//                Serial.print(",");
//                Serial.println(u4s_heartRateSendRollingCnt++);

                // ir だけ送信すればで良いためコメントアウト
//                Serial.print(", ");
//                Serial.println(u2t_red);

                u4s_heartRateSendCntOld = u4s_counter;
            }

        // Heart Rate Device から bpm を取得する場合
        } else {
            
            // Make sure to call update as fast as possible
            // (If you don't run at the fast, you will always get "0" output.)
            pulseOximeter.update();
            if (u4s_counter - u4s_heartRateSendCntOld >= (u4)0x00000005) {
                Serial.print("Heart rate:");
                Serial.print(pulseOximeter.getHeartRate());
                Serial.print("bpm / SpO2:");
                Serial.print(pulseOximeter.getSpO2());
                Serial.println("%");

                u4s_heartRateSendCntOld = u4s_counter;
            }
        }
    }
}

/**
 * シリアルデータの受信
 */
void serialReceiveManager()
{
    u1 u1t_receiveDataSize = Serial.available();

    // シリアル通信で受信した場合
    if (u1t_receiveDataSize > 0) {

        u1 u1t_rcvData = (u1)0x00;

        // バイナリで受信する場合
        if (SERIAL_RECEIVE_MODE_SW == 0) {

            // 先頭から 1byte 分取得し 2byte 目以降は破棄する
            u4 u4t_incomingByte = Serial.read();
            for (int i = 1; i <= u1t_receiveDataSize; i++) {
                Serial.read();  // dummy read
            }

            u1t_rcvData = (u1)u4t_incomingByte;

        // ASCII で受信する場合 (シリアルモニタで受信する場合)
        } else {

            // 先頭から 3byte 分取得し 4byte 目以降は破棄する
            u4 u4t_incomingByte[3] = {0x00000000, 0x00000000, 0x00000000};
            for (int i = 0; i <= u1t_receiveDataSize; i++) {
                if (i < 3) {
                    u4t_incomingByte[i] = Serial.read();
                } else {
                    Serial.read();  // dummy read
                }
            }
            
            for (int i = 0; i < 3; i++) {

                // 文字列を数値に変換
                u1 u1t_num = (u1)(u4t_incomingByte[i] - '0');

                // 0～9 の場合
                if (u1t_num >= 0x00 && u1t_num < 0x0A) {
                    u1t_rcvData = u1t_rcvData * (u1)0x0A + u1t_num;
                }
            }
        }

        Serial.print("received: ");
        Serial.println(u1t_rcvData);

        // 受信データを解析する
        parseReceveData(u1t_rcvData);
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
    if (u4t_nowTime > u4s_heartRateOldTime) {
        u4t_interval = u4t_nowTime - u4s_heartRateOldTime;
    } else {
        u4t_interval = (u4)0xFFFFFFFF - u4s_heartRateOldTime + u4t_nowTime + (u4)0x00000001;
    }

    // Beat Mode
    if (u1s_isBeat) {
        if (u4t_interval > cu4_HEART_RATE_BEAT_ARRAY[u1s_heartRateBeatMode][0]) {

            u1s_heartRateBeatMode++;           
            if (u1s_heartRateBeatMode < HEART_RATE_BEAT_MODE_NUM) {
                digitalWrite(MOTOR_PIN, cu4_HEART_RATE_BEAT_ARRAY[u1s_heartRateBeatMode][1]);
            } else {
                // InterVal Mode へ遷移するための準備
                digitalWrite(MOTOR_PIN, LOW);
                u1s_isBeat = false;
                
                Serial.println("Beat Mode -> Interval Mode");
            }
            u4s_heartRateOldTime = u4t_nowTime;
        }

    // Interval Mode
    } else {
        if (u4t_interval > u4s_heartRateInterval) {
            // u1t_port_val = digitalRead(MOTOR_PIN) == LOW ? HIGH : LOW;

            // Beat Mode へ遷移するための準備
            u1s_heartRateBeatMode = HEART_RATE_BEAT_MODE1;
            digitalWrite(MOTOR_PIN, cu4_HEART_RATE_BEAT_ARRAY[u1s_heartRateBeatMode][1]);
            u1s_isBeat = true;
            u4s_heartRateOldTime = u4t_nowTime;
            
            Serial.println("Interval Mode -> Beat Mode");
        }
    }
}

/**
 * ディスプレイの表示を制御する
 */
void displayManager() 
{
    sprite.fillScreen(BLACK);
    sprite.setCursor(10, 10);       //文字表示の左上位置を設定
    sprite.setTextSize(3);
    sprite.setTextColor(WHITE);     //文字色設定(背景は透明)(WHITE, BLACK, RED, GREEN, BLUE, YELLOW...) 
    sprite.print(HELP_MESSAGE_ARRAY[u1s_messageNumber]);

    sprite.pushSprite(0, 0); 
    
    // M5.Lcd.setTextColor(RED, BLACK); //文字色設定と背景色設定(WHITE, BLACK, RED, GREEN, BLUE, YELLOW...)
    // M5.Lcd.setCursor(10, 100); //文字表示の左上位置を設定
    // M5.Lcd.print("Hey Guys! \n\n We have a gift for you!");
}

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Private method
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
/**
 * 受信データを解析する
 */
static void parseReceveData(u1 u1t_rcvData)
{
    // heart rate data の場合
    if (u1t_rcvData <= 0x0F) {

        // 心拍数を mode で取得する場合
        if (true) {
            u4s_heartRateInterval = cu4_HEART_RATE_INTERVAL_ARRAY[u1t_rcvData];
            
        // 心拍数を bpm で取得する場合
        } else {
            
            // 0 割り対策
            if (u1t_rcvData != 0x00) {
                // bpm から 1回あたりの心拍の時間(ms) を取得
                u4s_heartRateInterval = 600000 / u1t_rcvData;
            }
        }

        Serial.print("interval: ");
        Serial.print(u4s_heartRateInterval);
        Serial.println("(ms)");

    // message data の場合
    } else if (u1t_rcvData <= 0xEF) {

        u1s_messageNumber = u1t_rcvData - 0x10;

        Serial.print("message No: ");
        Serial.println(u1s_messageNumber);

    // reserve data の場合
    } else {
        Serial.println("receive data");
    }
}


// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Service Layer ※ここで Application と MCAL の依存関係を排除
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
