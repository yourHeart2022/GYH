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
#include "MAX30100.h"   //心拍センサ用のArduinoライブラリ

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

#define DEBUG_SW                1

#define MOTOR_MODE1             0
#define MOTOR_MODE2             1
#define MOTOR_MODE3             2
#define MOTOR_MODE4             3
#define MOTOR_MODE5             4
#define MOTOR_MODE_NUM          5
#define MOTOR_MODE1_THRESHOLD   (u4)0x000186A0  //  100ms (LSB 1us)
#define MOTOR_MODE2_THRESHOLD   (u4)0x00030D40  //  200ms (LSB 1us)
#define MOTOR_MODE3_THRESHOLD   (u4)0x0007A120  //  500ms (LSB 1us)
#define MOTOR_MODE4_THRESHOLD   (u4)0x000F4240  // 1000ms (LSB 1us)
#define MOTOR_MODE5_THRESHOLD   (u4)0x001E8480  // 2000ms (LSB 1us)

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Const
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
const u4 cu4_MOTOR_MODE_ARRAY[MOTOR_MODE_NUM] = {
                                MOTOR_MODE1_THRESHOLD, 
                                MOTOR_MODE2_THRESHOLD, 
                                MOTOR_MODE3_THRESHOLD, 
                                MOTOR_MODE4_THRESHOLD, 
                                MOTOR_MODE5_THRESHOLD};

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
u4  u4s_thresholdForMotor;
u4  u4s_oldTimeForMotor;
u1  u1s_motorMode;

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Prototypes
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
void IRAM_ATTR timer_callback();
void motorManager();
static void changeMotorMode(u1);

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Initialize
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
void setup() {
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

    // setting motor
    pinMode(MOTOR_PIN,OUTPUT);
    u1s_motorMode           = (u1)MOTOR_MODE3;
    u4s_thresholdForMotor   = cu4_MOTOR_MODE_ARRAY[MOTOR_MODE3];
    u4s_oldTimeForMotor     = (u4)0x00000000;

    // setting common
    u4s_counter             = (u4)0x00000000;
    u4s_counterOld          = (u4)0x00000000;
}

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Main loop
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
void loop()
{
    motorManager();
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
 * モーターを制御
 * 
 */
void motorManager()
{
    u4 u4t_nowTime;
    u4 u4t_interval;
    u1 u1t_port_val;

    // 現在時刻取得
    u4t_nowTime = timerRead(timer0);

    // フリーランのオバーフロー処置
    if (u4t_nowTime > u4s_oldTimeForMotor) {
        u4t_interval = u4t_nowTime - u4s_oldTimeForMotor;
    } else {
        u4t_interval = (u4)0xFFFFFFFF - u4s_oldTimeForMotor + u4t_nowTime + (u4)0x00000001;
    }

    // TODO 暫定で 10 秒毎に MotorMode を切り替える
    //      最終的には心拍数に応じて切り替えれるようにする
    if (u4s_counter - u4s_counterOld > (u4)0x0000000A) {
        u1s_motorMode = ++u1s_motorMode < MOTOR_MODE_NUM ? u1s_motorMode : MOTOR_MODE1;
        changeMotorMode(u1s_motorMode);
        u4s_counterOld = u4s_counter;

        if (DEBUG_SW) {
            Serial.println("★motorMode の切替");
            Serial.print("★★motorMode=");
            Serial.println(u1s_motorMode);
        }
    }

    // 閾値を超えた場合、ポートの出力値を切替
    if (u4t_interval > u4s_thresholdForMotor) {
        u1t_port_val = digitalRead(MOTOR_PIN) == LOW ? HIGH : LOW;
        digitalWrite(MOTOR_PIN, u1t_port_val);        
        u4s_oldTimeForMotor = u4t_nowTime;

        if (DEBUG_SW) {
            Serial.println("ポート出力値切替");   
        }
    }
}

// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
// Private method
// ■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
static void changeMotorMode(u1 u1t_motorMode)
{
    if (u1t_motorMode >= MOTOR_MODE_NUM) {
        u1t_motorMode = (u1)MOTOR_MODE_NUM - (u1)0x01;
    }
    u4s_thresholdForMotor = cu4_MOTOR_MODE_ARRAY[u1t_motorMode];
}
