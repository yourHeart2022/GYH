"""gui_GYH

    接続された2台の心臓デバイスから光センサ値を読みとり、
    心拍数を推定して表示するプログラム


"""
import time
import sys
import threading
import numpy as np
from scipy.signal import find_peaks
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import tkinter as tk
import tkinter.font as font
from tkinter import ttk
from tkinter import messagebox
from statistics import variance
from playsound import playsound
from PIL import Image

import control_GYH as GYH
import topic_generator as tpg

port_left = 'COM6'
port_right = 'COM3'

LEFT_ONLY     = False     # 左側に接続されたGYHデバイスのみを使う
RIGHT_OBSERV = True      # 右側に接続されたGYHデバイスが傍観者モードになる
BPM_CHANGE_RATE = 0.05   # [%] 心拍レベルの変化率。このパーセンテージ以上変化したら、次のレベルとなる
IR_SEND_RATE = 10        # 心拍送信周期。心拍送信周期の入力が必要なパラメータに使用する
ENABLE_MAX_HEART = True # 心拍レベルが最大(10)の時に、特別なメッセージを表示させる


# グローバル変数、基本的にいじらない
message_offset = 0x0e
max_heart_message = 0xfe

# 婚姻届け
filename = "extra/picture/konintodoke.jpg"
if ENABLE_MAX_HEART:
    imgPIL = Image.open(filename)

class grabYourHeart():
    '''# class grabYourHeart

        心臓デバイスのクラス\n
        接続された心臓デバイスから光センサ値を読みとり、心拍数を推定する\n

    Args:
        # 引数\n
        port                 , String    , COMポート番号\n
        baud_rate            , int       , [Hz] ボーレート（デフォルト : 115200）\n
        timeout              , int       , [sec] UART通信のタイムアウト時間（デフォルト : 1）\n
        debug_mode           , bool      , 接続するデバイスをデバッグモードにするかどうか（デフォルト : False）\n

        IR_SEND_RATE         , int       , 心拍送信周期。心拍送信周期の入力が必要なパラメータに使用する

        # 測定用のパラメータ\n
        IR_SIGNAL_THRESH     , int       , [a.u.] この値以下のデータは無視する（反射光が十分な強度でないと指が接触していない）\n
        ERASE_DATA_LENGTH    , int       , [sec] エラーとなるデータ数がこの値以上ある場合は、指が離れたと仮定しデータを破棄する\n
        DATA_LIST_MAX_LENGTH , int       , [sec] 収録する最大のデータ長さ。このデータリストでピークカウントを行う\n
        FINGER_FIND_LENGTH   , int       , [sec] 収録されたデータ数がこの値以上の場合は、指が接触していると仮定し、心拍計算を行う\n
        FINGER_FIND_LENGTH_PRE, int      , [sec] 収録されたデータ数がこの値以上の場合は、指が接触していると仮定し、これまでのデータを破棄する。\
                                           指が接触してからしばらくのデータはノイズを多く含むため、破棄する必要がある。

        # 心拍推定用のパラメータ\n
        DISTANCE             , int       , ピークとピークの距離。これ未満の距離で検出されたピークは無視する\n
        PROMINANCE_LOW       , int       , 地形突出度。この高さ以上突出しているピークを検出する\n
        DATA_LIST_CALC_LENGTH, int       , 収録されたデータがこの値以上の場合、ピークカウントを行う\n
        STABLE_VARIANCE      , int       , 心拍データの分散がこの値未満であれば、心拍値は安定しているとみなす\n
        STABLE_JUDGE_LENGTH  , int       , [sec] 心拍計測されてからこの時間以上経過すると、心拍データの分散値を計算し、安定かどうか判断する\n

        # 他インスタンス変数\n
        ir_data_list         , list      , 測定した光センサのIR生データを格納するリスト\n
        peaks                , list      , 生データから検出したピークのリスト\n
        bpm                  , int       , 推定した心拍値beat_per_minutes\n
        finger_find_flag     , bool      , 指が検出された時に立てるフラグ\n
        finger_find_pre_flag , bool      , 最初のデータはノイズを多く含むため、finger_find_flagを立てる前に、指が検出されてからデータを破棄する
        button_push_counter  , int       , GYHデバイスのボタンが押された回数カウンタ\n
    '''


    def __init__(self, port, baud_rate = 115200, timeout = 1, debug_mode = False):
        self.port = port
        self.myGYHdevice = GYH.hr_device(port, baud_rate, debug_mode = debug_mode)

        # 心拍送信周期
        self.IR_SEND_RATE = IR_SEND_RATE # [Hz]
        
        # measurement_processメソッドのパラメータ
        self.IR_SIGNAL_THRESH       = 30000 
        self.ERASE_DATA_LENGTH      = 3    #[sec] 
        self.DATA_LIST_MAX_LENGTH   = 30   #[sec] 
        self.FINGER_FIND_LENGTH     = 5    #[sec]  
        self.FINGER_FIND_LENGTH_PRE = 3    #[sec]

        # calc_bpm_processメソッドのパラメータ
        self.USE_FFT         = False
        self.DISTANCE        = 33                 
        self.PROMINANCE_LOW  = 800          
        self.DATA_LIST_CALC_LENGTH = 10
        self.STABLE_VARIANCE     = 1.5
        self.STABLE_JUDGE_LENGTH = 5       #[sec]

        # 他インスタンス変数
        self.ir_data_list = []
        self.peaks        = []
        self.bpm          = 0
        self.finger_find_flag    = False
        self.finger_find_pre_flag= False
        self.bpm_stable_flag     = False
        self.bpm_generate_flag   = False
        self.button_push_counter = 0

    def measurement_process(self):
        '''# measurement_process

            NIRデータを取得し続ける関数\n
            取得したデータは、インスタンス変数ir_data_list(リスト)に書き込む

        '''

        self.myGYHdevice.connect()

        self.myGYHdevice.reset_buffer()
        # ゴミデータを捨てる
        for i in range(50):
            self.myGYHdevice.read_data()

        thresh_error_counter = 0
        erase_data_length_num      = self.ERASE_DATA_LENGTH * self.IR_SEND_RATE # 線形補完と無関係なパラメータに使うため、送信周期が必要
        data_list_max_length_num   = self.DATA_LIST_MAX_LENGTH * 100     # 100[Hz] 送信周期が長くても線形補完するから定数で良い
        finger_find_length_num     = self.FINGER_FIND_LENGTH * 100       # 100[Hz] 送信周期が長くても線形補完するから定数で良い
        finger_find_rength_pre_num = self.FINGER_FIND_LENGTH_PRE * 100   # 100[Hz] 送信周期が長くても線形補完するから定数で良い

        counter_prev = None

        while True:
            # データを取得
            try:
                data_temp = self.myGYHdevice.read_data_rev1()

                ir_temp = data_temp[0]
                counter_temp = int(data_temp[1]/10) #data_temp[1]

                # GYHデバイス側でボタンが押されたとき(全データ最大値)
                if ir_temp == 65535 and counter_temp == 6553:#0xFFFF:
                    print('button!')
                    self.button_push_counter += 1
                
                # 反射光が閾値以上の場合は、正しく測定できていると仮定しデータリストに格納する
                elif ir_temp > self.IR_SIGNAL_THRESH:

                    if counter_prev == None: # 最初のデータの時
                        dt = 1
                    else:
                        dt = counter_temp - counter_prev
                                
                    # データ間隔が10msecのとき
                    if dt == 1:
                        self.ir_data_list.append(ir_temp)
                        thresh_error_counter = 0

                    # データに抜けがあるとき
                    elif dt > 2:
                        # print('comp')
                        # 線形補完したデータを追加
                        for i in range(dt):
                            ir_data_comp = ((ir_prev - ir_temp) / dt) * i + ir_prev
                            self.ir_data_list.append(ir_data_comp)
                        thresh_error_counter = 0
                        # print(self.port + ' data comp' +str(dt))

                    # データが更新されていないとき
                    elif dt < 0:
                        counter_prev = 0
                    else:
                        pass
                        # print(self.port + ' data same')                 

                    ir_prev = ir_temp
                    counter_prev = counter_temp
            
                # 反射光が閾値以下の場合は、エラーカウンタを増やす
                else:
                    thresh_error_counter += 1


                # エラーカウンタがある長さ（erase_data_length_num）を超えたら、指が離れたため測定できないとみなし、これまでのデータを破棄する
                if thresh_error_counter > erase_data_length_num:
                    self.ir_data_list = []
                    counter_prev = None
                    thresh_error_counter = 0
                    print(self.port + ' no finger')
                    self.finger_find_flag = False
                    self.finger_find_pre_flag = False
                    
                # 指が接触してからある長さ（finger_find_rength_pre_num）までデータがたまったら、ノイズが多い最初のデータは破棄する
                if len(self.ir_data_list) >= finger_find_rength_pre_num and self.finger_find_pre_flag == False:
                    print(self.port + ' finger founded pre')
                    self.finger_find_pre_flag = True
                    self.ir_data_list = []

                # 指が接触し、最初のデータ破棄後に、ある長さ（finger_find_length_num）までデータがたまったら、指が定位置にあり正しく測定できているとみなす
                if len(self.ir_data_list) >= finger_find_length_num and self.finger_find_pre_flag == True and self.finger_find_flag == False:
                    print(self.port + ' finger founded')
                    self.finger_find_flag = True

                # ある一定のデータ量（data_list_max_length_num）を超えたら、一番古い値を削除していく
                ir_data_list_amount = len(self.ir_data_list) - data_list_max_length_num
                if ir_data_list_amount > 0:
                    for i in range(ir_data_list_amount):
                        self.ir_data_list.pop(0)
                else:
                    pass
            
            except Exception as e:
                print('measurement_process: ', e)
                # raise

    def calc_bpm_process(self):
        '''# calc_bpm_process

            インスタンス変数に格納されたNIRデータから心拍(bpm)に換算し続ける関数
            換算したbpm値は、インスタンス変数bpmに書き込む

        Note:
            USE_FFTをTrueにすることで30秒以上のデータにおいて、
            フーリエ変換したグラフから心拍を算出することができる。
            が、チューニングにも依存するが、ピークカウントのほうが正確っぽい
        '''
        bpm_prev1 = 60
        bpm_prev2 = 60
        bpm_list = []
        
        while True:

            # 所定のデータ長さ以上になったらピークカウントを行う
            if self.finger_find_flag:
                self.bpm_generate_flag = True

                data_tobe_processed = np.array(self.ir_data_list)
                # ピークをカウントするライブラリを使う
                self.peaks, _ = find_peaks(data_tobe_processed, prominence=(self.PROMINANCE_LOW, None), distance = self.DISTANCE)

                data_length_sec =  round(len(self.ir_data_list) / 100)          # 0.01[sec]周期, 送信周期が長くても線形補完するから定数で良い
                bpm_current = round((len(self.peaks)  * 60 ) / data_length_sec) # 60  [sec]

                # 過去値を3点の平均値をbeat per minutesとして出力
                bpm_temp = int((bpm_current + bpm_prev1 + bpm_prev2)/3)
                bpm_prev2 = bpm_prev1
                bpm_prev1 = bpm_current
                self.bpm  = bpm_temp

                # 一般的な心拍数の範囲に入っているかどうか評価
                if bpm_temp > 40 and bpm_temp < 150:
                    bpm_list.append(bpm_temp)
                # bpm値が安定しているかどうか評価
                if len(bpm_list) > 5:
                    bpm_variance = variance(bpm_list)
                    # print(bpm_variance)
                    if bpm_variance < self.STABLE_VARIANCE:
                        self.bpm_stable_flag = True
                        # print('stable')
                    else:
                        self.bpm_stable_flag = False
                        # print('unstable')
                    bpm_list.pop(0)

            else:
                self.bpm = None
                self.bpm_stable_flag = False
                self.bpm_generate_flag = False
                bpm_list = []
                bpm_prev1 = 60
                bpm_prev2 = 60

            time.sleep(1)

    def start_threads(self):
        '''# start_threads

            クラスメソッドmeasurement_processとcalc_bpm_processを平行に実行する関数\n
            この関数で上記メソッドを実行しなければならない。
        '''
        thread1 = threading.Thread(target = self.measurement_process)
        thread2 = threading.Thread(target = self.calc_bpm_process)
        # デーモン化して、メイン関数終了時にスレッドも終了できるようにする
        thread1.setDaemon(True)
        thread2.setDaemon(True)
        thread1.start()
        thread2.start()


def interact_GYH_process():
    '''# interact_GYH_process

        GYHデバイス同士の心拍交換や、\n
        GYHデバイスからのリクエスト時に話題の送信を行う関数\n

        無限ループのため、threadingで呼び出す\n
    
    '''
    bpm_base_l = None
    bpm_base_r = None
    button_push_counter_prev_l = 0
    button_push_counter_prev_r = 0
    topicgen_l = None
    topicgen_r = None
    # print('called!')
    while True:
        
        # ------------------------------------------------------------------------------
        #     Left側のGYHデバイスの処理
        # ------------------------------------------------------------------------------
        if grabYourHeart_left.bpm_generate_flag:

            # Left側が初めて安定になったタイミングの処理
            if grabYourHeart_left.bpm_stable_flag == True and bpm_base_l == None:
                print('@ left GYH, first stable')
                bpm_base_l = grabYourHeart_left.bpm
                level_max_l = 0
                level_max_prev_l = 3

                # Left側の最初のトピックを生成
                topicgen_l = tpg.topicGenerator()
                if bpm_base_r != None:
                    topic_l = topicgen_l.get_topic(0)
                    print('@ left GYH ', tpg.TOPIC_TO_NAME[str(topic_l)])
                
                    grabYourHeart_left.myGYHdevice.send_8bit_message(topic_l + message_offset)
                time.sleep(1)

            # Left側が安定になった後の心拍値の処理
            if bpm_base_l != None:
                # Left側のbpm変化レベルを算出
                # TODO: ヒステリシスの関数を通す
                level_l = estimate_bpm_level(grabYourHeart_left.bpm, bpm_base_l, BPM_CHANGE_RATE)
                print('@ left GYH, ', level_l)

                # Left側だけモードときは、bpm変化レベルをLeft側に送信
                if LEFT_ONLY:
                    grabYourHeart_left.myGYHdevice.send_8bit_data(level_l)
                # 通常はRightのGYHデバイスにbpm変化レベルを送信する
                else:
                    grabYourHeart_right.myGYHdevice.send_8bit_data(level_l)
                
                # ボタンが押されてから、次のボタンが押されるまでのbpm変化レベルの最大値を記録
                level_max_l = max(level_max_l, level_l)

                time.sleep(1)

                # 心拍がMAXになったら
                if level_l == 10 and ENABLE_MAX_HEART:
                    # Left側だけモードの場合は、Left側の心拍値からLeft側トピックを生成する
                    if LEFT_ONLY:
                        grabYourHeart_left.myGYHdevice.send_8bit_message(max_heart_message)
                    # 通常はRightのGYHデバイスにbpm変化レベルを送信する
                    else:
                        grabYourHeart_right.myGYHdevice.send_8bit_message(max_heart_message)
                    imgPIL.show()
                    playsound('extra/sound/Mendelssohn_WeddingMarch_short.mp3')

            # Left側ボタンが押されたらLeft側トピックを生成する
            if grabYourHeart_left.button_push_counter != button_push_counter_prev_l and bpm_base_l != None:

                # Left側だけモードの場合は、Left側の心拍値からLeft側トピックを生成する
                if LEFT_ONLY:
                    topic_l = topicgen_l.get_topic(level_max_l - level_max_prev_l)
                    try:
                        print('@ left GYH ', 'score_L = ', level_max_l - level_max_prev_l, ', topic = ', tpg.TOPIC_TO_NAME[str(topic_l)])
                
                        grabYourHeart_left.myGYHdevice.send_8bit_message(topic_l + message_offset)
                    except Exception as e:
                        print(e)
                    level_max_prev_l = level_max_l
                    level_max_l = 0

                # 通常はRight側GYHデバイスの心拍値からLeft側トピックを推定する
                else:
                    # Right側デバイスが動作している場合のみ実行
                    if level_max_r and level_max_prev_r != None:
                        topic_l = topicgen_l.get_topic(level_max_r - level_max_prev_r)
                        try:
                            print('@ right GYH ', 'score_R = ', level_max_r - level_max_prev_r, ', topic = ', tpg.TOPIC_TO_NAME[str(topic_l)])

                            grabYourHeart_left.myGYHdevice.send_8bit_message(topic_l + message_offset)
                        except Exception as e:
                            print(e)
                        level_max_prev_r = level_max_r
                        level_max_r = 0
                time.sleep(1)
                
            button_push_counter_prev_l = grabYourHeart_left.button_push_counter

        else:
            # 手が見つかっていないことをGYHデバイスに知らせる
            if LEFT_ONLY:
                grabYourHeart_left.myGYHdevice.send_8bit_data(0)
                grabYourHeart_left.myGYHdevice.send_8bit_data(0xf0)
            # 通常はRight側のGYHデバイスに送信する
            else:
                grabYourHeart_right.myGYHdevice.send_8bit_data(0)
                grabYourHeart_right.myGYHdevice.send_8bit_data(0xf0)
            
            bpm_base_l      = None
            level_max_l     = None
            level_max_prev_l= None
            
            # インスタンスがあれば削除
            if topicgen_l != None:
                del topicgen_l
                topicgen_l = None

        # ------------------------------------------------------------------------------
        #     Right側のGYHデバイスの処理
        # ------------------------------------------------------------------------------
        if grabYourHeart_right.bpm_generate_flag and LEFT_ONLY == False:

            # Rightg側が初めて安定になったタイミングの処理
            if grabYourHeart_right.bpm_stable_flag == True and bpm_base_r == None:
                print('@ right GYH, first stable')
                bpm_base_r = grabYourHeart_right.bpm
                level_max_r = 0
                level_max_prev_r = 3

                # Rightg側の最初のトピックを生成, 観察者モードでないとき
                if RIGHT_OBSERV == False:
                    topicgen_r = tpg.topicGenerator()
                    topic_r = topicgen_r.get_topic(0)
                    print('@ right GYH ', tpg.TOPIC_TO_NAME[str(topic_r)])
                    grabYourHeart_right.myGYHdevice.send_8bit_message(topic_r + message_offset)
                # 観察者モードのとき
                else:
                    pass
                time.sleep(1)

            # Rightg側が安定になった後の心拍値の処理
            if bpm_base_r != None:
                # Rightg側のbpm変化レベルを算出
                # TODO: ヒステリシスの関数を通す
                level_r = estimate_bpm_level(grabYourHeart_right.bpm, bpm_base_r, BPM_CHANGE_RATE)
                print('@ right GYH, ', level_r)

                # 通常はLeft側のGYHデバイスにbpm変化レベル送信する
                grabYourHeart_left.myGYHdevice.send_8bit_data(level_r)
                
                # ボタンが押されてから、次のボタンが押されるまでのbpm変化レベルの最大値を記録
                level_max_r = max(level_max_r, level_r)

                time.sleep(1)

                # 心拍がMAXになったら
                if level_r == 10 and ENABLE_MAX_HEART:
                    grabYourHeart_left.myGYHdevice.send_8bit_message(max_heart_message)

            # Rightg側ボタンが押されたらRight側トピックを生成する
            if grabYourHeart_right.button_push_counter != button_push_counter_prev_r and bpm_base_r != None:
                # 通常はLeft側GYHデバイスの心拍値から推定する, 観察者モードでないとき
                if level_max_l and level_max_prev_l != None and RIGHT_OBSERV == False:
                    topic_r = topicgen_r.get_topic(level_max_l - level_max_prev_l)
                    try:
                        print('@ left GYH ', 'score_L = ',level_max_l - level_max_prev_l, ', topic = ', tpg.TOPIC_TO_NAME[str(topic_r)])
                
                        grabYourHeart_right.myGYHdevice.send_8bit_message(topic_r + message_offset)
                    except Exception as e:
                            print(e)
                    level_max_prev_l = level_max_l
                    level_max_l = 0

            button_push_counter_prev_r = grabYourHeart_right.button_push_counter

        else:
            if LEFT_ONLY:
                # imgPIL.show()
                pass
            # 手が見つかっていないことをleft側GYHデバイスに知らせる
            else:
                grabYourHeart_left.myGYHdevice.send_8bit_data(0)
                grabYourHeart_left.myGYHdevice.send_8bit_data(0xf0)
            
            bpm_base_r      = None
            level_max_r     = None
            level_max_prev_r= None

            # インスタンスがあれば削除
            if topicgen_r != None:
                del topicgen_r
                topicgen_r = None

        # 1sec周期
        time.sleep(1)

def estimate_bpm_level(bpm_current, bpm_base, bpm_change_rate):
    '''# estimate_bpm_stage

        pbmの基準値からの変化率から、現在の心拍数を10段階評価する\n
        1 : 最低\n
        3 : 基準\n
        10: 最高\n

    Args:
        bpm_current    , int   , 現在のbpm値
        bpm_base       , int   , 基準のbpm値
        bpm_change_rate, float , 評価するための基準となる変化率
    
    returns:
        level  , int , 10段階評価値
    '''
    step_value = bpm_base * bpm_change_rate
    
    level  = int((bpm_current - bpm_base) // step_value )

    if level > 7:
        level = 7
    elif level < -2:
        level = -2
    
    # 基準値を3にする
    level = level + 3

    return level


def EXIT():
    '''# EXIT
        root のEXITボタンを押下したときに実行する関数\n
        アプリケーションを終了する
    '''
    sys.exit()


def int_filter(entry_name):
    '''# int_filter

        任意のTKinter Entryウィジェットに対し、\n
        そのEntryに入力された値がintegerかつ正数か判断する関数。\n
        合致していれば値を返す。違う場合はNoneを返す\n

    Args:
        entry_name , TKinter.Entry , Tkinter Entryウィジェット\n
    Returns:
        res        , integer       , Entry入力値をint化した値。入力値がint出ない場合はNone\n
        
    '''
    # Entryの値を読む
    value = entry_name.get().strip()
    # Entryの値を一度消す
    entry_name.delete(0, "end")

    if value.isdecimal() == True and int(value) > 0:
        # intの場合は値を維持する
        entry_name.insert(0, value)
        res  = int(value)
    else:
        # intでない場合は値を消したまま
        entry_name.insert(0, '')
        res = None
    return res

def int_filter_widget_wrapper(event, entry_name):
    '''# int_filter_widget_wrapper

        int_filter関数をTKinterから呼び出すためのラッパー関数
    '''
    int_filter(entry_name)

def update_params(grabYourHeart, whitch):
    '''# update_params

        各調整パラメータをグローバル変数に書き込む関数\n

    Args:
        grabYourHeart, object  , ターゲットの心臓デバイス\n
        whitch       , String  , 'left'か'right'、GUIの左側 or 右側を指定する
    '''
    if whitch == 'left':
        entry_IR_SIGNAL_THRESH = entry_IR_SIGNAL_THRESH_l
        entry_DISTANCE = entry_DISTANCE_l
        entry_PROMINANCE_LOW = entry_PROMINANCE_LOW_l
    else:
        entry_IR_SIGNAL_THRESH = entry_IR_SIGNAL_THRESH_r
        entry_DISTANCE = entry_DISTANCE_r
        entry_PROMINANCE_LOW = entry_PROMINANCE_LOW_r
    new_thresh = int_filter(entry_IR_SIGNAL_THRESH)
    new_distance = int_filter(entry_DISTANCE)
    new_prominance = int_filter(entry_PROMINANCE_LOW)

    # print(new_thresh, new_distance, new_prominance)

    grabYourHeart.IR_SIGNAL_THRESH = new_thresh
    grabYourHeart.DISTANCE         = new_distance
    grabYourHeart.PROMINANCE_LOW   = new_prominance

    print(grabYourHeart, ' parameter updated!')


def init_widgets(grabYourHeart, whitch):
    '''# init_widgets

        entryの値をグローバルの値で初期化する関数

    Args:
        grabYourHeart, object  , ターゲットの心臓デバイス\n
        whitch       , String  , 'left'か'right'、GUIの左側 or 右側を指定する
    '''

    if whitch == 'left':
        entry_IR_SIGNAL_THRESH = entry_IR_SIGNAL_THRESH_l
        entry_DISTANCE = entry_DISTANCE_l
        entry_PROMINANCE_LOW = entry_PROMINANCE_LOW_l
    else:
        entry_IR_SIGNAL_THRESH = entry_IR_SIGNAL_THRESH_r
        entry_DISTANCE = entry_DISTANCE_r
        entry_PROMINANCE_LOW = entry_PROMINANCE_LOW_r
    entry_IR_SIGNAL_THRESH.delete(0, "end")
    entry_IR_SIGNAL_THRESH.insert(0, str(grabYourHeart.IR_SIGNAL_THRESH))
    entry_DISTANCE.delete(0, "end")
    entry_DISTANCE.insert(0, str(grabYourHeart.DISTANCE))
    entry_PROMINANCE_LOW.delete(0, "end")
    entry_PROMINANCE_LOW.insert(0, str(grabYourHeart.PROMINANCE_LOW))
    
def fig_plot(frame):
    '''
        TKinterに配置したMatplotlib Figureにプロットする関数\n
        定期的に呼び出されるためアニメーションのグラフになる
    '''

    ax1.cla()
    ax2.cla()       

    # 左側が見つかった時
    if grabYourHeart_left.finger_find_flag == True and grabYourHeart_right.finger_find_flag == False:
        data_temp = np.array(grabYourHeart_left.ir_data_list)
        peak_temp = grabYourHeart_left.peaks

        # 生データの描画
        ax1.plot(data_temp)
        try:
            # カウントしたピークの描画
            ax1.plot(peak_temp, data_temp[peak_temp],"x")
        except Exception as e:
            print('fig_plot left: ', e)

        # bpmの表示
        label_bpm_show_l['text'] = str(grabYourHeart_left.bpm)
        label_bpm_show_r['text'] = '--'
        label_bpm_stable_r['text'] = ''

        # 心拍値が安定していれば表示する
        if grabYourHeart_left.bpm_stable_flag == True:
            label_bpm_stable_l['text'] = 'stable'
        else:
            label_bpm_stable_l['text'] = ''

    # 右側が見つかった時
    elif grabYourHeart_left.finger_find_flag == False and grabYourHeart_right.finger_find_flag == True:
        data_temp = np.array(grabYourHeart_right.ir_data_list)
        peak_temp = grabYourHeart_right.peaks

        # 生データの描画
        ax2.plot(data_temp)
        try:
            # カウントしたピークの描画
            ax2.plot(peak_temp, data_temp[peak_temp],"x")
        except Exception as e:
            print('fig_plot right: ', e)

        # bpmの表示
        label_bpm_show_l['text'] = '--'
        label_bpm_show_r['text'] = str(grabYourHeart_right.bpm)
        label_bpm_stable_l['text'] = ''

        # 心拍値が安定していれば表示する
        if grabYourHeart_right.bpm_stable_flag == True:
            label_bpm_stable_r['text'] = 'stable'
        else:
            label_bpm_stable_r['text'] = ''

    # 両側が見つかった時
    elif grabYourHeart_left.finger_find_flag == True and grabYourHeart_right.finger_find_flag == True:

        data_temp = np.array(grabYourHeart_left.ir_data_list)
        peak_temp = grabYourHeart_left.peaks

        # 生データの描画
        ax1.plot(data_temp)
        try:
            # カウントしたピークの描画
            ax1.plot(peak_temp, data_temp[peak_temp],"x")
        except Exception as e:
            print('fig_plot left: ', e)
        
        data_temp = np.array(grabYourHeart_right.ir_data_list)
        peak_temp = grabYourHeart_right.peaks

        # 生データの描画
        ax2.plot(data_temp)
        try:
            # カウントしたピークの描画
            ax2.plot(peak_temp, data_temp[peak_temp],"x")
        except Exception as e:
            print('fig_plot right: ', e)

        # bpmの表示
        label_bpm_show_l['text'] = str(grabYourHeart_left.bpm)
        label_bpm_show_r['text'] = str(grabYourHeart_right.bpm)

        # 心拍値が安定していれば表示する
        if grabYourHeart_left.bpm_stable_flag == True:
            label_bpm_stable_l['text'] = 'stable'
        else:
            label_bpm_stable_l['text'] = ''

        if grabYourHeart_right.bpm_stable_flag == True:
            label_bpm_stable_r['text'] = 'stable'
        else:
            label_bpm_stable_r['text'] = ''

    # 指が見つかっていないときはグラフ描画しない
    else:
        label_bpm_show_l['text'] = '--'
        label_bpm_show_r['text'] = '--'
        label_bpm_stable_l['text'] = ''
        label_bpm_stable_r['text'] = ''


if __name__ == '__main__':
    ##########################################################################################
    # 
    #      TKinter root
    #  
    ##########################################################################################
    #root parameter
    root = tk.Tk()
    root_width = 1280
    root_height = 750
    root.geometry(str(root_width) + "x" + str(root_height))
    root.title('Grab Your Heart')

    style = ttk.Style()
    current_theme =style.theme_use()
    style.theme_settings(current_theme, {"TNotebook.Tab": {"configure": {"padding": [20, 4], "font" : ('default', '14', 'bold')}}})

    font_highlight     = font.Font(size = 27, weight = "bold")
    font_highlight_MAX = font.Font(size = 72, weight = "bold")


    #tab parameter
    nb = ttk.Notebook(width = root_width, height = 440)
    tab1 = tk.Frame(nb)
    tab2 = tk.Frame(nb)
    nb.add(tab1, text = '1.INIT', padding = 3)
    nb.add(tab2, text = '2.MAIN', padding = 3)
    nb.pack(expand = 1, fill = 'both')

    #buttons
    button_exit = tk.Button(root, text = '  X  ', font = font.Font(size = 24), bg = '#CD101A', fg = '#FFFFFF', command = EXIT)
    button_exit.place(x = 1180, y = 0, height = 38)

    ##########################################################################################
    # 
    #      TKinter TAB 1
    #  
    ##########################################################################################
    #組み込むグラフを作成 
    fig = plt.figure(figsize = (12.8, 3.5))
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)
    canvas = FigureCanvasTkAgg(fig, master = tab1)
    canvas.get_tk_widget().place(x = 0, y = 10)

    #アニメーションをグラフに追加
    ani = anim.FuncAnimation(fig, fig_plot, interval = 1000)

    #---------------------GUI左側------------------------------------
    # Labels
    label_bpm_show_title_l = tk.Label(tab1, text = 'BPM : ', font = font_highlight)
    label_bpm_show_l       = tk.Label(tab1, text = '--', font = font_highlight_MAX)
    label_bpm_stable_l     = tk.Label(tab1, text = '', font = font.Font(size = 32, weight = 'bold'))
    label_bpm_show_title_l.place(x = 120, y = 400)
    label_bpm_show_l.place(x = 300, y = 370)
    label_bpm_stable_l.place(x = 470, y = 410)

    label_IR_SIGNAL_THRESH_l = tk.Label(tab1, text = 'THRESHOLD:', font = font_highlight)
    label_DISTANCE_l         = tk.Label(tab1, text = 'DISTANCE :', font = font_highlight)
    label_PROMINANCE_LOW_l   = tk.Label(tab1, text = 'PROMINANCE:', font = font_highlight)
    label_IR_SIGNAL_THRESH_l.place(x = 30, y = 520)
    label_DISTANCE_l.place(x= 30, y = 580)
    label_PROMINANCE_LOW_l.place(x = 30,  y = 640)

    #パラメータ入力エントリー
    entry_IR_SIGNAL_THRESH_l = tk.Entry(tab1, font = font_highlight)
    entry_DISTANCE_l         = tk.Entry(tab1, font = font_highlight)
    entry_PROMINANCE_LOW_l   = tk.Entry(tab1, font = font_highlight)
    entry_IR_SIGNAL_THRESH_l.place(x = 300, y = 520, width = 120)
    entry_DISTANCE_l.place(x = 300, y = 580, width = 120)
    entry_PROMINANCE_LOW_l.place(x = 300, y = 640, width = 120)
    entry_IR_SIGNAL_THRESH_l.bind(sequence = "<KeyRelease>", func = lambda event, entry_name = entry_IR_SIGNAL_THRESH_l : int_filter_widget_wrapper(event, entry_name))
    entry_DISTANCE_l.bind(sequence = "<KeyRelease>", func = lambda event, entry_name = entry_DISTANCE_l : int_filter_widget_wrapper(event, entry_name))
    entry_PROMINANCE_LOW_l.bind(sequence = "<KeyRelease>", func = lambda event, entry_name = entry_PROMINANCE_LOW_l : int_filter_widget_wrapper(event, entry_name))

    #パラメータ更新ボタン
    button_update_param_l = tk.Button(tab1, text = 'UPDATE', font = font_highlight, bg = '#99D9EA')#, command = update_params)
    button_update_param_l.place(x = 440 , y = 570, width = 170, height = 70)


    #---------------------GUI右側------------------------------------

    # Labels
    label_bpm_show_title_r = tk.Label(tab1, text = 'BPM : ', font = font_highlight)
    label_bpm_show_r       = tk.Label(tab1, text = '--', font = font_highlight_MAX)
    label_bpm_stable_r     = tk.Label(tab1, text = '', font = font.Font(size = 32, weight = 'bold'))
    label_bpm_show_title_r.place(x = 740, y = 400)
    label_bpm_show_r.place(x = 920, y = 370)
    label_bpm_stable_r.place(x = 1090, y = 410)

    label_IR_SIGNAL_THRESH_r = tk.Label(tab1, text = 'THRESHOLD:', font = font_highlight)
    label_DISTANCE_r         = tk.Label(tab1, text = 'DISTANCE :', font = font_highlight)
    label_PROMINANCE_LOW_r   = tk.Label(tab1, text = 'PROMINANCE:', font = font_highlight)
    label_IR_SIGNAL_THRESH_r.place(x = 670, y = 520)
    label_DISTANCE_r.place(x= 670, y = 580)
    label_PROMINANCE_LOW_r.place(x = 670,  y = 640)

    #パラメータ入力エントリー
    entry_IR_SIGNAL_THRESH_r = tk.Entry(tab1, font = font_highlight)
    entry_DISTANCE_r         = tk.Entry(tab1, font = font_highlight)
    entry_PROMINANCE_LOW_r   = tk.Entry(tab1, font = font_highlight)
    entry_IR_SIGNAL_THRESH_r.place(x = 940, y = 520, width = 120)
    entry_DISTANCE_r.place(x = 940, y = 580, width = 120)
    entry_PROMINANCE_LOW_r.place(x = 940, y = 640, width = 120)
    entry_IR_SIGNAL_THRESH_r.bind(sequence = "<KeyRelease>", func = lambda event, entry_name = entry_IR_SIGNAL_THRESH_r : int_filter_widget_wrapper(event, entry_name))
    entry_DISTANCE_r.bind(sequence = "<KeyRelease>", func = lambda event, entry_name = entry_DISTANCE_r : int_filter_widget_wrapper(event, entry_name))
    entry_PROMINANCE_LOW_r.bind(sequence = "<KeyRelease>", func = lambda event, entry_name = entry_PROMINANCE_LOW_r : int_filter_widget_wrapper(event, entry_name))

    #パラメータ更新ボタン
    button_update_param_r = tk.Button(tab1, text = 'UPDATE', font = font_highlight, bg = '#99D9EA')
    button_update_param_r.place(x = 1080 , y = 570, width = 170, height = 70)
    

    ##########################################################################################
    # 
    #      Main Process
    #  
    ##########################################################################################
    # GUI左側用のデバイスをインスタンス化し、ウィジェットを初期化する
    grabYourHeart_left = grabYourHeart(port_left)
    grabYourHeart_left.start_threads()
    init_widgets(grabYourHeart_left, 'left')
    button_update_param_l.bind(sequence="<ButtonRelease>", func = lambda event, device = grabYourHeart_left, whitch = 'left' : update_params(device, whitch))

    # GUI右側用のデバイスをインスタンス化し、ウィジェットを初期化する。
    if LEFT_ONLY:
        grabYourHeart_right = grabYourHeart(port_right, debug_mode= True)
    else:
        grabYourHeart_right = grabYourHeart(port_right)
        grabYourHeart_right.start_threads()
    init_widgets(grabYourHeart_right, 'right')
    button_update_param_r.bind(sequence="<ButtonRelease>", func = lambda event, device = grabYourHeart_right, whitch = 'right' : update_params(device, whitch))

    thread = threading.Thread(target = interact_GYH_process)
    # デーモン化して、メイン関数終了時にスレッドも終了できるようにする
    thread.setDaemon(True)
    thread.start()

    root.mainloop()

    