"""monitor_heartrate_2device

    接続された2台の心臓デバイスから光センサ値を読みとり、
    心拍数を推定して表示するプログラム

Todo:
    1.interact_GYH_process()関数の記述
      心拍データの基準値から10段階化してGYHデバイスに送信する機能
      ボタンが押されたときに、メッセージ番号を送信する機能
      そのとき、メッセージ番号と心拍変化をログする機能
    
    2.心拍変動値から、メッセージをチューニングする 
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
import random

import arduino_heartrate_device as GYH

port_left = 'COM3'
port_right = 'COM4'

USE_LEFT_ONLY   = True
BPM_CHANGE_RATE = 0.05

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

        # 測定用のパラメータ\n
        IR_SIGNAL_THRESH     , int       , [a.u.] この値以下のデータは無視する（反射光が十分な強度でないと指が接触していない）\n
        ERASE_DATA_LENGTH    , int       , [sec] エラーとなるデータ数がこの値以上ある場合は、指が離れたと仮定しデータを破棄する\n
        DATA_LIST_MAX_LENGTH , int       , [sec] 収録する最大のデータ長さ。このデータリストでピークカウントを行う\n
        FINGER_FIND_LENGTH   , int       , [sec] 収録されたデータ数がこの値以上の場合は、指が接触していると仮定し、心拍計算を行う\n

        # 心拍推定用のパラメータ\n
        DISTANCE             , int       , ピークとピークの距離。これ未満の距離で検出されたピークは無視する\n
        PROMINANCE_LOW       , int       , 地形突出度。この高さ以上突出しているピークを検出する\n
        DATA_LIST_CALC_LENGTH, int       , 収録されたデータがこの値以上の場合、ピークカウントを行う\n
        STABLE_VARIANCE      , int       , 心拍データの分散がこの値未満であれば、心拍値は安定しているとみなす

        # 他インスタンス変数\n
        ir_data_list         , list      , 測定した光センサのIR生データを格納するリスト\n
        peaks                , list      , 生データから検出したピークのリスト\n
        bpm                  , int       , 推定した心拍値beat_per_minutes\n
        finger_find_flag     , bool      , 指が検出された時に立てるフラグ\n
        button_push_counter  , int       , GYHデバイスのボタンが押された回数カウンタ\n
    '''


    def __init__(self, port, baud_rate = 115200, timeout = 1, debug_mode = False):
        self.port = port
        self.myGYHdevice = GYH.hr_device(port, baud_rate, debug_mode = debug_mode)
        
        # measurement_processメソッドのパラメータ
        self.IR_SIGNAL_THRESH     = 40000 
        self.ERASE_DATA_LENGTH    = 3     
        self.DATA_LIST_MAX_LENGTH = 30    
        self.FINGER_FIND_LENGTH   = 5     

        # calc_bpm_processメソッドのパラメータ
        self.USE_FFT         = False
        self.DISTANCE        = 33                 
        self.PROMINANCE_LOW  = 500          
        self.DATA_LIST_CALC_LENGTH = 10
        self.STABLE_VARIANCE = 2

        # 他インスタンス変数
        self.ir_data_list = []
        self.peaks        = []
        self.bpm          = 0
        self.finger_find_flag    = False
        self.bpm_stable_flag     = False
        self.button_push_counter = 0

    def measurement_process(self):
        '''# measurement_process

            NIRデータを取得し続ける関数\n
            取得したデータは、インスタンス変数ir_data_list(リスト)に書き込む

        '''

        self.myGYHdevice.connect()

        self.myGYHdevice.reset_buffer()
        # ゴミデータを捨てる
        for i in range(10):
            self.myGYHdevice.read_data()

        thresh_error_counter = 0
        erase_data_length_num = self.ERASE_DATA_LENGTH * 100
        data_list_max_length_num = self.DATA_LIST_MAX_LENGTH * 100
        finger_find_length_num   = self.FINGER_FIND_LENGTH * 100

        counter_prev = None

        while True:
            # データを取得
            try:
                data_temp = self.myGYHdevice.read_data_rev1()

                ir_temp = data_temp[0]
                counter_temp = data_temp[1]

                # GYHデバイス側でボタンが押されたとき(全データ最大値)
                if ir_temp == 0xFFFF and counter_temp == 0xFFFF:
                    self.button_push_counter += 1
                
                # 通常の処理。閾値以上の反射光の場合は、正しく測定できていると仮定しデータリストに格納する
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
                        # 線形補完したデータを追加
                        for i in range(dt):
                            ir_data_comp = ((ir_prev - ir_temp) / dt) * i + ir_prev
                            self.ir_data_list.append(ir_data_comp)
                        thresh_error_counter = 0
                        # print(self.port + ' data comp' +str(dt))

                    # データが更新されていないとき
                    elif dt == 0:
                        pass
                        # print(self.port + ' data same')                 

                    ir_prev = ir_temp
                    counter_prev = counter_temp
            
                # 閾値以下の場合は、エラーカウンタを増やす
                else:
                    thresh_error_counter += 1


                # エラーカウンタが設定値を超えたら、指が離れたため測定できないとみなし、これまでのデータを破棄する
                if thresh_error_counter > erase_data_length_num:
                    self.ir_data_list = []
                    counter_prev = None
                    thresh_error_counter = 0
                    print(self.port + ' no finger')
                    self.finger_find_flag = False
                    
                # ある長さまでデータがたまったら、指が定位置にあり正しく測定できているとみなす
                if len(self.ir_data_list) == finger_find_length_num:
                    print(self.port + ' finger founded')
                    self.finger_find_flag = True

                # データ量を超えたら、一番古い値を削除していく
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

                data_tobe_processed = np.array(self.ir_data_list)
                # ピークをカウントするライブラリを使う
                self.peaks, _ = find_peaks(data_tobe_processed, prominence=(self.PROMINANCE_LOW, None), distance = self.DISTANCE)

                data_length_sec =  round(len(self.ir_data_list) / 100)
                bpm_current = round((len(self.peaks)  * 60 ) / data_length_sec)

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
    
    '''
    if USE_LEFT_ONLY:
        bpm_base_l = None
        button_push_counter_prev = 0
        print('called!')
        while True:

            
            if grabYourHeart_left.finger_find_flag:

                # 初めて安定になったタイミングの処理
                if grabYourHeart_left.bpm_stable_flag == True and bpm_base_l == None:
                    print('first stable')
                    bpm_base_l = grabYourHeart_left.bpm
                    level_base = 3 # 3を基準のbpm値での基準値としている
                    level_diff_max = 0

                    # トピックの生成
                    topic_random_list = random.sample(range(100), 100)
                    topic = topic_random_list[0]
                    topic_random_list.pop(0)
                    # grabYourHeart_left.myGYHdevice.send_8bit_data(topic + 0x10)

                    dict_topic_bpmlevel = {}

                # 安定になった後の処理
                if bpm_base_l != None:
                    level_l = estimate_bpm_level(grabYourHeart_left.bpm, bpm_base_l, BPM_CHANGE_RATE)
                    print(level_l)

                    # TODO: ヒステリシスの関数を通す
                    # grabYourHeart_left.myGYHdevice.send_8bit_data(level_l)

                    # levelの変化値の評価
                    level_diff = level_l - level_base
                    if abs(level_diff) > abs(level_diff_max):
                        level_diff_max = level_diff

                # ボタンが押されたら
                if grabYourHeart_left.button_push_counter != button_push_counter_prev:
                    dict_topic_bpmlevel[str(topic)] = level_diff_max
                    print(dict_topic_bpmlevel)
                    # 次の話題を送信する
                    topic = topic_random_list[0]
                    topic_random_list.pop(0)
                    # grabYourHeart_left.myGYHdevice.send_8bit_data(topic + 0x10)

            else:
                bpm_base_l     = None
                # grabYourHeart_left.myGYHdevice.send_8bit_data(0)
            time.sleep(1)
    else:
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
    if USE_LEFT_ONLY:
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

    