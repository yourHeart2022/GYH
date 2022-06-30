"""monitor_heartrate_2device

    接続された2台の心臓デバイスから光センサ値を読みとり、
    心拍数を推定して表示するプログラム

Todo:
    1. 心拍データをArduinoデバイスに送信する
    2. measurement_processとcalc_bpm_processの関係の見直し
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

import arduino_heartrate_device as arduino

port_left = 'COM3'
port_right = 'COM5'

USE_SINGLE_DEVICE = True

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
        THRESH_ERROR_COUNTUP , int       , [sec] エラーカウンタ。無視されたデータがこのカウンタ値以上ある場合は、指が離れたと仮定しデータを破棄する\n
        DATA_LIST_MAX_LENGTH , int       , [sec] 収録する最大のデータ長さ。このデータリストでピークカウントを行う\n
        FINGER_FIND_LENGTH   , int       , [sec] 収録されたデータがこの値以上の場合は、指が接触していると仮定する\n

        # 心拍推定用のパラメータ\n
        DISTANCE             , int       , ピークとピークの距離。これ未満の距離で検出されたピークは無視する\n
        PROMINANCE_LOW       , int       , 地形突出度。この高さ以上突出しているピークを検出する\n
        DATA_LIST_CALC_LENGTH, int       , 収録されたデータがこの値以上の場合、ピークカウントを行う\n

        # 他インスタンス変数\n
        ir_data_list         , list      , 測定した光センサのIR生データを格納するリスト\n
        peaks                , list      , 生データから検出したピークのリスト\n
        bpm                  , int       , 推定した心拍値beat_per_minutes\n
        finger_find_flag     , bool      , 指が検出された時に立てるフラグ\n
    '''


    def __init__(self, port, baud_rate = 115200, timeout = 1, debug_mode = False):
        self.port = port
        self.myHRdevice = arduino.hr_device(port, baud_rate, debug_mode = debug_mode)
        
        # measurement_processメソッドのパラメータ
        self.IR_SIGNAL_THRESH     = 40000 
        self.THRESH_ERROR_COUNTUP = 3     
        self.DATA_LIST_MAX_LENGTH = 30    
        self.FINGER_FIND_LENGTH   = 5     

        # calc_bpm_processメソッドのパラメータ
        self.USE_FFT  = False
        self.DISTANCE = 33                 
        self.PROMINANCE_LOW = 500          
        self.DATA_LIST_CALC_LENGTH = 10    

        # 他インスタンス変数
        self.ir_data_list = []
        self.peaks        = []
        self.bpm          = None
        self.finger_find_flag = False
        self.beat_per_min_ave = 60

    def measurement_process(self):
        '''# measurement_process

            NIRデータを取得し続ける関数、データはインスタンス変数のリストに書き込む

        '''

        self.myHRdevice.connect()

        self.myHRdevice.reset_buffer()
        # ゴミデータを捨てる
        for i in range(10):
            self.myHRdevice.read_data()

        thresh_error_counter = 0
        thtesh_error_countup_num = self.THRESH_ERROR_COUNTUP * 100
        data_list_max_length_num = self.DATA_LIST_MAX_LENGTH * 100
        finger_find_length_num   = self.FINGER_FIND_LENGTH * 100

        counter_prev = None

        while True:
            # データを取得
            try:
                data_temp = self.myHRdevice.read_data_rev1()
            except:
                pass


            ir_temp = data_temp[0]
            counter_temp = data_temp[1]

            # 正しく反射光が得られているか、閾値で決める。閾値以上だとデータリストに格納する
            if ir_temp > self.IR_SIGNAL_THRESH:

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
                    print(self.port + ' data comp' +str(dt))

                # データが更新されていないとき
                elif dt == 0:
                    print(self.port + ' data same')                 

                ir_prev = ir_temp
                counter_prev = counter_temp
        
            # 閾値以下の場合は、エラーカウンタを増やす
            else:
                thresh_error_counter += 1


            # エラーカウンタが設定値を超えたら、これまでのデータを破棄し、グラフ描画を終了
            if thresh_error_counter > thtesh_error_countup_num:
                self.ir_data_list = []
                counter_prev = None
                thresh_error_counter = 0
                print(self.port + ' no finger')
                self.finger_find_flag = False
                
            # ある長さまでデータがたまったら、グラフ描画を開始
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

    def calc_bpm_process(self):
        '''# calc_bpm_process

            インスタンス変数に格納されたNIRデータから心拍(bpm)に換算し続ける関数

        Note:
            USE_FFTをTrueにすることで30秒以上のデータにおいて、
            フーリエ変換したグラフから心拍を算出することができる。
            が、チューニングにも依存するが、ピークカウントのほうが正確っぽい
        '''
        data_list_calc_lemgth_num = self.DATA_LIST_CALC_LENGTH * 100
        data_list_max_length_num = self.DATA_LIST_MAX_LENGTH * 100
        bpm_prev1 = 60
        bmp_prev2 = 60
        
        while True:

            # 所定のデータ長さ以上になったらピークカウントを行う
            if len(self.ir_data_list) > data_list_calc_lemgth_num:

                data_tobe_processed = np.array(self.ir_data_list)
                # ピークをカウントするライブラリを使う
                self.peaks, _ = find_peaks(data_tobe_processed, prominence=(self.PROMINANCE_LOW, None), distance = self.DISTANCE)

                data_length_sec =  round(len(self.ir_data_list) / 100)
                beat_per_min = round((len(self.peaks)  * 60 ) / data_length_sec)

                # 過去値を3点の平均値をbeat per minutesとして出力
                bpm_prev2 = bpm_prev1
                bpm_prev1 = beat_per_min
                self.beat_per_min_ave = int((beat_per_min + bpm_prev1 + bpm_prev2)/3)

                # TODO: もう片方のデバイスへ、BPM値を送る
                # self.myHRdevice.send_bpm(beat_per_min_ave)

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
    # TODO: grabYourHeartで個別に設定する
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
            print(e)

        # bpmの表示
        label_bpm_show_l['text'] = str(grabYourHeart_left.beat_per_min_ave)
        label_bpm_show_r['text'] = '--'

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
            print(e)

        # bpmの表示
        label_bpm_show_l['text'] = '--'
        label_bpm_show_r['text'] = str(grabYourHeart_right.beat_per_min_ave)

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
            print(e)
        
        data_temp = np.array(grabYourHeart_right.ir_data_list)
        peak_temp = grabYourHeart_right.peaks

        # 生データの描画
        ax2.plot(data_temp)
        try:
            # カウントしたピークの描画
            ax2.plot(peak_temp, data_temp[peak_temp],"x")
        except Exception as e:
            print(e)

        # bpmの表示
        label_bpm_show_l['text'] = str(grabYourHeart_left.beat_per_min_ave)
        label_bpm_show_r['text'] = str(grabYourHeart_right.beat_per_min_ave)

    # 指が見つかっていないときはグラフ描画しない
    else:
        label_bpm_show_l['text'] = '--'
        label_bpm_show_r['text'] = '--'
        # ani.event_source.stop()
    

    # 
    # if self.finger_find_flag == False:
    #     self.ani.event_source.stop()
    #     plt.cla()
    # # 指が見つかった時
    # else:
    #     plt.cla()
    #     data_temp = np.array(self.ir_data_list)
    #     peak_temp = self.peaks

    #     # 生データの描画
    #     plt.plot(data_temp)
    #     try:
    #         # カウントしたピークの描画
    #         plt.plot(peak_temp, data_temp[peak_temp],"x")
    #     except Exception as e:
    #         print(e)

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
    label_bpm_show_title_l.place(x = 120, y = 400)
    label_bpm_show_l.place(x = 300, y = 370)

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
    label_bpm_show_title_r.place(x = 740, y = 400)
    label_bpm_show_r.place(x = 920, y = 370)

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

    if USE_SINGLE_DEVICE:
        grabYourHeart_right = grabYourHeart(port_right)
    else:
        grabYourHeart_right = grabYourHeart(port_right, debug_mode = True)
        grabYourHeart_right.start_threads()
    init_widgets(grabYourHeart_right, 'right')
    button_update_param_r.bind(sequence="<ButtonRelease>", func = lambda event, device = grabYourHeart_right, whitch = 'right' : update_params(device, whitch))

    root.mainloop()

    