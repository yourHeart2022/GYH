# Arduino（ESP32）と通信して心拍データをファイルに保存するプログラム

import datetime
import serial
import csv
import time

class hr_device():
    '''# hr_device

        M5Stack用HeartRateModule（MAX30100搭載）を制御するGYHデバイスと通信するクラス

    Attrs:
        port      , String  , GYHデバイスのポート名\n
        baud_rate , int     , GYHデバイスとのシリアル通信ボーレート\n

    '''

    def __init__(self, port, baud_rate, timeout = 1, debug_mode = False):
        self.port       = port
        self.baud_rate  = baud_rate
        self.timeout    = timeout
        self.debug_mode = debug_mode
        self.retry_num  = 3

    def connect(self):
        '''# connect
        
            指定のポートを開くメソッド
        '''
        if self.debug_mode:
            print('debug_mode: ' +str(self.port) + ' connected!')
        else:
            try:
                self.target = serial.Serial(self.port, self.baud_rate, timeout = self.timeout)
                print(str(self.port) + ' connected!')
            except Exception as e:
                print(e)
                raise

    def disconnect(self):
        '''# disconnect

            指定のポートを閉じるメソッド

        '''
        if self.debug_mode:
            print('debug_mode: ' +str(self.port) + ' disconnected!')
        else:
            try:
                self.target.close()
                print(str(self.port) + ' disconnected!')
            except Exception as e:
                print(e)
                raise

    def reset_buffer(self):
        '''# reset_buffer
           
            シリアルバッファを消去するメソッド
        
        '''
        if self.debug_mode:
            print('debug_mode: ' +str(self.port) + ' reset input buffer')
        else:
            try:
                self.target.reset_input_buffer()
                self.target.flush()
                print(str(self.port) + ' reset input buffer')
            except Exception as e:
                print(e)
                # raise

    def read_data(self):
        '''# read_data

            GYHデバイスから出力されるIR, Red光データを読み取る関数

        Args:
            None\n
        Returns:
            data3 , list of int , count、IRおよびRed光の生データ\n
        '''
        if self.debug_mode:
            time.sleep(0.1)
            return [1, 55 , 44 ]
        else:
            try:
                data = self.target.readline()
                # print(data)
                data2 = data.strip().split(b",")
                data3 = [int(i) for i in data2]
                return data3
            except:
                pass
    
    def read_data_rev1(self):
        '''# read_data

            GYHデバイスから出力されるIR, カウンタデータを読み取る関数

        Args:
            None\n
        Returns:
            data3 , list of int , IRおよびカウンタの生データ\n
        '''
        if self.debug_mode:
            time.sleep(0.1)
            return [55 , 1]
        else:
            try:
                data = self.target.readline()
                # print(data)
                data2 = data.strip().split(b",")
                data3 = [int(data2[0]), int(data2[1])]
                return data3
            except:
                # pass
                print(data)

    def send_bpm(self, beat_per_min):
        '''# send_bpm

            GYHデバイスへbpm（1分間当たりの鼓動回数）を送信する関数

        Args:
            beat_per_min, int   , 1分間当たりの鼓動回数\n
        '''
        bpm_tobe_send = [beat_per_min & 0xff]
        if self.debug_mode:
            print('debug mode: write -> ' + str(bpm_tobe_send))
        else:
            try:
                self.target.write(bpm_tobe_send)
                # print(bpm_tobe_send)
            except:
                pass

    def send_8bit_data(self, data_8bit):
        '''# send_bpm

            GYHデバイスへ8ビットデータを送信する関数

        Args:
            beat_per_min, int   , 1分間当たりの鼓動回数\n
        '''
        data_tobe_send = [data_8bit & 0xff]
        if self.debug_mode:
            print('debug mode: write -> ' + str(data_tobe_send))
        else:
            try:
                self.target.write(data_tobe_send)
                # print('send_8bit_data', format(data_tobe_send[0], 'x'))
            except:
                pass

    def send_8bit_message(self, message_8bit):
        '''# send_bpm_v2

            GYHデバイスへメッセージを送信する関数
            

        Args:
            message_8bit, int   , 送信する8ビットメッセージ番号\n
        '''
        data_tobe_send = [message_8bit & 0xff]
        if self.debug_mode:
            print('debug mode: write -> ' + str(data_tobe_send))
        else:
            try:
                # 受信エラー回避のため念のため２回送信する
                for i in range(2):
                    self.target.write(data_tobe_send)
                    time.sleep(0.2)
                # print(data_8bit)
            except:
                pass

if __name__ == '__main__':

    port = "COM5"
    myHRdevice = hr_device(port, 115200)
    myHRdevice.connect()

    topic = 193
    message_offset    = 0x0e
    # myHRdevice.send_8bit_data(topic + message_offset)
    myHRdevice.send_8bit_data(3)
    time.sleep(10)
    #-----------------------------------------------------
    # IR, Redデータをファイルに保存する時に使用するプログラム
    #-----------------------------------------------------
    # SAMPLING_TOTAL_MINUTES = 3 #min

    # SAMPLING_RATE = 100        #Hz
    # SAMPLING_SEC  = 10         #sec

    # # 現在の時刻でファイル名を作る
    # now = datetime.datetime.now()
    # filename = now.strftime('%Y%m%d_%H%M%S') + '.csv'

    # f =  open('data/' + filename, 'a')
    # writer = csv.writer(f, lineterminator='\n')
    # # ヘッダーを書き込む
    # writer.writerow(['count', 'IR', 'RED'])

    # myHRdevice.reset_buffer()
    # # ゴミデータを捨てる
    # for i in range(10):
    #     myHRdevice.read_data()

    # # 指定の時間（min）の間、データを計測する
    # for i in range(SAMPLING_TOTAL_MINUTES * 60 * SAMPLING_RATE):

    #     data = myHRdevice.read_data()
    #     writer.writerow(data)

    # f.close()

    myHRdevice.disconnect()