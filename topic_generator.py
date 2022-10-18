import copy
import random

class topicGenerator():
    '''# class topiCgenerator\n

        GYHデバイスのための、関係性のあるトピックを生成するクラス\n
        
    Attrs:\n
        TOPIC_LIST_ROOT        ,   list   , 関係性のないトピックのリスト\n
        TOPIC_DICT_RELATIONSHIP, dictioary, トピック(key)と関係のあるトピックのリスト(value)

    '''

    TOPIC_LIST_ROOT = [1, 5, 10, 15, 18, 21, 25, 28, 29, 33, 37, 40, 42, 46, 49,\
        51, 54, 56, 57, 58, 60, 62, 65, 67, 70, 72, 76, 80, 84, 86, 88]

    TOPIC_DICT_RELATIONSHIP = {
        '1'  : [2, 3, 4],
        '2'  : [15, 10, 11],
        '3'  : [1, 15, 4, 2, 6, 90],
        '4'  : [3, 1, 91],
        '5'  : [6, 7, 8, 9],
        '6'  : [10, 5],
        '7'  : [20, 56, 92],
        '8'  : [84, 65],
        '9'  : [54, 52],
        '10' : [11, 6, 14, 2, 93],
        '11' : [2, 12, 13, 14],
        '12' : [24, 14, 58, 95],
        '13' : [11, 14, 57, 94],
        '14' : [22, 10, 93],
        '15' : [1, 3, 16, 17, 2],
        '16' : [17, 15, 25, 7],
        '17' : [18, 21, 28],
        '18' : [17, 19, 20],
        '19' : [57, 22],
        '20' : [52, 56],
        '21' : [17, 22, 23, 14, 24],
        '22' : [29, 1, 15],
        '23' : [33, 34, 38],
        '24' : [12, 95],
        '25' : [26, 27, 96],
        '26' : [25, 96],
        '27' : [25, 29, 31],
        '28' : [23, 22, 97],
        '29' : [30, 31, 20, 32, 92],
        '30' : [22],
        '31' : [29, 27, 32],
        '32' : [31],
        '33' : [23, 34, 35, 36, 98],
        '34' : [23, 35],
        '35' : [37, 38, 99],
        '36' : [33, 14, 42],
        '37' : [38, 39, 35],
        '38' : [23, 35, 37],
        '39' : [40, 41, 37],
        '40' : [41, 39],
        '41' : [40],
        '42' : [46, 43, 44, 45],
        '43' : [42, 46, 45],
        '44' : [46, 42, 45],
        '45' : [42, 43, 48, 46, 100],
        '46' : [42, 47, 48],
        '47' : [46, 42, 48],
        '48' : [46, 50, 42],
        '49' : [50, 48, 46],
        '50' : [48, 49],
        '51' : [53, 52, 20],
        '52' : [54, 51, 20],
        '53' : [21, 51, 22],
        '54' : [52, 55],
        '55' : [9],
        '56' : [20, 7, 101],
        '57' : [30, 22],
        '58' : [12, 14, 59, 102],
        '59' : [58, 60, 61, 62],
        '60' : [61, 28, 103],
        '61' : [60, 103],
        '62' : [63, 64],
        '63' : [62, 64],
        '64' : [62, 63],
        '65' : [66, 8],
        '66' : [65, 67],
        '67' : [68, 69],
        '68' : [67, 69],
        '69' : [68, 67],
        '70' : [71, 104],
        '71' : [70, 73, 74],
        '72' : [73, 74, 75, 76],
        '73' : [74, 72, 105],
        '74' : [72, 73, 106],
        '75' : [77, 72],
        '76' : [77, 78, 79],
        '77' : [76, 107],
        '78' : [79, 76, 77],
        '79' : [78, 76],
        '80' : [81, 82, 108],
        '81' : [82, 83],
        '82' : [83, 81],
        '83' : [82],
        '84' : [85, 109],
        '85' : [86, 87],
        '86' : [87],
        '87' : [40, 86],
        '88' : [89],
        '89' : [110, 111, 112],
    }

    def __init__(self):
        self.topic_list_root  = copy.deepcopy(self.TOPIC_LIST_ROOT)
        self.topic_dict_rel   = copy.deepcopy(self.TOPIC_DICT_RELATIONSHIP)
        self.topic_used_list  = []    
        self.topic_current    = None
        self.topic_prev       = None
        self.gen_counter      = 0
        self.root_flag        = False

    def get_topic(self, topic_result):
        '''# get_topic

            呼ばれたらトピックを提供する関数\n

        Args:
            topic_result  , int  , 提供中のトピックの結果の良し悪し。\n
                                 プラスだと良い、ゼロ以下だと悪い\n
        Returns:
            topic_current , int  , 次に提供するトピックの番号
        
        '''
        try:
            self.gen_counter += 1

            # 最初に呼ばれたときの処理
            if self.topic_current == None:
                self.root_flag = True
                topic = random.choice(self.topic_list_root)
                self.topic_list_root.remove(topic)

                self.topic_current = topic
                self.topic_used_list.append(topic)

            # 二回目以降に呼ばれたときの処理
            else:
                # ToDO: 前回、今回、を明示する
                try:
                    print('前回のトピック: ' ,TOPIC_TO_NAME[str(self.topic_prev)],  ',  今回のトピック: ', TOPIC_TO_NAME[str(self.topic_current)])
                except:
                    pass
                
                # 今回のトピックがウケた場合、今回のトピックに関連するトピックにする
                if topic_result > 0 and self.topic_current < 90:
                    serch_rel_topic = self.topic_current
                    print('今回のトピックがウケた場合、今回のトピックに関連するトピックにする')
                
                # 今回のトピックがウケなかった場合、もしくは終端トピック（関連がないトピック）だった場合
                else:
                    print('今回のトピックがウケなかった場合、もしくは終端トピック（関連がないトピック）だった場合')
                    # 今回トピックがルートトピックだった場合、もう一度ルートトピックのトピックを生成する
                    if self.root_flag == True:
                        print('今回トピックがルートトピックだった場合、もう一度ルートトピックのトピックを生成する')
                        topic = random.choice(self.topic_list_root)
                        self.topic_list_root.remove(topic)

                        self.topic_prev    = self.topic_current
                        self.topic_current = topic
                        self.topic_used_list.append(topic)

                        return self.topic_current

                    # 今回のトピックがルートトピックでなかった場合、前回のトピックに関連するトピックにする
                    else:
                        print('今回のトピックがルートトピックでなかった場合、前回のトピックに関連するトピックにする')
                        serch_rel_topic = self.topic_prev
                        self.topic_current = self.topic_prev

                # トピックをkeyとして候補listから関連トピックを生成するが、既出だった場合は生成し直す
                flag = True
                while flag:
                    try:
                        print('トピックをkeyとして候補listから関連トピックを生成するが、既出だった場合は生成し直す')
                        self.root_flag = False
                        topic = self.topic_dict_rel[str(serch_rel_topic)][0]
                        self.topic_dict_rel[str(serch_rel_topic)].remove(topic)
                        flag = topic in self.topic_used_list
                    
                # そのトピックkeyにはもう候補listがない場合、ルートのトピックを生成する
                    except IndexError as e:
                        print('そのトピックkeyにはもう候補listがない場合、ルートのトピックを生成する')
                        self.root_flag = True
                        topic = random.choice(self.topic_list_root)
                        self.topic_list_root.remove(topic)
                        flag = False

                self.topic_prev    = self.topic_current
                self.topic_current = topic
                self.topic_used_list.append(topic)

                # 生成したトピックがルートトピックにもあった場合は、重複防止のためルートトピックから削除する
                if topic in self.topic_list_root:
                    self.topic_list_root.remove(topic)

            return self.topic_current

        except IndexError as e:
            print('IndexError: topic_result = ', topic_result, ', topic_prev = ', self.topic_prev, ', topic_current = ', self.topic_current)
            return None

        except KeyError as e:
            print('KeyError: topic_result = ', topic_result, ', topic_prev = ', self.topic_prev, ', topic_current = ', self.topic_current)
            return None

        except Exception as e:
            print(e)
            raise


TOPIC_TO_NAME = {
 '1' : 'スポーツ観戦'
,'2':  '筋トレ'
,'3':  '映画鑑賞'
,'4':  'マイブーム'
,'5':  '読書'
,'6':  '英会話'
,'7':  '書道'
,'8':  '雑学'
,'9':  '美術'
,'10': 'ヨガ'
,'11': 'エクササイズ'
,'12': 'フィットネス'
,'13': 'ダイエット'
,'14': '美容'
,'15': 'ネイルアート'
,'16': '着付け'
,'17': '手芸'
,'18': '編み物'
,'19': 'チーズ'
,'20': '生け花'
,'21': 'ガーデニング'
,'22': '料理'
,'23': 'ファッション'
,'24': 'アウトドア'
,'25': 'ダンス'
,'26': 'サンバ'
,'27': 'パントマイム'
,'28': '雑貨'
,'29': '手品'
,'30': 'お菓子'
,'31': '大道芸'
,'32': '伝統芸能'
,'33': 'ライフスタイル'
,'34': 'サブカルチャー'
,'35': 'オタク'
,'36': 'DIY'
,'37': 'ギャル'
,'38': 'ゴスロリ'
,'39': 'アイドル'
,'40': 'お笑い芸人'
,'41': 'タレント'
,'42': 'マーケティング'
,'43': 'IT'
,'44': '経営戦略'
,'45': 'ビジネス'
,'46': 'コンサルティング'
,'47': 'マネジメント'
,'48': 'ベンチャー'
,'49': 'ファイナンス'
,'50': 'ファンド'
,'51': '盆栽'
,'52': '陶芸'
,'53': '園芸'
,'54': '古美術'
,'55': 'コレクション'
,'56': '茶道'
,'57': 'お酒'
,'58': 'ブライダル'
,'59': 'ショッピング'
,'60': 'レストラン'
,'61': 'カフェ'
,'62': 'ディズニーランド'
,'63': 'ユニバーサル・スタジオ・ジャパン'
,'64': '富士急ハイランド'
,'65': '人生相談'
,'66': '恋愛相談'
,'67': '飲み会'
,'68': '合コン'
,'69': 'お見合い'
,'70': '旅行'
,'71': '休暇'
,'72': '修学旅行'
,'73': '夏休み'
,'74': '冬休み'
,'75': '合宿'
,'76': '学校行事'
,'77': '部活動'
,'78': '文化祭'
,'79': '体育祭'
,'80': 'ペット'
,'81': '犬'
,'82': 'ネコ'
,'83': 'ウサギ'
,'84': 'マンガ'
,'85': 'ギャグ'
,'86': '一発ギャグ'
,'87': '持ちネタ'
,'88': 'ゲーム'
,'89': 'ファミコン'
,'90': '特技'
,'91': 'さかなクン'
,'92': '武道'
,'93': '自然食'
,'94': 'アンチエイジング'
,'95': 'ウォーキング'
,'96': '音楽'
,'97': '日用品'
,'98': 'エコロジー'
,'99': 'イケメン'
,'100': 'エンターテインメント'
,'101': '俳句'
,'102': 'マイホーム'
,'103': 'ブティック'
,'104': 'ホームステイ'
,'105': '放課後'
,'106': 'ゴールデンウィーク'
,'107': '運動部'
,'108': '観賞魚'
,'109': '絵本'
,'110': 'セガサターン'
,'111': 'ゲームボーイ'
,'112': 'プレイステーション'
}

if __name__ == '__main__':
    import collections

    myTopicGen = topicGenerator()
    topic_list = []

    while True:
        res = myTopicGen.gen_counter, myTopicGen.get_topic(1)
        if res[1] == None:
            break

        # print(res, '        ',  myTopicGen.topic_list_root)
        print(TOPIC_TO_NAME[str(res[1])])
        topic_list.append(res[1])

    print(collections.Counter(topic_list))

    #       2. トピック番号からトピック名に変換し、大体いい感じに話題が振れているか見てみる