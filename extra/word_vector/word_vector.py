# 単語間の類似度比較
# https://qiita.com/DancingEnginee1/items/b10c8ef7893d99aa53be

from gensim.models.word2vec import Word2Vec

model_path = './word2vec.gensim.model'
model = Word2Vec.load(model_path)


print('---------------------------------------------------')
print(model.wv.most_similar(['セガサターン']))
print('---------------------------------------------------')