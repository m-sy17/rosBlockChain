# ライブラリのインポート
import keras
import glob
import numpy as np
from sklearn.model_selection import train_test_split
from tensorflow.keras.utils import load_img, img_to_array
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D
from keras.layers import Dense, Dropout, Flatten
#from keras.utils import np_utils
from keras.utils import to_categorical
from tensorflow.keras.optimizers import Adam
import matplotlib.pyplot as plt
import time
import cv2

train_data_path = 'sakai_env_harf_1' # Colaboratoryにアップロードしたzipファイルを解凍後の、データセットのフォルダ名を入力
image_size_width = 84 # 画像サイズ
image_size_height = 42 # 画像サイズ
color_setting = 3  # 「1」はモノクロ・グレースケール。「3」はカラー。
folder = ['sakai_env_harf_city1','sakai_env_harf_park1'] # データセット画像のフォルダ名
class_number = len(folder)
print('今回のデータで分類するクラス数は「', str(class_number), '」です。')

# データセットの読み込みとデータ形式の設定・正規化・分割
X_image = []
Y_label = []
for index, name in enumerate(folder):
  read_data = train_data_path + '/' + name
  files = glob.glob(read_data + '/*.png')
  print('--- 読み込んだデータセットは', read_data, 'です。')

  for i, file in enumerate(files):
    if color_setting == 1:
      img = load_img(file, color_mode = 'grayscale' ,target_size=(image_size_height, image_size_width))
    elif color_setting == 3:
      img = load_img(file, color_mode = 'rgb' ,target_size=(image_size_height, image_size_width))
    array = img_to_array(img)
    X_image.append(array)
    Y_label.append(index)

X_image = np.array(X_image)
Y_label = np.array(Y_label)

X_image = X_image.astype('float32') / 255
#print(X_image.shape)
#Y_label = np_utils.to_categorical(Y_label, class_number)
Y_label = to_categorical(Y_label, class_number)

train_images, valid_images, train_labels, valid_labels = train_test_split(X_image, Y_label, test_size=0.10)
x_train = train_images
y_train = train_labels
x_test = valid_images
y_test = valid_labels

# モデルの作成 – 畳み込みニューラルネットワーク（CNN）・学習の実行等
model = Sequential()
model.add(Conv2D(16, (3, 3), padding='same',
          input_shape=(image_size_height, image_size_width, color_setting), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Conv2D(128, (3, 3), padding='same', activation='relu'))
model.add(Conv2D(256, (3, 3), padding='same', activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.5))
model.add(Flatten())
model.add(Dense(128, activation='relu'))
model.add(Dropout(0.25))
model.add(Dense(class_number, activation='softmax'))

model.summary()

model.compile(loss='categorical_crossentropy',
              optimizer=Adam(),
              metrics=['accuracy'])

start_time = time.time()

history = model.fit(x_train,y_train, batch_size=2, epochs=5, verbose=1, validation_data=(x_test, y_test))

#plt.plot(history.history['accuracy'])
plt.plot(history.history['val_accuracy'])
plt.title('Model accuracy')
plt.ylabel('Accuracy')
plt.xlabel('Epoch')
plt.grid()
plt.legend(['Validation'], loc='upper left')
plt.show()

#plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('Model loss')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.grid()
plt.legend(['Validation'], loc='upper left')
plt.show()

open('taking_Test_env_harf_1.json','w').write(model.to_json())
model.save_weights('taking_Test_env_harf_1.h5')

score = model.evaluate(x_test, y_test, verbose=0)
print('Loss:', score[0], '（損失関数値 - 0に近いほど正解に近い）')
print('Accuracy:', score[1] * 100, '%', '（精度 - 100% に近いほど正解に近い）')
print('Computation time（計算時間）:{0:.3f} sec（秒）'.format(time.time() - start_time))