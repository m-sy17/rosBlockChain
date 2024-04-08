#!/usr/bin/env python3
# coding: UTF-8
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
from tensorflow.keras.utils import load_img, img_to_array
from keras.models import model_from_json
from tensorflow.keras.optimizers import Adam
from cv_bridge import CvBridge
import cv2

#環境分類モデルを読み込み、画像から環境分類結果を出力する
class Classification:
    def __init__(self):
        # ROSノードの初期化
        rospy.init_node("classification_node")

        # モデルの読み込みとコンパイル
        model_architecture = 'taking_Test_env_harf_1.json'
        #model_architecture = 'cnn_model_symposia_harf3.json'
        with open(model_architecture, 'r') as json_file:
            loaded_model_json = json_file.read()

        self.loaded_model = model_from_json(loaded_model_json)

        model_weights = 'taking_Test_env_harf_1.h5'
        self.loaded_model.load_weights(model_weights)

        self.loaded_model.compile(loss='categorical_crossentropy',
                                  optimizer=Adam(),
                                  metrics=['accuracy'])

        # クラスラベルの設定
        self.class_labels = ['Park', 'City']

        # ディスプレイウィンドウの名前
        self.window_name = "Camera"

        # 画像データの初期化
        self.image_data = None
        #self.image_path = 'photo1.png'

        # ROSのPublisherとSubscriberの設定
        self.pub = rospy.Publisher("classification_env", String, queue_size=10)
        self.image_sub = rospy.Subscriber("camera_image", Image, self.image_callback)

    def image_callback(self, image_msg):
        try:
            #print(image_msg)
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(image_msg, "rgb8")
            img_flip = cv2.flip(cv_image, 0)
            img_84 = cv2.resize(img_flip, (84, 42), interpolation=cv2.INTER_AREA)
            self.image_data = img_84
            #print(self.image_data)
            """
            # ディスプレイ表示
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.imshow(self.window_name, img_84)
            cv2.waitKey(1)
            """

        except Exception as err:
            print(err)

    def load_and_preprocess_image(self):
        #print(self.image_data)
        if self.image_data is not None:
            """
            image_size = 84  # 画像サイズ（モデルの入力サイズに合わせる）
            color_setting = 3  # カラー画像を使用する場合は3、モノクロの場合は1に設定

            if color_setting == 1:
                img = load_img(self.image_path, color_mode='grayscale', target_size=(image_size, image_size))
            elif color_setting == 3:
                img = load_img(self.image_path, color_mode='rgb', target_size=(image_size, image_size))
            """

            array = img_to_array(self.image_data)
            #print(array)
            preprocessed_image = np.array([array])
            preprocessed_image = preprocessed_image.astype('float32') / 255  # 正規化

            return preprocessed_image

    def classify_image(self):
        new_image = self.load_and_preprocess_image()
        #print(new_image)

        if new_image is not None:
            predictions = self.loaded_model.predict(new_image)
            predicted_class_index = np.argmax(predictions)
            predicted_class_label = self.class_labels[predicted_class_index]

            msg = String()
            msg.data = predicted_class_label

            self.pub.publish(msg)

            rospy.loginfo("Publish : " + msg.data)

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.classify_image()
            r.sleep()

if __name__ == "__main__":
    Classification().run()