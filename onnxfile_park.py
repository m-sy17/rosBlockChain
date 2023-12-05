#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import onnxruntime
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time

class ONNXInference:
    def __init__(self):
        self.agent_distance = 0.0
        self.agent_angle = 0.0
        self.camera_image = None
        self.camera_brg_image = None
        self.angle_dis_array = []
        self.results=[]
        self.i = 0
        self._twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.tagName=""
        
    
    def tag_callback(self, data):
        rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)

        self.tagName = data.data
        # print('subscribe tag : ', self.tagName)
        # self.tag_received.set()  # タグを受信したらイベントを設定

    def display_image(self):
        
        if self.camera_image is not None:
            # ウィンドウサイズに合わせて画像をリサイズ
            window_name = "Camera"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.imshow(window_name, self.camera_image)
            cv2.waitKey(1)
            
    def image_callback(self, image_msg):
        #print(image_msg)
        #print('image subscribe')
        try:
            # self.camera_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            # print("画像の形状:", self.camera_image.shape)
            # print("画像のエンコーディング:", image_msg.encoding)
            self.bridge = CvBridge()
            # ROSのImageメッセージをNumPy配列に変換
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            #img_flipで上下左右反転
            img_flip = cv2.flip(cv_image, 0)
            # 画像を84x84にリサイズ
            ################################################################################
            # img_84 = cv2.resize(img_flip, (84, 84), interpolation=cv2.INTER_NEAREST)#####
            ######INTER_NEARESTをINTER_AREAにしたら映して欲しい物が映った。理由は不明#######
            ################################################################################
            img_84 = cv2.resize(img_flip, (84, 84), interpolation=cv2.INTER_AREA)
            #bgrで読み込んだのをrgbに変換する
            self.camera_image = cv2.cvtColor(img_84, cv2.COLOR_BGR2RGB)
            #Dockerから実行する際は次のコードをコメントアウトする
            self.display_image()
        except Exception as err:
            print(err)

    def pose_callback(self, pose_msg):
        #print('dis and angle subscribe')
        # 受け取ったPoseStampedメッセージから距離と角度を抽出
        self.agent_distance = pose_msg.pose.position.x
        self.agent_angle = pose_msg.pose.position.y
        #距離角度は正しく送れている
        print("distance : ", self.agent_distance, " angle : ", self.agent_angle)

    def inference(self):
        #session = onnxruntime.InferenceSession('park_finish.onnx') # ONNXモデルを読み込み
        #print('onnxfie')
        session = onnxruntime.InferenceSession('symposia_park4.onnx') # ONNXモデルを読み込み
        # 受け取ったデータを使ってONNXモデルで推論を実行
        if self.camera_image is not None:
            # カメラ画像を処理
            camera_image_input = self.camera_image.transpose((2, 0, 1))  # ONNX形式に変換（チャンネルが先頭）
            camera_image_input = camera_image_input[np.newaxis, ...]
            #camera_input = camera_image_input.astype(np.float32)
            # ニューラルネットの入力が０，１なので正規化
            camera_input=camera_image_input/255.0
            camera_input=camera_input.astype(np.float32)
            #print(camera_image_input)
            #print("camera OK")
        else:
            # 画像が利用できない場合の処理（例：ゼロで埋めるか、プレースホルダを使用するか、任意に決定してください）
            #camera_image_input = np.zeros((1, 3, 84, 84), dtype=np.float32)
            camera_input = np.zeros((1, 3, 84, 84), dtype=np.float32)
        distance_amgle_input = np.array([[self.agent_distance, self.agent_angle]], dtype=np.float32)
        #angle_input = np.array([[self.agent_angle]], dtype=np.float32)

        # 入力を準備して推論を実行
        inputs = {
            session.get_inputs()[0].name: camera_input,
            session.get_inputs()[1].name: distance_amgle_input
        }
        self.results = session.run([session.get_outputs()[2].name], inputs)
        # 速度
        linear= self.results[0][0][0]
        #print("speed: ", self.results[0][0][0])
        #linear=0
        if linear <= 0:
           linear=0

        # 角速度
        angular= self.results[0][0][1]
        #print("angle: ", -self.results[0][0][1])
        #angular=0
        # 推論結果を必要に応じて処理
        # （resultsにはONNXモデルの出力が含まれます）

        # if self.i == 2 and self.dis <= 0.5:
        #     #self._arrival_pub.publish(Bool())
        #     linear = 0
        #     angular = 0
        #     print('!-------------------------------reach-----------------------------!')
        # elif self.dis <= 0.5:
        #     self.i += 1
        
        print('速度: ', linear , ',角速度 ' , angular)
        #self.display_image()
        twist = Twist()
        twist.linear.x = 0.5*linear
        twist.angular.z = 0.5*angular
                
        return twist
    def publish(self, cmd_vel):    
        self._twist_pub.publish(cmd_vel)
        #print('can publish')
        print(cmd_vel)

if __name__ == '__main__':
    onnx_inference = ONNXInference()
    rospy.init_node('onnx_inference_node')
    rospy.Subscriber("tag_environment", String, onnx_inference.tag_callback)
    rospy.Subscriber("robot_distance_angle", PoseStamped, onnx_inference.pose_callback)
    
    rospy.Subscriber("camera_image", Image, onnx_inference.image_callback)
    
    
    # rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        cmd_vel = onnx_inference.inference()
        onnx_inference.publish(cmd_vel)
        time.sleep(0.1)
    rospy.spin()
