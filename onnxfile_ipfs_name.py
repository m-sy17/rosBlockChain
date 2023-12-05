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
import threading

# onnx呼び出し用に追記
from web3 import Web3
import json
import requests
import os

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
        
        self.tag_received = threading.Event()  # タグを受信したことを通知するためのイベント
        self.tagName = None
        self.modelName_dict = {}
        self.inferenceSession_dict = {}

        self.file_received = threading.Event()  # タグを受信したことを通知するためのイベント


        # smart contaract 
        self.web3 = Web3(Web3.HTTPProvider('http://localhost:7545'))
        self.web3.eth.default_account = self.web3.eth.accounts[4]
        
        with open("rosipfs_contract_address.json", "r") as f:
            contract_data = json.load(f)
        self.contract_address = contract_data["contract_address"]

        self.contract_abi = '''
        [
            {
                "anonymous": false,
                "inputs": [
                    {
                        "indexed": false,
                        "internalType": "address",
                        "name": "",
                        "type": "address"
                    },
                    {
                        "indexed": false,
                        "internalType": "uint256",
                        "name": "",
                        "type": "uint256"
                    }
                ],
                "name": "Received",
                "type": "event"
            },
            {
                "inputs": [
                    {
                        "internalType": "string",
                        "name": "_hashName",
                        "type": "string"
                    },
                    {
                        "internalType": "string",
                        "name": "_ipfsHash",
                        "type": "string"
                    }
                ],
                "name": "addHash",
                "outputs": [
                    {
                        "internalType": "address",
                        "name": "",
                        "type": "address"
                    }
                ],
                "stateMutability": "nonpayable",
                "type": "function"
            },
            {
                "inputs": [
                    {
                        "internalType": "address",
                        "name": "",
                        "type": "address"
                    }
                ],
                "name": "balance",
                "outputs": [
                    {
                        "internalType": "uint256",
                        "name": "",
                        "type": "uint256"
                    }
                ],
                "stateMutability": "view",
                "type": "function"
            },
            {
                "inputs": [
                    {
                        "internalType": "string",
                        "name": "_hashName",
                        "type": "string"
                    }
                ],
                "name": "getHash",
                "outputs": [
                    {
                        "internalType": "string",
                        "name": "",
                        "type": "string"
                    }
                ],
                "stateMutability": "payable",
                "type": "function"
            },
            {
                "inputs": [
                    {
                        "internalType": "string",
                        "name": "",
                        "type": "string"
                    }
                ],
                "name": "ipfsData",
                "outputs": [
                    {
                        "internalType": "string",
                        "name": "ipfsHash",
                        "type": "string"
                    },
                    {
                        "internalType": "address",
                        "name": "owner",
                        "type": "address"
                    }
                ],
                "stateMutability": "view",
                "type": "function"
            },
            {
                "stateMutability": "payable",
                "type": "receive"
            }
        ]
        '''
        
    
    def display_image(self):
        
        if self.camera_image is not None:
            # ウィンドウサイズに合わせて画像をリサイズ
            window_name = "Camera"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.imshow(window_name, self.camera_image)
            cv2.waitKey(1)
            
    def image_callback(self, image_msg):
        # print('image subscribe')
        try:
            self.bridge = CvBridge()
            # ROSのImageメッセージをNumPy配列に変換
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            #img_flipで上下左右反転
            img_flip = cv2.flip(cv_image, 0)
            img_84 = cv2.resize(img_flip, (84, 84), interpolation=cv2.INTER_AREA)
            self.camera_image = cv2.cvtColor(img_84, cv2.COLOR_BGR2RGB)
            #Dockerから実行する際は次のコードをコメントアウトする
            self.display_image()
        except Exception as err:
            print(err)

    def tag_callback(self, data):
        rospy.loginfo(rospy.get_caller_id()+" I heard %s",data.data)

        self.tagName = data.data
        # print('subscribe tag : ', self.tagName)
        self.tag_received.set()  # タグを受信したらイベントを設定

    def pose_callback(self, pose_msg):
        #print('dis and angle subscribe')
        # 受け取ったPoseStampedメッセージから距離と角度を抽出
        self.agent_distance = pose_msg.pose.position.x
        self.agent_angle = pose_msg.pose.position.y
        # print("distance : ", self.agent_distance, " angle : ", self.agent_angle)

    #connect contract & get onnx flie hash address
    def get_onnx_hash(self):
        # self.tag_received.wait()
        # contract = self.web3.eth.contract(address=self.contract_address, abi=self.contract_abi)
        # print("contract : ", contract)
        # # onnx_hash = contract.functions.getHash(str(self.tagName)).call()
        # onnx_hash = contract.functions.getHash(self.tagName).call()

        # # onnx_hash = contract.functions.getHash("Park").call()
        # print("get onnx ipfs hash")
        # print('onnx ipfs hash check : ', onnx_hash)
        # ipfs_url = f'https://ipfs.io/ipfs/{onnx_hash}'
        # response = requests.get(ipfs_url)
        # self.onnx_filename = f'downloaded_model_{self.tagName}.onnx'
        # f'{self.tagName}_onnx_filename'
        # print(self.onnx_filename)
        # with open(self.onnx_filename, 'wb') as onnx_file:
        #     onnx_file.write(response.content)
        # return self.onnx_filename
        # #self.file_received.set()
        self.tag_received.wait()
        contract = self.web3.eth.contract(address=self.contract_address, abi=self.contract_abi)
        print("contract : ", contract)
        onnx_hash = contract.functions.getHash(self.tagName).call()
        print("get onnx ipfs hash")
        print('onnx ipfs hash check : ', onnx_hash)
        onnx_filename = f'downloaded_model_{self.tagName}.onnx'
        print(onnx_filename)
        if not os.path.isfile(onnx_filename):
            # ファイルが存在しない場合のみダウンロード
            ipfs_url = f'https://ipfs.io/ipfs/{onnx_hash}'
            response = requests.get(ipfs_url)
            with open(onnx_filename, 'wb') as onnx_file:
                onnx_file.write(response.content)
        return onnx_filename
        
    def inference(self):
        #self.file_received.wait()
        # print('call inference')
        # self.model_path = self.get_onnx_hash()
        # # if not hasattr(self, 'onnx_filename') or not os.path.isfile(f'downloaded_model_{self.tagName}.onnx'):
        # #     # self.onnx_filename が存在しないか、ファイルが存在しない場合
        # #     self.model_path = self.get_onnx_hash()
        # #     print('set onnx model')
        # print('onnx model name : ', self.onnx_filename)
        # session = onnxruntime.InferenceSession(self.model_path) # ONNXモデルを読み込み

        if self.tagName in ["Park", "City"]:
            if self.tagName not in self.inferenceSession_dict:
                # モデルをダウンロード中は inference を無効にする
                self.inference_enabled = False
                onnx_filename = self.get_onnx_hash()
                print('set onnx model : ', onnx_filename)
                self.modelName_dict[self.tagName] = onnx_filename
                self.inferenceSession_dict[self.tagName] = onnxruntime.InferenceSession(onnx_filename)
                self.inference_enabled = True  # モデルのダウンロードが完了したら inference を有効にする

            if not self.inference_enabled:
                print('Model download in progress. Inference is disabled.')
                return None  # モデルがダウンロード中は推論を行わない

            # print('onnx model name : ', self.model_dict[self.tagName])
            session = self.inferenceSession_dict[self.tagName]
            print('set onnx model : ', self.modelName_dict[self.tagName])

            # ... (perform inference using the session) ...
                # 受け取ったデータを使ってONNXモデルで推論を実行
            if self.camera_image is not None:
                # カメラ画像を処理
                camera_image_input = self.camera_image.transpose((2, 0, 1))  # ONNX形式に変換（チャンネルが先頭）
                camera_image_input = camera_image_input[np.newaxis, ...]
                camera_input = camera_image_input.astype(np.float32)
            else:
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
            # print("speed: ", self.results[0][0][0])
            #linear=0
            if linear <= 0:
                linear=0

            # 角速度
            angular= self.results[0][0][1]
            #print("angle: ", -self.results[0][0][1])
            #angular=0
            # print('速度: ', linear , ',角速度 ' , angular)
            twist = Twist()
            twist.linear.x = 0.5*linear
            twist.angular.z = 0.5*angular
                    
            return twist
        else:
            print(f'Unsupported tag name: {self.tagName}')

        
    def publish(self, cmd_vel):    
        self._twist_pub.publish(cmd_vel)
        # print(cmd_vel)

if __name__ == '__main__':
    onnx_inference = ONNXInference()
    rospy.init_node('onnx_inference_node')
    rospy.Subscriber("tag_environment", String, onnx_inference.tag_callback)
    rospy.Subscriber("robot_distance_angle", PoseStamped, onnx_inference.pose_callback)
    rospy.Subscriber("camera_image", Image, onnx_inference.image_callback)
    
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        # cmd_vel = onnx_inference.inference()
        # onnx_inference.publish(cmd_vel)
        # # time.sleep(0.1)
        cmd_vel = onnx_inference.inference()
        if cmd_vel:  # 推論の結果がある場合のみ publish
            onnx_inference.publish(cmd_vel)
        rate.sleep()
    rospy.spin()


