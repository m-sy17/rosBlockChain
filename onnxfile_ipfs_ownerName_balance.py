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
# from eth import web3
import json
import requests
import csv

import os
import sys

class ONNXInference:
    def __init__(self, model_directory):
        self.agent_distance = 0.0
        self.agent_angle = 0.0
        self.camera_image = None
        self.camera_brg_image = None
        self.angle_dis_array = []
        self.results=[]
        self.i = 0
        self._twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.inference_enabled = True

        # タグ管理
        self.tag_received = threading.Event()  # タグを受信したことを通知するためのイベント
        self.tagName = None
        # モデル管理
        self.onnx_filename = None
        self.modelName_dict = {}
        self.inferenceSession_dict = {}

        # smart contaract 
        self.web3 = Web3(Web3.HTTPProvider('http://localhost:7545'))
        self.web3.is_connected()
        # self.web3.eth.default_account = self.web3.eth.accounts[4]
        self.default_account = self.web3.eth.accounts[2]
        
        # Ganacheの残高をCSVファイルにする関連
        self.accounts = self.web3.eth.accounts
        self.reach_count = 0
        self.csv_data = []
        for account in self.accounts:
            balance_wei = self.web3.eth.get_balance(account)
            balance_eth = self.web3.from_wei(balance_wei, 'ether')
            self.csv_data.append([account ,balance_eth])
        # print(self.csv_data)
        self.csv_file_path = os.path.join(os.getcwd(), "balances.csv")

        # ディレクトリ関連
        self.model_directory = model_directory
        if not os.path.exists(self.model_directory):
            os.makedirs(self.model_directory)
        self.file_delete = threading.Event()
        self.file_delete.set()

        # コントラクトのアドレス管理
        with open("rosipfs_contract_address.json", "r") as f:
            contract_data = json.load(f)
        self.contract_address = contract_data["contract_address"]
        print("contract addrsss" + self.contract_address)

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
        #print('image subscribe')
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

    def pose_callback(self, pose_msg):
        #print('dis and angle subscribe')
        # 受け取ったPoseStampedメッセージから距離と角度を抽出
        self.agent_distance = pose_msg.pose.position.x
        self.agent_angle = pose_msg.pose.position.y
        #print("distance : ", self.agent_distance, " angle : ", self.agent_angle)

    def tag_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id()+" I'm in the %s",data.data)
        self.tagName = data.data
        print('subscribe tag : ', self.tagName)
        self.tag_received.set()  # タグを受信したらイベントを設定
        
    def reach_callback(self, data):
        zero_vel = Twist()
        zero_vel.linear.x = 0
        zero_vel.linear.y = 0
        zero_vel.linear.z = 0
        zero_vel.angular.x = 0
        zero_vel.angular.y = 0
        zero_vel.angular.z = 0
        self._twist_pub.publish(zero_vel)
        rospy.loginfo(rospy.get_caller_id()+" I heard %s",data.data)
        self.reach_count += 1
        # file delete
        file_lists = os.listdir(self.onnx_filepath)
        for filename in file_lists:
            if filename.endswith('.onnx'):
                file_path = os.path.join(self.onnx_filepath, filename)
                os.remove(file_path)
        # os.remove(self.onnx_filename)
        self.modelName_dict.clear()
        self.inferenceSession_dict.clear()
        print(self.model_directory)
        print("delete onnx")
        if self.reach_count >= 4:
            self.recode_balues() 
        # time.sleep(30)
        
    #connect contract & get onnx flie hash address
    def get_onnx_hash(self):
        self.tag_received.wait()
        contract = self.web3.eth.contract(address=self.contract_address, abi=self.contract_abi)
        
        print("--------------------------------------------caontract info------------------------------------------------")
        print("contract : ", contract)
        transaction = {
            'from': self.default_account,
            # 'gas': 4000000,
            # 'gasPrice': self.web3.to_wei('21', 'gwei'),
            'to': self.contract_address,  # コントラクトのアドレスを指定
            # 'to': '0xf2667684bee512e23a0cb7c533276a7a06618732009a5b3a2a56ae6e85dcf3d2',
            'value': self.web3.to_wei(1, 'ether'),  # 送金するEtherの量
            'nonce': self.web3.eth.get_transaction_count(self.default_account),
        }
        print('transaction', transaction)
        tx_hash = self.web3.eth.send_transaction(transaction)
        print('tx_hash : ', tx_hash)
        # print('recipt : ', tx_recipt)
        onnx_hash = contract.functions.getHash(self.tagName).call({'from': self.default_account})
        print('onnx ipfs hash check : ', onnx_hash)
        do_transact = contract.functions.getHash(self.tagName).transact({'from': self.default_account})
        print('do_contract : ', do_transact)
        tx_receipt = self.web3.eth.wait_for_transaction_receipt(do_transact)

        print("--------------------------------------------caontract info------------------------------------------------")

        print("--------------------------------------------model file info-----------------------------------------------")
        self.onnx_filename = f'downloaded_model_{self.tagName}.onnx'
        print("file name", self.onnx_filename)
        self.onnx_filepath = os.getcwd()
        print("file path : ", self.onnx_filepath)
        print("--------------------------------------------model file info-----------------------------------------------")


        if not os.path.isfile(self.onnx_filepath):
            # ファイルが存在しない場合のみダウンロード
            print('--------------file download --------------')
            ipfs_url = f'https://ipfs.io/ipfs/{onnx_hash}'
            response = requests.get(ipfs_url)
            with open(self.onnx_filename, 'wb') as onnx_file:
                onnx_file.write(response.content)
            print('---------------fin download----------------')
        # アカウントごとに残高を取得し、CSVデータに追加
        account_count=0
        for account in self.accounts:    
            balance_wei = self.web3.eth.get_balance(account)
            balance_eth = self.web3.from_wei(balance_wei, 'ether')
            self.csv_data[account_count].append(balance_eth)
            account_count += 1
        self.inference_enabled = True
        return self.onnx_filename

    def inference(self):
        self.file_delete.wait()
        print('call inference')
        
        # self.model_path = self.get_onnx_hash()
        # print("get onnx file")
        # session = onnxruntime.InferenceSession(self.model_path) # ONNXモデルを読み込み

        if self.tagName in ["Park", "City"]:
            if self.tagName not in self.inferenceSession_dict:
                # モデルをダウンロード中は inference を無効にする
                # twist = Twist()
                # twist.linear.x = 0.0
                # twist.angular.z = 0.0
                # print('download twist', twist)
                self.file_delete.set()
                print('request ', self.tagName, ' onnx model')
                zero_vel = Twist()
                zero_vel.linear.x = 0
                zero_vel.linear.y = 0
                zero_vel.linear.z = 0
                zero_vel.angular.x = 0
                zero_vel.angular.y = 0
                zero_vel.angular.z = 0
                self._twist_pub.publish(zero_vel)
                self.onnx_filename = self.get_onnx_hash()
                # print('set onnx model, file name : ', self.onnx_filename)
                self.modelName_dict[self.tagName] = self.onnx_filename
                print("-------------dict info---------------")
                print('modelName_dict : ', self.modelName_dict)
                session= onnxruntime.InferenceSession(self.onnx_filename)
                self.inferenceSession_dict[self.tagName] = session
                print('inferenceSession_dict : ', self.inferenceSession_dict)
                print("-------------dict info---------------")
                print("reach count : ", self.reach_count)
                # self.inference_enabled = True  # モデルのダウンロードが完了したら inference を有効にする

            if not self.inference_enabled:
                print('Model download in progress. Inference is disabled.')
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
                # print('Returning zero twist:', twist)
                return twist  # モデルがダウンロード中は速度0、角速度0を返す
            # print('onnx model name : ', self.model_dict[self.tagName])
            session = self.inferenceSession_dict[self.tagName]
            # print('set onnx model : ', self.modelName_dict[self.tagName])

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
            #print("speed: ", self.results[0][0][0])
            #linear=0
            if linear <= 0:
                linear=0

            # 角速度
            angular= self.results[0][0][1]
            #print("angle: ", -self.results[0][0][1])
            #angular=0

            twist = Twist()
            twist.linear.x = 0.7*linear
            twist.angular.z = 0.7*angular
            # print('Returning twist:', twist)
                    
            return twist
        else:
            # print(f'Unsupported tag name: {self.tagName}')
            twist = Twist()
            # print('Returning zero twist:', twist)
            return twist

    def publish(self, cmd_vel):    
        # self._twist_pub.publish(cmd_vel)
        # print(cmd_vel)
        if self.inference_enabled:
            # モデルのダウンロード中は推論が無効なので、座標の更新はスキップ
            # モデルがダウンロード済みの場合だけ座標を更新する
            self._twist_pub.publish(cmd_vel)
            # print(cmd_vel)

    # CSVファイルに書き込む用の関数
    def recode_balues(self):
        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(self.csv_data)
        print("Balances updated in CSV file")

if __name__ == '__main__':
    model_directory = "/home/moriokalab/catkin_ws/src/hello/src/model_directory"
    onnx_inference = ONNXInference(model_directory)
    rospy.init_node('onnx_inference_node')
    rospy.Subscriber("tag_environment", String, onnx_inference.tag_callback)
    rospy.Subscriber("robot_distance_angle", PoseStamped, onnx_inference.pose_callback)
    rospy.Subscriber("camera_image", Image, onnx_inference.image_callback)
    rospy.Subscriber("target_reach", String, onnx_inference.reach_callback)
      
    rate = rospy.Rate(10) 
    # while not rospy.is_shutdown():
    #     cmd_vel = onnx_inference.inference()
    #     onnx_inference.publish(cmd_vel)
    #     # time.sleep(0.1)
    while not rospy.is_shutdown():
        # cmd_vel = onnx_inference.inference()
        # onnx_inference.publish(cmd_vel)
        # # time.sleep(0.1)
        cmd_vel = onnx_inference.inference()
        if cmd_vel:  # 推論の結果がある場合のみ publish
            onnx_inference.publish(cmd_vel)
        rate.sleep()
    rospy.spin()
    
    
    # 卒論終了時点の完成版