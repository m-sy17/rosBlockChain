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
        
        #self.tagName = None
        self.tag_received = threading.Event()  # タグを受信したことを通知するためのイベント
        self.tagName = None

        # smart contaract 
        self.web3 = Web3(Web3.HTTPProvider('http://localhost:7545'))
        self.web3.eth.default_account = self.web3.eth.accounts[4]
        
        with open("rosipfs_contract_address.json", "r") as f:
            contract_data = json.load(f)
        self.contract_address = contract_data["contract_address"]

        self.contract_abi = '''
		[
            {
                "inputs": [],
                "stateMutability": "nonpayable",
                "type": "constructor"
            },
            {
                "stateMutability": "payable",
                "type": "fallback"
            },
            {
                "inputs": [],
                "name": "fee",
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
                        "name": "_userAddress",
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
                        "internalType": "string",
                        "name": "name",
                        "type": "string"
                    }
                ],
                "stateMutability": "view",
                "type": "function"
            },
            {
                "inputs": [],
                "name": "ipfsHash",
                "outputs": [
                    {
                        "internalType": "string",
                        "name": "",
                        "type": "string"
                    }
                ],
                "stateMutability": "view",
                "type": "function"
            },
            {
                "inputs": [],
                "name": "owner",
                "outputs": [
                    {
                        "internalType": "address payable",
                        "name": "",
                        "type": "address"
                    }
                ],
                "stateMutability": "view",
                "type": "function"
            },
            {
                "inputs": [],
                "name": "sendFee",
                "outputs": [],
                "stateMutability": "nonpayable",
                "type": "function"
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
                        "name": "_name",
                        "type": "string"
                    },
                    {
                        "internalType": "string",
                        "name": "_x",
                        "type": "string"
                    }
                ],
                "name": "sendHash",
                "outputs": [],
                "stateMutability": "nonpayable",
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

    def tag_callback(self, tag):
        self.tagName = tag.data
        # print(self.tagName)
        self.tag_received.set()  # タグを受信したらイベントを設定

        #return self.tagName

    def pose_callback(self, pose_msg):
        #print('dis and angle subscribe')
        # 受け取ったPoseStampedメッセージから距離と角度を抽出
        self.agent_distance = pose_msg.pose.position.x
        self.agent_angle = pose_msg.pose.position.y
        #print("distance : ", self.agent_distance, " angle : ", self.agent_angle)
        
    # コントラクトのデプロイとスマコンのイニシャライズ
    # def initialize_smart_contract(self):
    #     contract = self.web3.eth.contract(abi=self.contract_abi, bytecode=self.contaract_bytecode)
    #     tx_hash = contract.constructor().transact()
    #     tx_receipt = self.web3.eth.wait_for_transaction_receipt(tx_hash)
    #     return tx_receipt.contractAddress
    
    # スマートコントラクトにIPFSハッシュを格納
    # def store_ipfs_hash_in_contract(self, ipfs_hash):
    #     contract = self.web3.eth.contract(address=self.contract_address, abi=self.contract_abi)
    #     tx_hash = contract.functions.sendHash(ipfs_hash).transact()
    #     self.web3.eth.wait_for_transaction_receipt(tx_hash)

    #def get_toriger_name(self):


    #connect contract & get onnx flie hash address
    def get_onnx_hash(self):
        self.tag_received.wait()
        contract = self.web3.eth.contract(address=self.contract_address, abi=self.contract_abi)
        transaction = {
			'from': self.web3.eth.default_account,
			'gas': 2000000,
			'gasPrice': self.web3.to_wei('21', 'gwei'),
			#'to': self.contract_address,  # コントラクトのアドレスを指定
			'to': self.web3.eth.accounts[6],
			'value': self.web3.to_wei(1, 'ether'),
		}
        print('transaction')
        tx_hash = self.web3.eth.send_transaction(transaction)
        print('tx_hash : ', tx_hash)
        tx_recipt = self.web3.eth.wait_for_transaction_recipt(tx_hash)
        print('recipt : ', tx_recipt)
        onnx_hash = contract.functions.getHash(self.tagName).call({'value': self.web3.to_wei(1, 'ether')})
        # onnx_hash = contract.functions.getHash(self.tagName).call()
        print('onnx ipfs hash check : ', onnx_hash)
        ipfs_url = f'https://ipfs.io/ipfs/{onnx_hash}'
        response = requests.get(ipfs_url)
        self.onnx_filename = '{self.tagName}_model.onnx'
        with open(self.onnx_filename, 'wb') as onnx_file:
            onnx_file.write(response.content)
        return self.onnx_filename
        

    def inference(self):
        print('call inference')
        self.model_path = self.get_onnx_hash()
        print("get onnx file")
        session = onnxruntime.InferenceSession(self.model_path) # ONNXモデルを読み込み
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
        twist.linear.x = 1.0*linear
        twist.angular.z = 1.0*angular
                
        return twist
    def publish(self, cmd_vel):    
        self._twist_pub.publish(cmd_vel)
        print(cmd_vel)

if __name__ == '__main__':
    onnx_inference = ONNXInference()
    rospy.init_node('onnx_inference_node')
    rospy.Subscriber("tag_environment", String, onnx_inference.tag_callback)
    rospy.Subscriber("robot_distance_angle", PoseStamped, onnx_inference.pose_callback)
    rospy.Subscriber("camera_image", Image, onnx_inference.image_callback)
    
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        cmd_vel = onnx_inference.inference()
        onnx_inference.publish(cmd_vel)
        # time.sleep(0.1)
    rospy.spin()


