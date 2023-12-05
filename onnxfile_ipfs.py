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

        # smart contaract 
        self.web3 = Web3(Web3.HTTPProvider('http://localhost:7545'))
        self.web3.eth.default_account = self.web3.eth.accounts[4]

        self.contract_abi = '''
        [
            {
                "inputs": [],
                "name": "getHash",
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
                "inputs": [
                    {
                        "internalType": "string",
                        "name": "x",
                        "type": "string"
                    }
                ],
                "name": "sendHash",
                "outputs": [],
                "stateMutability": "nonpayable",
                "type": "function"
            }
        ]
        '''
        self.contaract_bytecode = "608060405234801561001057600080fd5b5061067c806100206000396000f3fe608060405234801561001057600080fd5b50600436106100365760003560e01c8063d13319c41461003b578063dfb2993514610059575b600080fd5b610043610075565b60405161005091906101aa565b60405180910390f35b610073600480360381019061006e9190610315565b610107565b005b6060600080546100849061038d565b80601f01602080910402602001604051908101604052809291908181526020018280546100b09061038d565b80156100fd5780601f106100d2576101008083540402835291602001916100fd565b820191906000526020600020905b8154815290600101906020018083116100e057829003601f168201915b5050505050905090565b80600090816101169190610574565b5050565b600081519050919050565b600082825260208201905092915050565b60005b83811015610154578082015181840152602081019050610139565b60008484015250505050565b6000601f19601f8301169050919050565b600061017c8261011a565b6101868185610125565b9350610196818560208601610136565b61019f81610160565b840191505092915050565b600060208201905081810360008301526101c48184610171565b905092915050565b6000604051905090565b600080fd5b600080fd5b600080fd5b600080fd5b7f4e487b7100000000000000000000000000000000000000000000000000000000600052604160045260246000fd5b61022282610160565b810181811067ffffffffffffffff82111715610241576102406101ea565b5b80604052505050565b60006102546101cc565b90506102608282610219565b919050565b600067ffffffffffffffff8211156102805761027f6101ea565b5b61028982610160565b9050602081019050919050565b82818337600083830152505050565b60006102b86102b384610265565b61024a565b9050828152602081018484840111156102d4576102d36101e5565b5b6102df848285610296565b509392505050565b600082601f8301126102fc576102fb6101e0565b5b813561030c8482602086016102a5565b91505092915050565b60006020828403121561032b5761032a6101d6565b5b600082013567ffffffffffffffff811115610349576103486101db565b5b610355848285016102e7565b91505092915050565b7f4e487b7100000000000000000000000000000000000000000000000000000000600052602260045260246000fd5b600060028204905060018216806103a557607f821691505b6020821081036103b8576103b761035e565b5b50919050565b60008190508160005260206000209050919050565b60006020601f8301049050919050565b600082821b905092915050565b6000600883026104207fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff826103e3565b61042a86836103e3565b95508019841693508086168417925050509392505050565b6000819050919050565b6000819050919050565b600061047161046c61046784610442565b61044c565b610442565b9050919050565b6000819050919050565b61048b83610456565b61049f61049782610478565b8484546103f0565b825550505050565b600090565b6104b46104a7565b6104bf818484610482565b505050565b5b818110156104e3576104d86000826104ac565b6001810190506104c5565b5050565b601f821115610528576104f9816103be565b610502846103d3565b81016020851015610511578190505b61052561051d856103d3565b8301826104c4565b50505b505050565b600082821c905092915050565b600061054b6000198460080261052d565b1980831691505092915050565b6000610564838361053a565b9150826002028217905092915050565b61057d8261011a565b67ffffffffffffffff811115610596576105956101ea565b5b6105a0825461038d565b6105ab8282856104e7565b600060209050601f8311600181146105de57600084156105cc578287015190505b6105d68582610558565b86555061063e565b601f1984166105ec866103be565b60005b82811015610614578489015182556001820191506020850194506020810190506105ef565b86831015610631578489015161062d601f89168261053a565b8355505b6001600288020188555050505b50505050505056fea2646970667358221220a65a707112563a00e777fea0a151acaf6780abf2db445b47abbe7a1df26197e264736f6c63430008120033"
        # スマートコントラクトのイニシャライザーの呼び出し
        
        ipfs_hash = "QmdTHBrv81qo7i68BVQZjW66yRC6gQgLMvB4PETbCWFuDv"
        self.contract_address = None  # デプロイ後にコントラクトアドレスを設定
        self.initialize_smart_contract()
        self.contract_address = self.initialize_smart_contract()
        # IPFSハッシュとスマートコントラクトアドレスを設定
        
        self.store_ipfs_hash_in_contract(ipfs_hash)
        print("IPFS hash stored in contract")
        print('ipfs hash : ', ipfs_hash)
        self.model_path = self.get_onnx_hash()
        print("get onnx file")

    
    def display_image(self):
        
        if self.camera_image is not None:
            # ウィンドウサイズに合わせて画像をリサイズ
            window_name = "Camera"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.imshow(window_name, self.camera_image)
            cv2.waitKey(1)
            
    def image_callback(self, image_msg):
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
        print('---------------------tag receive-------------------')
        self.tagName = tag.data
        print(type(self.tagName), self.tagName)
        #self.tagName = f'{tagString}'
        print()
        return self.tagName
    
    def pose_callback(self, pose_msg):
        #print('dis and angle subscribe')
        # 受け取ったPoseStampedメッセージから距離と角度を抽出
        self.agent_distance = pose_msg.pose.position.x
        self.agent_angle = pose_msg.pose.position.y
        #print("distance : ", self.agent_distance, " angle : ", self.agent_angle)
        
    # コントラクトのデプロイとスマコンのイニシャライズ
    def initialize_smart_contract(self):
        contract = self.web3.eth.contract(abi=self.contract_abi, bytecode=self.contaract_bytecode)
        tx_hash = contract.constructor().transact()
        tx_receipt = self.web3.eth.wait_for_transaction_receipt(tx_hash)
        return tx_receipt.contractAddress
    
    # スマートコントラクトにIPFSハッシュを格納
    def store_ipfs_hash_in_contract(self, ipfs_hash):
        contract = self.web3.eth.contract(address=self.contract_address, abi=self.contract_abi)
        tx_hash = contract.functions.sendHash(ipfs_hash).transact()
        self.web3.eth.wait_for_transaction_receipt(tx_hash)

    def get_onnx_hash(self):
        contract = self.web3.eth.contract(address=self.contract_address, abi=self.contract_abi)
        onnx_hash = contract.functions.getHash().call()
        ipfs_url = f'https://ipfs.io/ipfs/{onnx_hash}'
        response = requests.get(ipfs_url)
        self.onnx_filename = 'downloaded_park_model.onnx'
        #print('receive onnx file')
        with open(self.onnx_filename, 'wb') as onnx_file:
            onnx_file.write(response.content)
        return self.onnx_filename
        

    def inference(self):
        
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
    rospy.Subscriber("robot_distance_angle", PoseStamped, onnx_inference.pose_callback)
    rospy.Subscriber("camera_image", Image, onnx_inference.image_callback)
    rospy.Subscriber("tag_environment", String, onnx_inference.tag_callback)
    
    rospy.init_node('onnx_inference_node')
    
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        cmd_vel = onnx_inference.inference()
        onnx_inference.publish(cmd_vel)
        time.sleep(0.1)
    rospy.spin()


