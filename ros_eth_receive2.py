#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# import onnxruntime
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

import requests
from PIL import Image
import io
from hexbytes import HexBytes


class ImageView:
    def __init__(self):
        # rospy.init_node('ros_eth_re')
        # rospy.Subscriber("tag_message", String, self.tag_callback)
        # smart contaract 
        self.web3 = Web3(Web3.HTTPProvider('http://localhost:7545'))
        self.web3.eth.default_account = self.web3.eth.accounts[3]
        
        with open("rosEth_contract_address.json", "r") as f:
            contract_data = json.load(f)
        self.tag_received = threading.Event()
        self.hash_received = threading.Event()
        self.tagName = None
        self.contract_address = contract_data["contract_address"]
        print(self.contract_address)
        rospy.loginfo("contract_address : " + self.contract_address)
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
					},
					{
						"internalType": "address",
						"name": "",
						"type": "address"
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
        self.contract = self.web3.eth.contract(address=self.contract_address, abi=self.contract_abi)
        # rospy.loginfo("contract : " + self.contract)
        print('fin init')
    
    def deploy_contract(self):
        self.contract = self.web3.eth.contract(address=self.contract_address, abi=self.contract_abi)
        print('contract : ', self.contract)
        tx_hash = self.web3.eth.send_transaction({
            'from': self.web3.eth.default_account,
            'gas': 2000000,
            'gasPrice':self.web3.to_wei('21', 'gwei'),
		})
        print("tx_haxh : ", tx_hash)
        tx_recipt = self.web3.eth.wait_for_transaction_receipt(tx_hash)
        return tx_recipt.contractAddress
    
    def tag_callback(self, tag):
        rospy.loginfo(rospy.get_caller_id()+" I heard %s",tag.data)
        self.tagName = tag.data
        # print(self.tagName, ' : ', type(self.tagName))
        rospy.loginfo(type(self.tagName))
        self.tag_received.set()
        self.show_image()

    def get_ipfs_hash(self):
        self.tag_received.wait()
        # print("start get_ipfs_hash")
        rospy.loginfo("start get_ipfs_hash")

        # コントラクトの関数呼び出しでIPFSハッシュを取得
        ipfs_hash = self.contract.functions.getHash("A").call({'from': self.web3.eth.default_account})
        print("ipfs_hash")
        # rospy.loginfo('ipfs_hash : ', ipfs_hash)
        # self.hash_received.set()
        # time.sleep(30)
        do_transact = self.contract.functions.getHash('A').transact({'from': self.web3.eth.default_account})
        # print(self.tagName)
        # ipfs_hash = self.contract.functions.getHash(self.tagName).call()
        # do_transact = self.contract.functions.getHash(self.tagName).transact()
        # rospy.loginfo("do_transaction : ", do_transact)
        print("do_contract")
        tx_receipt = self.web3.eth.wait_for_transaction_receipt(do_transact)


        # コントラクトの残高を確認
        contract_balance = self.get_contract_balance()
        print('contract balance', contract_balance)

        print("finish transaction")
        return ipfs_hash

    # コントラクトの残高を確認
    def get_contract_balance(self):
        balance_wei = self.web3.eth.get_balance(self.contract_address)
        balance_eth = self.web3.from_wei(balance_wei, 'ether')
        return balance_eth

    def show_image(self):
        print("start show_image")
        ipfs_hash = self.get_ipfs_hash()
        print(f"IPFS Hash: {ipfs_hash}")

        # IPFSから画像を取得
        ipfs_url = f'https://ipfs.io/ipfs/{ipfs_hash}'
        response = requests.get(ipfs_url)

        # 画像を表示
        if response.status_code == 200:
            image_bytes = response.content
            image = Image.open(io.BytesIO(image_bytes))
            image.show()
        else:
            print("IPFSから画像を取得できませんでした。")

if __name__ == '__main__':
    image_view = ImageView()
    rospy.init_node('ros_eth_re')
    rospy.Subscriber("tag_message", String, image_view.tag_callback)

    # contract_address = image_view.deploy_contract()
    image_view.show_image()
    # print(f"Deployed contract at address: {contract_address}")
    # while not rospy.is_shutdown():
    #     image_view.show_image()
    rospy.spin()