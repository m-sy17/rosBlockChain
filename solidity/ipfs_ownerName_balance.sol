// SPDX-License-Identifier: MIT
pragma solidity ^0.8.3;

contract IPFS {

    uint amount=0;
    event Received(address, uint);

    // 格納する情報　ハッシュ値のアドレス　共有者
    struct IpfsData {
        string ipfsHash;
        address owner;
    }

    mapping(string => IpfsData) public ipfsData;
    mapping(address => uint256) public balance; // アカウントごとの残高を格納するマッピング

    receive() external payable {
        emit Received(msg.sender, msg.value);
        if (balance[msg.sender] == 0){
            balance[msg.sender] = msg.value;
        }
        balance[msg.sender] += msg.value;
        amount +=msg.value;
    }

    function addHash(string memory _hashName, string memory _ipfsHash) public returns (address){
        ipfsData[_hashName] = IpfsData(_ipfsHash, msg.sender);
        // 共有したアカウントのアドレスを確認する
        address accountAddress = msg.sender;
        return accountAddress;
    }


    function getHash(string memory _hashName) public payable returns (string memory) {
        uint fee = 1.0 ether;
        IpfsData storage data = ipfsData[_hashName];
        require(fee <= balance[msg.sender]);
        //マップの共有主にfeeを送信
        (bool success, ) = payable(ipfsData[_hashName].owner).call{value: fee}("");
        require(success, "Failed to send Ethere");
        balance[msg.sender] -= fee;
        // return ipfsData[_hashName].ipfsHash;
        return (data.ipfsHash);
    }
}
