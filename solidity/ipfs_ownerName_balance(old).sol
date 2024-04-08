// SPDX-License-Identifier: MIT
pragma solidity ^0.8.3;

contract IPFS {

    uint amount=0;
    event Received(address, uint);

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
        // require(bytes(_ipfsHash).length > 0, "IPFS hash must not be empty");
        ipfsData[_hashName] = IpfsData(_ipfsHash, msg.sender);
        // uint256 accountBalance = check_balance(msg.sender);
        address accountAddress = msg.sender;
        return accountAddress;
    }

    function showIpfsData(string memory _hashName) public view returns (string memory, address) {
        IpfsData memory data = ipfsData[_hashName];
        return (data.ipfsHash, data.owner);
    }

    function getHash(string memory _hashName) public payable returns (string memory) {
        uint fee = 1.0 ether;
        IpfsData storage data = ipfsData[_hashName];
        // require(data.owner != address(0), "No sender recorded for the given hash name");
        // require(msg.value > 0, "Send ETH to retrieve IPFS hash");

        require(fee <= balance[msg.sender]);
        //マップの共有主にfeeを送信
        //payable(maps[_name].owner).transfer(fee);
        (bool success, ) = payable(ipfsData[_hashName].owner).call{value: fee}("");
        require(success, "Failed to send Ethere");

        balance[msg.sender] -= fee;

        // payable(data.sender).transfer(msg.value);
        // // アカウントの残高を更新
        // balance[data.sender] -= msg.value;

        return (data.ipfsHash);
    }
}
