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


    function getHash(string memory _hashName) public payable returns (string memory) {
        IpfsData storage data = ipfsData[_hashName];
        return data.ipfsHash;
    }
}
