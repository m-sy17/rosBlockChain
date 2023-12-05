// SPDX-License-Identifier: MIT
//スマートコントラクトの所有者にEthを送金する処理を追加したコード
pragma solidity ^0.8.3;

contract IPFS {
    address payable public owner;  // スマートコントラクトの所有者
    uint amount; //コントラクトの残高
    string public ipfsHash;
    uint  public fee;
    
    struct IpfsData {
        string ipfsHash;
        string name;
    }

    mapping(string => IpfsData) public ipfsData;
    
    constructor() {
        owner = payable(msg.sender);  // コントラクトをデプロイしたアカウントを所有者とする
    }
    receive() external payable {}
    fallback() external payable {}

    modifier onlyOwner() {
        require(msg.sender == owner, "Only the owner can call this function");
        _;
    }

    function sendFee() public {
        fee = 1;
    }

    function sendHash(string memory _hashName, string memory _name, string memory _x) public {
        ipfsData[_hashName] = IpfsData(_x, _name);
    }
    function getHash(string memory _userAddress) public payable returns (string memory) {
        require(msg.value > 0, "Send ETH to retrieve IPFS hash");
        owner.transfer(msg.value); // ETHを所有者に転送
        return ipfsData[_userAddress].ipfsHash;
    }
}