#include <iostream>
#include <windows.h>
#include "PN5180.h"
#include "mcp2210.h"
#include "Debug.h"

using namespace std;

int LoadRFIDConfig(void* deviceHandle, uint8_t tx_config, uint8_t rx_config) {
	uint8_t cmd[] = { CMD_LOAD_RF_CONFIG, tx_config, rx_config };
	auto res = SPIGenericWrite(deviceHandle, cmd, 3);
	if (res != E_SUCCESS) {
		return NFC_ERROR;
	}
	return NFC_SUCCESS;
}

void ReadRegister(void* deviceHandle, uint8_t reg, uint32_t* output) {
	uint8_t tx_buf[] = { 0x04, reg };
	uint8_t rx_buf[4];
	SPIGenericTransfer(deviceHandle, tx_buf, 2, rx_buf, 4);
	*output = rx_buf[3];
	*output <<= 8;
	*output |= rx_buf[2];
	*output <<= 8;
	*output |= rx_buf[1];
	*output <<= 8;
	*output |= rx_buf[0];
}

int SetRegister(void* deviceHandle, uint8_t reg, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
	uint8_t tx_buf[6] = { CMD_WRITE_REGISTER, reg, byte1, byte2, byte3, byte4};
	auto res = SPIGenericWrite(deviceHandle, tx_buf, 6);
	if (res != E_SUCCESS)
		return NFC_ERROR;
	return NFC_SUCCESS;
}

uint8_t GetTransceiveState(void* deviceHandle) {
	uint32_t rfStatus;
	ReadRegister(deviceHandle, REG_RF_STATUS, &rfStatus);
	uint8_t state = ((rfStatus >> 24) & 0x07);
	return state;
}

int32_t GetIRQStatus(void* deviceHandle) {
	uint32_t output;
	ReadRegister(deviceHandle, REG_IRQ_STATUS, &output);
	return output;
}

int ClearIRQStatus(void* deviceHandle, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
	auto res = SetRegister(deviceHandle, REG_IRQ_CLEAR, byte1, byte2, byte3, byte4);
	if (res != NFC_SUCCESS)
		return NFC_ERROR;
	return NFC_SUCCESS;
}

int WriteRegisterAndMask(void* deviceHandle, uint8_t reg, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
	WaitForBusyStatus(deviceHandle);
	uint8_t tx_buf[6] = { CMD_WRITE_REGISTER_AND_MASK, reg, byte1, byte2, byte3, byte4 }; // system follows little endien
	auto res = SPIGenericWrite(deviceHandle, tx_buf, 6);
	if (res != E_SUCCESS)
		return NFC_ERROR;
	return NFC_SUCCESS;
}

int WriteRegisterOrMask(void* deviceHandle, uint8_t reg, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
	WaitForBusyStatus(deviceHandle);
	uint8_t tx_buf[6] = { CMD_WRITE_REGISTER_OR_MASK, reg, byte1, byte2, byte3, byte4 }; // system follows little endien
	auto res = SPIGenericWrite(deviceHandle, tx_buf, 6);
	if (res != E_SUCCESS)
		return NFC_ERROR;
	return NFC_SUCCESS;
}

int SendData(void* deviceHandle, uint8_t* cmd, uint8_t cmd_len) {
	uint8_t* tx_cmd = (uint8_t*)malloc((cmd_len + 2) * sizeof(uint8_t));
	tx_cmd[0] = CMD_SEND_DATA;
	tx_cmd[1] = 0x00;
	for (int i = 0; i < cmd_len; i++)
		tx_cmd[i + 2] = cmd[i];


	WriteRegisterAndMask(deviceHandle, REG_SYSTEM_CONFIG, 0xF8, 0xFF, 0xFF, 0xF0);
	WriteRegisterOrMask(deviceHandle, REG_SYSTEM_CONFIG, 0x03, 0x00, 0x00, 0x00);

	auto irq_status = GetIRQStatus(deviceHandle);
	auto tx_state = GetTransceiveState(deviceHandle);
	if (PN5180_TS_WaitTransmit != PN5180TransceiveStat(tx_state))
		return 0;

	auto res = SPIGenericWrite(deviceHandle, tx_cmd, cmd_len+2);
	if (res != E_SUCCESS)
		return NFC_ERROR;
	free(tx_cmd);
	tx_cmd = NULL;
	return NFC_SUCCESS;
}


int TurnRFOn(void* deviceHandle) {
	uint8_t senseCmd[] = { CMD_WRITE_EEPROM, 0x3B, 0xFF };
	auto res = SPIGenericWrite(deviceHandle, senseCmd, 3);

	uint8_t tx_data[] = { CMD_RF_ON, 0x00 };
	res = SPIGenericWrite(deviceHandle, tx_data, 2);
	if (res != E_SUCCESS) {
		return NFC_ERROR;
	}

	uint32_t irqStat;
	do {
		irqStat = GetIRQStatus(deviceHandle);
	} while ((TX_RFON_IRQ_STAT & irqStat) == 0);

	ClearIRQStatus(deviceHandle, 0x00, 0x02, 0x00, 0x00);  
	return NFC_SUCCESS;
}

int LoadRFIDSettings(void* deviceHandle) {
	int res;
	// Load ISO15693 standard
	res = LoadRFIDConfig(deviceHandle, 0x0D, 0x8D);

	// Turn RF On
	res |= TurnRFOn(deviceHandle);

	// Idle & stop com cmd
	res |= WriteRegisterAndMask(deviceHandle, REG_SYSTEM_CONFIG, 0xF8, 0xFF, 0xFF, 0xFF);

	//Transceive command
	res |= WriteRegisterOrMask(deviceHandle, REG_SYSTEM_CONFIG, 0x03, 0x00, 0x00, 0x00);

	if (res != E_SUCCESS)
		return NFC_ERROR;
	return NFC_SUCCESS;
}

int SetGearNumber(void* deviceHandle, unsigned int numberOfGear) {
	uint8_t cmd[] = {CMD_WRITE_EEPROM, 0x81, numberOfGear};
	return SPIGenericWrite(deviceHandle, cmd, 3);
}

void GetRxGain(void* deviceHandle) {
	uint32_t rx_data;
	ReadRegister(deviceHandle, 0x22, &rx_data);
	cout << "RX gain output:" << rx_data << endl;
}

int SetRFConfig(void* deviceHandle, unsigned char* pGpioPinDes, unsigned int* pdfltGpioOutput, unsigned int* pdfltGpioDir, unsigned char* prmtWkupEn, unsigned char* pintPinMd, unsigned char* pspiBusRelEn) {
	int res;
	res = ResetDevice(deviceHandle, pGpioPinDes, pdfltGpioOutput, pdfltGpioDir, prmtWkupEn, pintPinMd, pspiBusRelEn);
	res |= LoadRFIDSettings(deviceHandle);

	if (res != E_SUCCESS)
		return NFC_ERROR;

	//SetGearNumber(deviceHandle, 0x09);
	//GetRxGain(deviceHandle);
	return NFC_SUCCESS;
}

int GetRxStatus(void* deviceHandle) {
	uint32_t rx_status;
	ReadRegister(deviceHandle, REG_RX_STATUS, &rx_status);
	return (uint16_t)(rx_status & 0x000001ff);
}

int ReadData(void* deviceHandle, int len, uint8_t* buffer) {
	if (len > 508)
		return NFC_ERROR;

	uint8_t cmd[] = {CMD_READ_DATA, 0x00};
	auto res = SPIGenericTransfer(deviceHandle, cmd, 2, buffer, len);
	if (res != E_SUCCESS)
		return NFC_ERROR;
	return NFC_SUCCESS;
}


int NFCReadCard_ISO15693(void* deviceHandle, uint8_t* uid) {
	uint8_t cmd[3] = {0x26, 0x01, 0x00};
	SendData(deviceHandle, cmd, 3);

	//Check if IRQ detects a card
	auto irq_status = GetIRQStatus(deviceHandle);
	if ((irq_status & RX_SOF_DET_IRQ_STAT) == 0)
		return 0;

	while ((irq_status & RX_IRQ_STAT) == 0) {
		irq_status = GetIRQStatus(deviceHandle);
	}

	//get Uid
	uint16_t uidLen = GetRxStatus(deviceHandle);
	uint8_t rx_data[10];
	auto res = ReadData(deviceHandle, uidLen, rx_data);
	if (res != E_SUCCESS)
		return 0;

	for (int i = 0; i < 8; i++) {
		uid[i] = rx_data[2 + i];
	}
	
	return uidLen-2;
}

int GetNFCSystemInfo(void* deviceHandle, uint8_t* uid, int uid_len, uint8_t* blockSize, uint8_t* numBlocks) {
	uint8_t readSystemCmd[] = { 0x22, 0x2b, 1, 2, 3, 4, 5, 6, 7, 8 };
	for (int i = 0; i < uid_len; i++)
		readSystemCmd[i + 2] = uid[i];
	SendData(deviceHandle, readSystemCmd, 10);

	auto irq_status = GetIRQStatus(deviceHandle);
	if ((irq_status & RX_SOF_DET_IRQ_STAT) == 0)
		return 0;
	while ((irq_status & RX_IRQ_STAT) == 0) {
		irq_status = GetIRQStatus(deviceHandle);
	}

	uint16_t systemLen = GetRxStatus(deviceHandle);
	uint8_t readBuffer[255];
	auto res = ReadData(deviceHandle, systemLen, readBuffer);
	if (res != E_SUCCESS)
		return 0;
	uint8_t* p = &readBuffer[10];
	uint8_t infoFlags = readBuffer[1];

	if (infoFlags & 0x01) {   // DSFID flag
		*p++;
	}
	if (infoFlags & 0x02) {   // AFI flag
		*p++;
	}

	if (infoFlags & 0x04) {
		*numBlocks = *p++;
		*blockSize = *p++;
		*blockSize = (*blockSize) & 0x1f;
		*blockSize = *blockSize + 1; // range: 1-32
		*numBlocks = *numBlocks + 1; // range: 1-256
	}
	return NFC_SUCCESS;
}

int ReadSingleBlock(void* deviceHandle, uint8_t* uid, uint8_t uid_len, uint8_t blockNum, uint8_t blockSize, uint8_t* readBuffer) {
	uint8_t cmdReadSingleBlock[] = {0x22,0x20, 1,2,3,4,5,6,7,8, blockNum};
	for (int i = 0; i < uid_len; i++) 
		cmdReadSingleBlock[i + 2] = uid[i];

	SendData(deviceHandle, cmdReadSingleBlock, uid_len+3);

	auto irq_status = GetIRQStatus(deviceHandle);
	if ((irq_status & RX_SOF_DET_IRQ_STAT) == 0)
		return 0;

	uint16_t dataLen = GetRxStatus(deviceHandle);
	auto res = ReadData(deviceHandle, dataLen, readBuffer);
	if (res != E_SUCCESS)
		return 0;
	return 1;

}

int ReadCardData(void* deviceHandle, uint8_t* uid, uint8_t uid_len, uint8_t blockSize, uint8_t numBlocks, uint8_t* readBuffer) {
	for (int i = 0; i < numBlocks; i++) {
		auto res = ReadSingleBlock(deviceHandle, uid, uid_len, i, blockSize, readBuffer);
		cout << "Block #" << i << ": ";
		for (int j=0; j < blockSize+1; j++) {
			cout << static_cast<char>(readBuffer[j]) << " ";
		}
		cout << endl;
	}
	return 1;
}

bool PN5180::setupDevice(nfc3* device) {
	try {
		int res;
		res = GetDevicePins(device->deviceHandle,
			device->pGpioPinDes,
			&(device->pdfltGpioOutput),
			&(device->pdfltGpioDir),
			&(device->prmtWkupEn),
			&(device->pintPinMd),
			&(device->pspiBusRelEn));
		if (res != E_SUCCESS)
			return false;

		res = SetDevicePins(device->deviceHandle,
			device->pGpioPinDes,
			&(device->pdfltGpioOutput),
			&(device->pdfltGpioDir),
			&(device->prmtWkupEn),
			&(device->pintPinMd),
			&(device->pspiBusRelEn));
		if (res != E_SUCCESS)
			return false;
		return true;
	}
	catch (string msg) {
		throw PN5180_INIT_ERROR;
	}

}

bool PN5180::resetDevice(nfc3* device) {
	try {
		auto res = ResetDevice(device->deviceHandle,
			device->pGpioPinDes,
			&(device->pdfltGpioOutput),
			&(device->pdfltGpioDir),
			&(device->prmtWkupEn),
			&(device->pintPinMd),
			&(device->pspiBusRelEn));
		if (res != E_SUCCESS)
			return false;
		return true;
	}
	catch (string msg) {
		throw PN5180_INIT_ERROR;
	}
}

PN5180ReturnStatus PN5180::initDevice(nfc3* device) {
	try {
		bool isConnected = CheckSPI();
		if (!isConnected)
			return PN5180_DEVICE_NOT_FOUND;

		void* deviceHandle = OpenSPIByIndex();
		if (deviceHandle == NULL)
			return PN5180_DEVICE_NOT_FOUND;
		else
			device->deviceHandle = deviceHandle;

		bool isPinSet = setupDevice(device);
		if (!isPinSet)
			return PN5180_INIT_ERROR;

		bool isReset = resetDevice(device);
		if (!isReset)
			return PN5180_RESET_ERROR;
		return PN5180_SUCCESS;
	}
	catch (string msg){
		return PN5180_INIT_ERROR;
	}
	
}

PN5180ReturnStatus PN5180::endDevice(nfc3* device) {

	return PN5180_SUCCESS;
}

PN5180ReturnStatus PN5180::checkHardwareVersion(nfc3* device) {
	void* deviceHandle = device->deviceHandle;
	int res;
	uint8_t hardwareVersion[3];
	uint8_t txData[] = { 0x07, 0x12, 2 };
	res = SPIGenericTransfer(deviceHandle, txData, 3, hardwareVersion, 2);
	uint16_t dataout;
	dataout = hardwareVersion[0];
	dataout <<= 8;
	dataout |= hardwareVersion[1];
	cout << "Hardware Version:" << dataout << endl;
	if (dataout >= 10)
		return PN5180_VERSION_ERROR;

	txData[1] = 0x14;
	res = SPIGenericTransfer(deviceHandle, txData, 3, hardwareVersion, 2);
	dataout = hardwareVersion[0];
	dataout <<= 8;
	dataout |= hardwareVersion[1];
	cout << "EEPROM Version:" << dataout << endl;
	if (dataout >= 1000)
		return PN5180_VERSION_ERROR;
	
	return PN5180_SUCCESS;
}

PN5180ReturnStatus PN5180::getIRQStatus(nfc3* device) {

	return PN5180_SUCCESS;
}

PN5180ReturnStatus PN5180::clearIRQStatus(nfc3* device) {

	return PN5180_SUCCESS;
}

PN5180ReturnStatus PN5180::sendData(nfc3* device) {

	return PN5180_SUCCESS;
}

PN5180ReturnStatus PN5180::readData(nfc3* device) {
	return PN5180_SUCCESS;
}


