// RFID_Test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <windows.h>
#include "mcp2210.h"
#include "PN5180.h"
#include "PN5180ISO14443.h"
#include "PN5180ISO15693.h"
#include "Debug.h"

using namespace std;

void DisplayErrorMessage(string message) {
	std::cout << message << endl;
}

void DisplayDataLittleEndian(uint8_t* data, int len) {
	for (int i = len - 1; i > -1; i--) {
		auto output = formatHex(data[i]);
		std::cout << output << " ";
	}
	std::cout << endl;
}


int main()
{
	
	std::cout << "Program started" << endl;
	nfc3 device;
	PN5180ISO15693 com;

	auto comStatus = com.startDevice(&device);
	if (comStatus != PN5180_SUCCESS) {
		cout << "Cannot init device" << endl;
		return 0;
	}
	
	
	cout << "Program ended" << endl;
	return 0;
}


/*
std::cout << "Connection started..." << std::endl;

bool isConnected = CheckSPI();
if (!isConnected) {
	cout << "Hardware not connected. Program terminated." << endl;
	return 0;
}
void* deviceHandle = OpenSPIByIndex();
if (deviceHandle == NULL) {
	cout << "Device Handle error: Could not access chip" << endl;
	return 0;
}

//Get the current GPIO configuration
int res;
unsigned char pGpioPinDes[MCP2210_GPIO_NR];
unsigned int pdfltGpioOutput;
unsigned int pdfltGpioDir;
unsigned char prmtWkupEn;
unsigned char pintPinMd;
unsigned char pspiBusRelEn;
res = GetDevicePins(deviceHandle, pGpioPinDes, &pdfltGpioOutput, &pdfltGpioDir, &prmtWkupEn, &pintPinMd, &pspiBusRelEn);

if (res != E_SUCCESS){
	Mcp2210_Close(deviceHandle);
	DisplayErrorMessage("Cannot get device pins");
	return -1;
}

res = SetDevicePins(deviceHandle, pGpioPinDes, &pdfltGpioOutput, &pdfltGpioDir, &prmtWkupEn, &pintPinMd, &pspiBusRelEn);
if (res != E_SUCCESS) {
	Mcp2210_Close(deviceHandle);
	DisplayErrorMessage("Cannot Set device pins");
	return -1;
}

res = ResetDevice(deviceHandle, pGpioPinDes, &pdfltGpioOutput, &pdfltGpioDir, &prmtWkupEn, &pintPinMd, &pspiBusRelEn);

uint8_t txData[] = { 0x07, 0x12, 2};
uint8_t receive[3];
res = SPIGenericTransfer(deviceHandle, txData, 3, receive, 2);
uint16_t dataout;
dataout = receive[0];
dataout <<= 8;
dataout |= receive[1];
cout << "Firmware Version: " << dataout << endl;
Sleep(LONG_DELAY);

txData[1] = 0x14;
res = SPIGenericTransfer(deviceHandle, txData, 3, receive, 2);
dataout = receive[0];
dataout <<= 8;
dataout |= receive[1];
cout << "EEPROM Version: " << dataout << endl;
Sleep(LONG_DELAY);

cout << "Configure NFC ..." << endl;
res = SetRFConfig(deviceHandle, pGpioPinDes, &pdfltGpioOutput, &pdfltGpioDir, &prmtWkupEn, &pintPinMd, &pspiBusRelEn);
if (res != NFC_SUCCESS) {
	Mcp2210_Close(deviceHandle);
	DisplayErrorMessage("Reset device error - Program terminated");
	return -1;
}
Sleep(DELAY);

cout << "---------------------------------------------------" << endl;
for (int i = 0; i< 1000; i++) {
	uint8_t uid[8];
	auto uid_len = NFCReadCard_ISO15693(deviceHandle, uid);
	if (uid_len <= 0) {
		//Sleep(DELAY);
		//continue;
		cout << "No Card" << endl;
	}
	else {
		cout << "Card Detected" << endl;
	}
	/*
	cout << "Uid: ";
	DisplayDataLittleEndian(uid, uid_len);

	uint8_t blockSize, numBlocks;
	GetNFCSystemInfo(deviceHandle, uid, uid_len, &blockSize, &numBlocks);
	cout << "Block size:" << int(blockSize) << " Block num:" << int(numBlocks) << endl;

	if (blockSize <= 0 || numBlocks <= 0) {
		cout << "Card no memory ability" << endl;
		cout << "-----------------------------------------------" << endl;
		Sleep(DELAY);
		continue;
	}
	uint8_t* readBuffer = (uint8_t*) malloc ((blockSize) * sizeof(uint8_t));
	auto cardData = ReadCardData(deviceHandle, uid, uid_len,blockSize, numBlocks, readBuffer);
	cout << "-----------------------------------------------" << endl;
	Sleep(DELAY);
	*/

	// Test ISO14443 
	/*PN5180ISO14443 nfc;
	nfc3 device;
	auto isConnected = nfc.initDevice(&device);
	if (!isConnected) {
		cout << "Cannot connect to device" << endl;
		return -1;
	}

	nfc.readHardwareVersion(&device);
	cout << "-------------------------------------" << endl;
	uint8_t output[1];
	uint8_t kind = 1;
	nfc.activeTypeA(&device, output, kind);

	nfc.endDevice(&device);
	*/