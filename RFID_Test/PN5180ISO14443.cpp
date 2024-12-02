#include <iostream>
#include <Windows.h>
#include "PN5180ISO14443.h"
#include "mcp2210.h"

using namespace std;

/*
bool PN5180ISO14443::initDevice(nfc3* device) {
	// check for the device
	bool isConnected = CheckSPI();
	if (!isConnected) {
		cout << "Hardware not connected. Program terminated." << endl;
		return false;
	}

	void* deviceHandle = OpenSPIByIndex();
	if (deviceHandle == NULL)
		return false;
	else
		device->deviceHandle = deviceHandle;

	auto isPinSet = setupDevice(device);
	if (!isPinSet)
		return false;

	auto isReset = resetDevice(device);
	if (!isReset)
		return false;

	return true;
}

bool PN5180ISO14443::endDevice(nfc3* device) {
	if (device->deviceHandle == NULL)
		return true;
	Mcp2210_Close(device->deviceHandle);
	return true;
}

bool PN5180ISO14443::setupDevice(nfc3* device) {
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

bool PN5180ISO14443::resetDevice(nfc3* device) {
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

void PN5180ISO14443::readHardwareVersion(nfc3* device) {
	void* deviceHandle = device -> deviceHandle;
	int res;

	uint8_t txData[] = { 0x07, 0x12, 2 };
	uint8_t receive[3];
	res = SPIGenericTransfer(deviceHandle, txData, 3, receive, 2);
	uint16_t dataout;
	dataout = receive[0];
	dataout <<= 8;
	dataout |= receive[1];
	cout << "Firmware Version: " << dataout << endl;
	Sleep(DELAY);

	txData[1] = 0x14;
	res = SPIGenericTransfer(deviceHandle, txData, 3, receive, 2);
	dataout = receive[0];
	dataout <<= 8;
	dataout |= receive[1];
	cout << "EEPROM Version: " << dataout << endl;
	Sleep(DELAY);
}

bool PN5180ISO14443::loadRFConfig(nfc3* device) {
	uint8_t cmd[] = { CMD_LOAD_RF_CONFIG, 0x00, 0x80};
	auto res = SPIGenericWrite(device->deviceHandle, cmd, 3);
	if (res != E_SUCCESS)
		return false;
	return true;
}

bool PN5180ISO14443::turnRFOn(nfc3* device) {
	uint8_t cmd[] = { CMD_RF_ON, 0x00 };
	auto res = SPIGenericWrite(device->deviceHandle, cmd, 2);
	if (res != E_SUCCESS)
		return false;
	return true;
}

bool PN5180ISO14443::turnRFOff(nfc3* device) {
	return true;
}


TransceiveErrorCode PN5180ISO14443::activeTypeA(nfc3* device, uint8_t* data_out, uint8_t kind) {
	if (!resetDevice(device))
		return Reset_Error;
	
	if (!loadRFConfig(device))
		return Load_Config_Error;

	if (turnRFOn(device))
		return RF_Active_Error;

	return Success;
}

*/
