#include <iostream>
#include <windows.h>
#include "mcp2210_dll_um.h"
#include "mcp2210.h"
#include "PN5180.h"
using namespace std;


bool CheckSPI() {
	int devCount = Mcp2210_GetConnectedDevCount(DEFAULT_VID, DEFAULT_PID);
	if (devCount > 0) {
		return true;
	}
	return false;
}

/*
int GetLastError() {
	return Mcp2210_GetLastError();
}*/

void* OpenSPIByIndex() {
	wchar_t* devPathsize = NULL;
	void* deviceHandle = Mcp2210_OpenByIndex(DEFAULT_VID, DEFAULT_PID, 0, devPathsize, 0); //try to open the first device, not using path
	auto res = Mcp2210_GetLastError();
	if (res != E_SUCCESS)
		return NULL;
	return deviceHandle;
}

int SetSPIConfig(void* deviceHandle) {
	unsigned int baudrate = BAUD_RATE;
	unsigned int pidleCsVal = 0x01;
	unsigned int pactiveCsVal = 0;
	unsigned int pCsToDataDly = 0;
	unsigned int pdataToCsDly = 0;
	unsigned int pdataToDataDly = 0;
	unsigned int ptxferSize = 2;
	unsigned char pspiMd = MCP2210_SPI_MODE0;
	auto res = Mcp2210_SetSpiConfig(deviceHandle,
		MCP2210_VM_CONFIG,
		&baudrate,
		&pidleCsVal,
		&pactiveCsVal,
		&pCsToDataDly,
		&pdataToCsDly,
		&pdataToDataDly,
		&ptxferSize,
		&pspiMd);
	if (res != E_SUCCESS)
		return -1;
	return E_SUCCESS;
}

int GetDevicePins(void* deviceHandle, unsigned char *pGpioPinDes, unsigned int *pdfltGpioOutput, unsigned int *pdfltGpioDir, unsigned char *prmtWkupEn, unsigned char *pintPinMd, unsigned char *pspiBusRelEn) {
	auto resCode = Mcp2210_GetGpioConfig(deviceHandle, MCP2210_VM_CONFIG, pGpioPinDes, pdfltGpioOutput, pdfltGpioDir, prmtWkupEn, pintPinMd, pspiBusRelEn);
	if (resCode != E_SUCCESS)
		return -1;
}

int SetDevicePins(void* deviceHandle, unsigned char* pGpioPinDes, unsigned int* pdfltGpioOutput, unsigned int* pdfltGpioDir, unsigned char* prmtWkupEn, unsigned char* pintPinMd, unsigned char* pspiBusRelEn) {
	pGpioPinDes[0] = MCP2210_PIN_DES_CS; // set GP0 as CS to match RFID CS
	pGpioPinDes[1] = MCP2210_PIN_DES_GPIO; // set GP1 as GPIO to match RFID rst
	pGpioPinDes[4] = MCP2210_PIN_DES_GPIO; // set GP4 as GPIO to match RFID bsy
	pGpioPinDes[5] = MCP2210_PIN_DES_GPIO;
	*pdfltGpioDir = GP_MASK;
	*pdfltGpioOutput = *pdfltGpioOutput | (RESET_MASK); // set reset line high
	//*pdfltGpioOutput = *pdfltGpioOutput | (CHIP_SELECT);
	auto res = Mcp2210_SetGpioConfig(deviceHandle, MCP2210_VM_CONFIG, pGpioPinDes, *pdfltGpioOutput, *pdfltGpioDir, *prmtWkupEn, *pintPinMd, *pspiBusRelEn);
	if (res != E_SUCCESS)
		return -1;

	res = SetSPIConfig(deviceHandle);
	if (res != E_SUCCESS)
		return -1;
	return E_SUCCESS;
}

int TransferSPIData(void* deviceHandle, unsigned char* pdataTx, unsigned char *pdataRx, unsigned int ptxferSize){
	unsigned int baudRate = BAUD_RATE;
	unsigned int csmask = 0x01;
	auto res = Mcp2210_xferSpiData(deviceHandle, pdataTx, pdataRx, &baudRate, &ptxferSize, csmask);
	if (res != E_SUCCESS)
		return -1;
}

int TransferSPIDataEx(void* deviceHandle, unsigned char* txData, unsigned char* rxData, unsigned int txSize) {
	unsigned int csmask = 0x01;
	unsigned int baudRate = BAUD_RATE;
	unsigned int pidleCsVal = 1;
	unsigned int pactiveCsVal = 0;
	unsigned int pCsToDataDly = 0;
	unsigned int pDataTocsDly = 0;
	unsigned int pDataToDataDly = 0;
	unsigned char pspiMd = MCP2210_SPI_MODE0;
	auto res = Mcp2210_xferSpiDataEx(deviceHandle, txData, rxData, &baudRate,
		&txSize, csmask, &pidleCsVal, &pactiveCsVal, &pCsToDataDly, &pDataTocsDly, &pDataToDataDly, &pspiMd);
	WaitForBusyStatus(deviceHandle);
	if (res != E_SUCCESS)
		return -1;
	return 0;
}

int ResetDevice(void* deviceHandle, unsigned char* pGpioPinDes, unsigned int* pdfltGpioOutput, unsigned int* pdfltGpioDir, unsigned char* prmtWkupEn, unsigned char* pintPinMd, unsigned char* pspiBusRelEn) {
	// set pin low
	*pdfltGpioOutput = *pdfltGpioOutput & (~RESET_MASK);
	auto res = Mcp2210_SetGpioConfig(deviceHandle, MCP2210_VM_CONFIG, pGpioPinDes, *pdfltGpioOutput, *pdfltGpioDir, *prmtWkupEn, *pintPinMd, *pspiBusRelEn);
	// set pin high
	*pdfltGpioOutput = *pdfltGpioOutput | (RESET_MASK);
	res = Mcp2210_SetGpioConfig(deviceHandle, MCP2210_VM_CONFIG, pGpioPinDes, *pdfltGpioOutput, *pdfltGpioDir, *prmtWkupEn, *pintPinMd, *pspiBusRelEn);
	if (res != E_SUCCESS) {
		return -1;
	}
	
	// get IRQ status to insure the state of the device
	uint32_t irq_status;
	do {
		irq_status = GetIRQStatus(deviceHandle);
		Sleep(SHORT_DELAY);
	} while ((irq_status & IDLE_IRQ_STAT) == 0);

	ClearIRQStatus(deviceHandle, 0xFF, 0xFF, 0xFF, 0xFF);
	return E_SUCCESS;
}

void WaitForBusyStatus(void* deviceHandle) {
	unsigned int pinVal;
	auto res = Mcp2210_GetGpioPinVal(deviceHandle, &pinVal);
	int gpo4Status = (pinVal >> 4) % 2;
	while (res == E_SUCCESS && (pinVal >> 4) % 2) {
		Sleep(SHORT_DELAY);
		auto res = Mcp2210_GetGpioPinVal(deviceHandle, &pinVal);
		if (res != E_SUCCESS)
			return;
	}
}

int SelectBoard(void* deviceHandle, unsigned char* pGpioPinDes, unsigned int* pdfltGpioOutput, unsigned int* pdfltGpioDir, unsigned char* prmtWkupEn, unsigned char* pintPinMd, unsigned char* pspiBusRelEn) {
	*pdfltGpioOutput = *pdfltGpioOutput | (CHIP_SELECT);
	auto res = Mcp2210_SetGpioConfig(deviceHandle, MCP2210_VM_CONFIG, pGpioPinDes, *pdfltGpioOutput, *pdfltGpioDir, *prmtWkupEn, *pintPinMd, *pspiBusRelEn);
	if (res != E_SUCCESS) {
		return -1;
	}
	return E_SUCCESS;
}

int DeselectBoard(void* deviceHandle, unsigned char* pGpioPinDes, unsigned int* pdfltGpioOutput, unsigned int* pdfltGpioDir, unsigned char* prmtWkupEn, unsigned char* pintPinMd, unsigned char* pspiBusRelEn) {
	*pdfltGpioOutput = *pdfltGpioOutput & (~CHIP_SELECT);
	auto res = Mcp2210_SetGpioConfig(deviceHandle, MCP2210_VM_CONFIG, pGpioPinDes, *pdfltGpioOutput, *pdfltGpioDir, *prmtWkupEn, *pintPinMd, *pspiBusRelEn);
	if (res != E_SUCCESS) {
		return -1;
	}
	return E_SUCCESS;
}

int SPIGenericWrite(void* deviceHandle, uint8_t* tx_data, uint8_t len) {
	int res;
	uint8_t* rx_data = (uint8_t*)malloc(DUMMY_ARRAY_SIZE * sizeof(uint8_t));
	res = TransferSPIDataEx(deviceHandle, tx_data, rx_data, len);
	if (res != E_SUCCESS) {
		return -1;
	}
	//WaitForBusyStatus(deviceHandle);
	free(rx_data);
	rx_data = NULL;
	return E_SUCCESS;
}

int SPIGenericRead(void* deviceHandle, uint8_t* rx_data, uint8_t len) {
	int res;
	uint8_t* tx_data = (uint8_t*)malloc(DUMMY_ARRAY_SIZE * sizeof(uint8_t));
	for (int count = 0; count < len; count++) {
		tx_data[count] = (uint8_t)DUMMY;
	}
	res = TransferSPIDataEx(deviceHandle, tx_data, rx_data, len);
	if (res != E_SUCCESS) {
		return -1;
	}
	//WaitForBusyStatus(deviceHandle);
	free(tx_data);
	tx_data = NULL;
	return E_SUCCESS;
}

int SPIGenericTransfer(void * deviceHandle, uint8_t* tx_data, uint8_t tx_len, uint8_t* rx_data, uint8_t rx_len) {
	int res;
	res = SPIGenericWrite(deviceHandle, tx_data, tx_len);
	if (res != E_SUCCESS) {
		return -1;
	}
	
	res = SPIGenericRead(deviceHandle, rx_data, rx_len);
	if (res != E_SUCCESS) {
		return -1;
	}
	return E_SUCCESS;
}


void CloseConnection(void* deviceHandle) {
	Mcp2210_Close(deviceHandle);
}











