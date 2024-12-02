#ifndef MCP2210_H
#define MCP2210_H

#include "mcp2210_dll_um.h"
#include <stdint.h>

#define DEFAULT_VID 0x4D8
#define DEFAULT_PID 0xDE
#define CHIP_SELECT 0x01
#define GP_MASK 509
#define RESET_MASK 0x2
#define SHORT_DELAY 1 // milliseconds
#define DELAY 1 // milliseconds
#define LONG_DELAY 1 // milliseconds
#define BAUD_RATE 115200
#define DUMMY 0xFF
#define NFC_ERROR -1 // error code for nfc
#define NFC_SUCCESS 0
#define DUMMY_ARRAY_SIZE 257

// check SPI connection
bool CheckSPI();

void* OpenSPIByIndex();

void CloseConnection(void* deviceHandle);

// Set Device SPI sending configuration
int SetSPIConfig(void* deviceHandle);

// Get the pins settings for the hardware
int GetDevicePins(void* deviceHandle, unsigned char* pGpioPinDes, unsigned int* pdfltGpioOutput, unsigned int* pdfltGpioDir, unsigned char* prmtWkupEn, unsigned char* pintPinMd, unsigned char* pspiBusRelEn);

// set the pins to what they are associated to on te RFID chip
int SetDevicePins(void* deviceHandle, unsigned char* pGpioPinDes, unsigned int* pdfltGpioOutput, unsigned int* pdfltGpioDir, unsigned char* prmtWkupEn, unsigned char* pintPinMd, unsigned char* pspiBusRelEn);

int TransferSPIData(void* deviceHandle, unsigned char* pdataTx, unsigned char* pdataRx, unsigned int ptxferSize);

int TransferSPIDataEx(void* deviceHandle, unsigned char* txData, unsigned char* rxData, unsigned int txSize);

// reset device
int ResetDevice(void* deviceHandle, unsigned char* pGpioPinDes, unsigned int* pdfltGpioOutput, unsigned int* pdfltGpioDir, unsigned char* prmtWkupEn, unsigned char* pintPinMd, unsigned char* pspiBusRelEn);

int SPIGenericTransfer(void* deviceHandle, uint8_t* tx_data, uint8_t tx_len, uint8_t* rx_data, uint8_t rx_len);

int SPIGenericWrite(void* deviceHandle, uint8_t* tx_data, uint8_t len);

int SPIGenericRead(void* deviceHandle, uint8_t* rx_data, uint8_t len);

void WaitForBusyStatus(void* deviceHandle);

#endif 