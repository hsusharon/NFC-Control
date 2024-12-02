#ifndef PN5180_H
#define PN5180_H

#include <iostream>
#include <windows.h>

// PN5180 Commands
#define CMD_WRITE_REGISTER 0x00
#define CMD_WRITE_REGISTER_OR_MASK 0x01
#define CMD_WRITE_REGISTER_AND_MASK 0x02
#define CMD_WRITE_REGISTER_MULTIPLE 0x03
#define CMD_READ_REGISTER 0x04
#define CMD_READ_REGISTER_MULTIPLE 0x05
#define CMD_WRITE_EEPROM 0x06
#define CMD_READ_EEPROM 0x07
#define CMD_WRITE_TX_DATA 0x08
#define CMD_SEND_DATA 0x09
#define CMD_READ_DATA 0x0A
#define CMD_SWITCH_MODE 0x0B
#define CMD_LOAD_RF_CONFIG 0x11
#define CMD_RF_ON 0x16
#define CMD_RF_OFF 0x17

#define REG_SYSTEM_CONFIG 0x00
#define REG_IRQ_STATUS 0x02
#define REG_IRQ_CLEAR 0x03
#define REG_RX_STATUS 0x13
#define REG_RF_STATUS 0x1D

// PN5180 IRQ_STATUS
#define RX_IRQ_STAT         (1<<0)  // End of RF rececption IRQ
#define TX_IRQ_STAT         (1<<1)  // End of RF transmission IRQ
#define IDLE_IRQ_STAT       (1<<2)  // IDLE IRQ
#define RFOFF_DET_IRQ_STAT  (1<<6)  // RF Field OFF detection IRQ
#define RFON_DET_IRQ_STAT   (1<<7)  // RF Field ON detection IRQ
#define TX_RFOFF_IRQ_STAT   (1<<8)  // RF Field OFF in PCD IRQ
#define TX_RFON_IRQ_STAT    (1<<9)  // RF Field ON in PCD IRQ
#define RX_SOF_DET_IRQ_STAT (1<<14) // RF SOF Detection IRQ

enum PN5180TransceiveStat {
	PN5180_TS_Idle = 0,
	PN5180_TS_WaitTransmit = 1,
	PN5180_TS_Transmitting = 2,
	PN5180_TS_WaitReceive = 3,
	PN5180_TS_WaitForData = 4,
	PN5180_TS_Receiving = 5,
	PN5180_TS_LoopBack = 6,
	PN5180_TS_RESERVED = 7
};

enum PN5180ReturnStatus {
	PN5180_SUCCESS = 0,
	PN5180_GEN_ERROR = 1,
	PN5180_DEVICE_NOT_FOUND = 2,
	PN5180_INIT_ERROR = 3,
	PN5180_RESET_ERROR = 4,
	PN5180_VERSION_ERROR = 5
};

class nfc3 {
public:
	void* deviceHandle;
	unsigned char pGpioPinDes[9];
	unsigned int pdfltGpioOutput;
	unsigned int pdfltGpioDir;
	unsigned char prmtWkupEn;
	unsigned char pintPinMd;
	unsigned char pspiBusRelEn;
};

class PN5180{
public:
	PN5180ReturnStatus initDevice(nfc3* device);
	PN5180ReturnStatus endDevice(nfc3* device);
	PN5180ReturnStatus checkHardwareVersion(nfc3* device);
	PN5180ReturnStatus getIRQStatus(nfc3* device);
	PN5180ReturnStatus clearIRQStatus(nfc3* device);
	PN5180ReturnStatus sendData(nfc3* device);
	PN5180ReturnStatus readData(nfc3* device);
	PN5180ReturnStatus transceiveData(nfc3* device, uint8_t* tx_data, uint8_t tx_len, uint8_t* rx_data, uint8_t rx_len);
	PN5180ReturnStatus readRegister(nfc3* device, uint8_t reg, uint32_t data);
	PN5180ReturnStatus writeRegister(nfc3* device, uint8_t reg, uint8_t* data);
	PN5180ReturnStatus writeRegisterWithOrMask(nfc3* device, uint8_t reg, uint8_t* data);
	PN5180ReturnStatus writeRegisterwithAndMask(nfc3* device, uint8_t reg, uint8_t* data);

private:
	bool setupDevice(nfc3* device);
	bool resetDevice(nfc3* device);
};


int LoadRFIDConfig(void* deviceHandle, uint8_t tx_config, uint8_t rx_config);

void ReadRegister(void* deviceHandle, uint8_t reg, uint32_t* output);

int32_t GetIRQStatus(void* deviceHandle);

int ClearIRQStatus(void* deviceHandle, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);

int SetRFConfig(void* deviceHandle, unsigned char* pGpioPinDes, unsigned int* pdfltGpioOutput, unsigned int* pdfltGpioDir, unsigned char* prmtWkupEn, unsigned char* pintPinMd, unsigned char* pspiBusRelEn);

int NFCReadCard_ISO15693(void* deviceHandle, uint8_t* data_out);

int GetNFCSystemInfo(void* deviceHandle, uint8_t* uid, int uid_len, uint8_t* blockSize, uint8_t* numBlocks);

int ReadCardData(void* deviceHandle, uint8_t* uid, uint8_t uid_len,uint8_t blockSize, uint8_t numBlocks, uint8_t* readBuffer);

#endif