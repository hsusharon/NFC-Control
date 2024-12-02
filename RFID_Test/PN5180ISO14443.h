#pragma once
#ifndef PN5180ISO14443_H
#define PN5180ISO14443_H

#include <iostream>
#include <Windows.h>
#include "PN5180.h"


enum TransceiveErrorCode {
	Success = 0,
	Reset_Error = 1,
	Load_Config_Error = 2,
	RF_Active_Error = 3
};

class PN5180ISO14443 : public PN5180 {
public:
	bool initDevice(nfc3* device);
	bool endDevice(nfc3* device);
	bool resetDevice(nfc3* device);
	void readHardwareVersion(nfc3* device);
	TransceiveErrorCode activeTypeA(nfc3* device, uint8_t* data_out, uint8_t kind);

private:
	bool setupDevice(nfc3* device);
	bool loadRFConfig(nfc3* device);
	bool turnRFOn(nfc3* device);
	bool turnRFOff(nfc3* device);
};


#endif // ! PN5180ISO14443_H