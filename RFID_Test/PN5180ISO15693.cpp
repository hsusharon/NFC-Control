#include "PN5180ISO15693.h"


PN5180ReturnStatus PN5180ISO15693::startDevice(nfc3* device) {

	initDevice(device);

	return PN5180_SUCCESS;
}


