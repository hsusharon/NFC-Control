#include "PN5180.h"


class PN5180ISO15693: public PN5180{
public:
	PN5180ReturnStatus startDevice(nfc3* device);

};
