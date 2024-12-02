
#include <inttypes.h>
#include "Debug.h"


static const char hexChar[] = "0123456789ABCDEF";
static char hexBuffer[9];

char* formatHex(const uint8_t val) {
    hexBuffer[0] = hexChar[val >> 4];
    hexBuffer[1] = hexChar[val & 0x0f];
    hexBuffer[2] = '\0';
    return hexBuffer;
}

char* formatHex(const uint16_t val) {
    hexBuffer[0] = hexChar[(val >> 12) & 0x0f];
    hexBuffer[1] = hexChar[(val >> 8) & 0x0f];
    hexBuffer[2] = hexChar[(val >> 4) & 0x0f];
    hexBuffer[3] = hexChar[val & 0x0f];
    hexBuffer[4] = '\0';
    return hexBuffer;
}

char* formatHex(uint32_t val) {
    for (int i = 7; i >= 0; i--) {
        hexBuffer[i] = hexChar[val & 0x0f];
        val = val >> 4;
    }
    hexBuffer[8] = '\0';
    return hexBuffer;
}
