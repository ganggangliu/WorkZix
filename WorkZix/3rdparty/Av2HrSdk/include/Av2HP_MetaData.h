#pragma once

#include "Av2HP_define.h"

class Av2HP_MetaData
{
public:
    Av2HP_MetaData(void);
    ~Av2HP_MetaData(void);

public:
    static unsigned int s_uiCyclicCounter;

public:
    void SetCyclicCounter();

public:
    void ToBuf(CAN_MESSAGE& canMessage)
    {
#ifdef BLOCK_HANDLER

        memcpy_s(canMessage, sizeof(canMessage), this, sizeof(*this));

#else

        canMessage[0] = (unsigned char)(iMessageType << 5) | (unsigned char)(iMapProvider << 2) | (unsigned char)(iCountryCode >> 8);
        canMessage[1] = (unsigned char)(iCountryCode & 0xFF);
        canMessage[2] = (unsigned char)(iCyclicCounter << 6) | (unsigned char)(iMajorProtocolVersion << 4) | (unsigned char)(iMinorProtocolSubVersion << 1) | (unsigned char)(iHardwareVersion >> 8);
        canMessage[3] = (unsigned char)(iHardwareVersion & 0xFF);
        canMessage[4] = (unsigned char)(iDrivingSide << 7) | (unsigned char)(iRegionCode >> 8);
        canMessage[5] = (unsigned char)(iRegionCode & 0xFF);
        canMessage[6] = (unsigned char)(iMapVersionQuarter << 6) | iMapVersionYear;
        canMessage[7] = (unsigned char)(iReserved << 5) | (unsigned char)(iSpeedUnits << 4) | iMinorProtocolVersion;

#endif
    }

    void FromBuf(CAN_MESSAGE& canMessage)
    {
#ifdef BLOCK_HANDLER

        memcpy_s(this, sizeof(*this), canMessage, sizeof(canMessage));

#else

        iMessageType                = UINT8_LEFT(canMessage[0], 3);

        iMapProvider                = UINT8_MID(canMessage[0], 2, 3);
        iCountryCode                = UINT8_RIGHT(canMessage[0], 2);
        iCountryCode                <<= 8;
        iCountryCode                |= canMessage[1];


        iCyclicCounter              = UINT8_LEFT(canMessage[2], 2);
        iMajorProtocolVersion       = UINT8_MID(canMessage[2], 4, 2);
        iMinorProtocolSubVersion    = UINT8_MID(canMessage[2], 1, 3);

        iHardwareVersion            = UINT8_RIGHT(canMessage[2], 1);
        iHardwareVersion            <<= 8;
        iHardwareVersion            |= canMessage[3];

        iDrivingSide                = UINT8_LEFT(canMessage[4], 1);

        iRegionCode                 = UINT8_RIGHT(canMessage[4], 7);
        iRegionCode                 <<= 8;
        iRegionCode                 |= canMessage[5];

        iMapVersionQuarter          = UINT8_LEFT(canMessage[6], 2);
        iMapVersionYear             = UINT8_RIGHT(canMessage[6], 6);

        iSpeedUnits                 = UINT8_MID(canMessage[7], 4, 1);

        iMinorProtocolVersion       = UINT8_RIGHT(canMessage[7], 4);

#endif
    }

public:
    // byte0 & byte1
    unsigned int iMessageType 	            : 3;
    unsigned int iMapProvider 	            : 3;
    unsigned int iCountryCode 	            : 10;

    // byte2 & byte3
    unsigned int iCyclicCounter 	        : 2;
    unsigned int iMajorProtocolVersion 	    : 2;
    unsigned int iMinorProtocolSubVersion 	: 3;
    unsigned int iHardwareVersion 	        : 9;

    // byte4 & byte5
    unsigned int iDrivingSide 	            : 1;
    unsigned int iRegionCode 	            : 15;

    // byte6
    unsigned int iMapVersionQuarter 	    : 2;
    unsigned int iMapVersionYear 	        : 6;

    // byte7
    unsigned int iReserved 	                : 3;
    unsigned int iSpeedUnits 	            : 1;
    unsigned int iMinorProtocolVersion 	    : 4;
};
