#pragma once

#include "Av2HP_define.h"

#define POSITION_AGE_NA                 (511)
#define POSITION_SPEED_NA               (511)
#define POSITION_RELATIVE_HEADING_NA    (255)
#define POSITION_PROBABILITY_NA         (31)
#define POSITION_CONFIDENCE_NA          (7)

enum Av2HP_Position_Index
{
    Av2HP_PI_0,
    Av2HP_PI_1,
    Av2HP_PI_2,
    Av2HP_PI_3
};

class Av2HP_Position
{
public:
	Av2HP_Position(void);
	~Av2HP_Position(void);

public:
    static unsigned int s_uiCyclicCounter;

public:
    void SetCyclicCounter();

public:
    void ToBuf(CAN_MESSAGE& canMessage);
    void FromBuf(CAN_MESSAGE& canMessage);
    void ToString(kn_string& str);

public:
    
#ifdef ADASIS_NISSAN

    // byte0
    unsigned int iMessageType		: 3;
    unsigned int iCyclicCounter		: 2;
    unsigned int iPositionIndex		: 2;
    unsigned int iReserved1			: 1;

    // byte1 & byte2
    unsigned int iOffset			: 16;

    // byte3 & byte4 & byte5
    unsigned int iPathIndex			: 6;
    unsigned int iPositionAge		: 9;
    unsigned int iSpeed				: 9;

    // byte6
    unsigned int iCurrentLane		: 4;
    unsigned int iReserved2			: 4;

    // byte7
    unsigned int iProbability		: 5;
    unsigned int iPositionConfidence: 3;

    //unsigned int iRelativeHeading	: 8;

#else

    // byte0 & byte1
    unsigned int iMessageType		: 3;
    unsigned int iOffset			: 13;

    // byte2
    unsigned int iCyclicCounter		: 2;
    unsigned int iPathIndex			: 6;

    // byte3 & byte4
    unsigned int iPositionIndex		: 2;
    unsigned int iProbability		: 5;
    unsigned int iPositionAge		: 9;

    // byte5 & byte6
    unsigned int iReserved			: 1;
    unsigned int iPositionConfidence: 3;
    unsigned int iCurrentLane		: 3;
    unsigned int iSpeed				: 9;

    // byte7
    unsigned int iRelativeHeading	: 8;

#endif

public:
    int iAbsOffset;
	int iAbsPathIndex;
};
