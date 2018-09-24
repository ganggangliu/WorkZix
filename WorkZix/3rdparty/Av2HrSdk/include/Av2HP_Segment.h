#pragma once

#include "Av2HP_define.h"

class Av2HP_Segment
{
public:
    Av2HP_Segment(void);
    ~Av2HP_Segment(void);

public:
    static unsigned int s_uiCyclicCounter;

public:
    void SetCyclicCounter();

    void SetRetransmission(bool bRetransmission)
    {
        iRetransmission = bRetransmission;
    }

    bool IsRetransmission()
    {
        return iRetransmission;
    }

public:
    void ToBuf(CAN_MESSAGE& canMessage);
    void FromBuf(CAN_MESSAGE& canMessage);
    void ToString(kn_string& str);

public:

#ifdef ADASIS_NISSAN

    // byte0
    unsigned int iMessageType 				        : 3;
    unsigned int iCyclicCounter 				    : 2;
    unsigned int iFunctionalRoadClass 			    : 3;

    // byte1 & byte2
    unsigned int iOffset 				            : 16;

    // byte3
    unsigned int iPathIndex 				        : 6;
    unsigned int iTunnel 				            : 2;

    // byte4
    unsigned int iBridge 				            : 2;
    unsigned int iNoPassingArea                     : 2;
    unsigned int iFormOfWay 				        : 4;

    // byte5
    unsigned int iEffectiveSpeedLimit 			    : 5;
    unsigned int iEffectiveSpeedLimitType 		    : 3;

    // byte6
    unsigned int iNumberOfLanesInDrivingDirection 	: 4;
    unsigned int iNumberOfLanesInOppositeDirection 	: 2;
    unsigned int iPartOfCalculatedRoute 			: 2;

    // byte7
    unsigned int iRelativeProbability 			    : 5;
    unsigned int iUpdate 				            : 1;
    unsigned int iRetransmission 			        : 1;
    unsigned int iReserved 				            : 1;

    //unsigned int iDividedRoad 				        : 2;
    //unsigned int iBuiltUpArea 				        : 2;
    //unsigned int iComplexIntersection 			    : 2;

#else

    // byte0 & byte1
    unsigned int iMessageType 				        : 3;
    unsigned int iOffset 				            : 13;

    // byte2
    unsigned int iCyclicCounter 				    : 2;
    unsigned int iPathIndex 				        : 6;

    // byte3
    unsigned int iTunnel 				            : 2;
    unsigned int iBridge 				            : 2;
    unsigned int iBuiltUpArea 				        : 2;
    unsigned int iRetransmission 			        : 1;
    unsigned int iUpdate 				            : 1;

    // byte4
    unsigned int iRelativeProbability 			    : 5;
    unsigned int iFunctionalRoadClass 			    : 3;

    // byte5
    unsigned int iPartOfCalculatedRoute 			: 2;
    unsigned int iComplexIntersection 			    : 2;
    unsigned int iFormOfWay 				        : 4;

    // byte6
    unsigned int iEffectiveSpeedLimit 			    : 5;
    unsigned int iEffectiveSpeedLimitType 		    : 3;

    // byte7
    unsigned int iReserved 				            : 1;
    unsigned int iDividedRoad 				        : 2;
    unsigned int iNumberOfLanesInOppositeDirection 	: 2;
    unsigned int iNumberOfLanesInDrivingDirection 	: 3;

#endif

public:
    int iAbsOffset;
	int iAbsPathIndex;
};
