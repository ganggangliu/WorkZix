#pragma once

#include "Av2HP_define.h"

#define STUB_TURN_ANGLE_NA          (255)
#define STUB_RIGHTOFWAY_NA          (3)

class Av2HP_Stub
{
public:
    Av2HP_Stub(void);
    ~Av2HP_Stub(void);

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
    unsigned int iMessageType 	                    : 3;                        
    unsigned int iCyclicCounter 	                : 2;                        
    unsigned int iRetransmission                    : 1;                        
    unsigned int iPartOfCalculatedRoute 	        : 2;            

    // byte1 & byte2
    unsigned int iOffset 	                        : 16;  

    // byte3 & byte4
    unsigned int iPathIndex 	                    : 6;                            
    unsigned int iSubPathIndex                      : 6; 
    unsigned int iFormOfWay 	                    : 4;   

    // byte5
    unsigned int iTurnAngle 	                    : 8;  

    // byte6
    unsigned int iNumberOfLanesInDrivingDirection 	: 4;
    unsigned int iNumberOfLanesInOppositeDirection 	: 2;
    unsigned int iRightOfWay 	                    : 2;                        

    // byte7
    unsigned int iRelativeProbability               : 5;                
    unsigned int iFunctionalRoadClass 	            : 3;                

    //unsigned int iUpdate 	                        : 1;                                
    //unsigned int iComplexIntersection 	            : 2;                
    //unsigned int iLastStubAtOffset 	                : 1;

#else

    // byte0 & byte1
    unsigned int iMessageType 	                    : 3;                        
    unsigned int iOffset 	                        : 13;  

    // byte2
    unsigned int iCyclicCounter 	                : 2;                        
    unsigned int iPathIndex 	                    : 6;                            

    // byte3
    unsigned int iSubPathIndex                      : 6; 
    unsigned int iRetransmission                    : 1;                        
    unsigned int iUpdate 	                        : 1;                                

    // byte4
    unsigned int iRelativeProbability               : 5;                
    unsigned int iFunctionalRoadClass 	            : 3;                

    // byte5
    unsigned int iPartOfCalculatedRoute 	        : 2;            
    unsigned int iComplexIntersection 	            : 2;                
    unsigned int iFormOfWay 	                    : 4;   

    // byte6
    unsigned int iTurnAngle 	                    : 8;                            

    // byte7
    unsigned int iLastStubAtOffset 	                : 1;
    unsigned int iRightOfWay 	                    : 2;                        
    unsigned int iNumberOfLanesInOppositeDirection 	: 2;
    unsigned int iNumberOfLanesInDrivingDirection 	: 3;

#endif

public:
    int iAbsOffset;
	int iAbsPathIndex;
};


class Av2HP_Stub2
{
public:
    Av2HP_Stub2(void);
    ~Av2HP_Stub2(void);

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

    // byte0
    unsigned int iMessageType 	                    : 3;                        
    unsigned int iCyclicCounter 	                : 2;                        
    unsigned int iRetransmission                    : 1;                        
    unsigned int iPartOfCalculatedRoute 	        : 2;            

    // byte1 & byte2
    unsigned int iOffset 	                        : 16;  

    // byte3 & byte4 & byte5
    unsigned int iPathIndex 	                    : 6;                            
    unsigned int iSubPathIndex                      : 6; 
    unsigned int iLaneToEnter  	                    : 10; 
    unsigned int iReserved                          : 2;

    // byte6 & byte7
    unsigned int iOffsetToEnd                       : 16;

public:
    int iAbsOffset;
	int iAbsPathIndex;
};
