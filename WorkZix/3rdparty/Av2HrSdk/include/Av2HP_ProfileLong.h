#pragma once

#include "Av2HP_define.h"

enum ProfileLongType
{
    PLT_NA                                      ,
    PLT_LONGITUDE                               ,
    PLT_START   = PLT_LONGITUDE                   ,
    PLT_LATITUDE                                ,
    PLT_END     = PLT_LATITUDE                  ,
    PLT_ALTITUDE                                ,
    PLT_CONTROL_POINT_LONGITUDE                 ,
    PLT_CONTROL_POINT_LATITUDE                  ,
    PLT_CONTROL_POINT_ALTITUDE                  ,
    PLT_LINK_IDENTIFIER                         ,
    PLT_TRAFFIC_SIGN                            ,
    PLT_TRUCK_SPEED_LIMITS                      ,
    PLT_RESERVED_FOR_STANDARD_TYPES_10          ,
    PLT_RESERVED_FOR_STANDARD_TYPES_11          ,
    PLT_RESERVED_FOR_STANDARD_TYPES_12          ,
    PLT_RESERVED_FOR_STANDARD_TYPES_13          ,
    PLT_RESERVED_FOR_STANDARD_TYPES_14          ,
    PLT_RESERVED_FOR_STANDARD_TYPES_15          ,
    PLT_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_16   ,
    PLT_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_17   ,
    PLT_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_18   ,
    PLT_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_19   ,
    PLT_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_20   ,
    PLT_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_21   ,
    PLT_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_22   ,
    PLT_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_23   ,
    PLT_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_24   ,
    PLT_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_25   ,
    PLT_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_26   ,
    PLT_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_27   ,
    PLT_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_28   ,
    PLT_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_29   ,
    PLT_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_30   ,
    PLT_OFFSET_CORRECTION                       ,
    PLT_MAX
};

class Av2HP_ProfileLong
{
public:
    Av2HP_ProfileLong(void);
    virtual ~Av2HP_ProfileLong(void);

public:
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
    unsigned int iMessageType 	: 3;
    unsigned int iCyclicCounter : 2;
    unsigned int iRetransmission: 1;
    unsigned int iProfileType 	: 2;

    // byte1
    unsigned int iPathIndex 	: 6;
    unsigned int iControlPoint 	: 1;
    unsigned int iUpdate 	    : 1;

    // byte2 & byte3
    unsigned int iOffset 	    : 16;


    // byte4、byte5、byte6、byte7
    unsigned int iValue 	    : 32;

#else

    // byte0 & byte1
    unsigned int iMessageType 	: 3;
    unsigned int iOffset 	    : 13;

    // byte2
    unsigned int iCyclicCounter : 2;
    unsigned int iPathIndex 	: 6;

    // byte3
    unsigned int iProfileType 	: 5;
    unsigned int iControlPoint 	: 1;
    unsigned int iRetransmission: 1;
    unsigned int iUpdate 	    : 1;

    // byte4、byte5、byte6、byte7
    unsigned int iValue 	    : 32;

#endif

public:
    int iAbsOffset;
	int iAbsPathIndex;
};

class Av2HP_Longitude : public Av2HP_ProfileLong
{
public:
    Av2HP_Longitude();
    virtual ~Av2HP_Longitude();

public:
    static unsigned int s_uiCyclicCounter;

public:
    void SetCyclicCounter();
};

class Av2HP_Latitude  : public Av2HP_ProfileLong
{
public:
    Av2HP_Latitude();
    virtual ~Av2HP_Latitude();

public:
    static unsigned int s_uiCyclicCounter;

public:
    void SetCyclicCounter();
};

class Av2HP_ProfileLongOffsetCorrection : public Av2HP_ProfileLong
{
public:
    Av2HP_ProfileLongOffsetCorrection();
    virtual ~Av2HP_ProfileLongOffsetCorrection();

public:
    static unsigned int s_uiCyclicCounter;

public:
    void SetCyclicCounter();
};
