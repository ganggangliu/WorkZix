#pragma once

#include "Av2HP_define.h"

#define PS_VALUE0_INVALID                       (0x3FF)         // 1023
#define PS_VALUE1_INVALID                       (0x3FF)         // 1023

#ifdef ADASIS_NISSAN
    
    #define PS_DISTANCE1_INVALID                    (0x3FFF)    // 16383, 14 bits, MAX: 0x3FAC (16300)

#else

    #define PS_DISTANCE1_INVALID                    (0x3FF)      // 1023, 10 bits

#endif // ADASIS_NISSAN

#ifdef ADASIS_NISSAN

enum ProfileShortType
{
    PST_NA                                      ,
    PST_CURVATURE                               ,
    PST_SLOPE_LINEAR                            ,
    PST_VARIABLE_SPEED_SIGN_POSITION
};

#else

enum ProfileShortType
{
    PST_NA                                      ,
    PST_CURVATURE                               ,
    PST_ROUTE_NUMBER_TYPES                      ,
    PST_SLOPE_STEP                              ,
    PST_SLOPE_LINEAR                            ,
    PST_ROAD_ACCESSIBILITY                      ,
    PST_ROAD_CONDITION                          ,
    PST_VARIABLE_SPEED_SIGN_POSITION            ,
    PST_HEADING_CHANGE                          ,
    PST_RESERVED_FOR_STANDARD_TYPES_09          ,
    PST_RESERVED_FOR_STANDARD_TYPES_10          ,
    PST_RESERVED_FOR_STANDARD_TYPES_11          ,
    PST_RESERVED_FOR_STANDARD_TYPES_12          ,
    PST_RESERVED_FOR_STANDARD_TYPES_13          ,
    PST_RESERVED_FOR_STANDARD_TYPES_14          ,
    PST_RESERVED_FOR_STANDARD_TYPES_15          ,
    PST_STOP_LINE                               ,
    PST_START   = PST_STOP_LINE                 ,
    PST_TRAFFIC_LIGHT                           ,
    PST_WAYPOINT_PROFILE                        ,
    PST_LANE_PROFILE                            ,
    PST_REGULATION_LANEMARK,
    PST_WARNING_LANEMARK,
    PST_END = PST_WARNING_LANEMARK,
    PST_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_22   ,
    PST_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_23   ,
	PST_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_24   ,
    PST_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_25   ,
    PST_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_26   ,
    PST_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_27   ,
    PST_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_28   ,
    PST_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_29   ,
    PST_RESERVED_FOR_SYSTEM_SPECIFIC_TYPES_30   ,
    PST_OFFSET_CORRECTION                       ,
    PST_MAX
};

#endif // ADASIS_NISSAN

class Av2HP_ProfileShort
{
public:
    Av2HP_ProfileShort(void);
    virtual ~Av2HP_ProfileShort(void);

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
    unsigned int iMessageType 	    : 3;
    unsigned int iCyclicCounter 	: 2;
    unsigned int iRetransmission 	: 1;
    unsigned int iProfileType       : 2;

    // byte1 & byte2
    unsigned int iOffset 	        : 16;

    // byte3 & byte4
    unsigned int iPathIndex 	    : 6;
    unsigned int iValue0 	        : 10;

    // byte5、byte6、byte7
    unsigned int iDistance1 	    : 14;
    unsigned int iValue1 	        : 10;

#else

    // byte0 & byte1
    unsigned int iMessageType 	    : 3;
    unsigned int iOffset 	        : 13;

    // byte2
    unsigned int iCyclicCounter 	: 2;
    unsigned int iPathIndex 	    : 6;

    // byte3
    unsigned int iProfileType 	    : 5;
    unsigned int iControlPoint 	    : 1;
    unsigned int iRetransmission 	: 1;
    unsigned int iUpdate 	        : 1;

    // byte4、byte5、byte6、byte7
    unsigned int iAccuracy 	        : 2;
    unsigned int iDistance1 	    : 10;
    unsigned int iValue0 	        : 10;
    unsigned int iValue1 	        : 10;

#endif

public:
    int iAbsOffset;
	int iAbsPathIndex;
};

class Av2HP_Curvature : public Av2HP_ProfileShort
{
public:
    Av2HP_Curvature();
    virtual ~Av2HP_Curvature();

public:
    static unsigned int s_uiCyclicCounter;

public:
    void SetCyclicCounter();
};

class Av2HP_Slope : public Av2HP_ProfileShort
{
public:
    Av2HP_Slope();
    virtual ~Av2HP_Slope();

public:
    static unsigned int s_uiCyclicCounter;

public:
    void SetCyclicCounter();
};

class Av2HP_VariableSpeedSignPosition : public Av2HP_ProfileShort
{
public:
    Av2HP_VariableSpeedSignPosition();
    virtual ~Av2HP_VariableSpeedSignPosition();

public:
    static unsigned int s_uiCyclicCounter;

public:
    void SetCyclicCounter();
};

//add for GQ by lihaifeng on 20150720
class Av2HP_TrafficLight : public Av2HP_ProfileShort
{
public:
    Av2HP_TrafficLight();
    virtual ~Av2HP_TrafficLight();

public:
    static unsigned int s_uiCyclicCounter;

public:
    void SetCyclicCounter();
};

class Av2HP_HAStopLine : public Av2HP_ProfileShort
{
public:
    Av2HP_HAStopLine();
    virtual ~Av2HP_HAStopLine();

public:
    static unsigned int s_uiCyclicCounter;

public:
    void SetCyclicCounter();
};

/*k001 add by runz for LaneMark 20151212(begin)*/
class Av2HP_HAWarningLaneMark : public Av2HP_ProfileShort
{
public:
	Av2HP_HAWarningLaneMark();
	virtual ~Av2HP_HAWarningLaneMark();

public:
	static unsigned int s_uiCyclicCounter;

public:
	void SetCyclicCounter();
};

class Av2HP_HARegulationLaneMark : public Av2HP_ProfileShort
{
public:
	Av2HP_HARegulationLaneMark();
	virtual ~Av2HP_HARegulationLaneMark();

public:
	static unsigned int s_uiCyclicCounter;

public:
	void SetCyclicCounter();
};


/*k001 add by runz for LaneMark 20151212(end)*/

class Av2HP_StopLine : public Av2HP_ProfileShort
{
public:
    Av2HP_StopLine();
    virtual ~Av2HP_StopLine();

public:
    static unsigned int s_uiCyclicCounter;

public:
    void SetCyclicCounter();
};


class Av2HP_ProfileShortOffsetCorrection : public Av2HP_ProfileShort
{
public:
    Av2HP_ProfileShortOffsetCorrection();
    virtual ~Av2HP_ProfileShortOffsetCorrection();

public:
    static unsigned int s_uiCyclicCounter;

public:
    void SetCyclicCounter();
};

class Av2HP_WaypointProfile : public Av2HP_ProfileShort
{
public:
    Av2HP_WaypointProfile();
    virtual ~Av2HP_WaypointProfile();

public:
    static unsigned int s_uiCyclicCounter;

public:
    void SetCyclicCounter();
};

class Av2HP_LaneProfile : public Av2HP_ProfileShort
{
public:
    Av2HP_LaneProfile();
    virtual ~Av2HP_LaneProfile();

public:
    static unsigned int s_uiCyclicCounter;

public:
    void SetCyclicCounter();
};

