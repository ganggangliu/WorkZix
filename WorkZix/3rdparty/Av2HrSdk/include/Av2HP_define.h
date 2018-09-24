#ifndef _AV2HP_DEFINE_H_
#define _AV2HP_DEFINE_H_

#include <set>
#include <deque>
#include <string>

#ifdef WIN32

#include "windows.h"
#include <tchar.h>

#else

#endif

typedef std::wstring    kn_string;
typedef wchar_t         kn_char;

#define ADASIS_GQ
//#define ADASIS_NISSAN

#if 1/*def ADASIS_NISSAN*/

#define ADASIS_HP       // High precision map data.

#endif

#ifdef ADASIS_NISSAN

#define AREA_OPG

#endif

#define UINT8_RIGHT(value, count)       (unsigned char)(value << (8 - count) >> (8 - count))
#define UINT8_LEFT(value, count)        (unsigned char)(value >> (8 - count))
#define UINT8_MID(value, first, count)  (unsigned char)(value << (8 - (first + count)) >> (8 - count)) // first : the index from low bit (or right).

//#define BLOCK_HANDLER

#define CYCLIC_COUNTER_MODE                                 (4)
#define CYCLIC_COUNTER_MAX                                  (3)
#define FUNCTIONAL_ROAD_CLASS_NA                            (7)
#define RELATIVE_PROBABILITY_NA 		                    (31)
#define FORM_OF_WAY_NA                                      (15)

#ifndef ADASIS_NISSAN

    #define NUMBER_OF_LANES_IN_DRIVING_DIRECTION_NA         (7)

#else

    #define NUMBER_OF_LANES_IN_DRIVING_DIRECTION_NA         (0xF)

#endif

#define NUMBER_OF_LANES_IN_OPPOSITE_DIRECTION_NA            (3)
#define NUMBER_OF_LANES_INVALID                             (-1)

#define COMPLEX_INTERSECTION_NA 		                    (3)
#define PART_OF_CALCULATED_ROUTE_NA	                        (3)

#ifdef ADASIS_NISSAN

#define OFFSET_INVALID                                      (0xFFFF)    // 65535 16 bits
#define OFFSET_MODEL                                        (0xFFDD)    // 65501 MAX: 65500

#ifndef ADASIS_HP
#define OFFSET_INNER_UNIT                                   (1)         // 1m
#else
#define OFFSET_INNER_UNIT                                   (100)       // 0.01m
#endif
#define OFFSET_UNIT                                         (10)        // 0.1m

#else

#define OFFSET_INVALID                                      (0x1FFF)    // 8191  13 bits
#define OFFSET_MODEL                                        (0x1FFF)    // 8191  MAX: 8190

#ifdef ADASIS_HP

#define OFFSET_INNER_UNIT                                   (100)       // m    
#define OFFSET_UNIT                                         (1)         // m

#else

#define OFFSET_INNER_UNIT                                   (1)         // m
#define OFFSET_UNIT                                         (1)         // m

#endif

#endif // ADASIS_NISSAN

#define PATH_INDEX_UNKOWN                                   (0)
#define PATH_INDEX_POSITION_NOT_IN_DIGITIZED_AREA           (1)
#define PATH_INDEX_POSITION_NOT_ON_ROAD                     (2)
#define PATH_INDEX_POSITION_SUB_SYSTEM_NOT_CALIBRATED       (3)
#define PATH_INDEX_SEGMENT_CURRENT_VEHICLE_POSITION         (4)
#define PATH_INDEX_STUB_NOT_FURTHER_BE_APPENDED             (5)
#define PATH_INDEX_STUB_CONTINUATION                        (6)
#define PATH_INDEX_RESERVED                                 (7)

#define INIT_PATH_INDEX                                     (8)
#define MAX_PATH_INDEX                                      (64)

#define PATH_INDEX_BITS                                     (6)
#define PATH_INDEX_MASK                                     (0x3F)

#define MAX_LENGTH_TRANSMITTED_PATH                         (2000)  // 2km
#define DS_TRAILING_LENGTH                                  (100)   // 100m The distance back from the vehicle before entities can be deleted from the data store of AV2HR. This value must be consistent with the corresponding setting of the AV2HP.
#define META_DATA_MESSAGE_PERIOD                            (5000)  // 5s

#define MAX_SUB_PATH_LEVEL                                  (3)
#define MAX_PATH_DEPTH                                      (10)

#define CONTINUATION_STUB_PATH_INDEX                        (6)     // Value 6: As opposed to normal STUBs connecting side-branches to the main path, a STUB with a SUB-PATH INDEX of 6 provides information for the continuation of the main path such as TURN ANGLE and RELATIVE PROBABILITY. Most of the content of such a STUB message (such as NUMBER OF LANES) may be redundant as there is already a SEGMENT message with the same data. We will refer to this kind of stub as a continuation stub.

#define STUB_TURN_ANGLE_UNIT                                (360.0 / 254)
#define STUB_RELATIVE_PROBABILITY_UNIT                      (100.0 / 30)
#define STUB_2_ENTER_LANE_MAX                               (10)

#define POSITION_SPEED_UNIT                                 (0.2)   // m/s
#define POSITION_SPEED_MIN                                  (-12.9) // m/s
#define POSITION_SPEED_MAX                                  (89.2) // m/s
#define POSITION_SPEED_MULTIPLIER                           (10)
#define POSITION_SPEED_NA                                   (511)

#define POSITION_HEADING_UNIT                               (360.0 / 254) 
#define POSITION_HEADING_UNKNOWN                            (254) 
#define POSITION_HEADING_NA                                 (255) 

#define POSITION_PROBABILITY_UNKNOWN                        (0)
#define POSITION_PROBABILITY_NA                             (31)
#define POSITION_PROBABILITY_UNNIT                          (100.0 / 30)

#define POSITION_CONFIDENCE_NA                              (7)
#define POSITION_AGE_MAX                                    (2545)
#define POSITION_AGE_UNIT                                   (5) // ms

#define SEGMENT_TUNNEL_NA                                   (3)
#define SEGMENT_BRIDGE_NA                                   (3)
#define SEGMENT_DIVIDED_ROAD_NA                             (3)
#define SEGMENT_BUILT_UP_AREA_NA                            (3)
#define SEGMENT_NO_PASSING_AREA_UNKNOWN                     (2)
#define SEGMENT_NO_PASSING_AREA_NA                          (3)

#define META_DATA_COUNTRY_CODE_UNKNOWN                      (0)
#define META_DATA_REGION_CODE_UNKNOWN                       (0)
#define META_DATA_HARDWARE_VERSION_UNKNOWN                  (0)
#define META_DATA_MAP_VERSION_YEAR_NA                       (63)

#define LONGITUDE_MULTIPLIER                                (10000000)
#define LATITUDE_MULTIPLIER                                 (10000000)

#define DEGREE_TO_1024_SECOND_MULTIPLIER                    (3686400)   /* 1024 * 60 * 60 */

#ifdef ADASIS_GQ

#define STOP_LINE_DIS                                       (2) // 2m

#endif

#define ROAD_RIGHT_RADIUS   (20)

enum ErrorCode
{
    EC_ADASIS_OK,
    EC_ADASIS_VECHICLE_IS_NOT_ON_ROAD,
    EC_ADASIS_MAX
};

ErrorCode SetErrorCode(ErrorCode ec);
ErrorCode GetErrorCode();

enum Av2HP_Mode
{
    Av2HP_MINI,                                 // Where clients are only interested in the attributes of the current link, Av2HP could decide not to send the ADAS Horizon in the form of Paths. Instead, a SEGMENT message with a PATH INDEX of 4 describes the attributes of the link where the vehicle is currently located.
    Av2HP_MAIN_PATH_ONLY,                       // Most applications will actually only need the main path. Therefore, in many installations it will be enough if the ADASIS v2 Horizon Provider (Av2HP) sends only the main path to the client.
    Av2HP_MAIN_PATH_WITH_STUBS,                 // Horizon Provider would be the Main Path with attached stubs, which contain basic intersection information.
    Av2HP_MAIN_PATH_WITH_FIRST_LEVEL_SUB_PATHS, // If the ADAS application also needs information for sub-paths, the ADASIS v2 Horizon Provider may offer extended configuration levels, also transmitting preview information reaching into sub path structures. This is done by using "stubs" which indicate the start of a new path attached to a parent path, and which contain basic information about the attached road (e.g. turn angle, road class).
    Av2HP_FULL_HORIZON                          // If the client needs even more information and has enough memory to store it, the ADASIS v2 Horizon Provider can be configured to transmit the full available map information with higher-level sub paths. In that case, the client will never be in a "blind" state and will always have immediate map information available when the vehicle leaves a path.
};

enum Av2HP_Message_Type
{
	Av2HP_MT_SYSTEM_SPECIFIC  = 0,
	Av2HP_MT_POSITION,
	Av2HP_MT_SEGMENT,
	Av2HP_MT_STUB,
	Av2HP_MT_PROFILE_SHORT,
	Av2HP_MT_PROFILE_LONG,
	Av2HP_MT_META_DATA,
	Av2HP_MT_RESERVED
};

enum Av2HP_Path_Index
{
    Av2HP_PI_UNKNOWN,
    Av2HP_PI_POSITION_NOT_IN_DIGITIZED_AREA,
    Av2HP_PI_NOT_ON_ROAD,
    Av2HP_PI_NOT_CALIBRATED,
    Av2HP_PI_SEGMENT,
    Av2HP_PI_STUB_NOT_APPENDED,
    Av2HP_PI_STUB_HEADING_CHANGE_RELATIVE_PROBABILITY,
    Av2HP_PI_RESERVED
};

enum FormOfWay
{
    FOW_UNKNOWN,
    FOW_FREEWAY_OR_CONTROLLED_ACCESS_ROAD,                                  // Freeway or Controlled Access road that is not a slip road/ramp
    FOW_MULTIPLE_CARRIAGEWAY_OR_MULTIPLY_DIGITIZED_ROAD,                    // Multiple Carriageway or Multiply Digitized Road
    FOW_SINGLE_CARRIAGEWAY,                                                 // Single Carriageway (default)
    FOW_ROUNDABOUT_CIRCLE,                                                  // Roundabout Circle
    FOW_TRAFFIC_SQUARE_SPECIAL_TRAFFIC_FIGURE,                              // Traffic Square/Special Traffic Figure
#ifdef ADASIS_GQ
    FOW_U_TURN,                                                             // Reserved
    FOW_RIGHT_TURN,                                                         // Reserved
#else
    FOW_RESERVED_1,                                                         // Reserved
    FOW_RESERVED_2,                                                         // Reserved
#endif
    FOW_PARALLEL_ROAD_SPECIAL_SLIP_ROAD_RAMP,                               // Parallel Road (as special type of a slip road/ramp)
    FOW_SLIP_ROAD_RAMP_ON_FREEWAY_CONTROLLED_ACCESS_ROAD,                   // Slip Road/Ramp on a Freeway or Controlled Access road
    FOW_SLIP_ROAD_RAMP_NOT_ON_FREEWAY_CONTROLLED_ACCESS_ROAD,               // Slip Road/Ramp (not on a Freeway or Controlled Access road)
    FOW_SERVICE_ROAD_OR_FRONTAGE_ROAD,                                      // Service Road or Frontage Road
    FOW_ENTRANCE_EXIT_CAR_PARK,                                             // Entrance to or exit of a Car Park
    FOW_ENTRANCE_EXIT_SERVICE,                                              // Entrance to or exit to Service
    FOW_PEDESTRIAN_ZONE,                                                    // Pedestrian Zone
    FOW_NA
};

/*
Functional Road Class indicates relative importance of the road in the routing network and lower values indicate higher priority. This information is usually map-provider specific. Regular values are 1-6; value 0 is unknown, value 7 indicates N/A.
*/
enum FunctionalRoadClass
{
    FRC_UNKNOWN,
    FRC_1,
    FRC_2,
    FRC_3,
    FRC_4,
    FRC_5,
    FRC_6,
    FRC_NA
};

enum EffectiveSpeedLimit
{
    ESL_UNKNOWN,
    ESL_5,
    ESL_7,
    ESL_10,
    ESL_15,
    ESL_20,
    ESL_25,
    ESL_30,
    ESL_35,
    ESL_40,
    ESL_45,
    ESL_50,
    ESL_55,
    ESL_60,
    ESL_65,
    ESL_70,
    ESL_75,
    ESL_80,
    ESL_85,
    ESL_90,
    ESL_95,
    ESL_100,
    ESL_105,
    ESL_110,
    ESL_115,
    ESL_120,
    ESL_130,
    ESL_140,
    ESL_150,
    ESL_160,
    ESL_UNLIMITED,
    ESL_INVALID
};

enum EffectiveSpeedLimitType
{
    ESLT_IMPLICIT,
    ESLT_EXPLICIT_ON_TRAFFIC_SIGN,
    ESLT_EXPLICIT_BY_NIGHT,
    ESLT_EXPLICIT_BY_DAY,
    ESLT_EXPLICIT_TIME_OF_DAY,
    ESLT_EXPLICIT_RAIN,
    ESLT_EXPLICIT_SNOW,
    ESLT_UNKNOWN,
};

#ifndef ADASIS_NISSAN

enum CurrentLane 
{
    CL_UNKNOWN,
    CL_EMERGENCY_LANE,
    CL_SINGLE_LANE_ROAD,
    CL_LEFT_MOST_LANE,
    CL_RIGHT_MOST_LANE,
    CL_MIDDLE_LANE,                 // One of middle lanes on road with three or more lanes
    CL_RESERVED,
    CL_NA,
    CL_MAX
};

#else

enum CurrentLane
{
    CL_UNKNOWN,
    CL_EMERGENCY_LANE,
    CL_SINGLE_LANE_ROAD,
    CL_LEFT_MOST_LANE,
    CL_RIGHT_MOST_LANE,
    CL_2ND_LANE,
    CL_3ND_LANE,
    CL_4TH_LANE,
    CL_5TH_LANE,
    CL_6TH_LANE,
    CL_7TH_LANE,
    CL_8TH_LANE,
    CL_9TH_LANE,
    CL_10TH_LANE,
    CL_RESERVED1,
    CL_RESERVED2,
    CL_MAX
};

#endif

enum CANMessageID
{
#ifdef ADASIS_NISSAN

    CAN_MESSAGE_ID_POSITION         = 430,
    CAN_MESSAGE_ID_SEGMENT          = 431,
    CAN_MESSAGE_ID_STUB             = 432,
    CAN_MESSAGE_ID_STUB_2           = 433,
    CAN_MESSAGE_ID_PROFILE_SHORT    = 444,
    CAN_MESSAGE_ID_PROFILE_LONG     = 445,
    CAN_MESSAGE_ID_META_DATA        = 450

#else

#if 0
    CAN_MESSAGE_ID_POSITION         = 501,
    CAN_MESSAGE_ID_SEGMENT          = 502,
    CAN_MESSAGE_ID_STUB             = 503,
    CAN_MESSAGE_ID_PROFILE_SHORT    = 504,
    CAN_MESSAGE_ID_PROFILE_LONG     = 505,
    CAN_MESSAGE_ID_META_DATA        = 506
#endif

    CAN_MESSAGE_ID_POSITION         = 0x210,
    CAN_MESSAGE_ID_SEGMENT          = 0x220,
    CAN_MESSAGE_ID_STUB             = 0x230,
    CAN_MESSAGE_ID_PROFILE_SHORT    = 0x240,
    CAN_MESSAGE_ID_PROFILE_LONG     = 0x250,
    CAN_MESSAGE_ID_META_DATA        = 0x260

#endif
};

enum VariableSpeedSignPosition
{
    VSSP_UNKNOWN,
    VSSP_ROAD_LEFT,
    VSSP_ROAD_RIGHT,
    VSSP_ABOVE_ROAD,
    VSSP_ABOVE_EMERGENCY,
    VSSP_1ST_LANE,
    VSSP_2ND_LANE,
    VSSP_3RD_LANE,
    VSSP_4TH_LANE,
    VSSP_5TH_LANE
};

//add for GQ by lihaifeng on 20150720
enum HTrafficLightPosition
{
    HTLP_UNKNOWN,
    HTLP_ROAD_LEFT,
    HTLP_ROAD_RIGHT,
    HTLP_ABOVE_ROAD,
    HTLP_ABOVE_EMERGENCY,
    HTLP_1ST_LANE,
    HTLP_2ND_LANE,
    HTLP_3RD_LANE,
    HTLP_4TH_LANE,
    HTLP_5TH_LANE
};

/*k001 add by runz for laneMark 20151212(begin)*/
enum HWarningPosition
{
	WARNING_UNKNOWN,
	WARNING_ROAD_LEFT,
	WARNING_ROAD_RIGHT,
	WARNING_ABOVE_ROAD,
	WARNING_ABOVE_EMERGENCY,
	WARNING_1ST_LANE,
	WARNING_2ND_LANE,
	WARNING_3RD_LANE,
	WARNING_4TH_LANE,
	WARNING_5TH_LANE,
	WARNING_6TH_LANE,
	WARNING_7TH_LANE,
	WARNING_8TH_LANE,
	WARNING_9TH_LANE,
	WARNING_10TH_LANE,
	WARNING_11TH_LANE,
	WARNING_12TH_LANE,
	WARNING_13TH_LANE,
	WARNING_14TH_LANE,
	WARNING_15TH_LANE,
	WARNING_16TH_LANE,
	WARNING_17TH_LANE,
	WARNING_18TH_LANE,
	WARNING_19TH_LANE,
	WARNING_20TH_LANE,
	WARNING_21TH_LANE,
	WARNING_22TH_LANE,
	WARNING_23TH_LANE,
	WARNING_NA
};

enum HRegulationPosition
{
	REGULATION_UNKNOWN,
	REGULATION_ROAD_LEFT,
	REGULATION_ROAD_RIGHT,
	REGULATION_ABOVE_ROAD,
	REGULATION_ABOVE_EMERGENCY,
	REGULATION_1ST_LANE,
	REGULATION_2ND_LANE,
	REGULATION_3RD_LANE,
	REGULATION_4TH_LANE,
	REGULATION_5TH_LANE,
	REGULATION_6TH_LANE,
	REGULATION_7TH_LANE,
	REGULATION_8TH_LANE,
	REGULATION_9TH_LANE,
	REGULATION_10TH_LANE,
	REGULATION_11TH_LANE,
	REGULATION_12TH_LANE,
	REGULATION_13TH_LANE,
	REGULATION_14TH_LANE,
	REGULATION_15TH_LANE,
	REGULATION_16TH_LANE,
	REGULATION_17TH_LANE,
	REGULATION_18TH_LANE,
	REGULATION_19TH_LANE,
	REGULATION_20TH_LANE,
	REGULATION_21TH_LANE,
	REGULATION_22TH_LANE,
	REGULATION_23TH_LANE,
	REGULATION_NA
};
/*k001 add by runz for laneMark 20151212(end)*/

enum SegmentProcessCase
{
	SPC_GENERATE_SEGMENT_MESSAGE,
	SPC_SHAPE_POINT,
	SPC_CURVATURE,
	SPC_SLOPE,
	SPC_VARIABLE_SPEED_SIGN_POSITION,
	SPC_GUIDE,
	SPC_MAX
};

enum RequestAdasisCommand
{
    RAC_TRANSMISSION,
    RAC_RETRANSMISSION,
    RAC_STOP_TRANSMISSION,
    RAC_CLEAR,
    RAC_ENCRYPT_COORD,
    RAC_NONE
};

enum WayPointType
{
    WPT_CHECK_POINT,    // 无人驾驶车辆在该条车道上必须经过的检测点
    WPT_EXIT_POINT,     // 出口点
    WPT_ENTRY_POINT,    // 入口点
    //WPT_U_TURN,         // 掉头处的路点
    WPT_STOP_LINE,      // 表示和停止线有关的路点
    WPT_TRAFFIC_LIGHT,  // 红绿灯
    WPT_CANDIDATE_ENTRY_POINT,   // 候补Entry way point
	
	/*k001 add by runz 20151027(begin)*/
	WPT_CROSS_WALK,     //人行横道
	/*k001 add by runz 20151027(end)*/

	/*k001 add by runz 20160104 for adding lane change point(begin)*/
	WPT_LANECHANGE_POINT
	/*k001 add by runz 20160104 for adding lane change point(end)*/
};

enum LaneChangeFlag
{
    LCF_NONE,
    LCF_LEFT,
    LCF_RIGHT,
    LCF_NA
};

// MetaData
enum CountryCode
{
    CC_UNKNOWN,
    CC_CN               = 156,
    CC_JAPAN            = 81
};

enum RegionCode
{
    META_DATA_RC_UNKNOWN,
};

enum DrivingSide
{
    DS_LEFT,
    DS_RIGHT
};

enum SpeedUnit
{
    SU_MPH,
    SU_KMPH
};

enum MapProvider
{
    MP_UNKNOWN  ,
    MP_NAVTEQ   ,
    MP_TOMTOM   ,
    MP_ZENRIN   ,
    MP_IPC      ,
    MP_NAVINFO  ,
    MP_OTHER    ,
    MP_NA       ,
};

enum MapVersionQuarter
{
    MVQ_SPRING,
    MVQ_SUMMER,
    MVQ_AUTUMN,
    MVQ_WINTER
};

class HAdasisConfig;
class HAdasisLoger;

struct HVehicleStatus;
struct HRouteStatus;
struct HAv2HRStatus;
struct HAdasisMessageCounter;
struct HAdasisStatus;
class HAdasisDiagnosis;

//class POSITION;

struct PathLink;
class KLink;

class Av2HP_Position;
class Av2HP_Stub;
class Av2HP_Stub2;
class Av2HP_Segment;
class Av2HP_ProfileShort;
class Av2HP_ProfileLong;
class Av2HP_MetaData;

class Av2HP_Longitude;
class Av2HP_Latitude;
class Av2HP_Curvature;
class Av2HP_Slope;
class Av2HP_VariableSpeedSignPosition;
class Av2HP_StopLine;

class HPath;
class HPathLink;

class HPosition;
class HStub;
class HSegment;
class HSpot;
class HIntersection;

class HAdasisTransceiver;
class HCanTransceiver;
class HLcmTransceiver;

typedef unsigned char                                       CAN_MESSAGE[8];

typedef std::deque<HPathLink*>                              HPathLinkDeque;
typedef std::deque<HStub*>                                  HStubDeque;
typedef std::set<HStub*>                                    HStubSet;
typedef std::deque<HSegment*>                               HSegmentDeque;
typedef std::deque<HSpot*>                                  HSpotDeque;
typedef std::deque<HPath*>                                  HPathDeque;
typedef std::deque<HIntersection*>                          HIntersectionDeque;

typedef std::deque<Av2HP_Position*>                         Av2HP_PositonDeque;
typedef std::deque<Av2HP_Stub*>                             Av2HP_StubDeque;
typedef std::deque<Av2HP_Stub2*>                            Av2HP_Stub2Deque;
typedef std::deque<Av2HP_Segment*>                          Av2HP_SegmentDeque;
typedef std::deque<Av2HP_ProfileShort*>                     Av2HP_ProfileShortDeque;
typedef std::deque<Av2HP_ProfileLong*>                      Av2HP_ProfileLongDeque;
typedef std::deque<Av2HP_Longitude*>                        Av2HP_LongitudeDeque;
typedef std::deque<Av2HP_Latitude*>                         Av2HP_LatitudeDeque;
typedef std::deque<Av2HP_Curvature*>                        Av2HP_CurvatureDeque;
typedef std::deque<Av2HP_Slope*>                            Av2HP_SlopeDeque;
typedef std::deque<Av2HP_VariableSpeedSignPosition*>        Av2HP_VariableSpeedSignPositionDeque;
typedef std::deque<Av2HP_StopLine*>                         Av2HP_StopLineDeque;

typedef std::set<unsigned int>                              HLinkIdSet;

typedef std::set<PathLink*>                                 PathLinkSet;

#endif // _AV2HP_DEFINE_H_
