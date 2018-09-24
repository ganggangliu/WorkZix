#ifndef _Av2HR_TYPE_H_
#define _Av2HR_TYPE_H_

#include "Av2HP_define.h"
#include "stdint.h"

#include "Av2HP_Position.h"
#include "Av2HP_Stub.h"
#include "Av2HP_MetaData.h"
#include "Av2HP_ProfileLong.h"
#include "Av2HP_ProfileShort.h"
#include "Av2HP_Segment.h"

#include <map>

#define ADA_DELETE(ptr) if (ptr != NULL) {delete ptr;ptr = NULL;}

#define INVALID_OFFSET                       (0x1FFF)

#define INVALID_PATHINDEX                    (7)
#define MAX_PATHINDEX                        (64)

#define RESERVED_SUB_PATHINDEX               (7)

#define INIT_PATHINDEX                       (8)
#define INIT_PATHINDEX_GENERATION            (0)

#define PROFIELTYPE_OFFSET_CORRECTION        (31)

enum Av2HR_ProfileType
{
    // Segment
	Av2HR_PT_FUNCTIONAL_CLASS,
	Av2HR_PT_FORM_OF_WAY,
	Av2HR_PT_EFFECTIVE_SPEED_LIMIT,
	Av2HR_PT_EFFECTIVE_SPEED_LIMIT_TYPE,
	Av2HR_PT_NUMBER_OF_LANES_IN_DRIVING_DIRECTION,
	Av2HR_PT_NUMBER_OF_LANES_IN_OPPOSITE_DIRECTION,
	Av2HR_PT_TUNNEL,
	Av2HR_PT_BRIDGE,
	Av2HR_PT_DIVIDED_ROAD,
	Av2HR_PT_BUILT_UP_AREA,
	Av2HR_PT_COMPLEX_INTERSECTION,
	Av2HR_PT_PART_OF_CALCULATED_ROUTE,

    // Profile Short
    Av2HR_PT_CURVATURE,
    Av2HR_PT_ROUTE_NUMBER,
    Av2HR_PT_SLOPE,
    Av2HR_PT_ROAD_ACCESSIBILITY,
    Av2HR_PT_ROAD_CONDITION,
    Av2HR_PT_VARIABLE_SPEED_SIGN_POSITION,
	Av2HR_PT_HEADING_CHANGE = 20,
	
	/*非标准的ADASIS ProfileShort(begin)*/
	Av2HR_PT_WAYPOINT = 18,
	Av2HR_PT_LANE = 19,
	/*非标准的ADASIS ProfileShort(end)*/
    
	// Profile Long
    Av2HR_PT_LONGITUDE = 100,
    Av2HR_PT_LATITUDE
};

class Av2HR_Profile
{
public:
    bool    m_bLong;

    union 
    {
        Av2HP_ProfileLong*   pProfileLong;
        Av2HP_ProfileShort*  pProfileShort;
    } m_profile;
};

class Av2HR_ProfileDesc
{
public:
    Av2HR_ProfileType   m_eType;
    union
    {
		//seg
		unsigned int functionRoadClass;
		unsigned int formOfWay;
		unsigned int effectiveLimitSpeed;
		unsigned int effectiveLimitSpeedType;
		unsigned int numberOfLanesIndrivingDirection;
		unsigned int numberOfLanesInOppositeDirection;
		unsigned int tunnel;
		unsigned int bridge;
		unsigned int divideRoad;
		unsigned int buildUpArea;
		unsigned int complexIntersection;
		unsigned int partOfCalculatedRoute;
		
		//profileShort
		unsigned int curvature;
		unsigned int routeNumber;
		unsigned int slop;
		unsigned int roadAccessbility;
		unsigned int roadCondition;
		unsigned int speedSignPosition;
		unsigned int headChange;
		
		/*非标准的ADASIS协议(ProfileShort)*/
		unsigned int wayPoint;          //所有消息都绑定在profileType = 18的value0中	
		unsigned int enterPoint;        //入口点
		unsigned int leavePoint;        //出口点
		unsigned int stopLine;          //停止线
		unsigned int trafficLight;      //红绿灯
		unsigned int zebraCross;        //斑马线
		unsigned int decelerationZone;  //减速带

		unsigned int LaneInfo;          //所有消息都绑定在profileType = 19的value0中
	    unsigned int laneChange;        //表示当前车道相对前一车道的变道信息
		unsigned int inWhichLane;       //表示当前车道是第几车道
		unsigned int laneChangeInfo;    //表示当前车道的变道属性
		unsigned int LaneWidth;         //所有消息都绑定在profileType = 19的value1中
		/*非标准的ADASIS协议(ProfileShort)*/
		
		//profile long
		unsigned int lontitude;
		unsigned int latitude;		
    } m_profileDesc;
};

class Av2HR_Message
{
public:
    CAN_MESSAGE         m_msg;
};

class IMessageProvider;
class IMessageObserver;

typedef int                 av2hr_e;
typedef int                 av2hr_pathid_t;
typedef int                 av2hr_offset_t;
typedef bool                bool_t;

typedef Av2HP_Position      av2hr_position_t;
typedef Av2HP_Stub          av2hr_stub_t;
typedef Av2HR_ProfileDesc   av2hr_profile_desc_t;
typedef MapProvider         av2hr_mapprovider_e;
typedef Av2HP_Segment       av2hr_segment_t;
typedef Av2HR_Profile       av2hr_profile_t;
typedef Av2HP_ProfileLong   av2hr_profile_long_t;
typedef Av2HP_ProfileShort  av2hr_profile_short_t;

typedef Av2HP_MetaData      av2hr_metadata_t;
typedef Av2HP_Message_Type  av2hr_msgtype_e;
//typedef uint8_t*            av2hr_canframe_t; 
typedef Av2HR_Message       av2hr_canframe_t;

//segment
typedef std::map<unsigned int, av2hr_segment_t*>      HRSegAbsOffsetMap; 
typedef std::pair<unsigned int, av2hr_segment_t*>	  HRSegAbsOffsetMapPair;

typedef std::map<unsigned int, HRSegAbsOffsetMap>     HRSegAbsPathIndexMap;
typedef std::pair<unsigned int, HRSegAbsOffsetMap>	  HRSegAbsPathIndexMapPair;

//stub
typedef std::map<unsigned int, av2hr_stub_t*>         HRStubAbsOffsetMap; 
typedef std::pair<unsigned int, av2hr_stub_t*>	      HRStubAbsOffsetMapPair;

typedef std::map<unsigned int, HRStubAbsOffsetMap>    HRStubAbsPathIndexMap;
typedef std::pair<unsigned int, HRStubAbsOffsetMap>	  HRStubAbsPathIndexMapPair;

//lontitude
typedef std::map<unsigned int, Av2HP_ProfileLong*>    HRLonAbsOffsetMap; 
typedef std::pair<unsigned int, Av2HP_ProfileLong*>	  HRLonAbsOffsetMapPair;

typedef std::map<unsigned int, HRLonAbsOffsetMap>     HRLonAbsPathIndexMap;
typedef std::pair<unsigned int, HRLonAbsOffsetMap>	  HRLonAbsPathIndexMapPair;

//latitude
typedef std::map<unsigned int, Av2HP_ProfileLong*>    HRLatAbsOffsetMap; 
typedef std::pair<unsigned int, Av2HP_ProfileLong*>   HRLatAbsOffsetMapPair;

typedef std::map<unsigned int, HRLatAbsOffsetMap>      HRLatAbsPathIndexMap;
typedef std::pair<unsigned int, HRLatAbsOffsetMap>	   HRLatAbsPathIndexMapPair;

//profile short
typedef std::map<unsigned int, Av2HP_ProfileShort*>    HRProfileShortAbsOffsetMap; 
typedef std::pair<unsigned int, Av2HP_ProfileShort*>   HRProfileShortAbsOffsetMapPair;

typedef std::map<unsigned int, HRProfileShortAbsOffsetMap>      HRProfileShortAbsPathIndexMap;
typedef std::pair<unsigned int, HRProfileShortAbsOffsetMap>	   HRProfileShortPathIndexMapPair;

typedef std::deque<av2hr_stub_t*>   StubDeque;
typedef std::set<av2hr_stub_t*>     StubSet;



//struct av2hr_location_t
//{
//    av2hr_pathid_t  m_pathId;
//    av2hr_offset_t  m_offset;
//};

struct av2hr_pathIndxGeneration_t            //用于计算绝对的pathIndex用
{
	unsigned char subPathIndex;
	unsigned int  generation;	
};

struct av2hr_segWithOffsetIndex_t            //用于计算Segment的绝对的offset
{
	unsigned int    offsetIndex; 
	av2hr_segment_t seg;
};

struct av2hr_posWithOffsetIndex_t            //用于计算Segment的绝对的offset
{
	unsigned int     offsetIndex;  
	av2hr_position_t pos;
};

struct av2hr_stubWithOffsetIndex_t           //用于计算Stub的绝对的offset
{
	unsigned int offsetIndex;  
	av2hr_stub_t stub;
};

struct av2hr_lonWithOffsetIndex_t             //用于计算经度的绝对的offset
{
	unsigned int         offsetIndex;  
	av2hr_profile_long_t lon;
};

struct av2hr_latWithOffsetIndex_t             //用于计算维度的绝对的offset
{
	unsigned int         offsetIndex;  
	av2hr_profile_long_t lat;
};

struct av2hr_profileShortWithOffsetIndex_t    //用于计算profile short的绝对的offset
{
	unsigned int          offsetIndex;  
	av2hr_profile_short_t profileShort;
};

#endif  // _Av2HR_TYPE_H_
