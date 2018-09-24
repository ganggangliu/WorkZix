
#ifndef _ADM_NUDT_API_H_
#define _ADM_NUDT_API_H_

#include <vector>

typedef signed char       int8_t;
typedef signed short      int16_t;
typedef signed int        int32_t;
typedef unsigned char     uint8_t;
typedef unsigned short    uint16_t;
typedef unsigned int      uint32_t;

#ifdef WIN32
typedef signed long long       int64_t;
typedef unsigned long long     uint64_t;
#endif

#define PATH_INDEX_INVALID                 (int32_t(0xFFFFFFFF))
#define OFFSET_REFERENCE_PATH_LINK_INVALID (int32_t(0xFFFFFFFF))
#define COUNTER_INVALID                    (int32_t(0xFFFFFFFF))

#define ALTITUDE_INVALID   (int32_t(0xFFFFFFFF))
#define LONGITUDE_INVALID  (int32_t(0xFFFFFFFF))
#define LATITUDE_INVALID   (int32_t(0xFFFFFFFF))

#define HEADING_CHANGE_UNKNOWN (int32_t(0xFFFFFFFF))
#define SLOPE_UNKNOWN          (int16_t(0xFFFF))
#define CURVATURE_UNKNOWN      (int32_t(0xFFFFFFFF))
#define LIMIT_SPEED_UNKNOWN    (int16_t(0xFFFF))

#define LANE_INDEX_INVALID                          (int8_t(0xFF))
#define REFERENCE_LANE_LINE_INDEX_INVALID           (int8_t(0xFF))
#define OFFSET_REFERENCE_LANE_LINE_INVALID          (int16_t(0xFFFF))
#define HEADING_CHANGES_REFERENCE_LANE_LINE_INVALID (int32_t(0xFFFFFFFF))
#define CONFIDENCE_INVALID                          (int8_t(0xFF))

#define LANE_LINE_INDEX_INVALID  (int8_t(0xFF))
#define OUTLINE_TYPE_INVALID     (int8_t(0xFF))
#define OUTLINE_PROPERTY_INVALID (int8_t(0xFF))

#define ADM_POI_MAJOR_TYPE_INVALID (int8_t(0xFF))
#define ADM_POI_MINOR_TYPE_INVALID (int8_t(0xFF))
#define ADM_POI_PLACE_TYPE_INVALID (int8_t(0xFF))

class Av2HR_ADM_VEHICLE_STATE
{
public:
    int64_t    m_tmSinceEpoch;  // Unit:ms, timestamp (micrseconds since the epoch)
    int32_t    m_iLongitude;    // Unit:0.0000001degrees, Longitude in degrees
    int32_t    m_iLatitude;     // Unit:0.0000001degrees, Latitude in degrees
    int32_t    m_iAltitude;     // Unit:0.01m, Altitude in meters above WGS84 reference ellipsoid
    int32_t    m_iSpeed;        // Unit:0.1m/s, Speed of the vehicle
    int32_t    m_iHeading;      // Unit:0.1degrees, Heading of the vehicle
};

class Av2HR_ADM_CAR_POSITION
{
public:
    int32_t    m_iPathIndex;
    int32_t    m_iOffset;                          //Unit: cm

    int8_t     m_iLaneIndex;
    int8_t     m_iReferenceLaneLineIndex;
    int16_t    m_iReferenceLaneLineOffset;         //Unit: cm
    int32_t    m_iReferenceLaneLineHeadingChanges; // Unit:0.01degrees
    int8_t     m_iConfidence;

public:
	Av2HR_ADM_CAR_POSITION()
	{
		m_iPathIndex                       = PATH_INDEX_INVALID;
		m_iOffset                          = OFFSET_REFERENCE_PATH_LINK_INVALID;
		m_iLaneIndex                       = LANE_INDEX_INVALID;
		m_iReferenceLaneLineIndex          = REFERENCE_LANE_LINE_INDEX_INVALID;
		m_iReferenceLaneLineOffset         = OFFSET_REFERENCE_LANE_LINE_INVALID;
		m_iReferenceLaneLineHeadingChanges = HEADING_CHANGES_REFERENCE_LANE_LINE_INVALID;
		m_iConfidence                      = CONFIDENCE_INVALID;
	}
};

class Av2HR_ADM_POINT
{
public:
    int32_t    m_ulLongitude;//Unit:0.0000001degrees
    int32_t    m_ulLatitude;
    int32_t    m_ulAltitude;

public:
	Av2HR_ADM_POINT()
	{
		m_ulLongitude = LONGITUDE_INVALID;
		m_ulLatitude  = LATITUDE_INVALID;
		m_ulAltitude  = ALTITUDE_INVALID;
	}
	Av2HR_ADM_POINT(int32_t ulLongitude, int32_t ulLatitude,  int32_t ulAltitude)
	{
		m_ulLongitude = ulLongitude;
		m_ulLatitude = ulLatitude;
		m_ulAltitude = ulAltitude;
		m_ulAltitude = ALTITUDE_INVALID;
	}
};

class Av2HR_ADM_OUTLINE
{
public:
    int8_t     m_iLaneLineIndex;
    int8_t     m_iOutlineType;
    int8_t     m_iOutlineProperty;

    int16_t    m_usPointNum;
    std::vector< Av2HR_ADM_POINT > m_pointString;

public:
	Av2HR_ADM_OUTLINE()
	{
		m_iLaneLineIndex   = LANE_LINE_INDEX_INVALID;
		m_iOutlineType     = OUTLINE_TYPE_INVALID;
		m_iOutlineProperty = OUTLINE_PROPERTY_INVALID;

		m_usPointNum = 0;
	}
};

class Av2HR_ADM_LANE_LINE_PROFILE
{
public:
    int32_t    m_iPathIndex;
    int32_t    m_iOffset;

    int8_t     m_iLaneLineNum;
    std::vector< Av2HR_ADM_OUTLINE > m_outlineString;

public:
	Av2HR_ADM_LANE_LINE_PROFILE()
	{
		m_iPathIndex = PATH_INDEX_INVALID;
		m_iOffset = OFFSET_REFERENCE_PATH_LINK_INVALID;

		m_iLaneLineNum = 0;
	}
};

class Av2HR_ADM_LANE_LINE_PROFILE_STRING
{
public:
    int32_t    m_iPathIndex;
    int32_t    m_iOffset;             //unit:cm

    int8_t     m_iLaneLineProfileNum;
    std::vector< Av2HR_ADM_LANE_LINE_PROFILE > m_laneLineProfileString;

public:
	Av2HR_ADM_LANE_LINE_PROFILE_STRING()
	{
		m_iPathIndex          = PATH_INDEX_INVALID;
		m_iOffset             = OFFSET_REFERENCE_PATH_LINK_INVALID;
		m_iLaneLineProfileNum = 0;
	}
};

class Av2HR_ADM_POI
{
public:
    int8_t     m_iMajorType;
    int8_t     m_iMinorType;
    int8_t     m_iPlaceType;
    int32_t    m_iOffset;
    Av2HR_ADM_POINT m_crd;
    int32_t    m_iValue;

public:
	Av2HR_ADM_POI()
	{
		m_iMajorType = ADM_POI_MAJOR_TYPE_INVALID;
		m_iMinorType = ADM_POI_MINOR_TYPE_INVALID;
		m_iPlaceType = ADM_POI_PLACE_TYPE_INVALID;
		m_iOffset = OFFSET_REFERENCE_PATH_LINK_INVALID;
		m_iValue = int32_t(0xFFFFFFFF);
	}
};

class Av2HR_ADM_POI_LIST
{
public:
    int32_t    m_iPathIndex;
    int32_t    m_iOffset;

    int16_t    m_iPOINum;
    std::vector< Av2HR_ADM_POI > m_poiList;
public:
	Av2HR_ADM_POI_LIST()
	{
		m_iPathIndex = PATH_INDEX_INVALID;
		m_iOffset    = OFFSET_REFERENCE_PATH_LINK_INVALID;

		m_iPOINum    = 0;
	}
};

class Av2HR_ADM_ELEVATION_GRID_MAP
{
public:
    int32_t    m_iPathIndex;
    int32_t    m_iOffset;

    int32_t    m_iLeft;
    int32_t    m_iRight;
    int32_t    m_iTop;
    int32_t    m_iBottom;

    int32_t    m_iRow;
    int32_t    m_iCol;

    int32_t    m_iNum;
    std::vector< int32_t > m_indexString;
    std::vector< uint8_t > m_elevationString;

public:
	Av2HR_ADM_ELEVATION_GRID_MAP()
	{
		m_iPathIndex = PATH_INDEX_INVALID;
		m_iOffset    = OFFSET_REFERENCE_PATH_LINK_INVALID;

		m_iLeft   = 0;
		m_iRight  = 0;
		m_iTop    = 0;
		m_iBottom = 0;

		m_iRow = 0;
		m_iCol = 0;

		m_iNum = 0;
	}
};

class Av2HR_ADM_GUIDE_POINT_INFO
{
public:
    Av2HR_ADM_POINT m_crd;

    int32_t    m_iHeading;         //引导点处道路的方向，单位：0.01degrees
    int32_t    m_iCurvature;       //引导点处道路的曲率，单位：0.00001 (1/m)
    int16_t    m_iSlope;           //引导点处道路的坡度，单位：0.1

    int16_t    m_iLimitSpeed;      //引导点处道路的限制值，单位：1 km/h。

public:
	Av2HR_ADM_GUIDE_POINT_INFO()
	{
		m_iHeading          = HEADING_CHANGE_UNKNOWN;
		m_iCurvature        = CURVATURE_UNKNOWN;
		m_iSlope            = SLOPE_UNKNOWN;
		m_iLimitSpeed       = LIMIT_SPEED_UNKNOWN;
	}
};

class Av2HR_ADM_GUIDE_POINT_LIST
{
public:
    int32_t    m_iPathIndex;
    int32_t    m_iOffset;          // Unit:cm

    int16_t    m_iGuidePointNum;
    std::vector< Av2HR_ADM_GUIDE_POINT_INFO > m_guidePointList;
public:
	Av2HR_ADM_GUIDE_POINT_LIST()
	{
		m_iPathIndex = PATH_INDEX_INVALID;
		m_iOffset    = OFFSET_REFERENCE_PATH_LINK_INVALID;
		m_iGuidePointNum = 0;
	}
};

int Av2HR_setVehicleState(Av2HR_ADM_VEHICLE_STATE* pAdmVehicleState);

int Av2HR_getCarPosition(Av2HR_ADM_CAR_POSITION* pAdmCarPosition);
int Av2HR_getLaneLineProfile(Av2HR_ADM_LANE_LINE_PROFILE_STRING* pAdmLaneLineProfileString);
int Av2HR_getPOIList(Av2HR_ADM_POI_LIST* pAdmPoiList);
int Av2HR_getElevationGridMap(Av2HR_ADM_ELEVATION_GRID_MAP* pAdmElevationGridMap);
int Av2HR_getGuidePointList(Av2HR_ADM_GUIDE_POINT_LIST* pAdmGuidePointList);

int ADM_NUDT_API_Initialize();

#endif  // _ADM_NUDT_API_H_
