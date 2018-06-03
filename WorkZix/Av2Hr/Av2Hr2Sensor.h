#include <Windows.h>
#include "IDataStore.h"
#include "IMessageCodec.h"
#include "IMessageObserver.h"
#include "IMessageProvider.h"
#include "adm_kotei_api.h"
#include "LcmReceiver.h"
#include "LCM_NAVI_REQUIRE_INFO.hpp"
#include "LCM_NAVI_TO_SENSE_INFO.hpp"
#include "LCM_TRAFIC_LIGHT_REQUIRE.hpp"
#include "GisTransform.h"
#include "LibGuiOpenGL.h"

using namespace std;

#if _DEBUG
#pragma comment(lib,"LibGuiOpenGLD.lib")
#else
#pragma comment(lib,"LibGuiOpenGL.lib")
#endif

struct CAv2HrParam
{
	double dLongitudeOffSet;
	double dLatitudeOffSet;
	bool bIsLaneChangeEnable;
	bool bTLEnable;
	double dTLMaxDist;
	double dTLMinHeight;
	double dTLMaxAngle;
	double dSLMaxDist;
	double dSLMaxLateralDist;
	double dSLOffSet;
	double dChangeLaneShortenDist;
	CAv2HrParam()
	{
		dLongitudeOffSet = -0.0000060;
		dLatitudeOffSet = +0.0000010;
		bIsLaneChangeEnable= true;
		bTLEnable = true;
		dTLMaxDist = 200.0;
		dTLMinHeight = 5.0;
		dTLMaxAngle = 20.0;
		dSLMaxDist = 100.0;
		dSLMaxLateralDist = 5.0;
		dSLOffSet = 3.5;
		dChangeLaneShortenDist = 0.0;
	}
	int ReadParam();
	int WriteParam();
	void PrintParam();
};

class CVirtualObjectList
{
public:
	CVirtualObjectList();
	~CVirtualObjectList();

	long Init(char* pszPath);
	int TransData(LCM_NAVI_REQUIRE_INFO GpsRequir, vector<LCM_NAVI_OBJECT>& Objs);

public:
	//6：交通灯  7：停止线  8：减速带  
	//20：禁止变道区域
	//30：禁止左侧预测  31：禁止右侧预测
	//11：路口信息
	Mat m_ObjList;//0:MinorType 1:Lat 2:Long 3:Alti 4:MajType 5:nValue 6:reserve
};

class CLaneJoint
{
public:
	int8_t m_bIsCalculatedRoute;
	std::list<Av2HR_ADM_WAYPOINT> m_waypointList;
};

class CPathJoint
{
public:
	CPathJoint(CAv2HrParam Param)
	{
		m_Param = Param; 
	}
	vector<CLaneJoint> Av2HR_ADM_LANE_2_CLaneJoint(Av2HR_ADM_LANE_SECTION& LaneSection_in);
	vector<CLaneJoint> GetParallelPaths(Av2HR_ADM_LANE_SECTION& LaneSection_in);
	void FeedBase(Av2HR_ADM_LANE_SECTION& LaneSection_in);
	void FeedDataFront(Av2HR_ADM_LANE_SECTION& LaneSection_in);
	void FeedDataBack(Av2HR_ADM_LANE_SECTION& LaneSection_in);
	Av2HR_ADM_LANE_SECTION GetFinalLaneSection(LCM_NAVI_REQUIRE_INFO& Req_in);

	vector<CLaneJoint> m_Paths;
	CAv2HrParam m_Param;
};

class Av2HR_Package_Data
{
public:
	Av2HR_ADM_VEHICLE_STATE VS;
	Av2HR_ADM_LANE_LINE_PROFILE_STRING ProfileString;
	Av2HR_ADM_POI_LIST PoiList;
	Av2HR_ADM_LANE_SECTION_LIST LaneSectionList;
	void Init()
	{
		Av2HR_ADM_VEHICLE_STATE VS_;
		Av2HR_ADM_LANE_LINE_PROFILE_STRING ProfileString_;
		Av2HR_ADM_POI_LIST PoiList_;
		Av2HR_ADM_LANE_SECTION_LIST LaneSectionList_;
		VS = VS_;
		ProfileString = ProfileString_;
		PoiList = PoiList_;
		LaneSectionList = LaneSectionList_;
	}
};

class CAv2Hr2Sensor
{
public:
	CAv2Hr2Sensor();
	~CAv2Hr2Sensor();
	
	int Start();
	int UpdataNaviData(Av2HR_Package_Data& Pack_in);
	CVirtualObjectList m_ObjOpr;
	CAv2HrParam m_Param;

friend void WINAPI NaviReqCallBack(void* pData, void* pUser);

private:
	int DrawOpengl(Av2HR_Package_Data& AvData);
	int FillDataEx(Av2HR_Package_Data& Pack_in, LCM_NAVI_REQUIRE_INFO& Req_in, LCM_NAVI_TO_SENSE_INFO& Pack_out);
	int AddObjects(LCM_NAVI_REQUIRE_INFO& Req_in, LCM_NAVI_TO_SENSE_INFO& Pack);
	void TrafficLightRequire(LCM_NAVI_TO_SENSE_INFO& NaviInfo);
	LCM_POINT2D_F TransPoint(CGisTransform& GisOpr, Av2HR_ADM_POINT& pt);
	void LaneChangeDetect(LCM_NAVI_TO_SENSE_INFO& NaviInfo);
	CGisTransform m_GlobalTran;
	CLcmRevicer<LCM_NAVI_REQUIRE_INFO> m_Req;
	CLcmRevicer<LCM_NAVI_TO_SENSE_INFO> m_NaviSend;
	bool m_bIsBusy;
	CRITICAL_SECTION m_cs;
	Av2HR_Package_Data m_Pack;
	double m_dRangeDistMin;
	double m_dRangeDistMax;
	LIBGUIOPENGL_HANDLE m_Opengl;
	CLcmRevicer<LCM_TRAFIC_LIGHT_REQUIRE> m_TrafficLightReqSend;
};
