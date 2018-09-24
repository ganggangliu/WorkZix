#include "LcmReceiver.h"
#include "LCM_NAVI_REQUIRE_INFO.hpp"
#include "LCM_NAVI_TO_SENSE_INFO.hpp"
#include "LCM_GPS_DATA.hpp"
#include "LCM_TRAFIC_LIGHT_REQUIRE.hpp"
#include "GisTransform.h"
#include <vector>
#include <fstream>
using namespace std;

class CVirtualLane
{
public:
	CVirtualLane();
	~CVirtualLane();

	long Init(char* pszPath, CGisTransform* pGisOprMap);
	LCM_NAVI_TO_SENSE_INFO TransData(LCM_NAVI_REQUIRE_INFO GpsRequir);
	LCM_NAVI_TO_SENSE_INFO TransDataNotLoop(LCM_NAVI_REQUIRE_INFO GpsRequir);
	int GetCurInd();
	
	int nId;
	int Flag;
	vector<LCM_GPS_DATA> VecGpsOri;
	vector<LCM_GPS_DATA> VecGps;
	vector<LCM_GPS_DATA> VecLineLeft;
	vector<LCM_GPS_DATA> VecLineRight;
	int nBranchStart;
	int nBranchEnd;

private:
	double GetDistance(double dLat0, double dLat1, double dLon0, double dLon1 );
	void ReadTrackLog(char* pszPath);
	long ParseImuData(char* pBuffer, LCM_GPS_DATA* pImu);
	long FittingCurvesWithBSplineOrder2(vector<LCM_GPS_DATA>& Track_in, vector<LCM_GPS_DATA>& Track_out);
	
	long m_nNearIndStart;
	long m_nNearIndEnd;
	long m_nNearInd;
	double m_dMaxTrackDist;
	double m_dDistLine2Path;
	CGisTransform* m_pGisOprMap;
};

class CVirtualObjectList
{
public:
	CVirtualObjectList();
	~CVirtualObjectList();

	long Init(char* pszPath, CGisTransform* pGisOprMap);
	int TransData(LCM_NAVI_REQUIRE_INFO GpsRequir, vector<LCM_NAVI_OBJECT>& Objs);

public:
	
	CGisTransform* m_pGisOprMap;
	Mat m_ObjList;//0:MinorType 1:Lat 2:Long 3:Alti 4:MajType 5:nValue 6:reserve
	Mat m_ObjListTrans;
};

class CVirtualNavi
{
public:
	CVirtualNavi();
	~CVirtualNavi();

	void Init(char* pszPath);
	long AddObjects(char* pszPath);
	long AddBranch(char* pszPath, int nLaneSide);
	LCM_NAVI_TO_SENSE_INFO TransData(LCM_NAVI_REQUIRE_INFO GpsRequir);
	void TrafficLightRequire(LCM_NAVI_TO_SENSE_INFO& NaviInfo);

public:
	CVirtualLane m_MainLane;
	vector<CVirtualLane> m_Branches;
	CGisTransform m_GisOprMap;
	CVirtualObjectList m_ObjectList;
	CLcmRevicer<LCM_TRAFIC_LIGHT_REQUIRE> m_TrafficLightReqSend;
};