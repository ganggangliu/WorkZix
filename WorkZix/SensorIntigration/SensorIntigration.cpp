#include "windows.h"
#include <list>
#include "LcmReceiver.h"
#include "LCM_NAVI_TO_SENSE_INFO.hpp"
#include "LCM_NAVI_REQUIRE_INFO.hpp"
#include "LCM_SENSOR_FUSION_PACKAGE.hpp"
#include "LIDAR_RADAR_INFO.hpp"
#include "LCM_TRAFIC_LIGHT_RESULT.hpp"
#include "GpsManager.h"
#include "IbeoLidarOpr.h"
#include <iostream>
#include "ProjectOpr.h"
#include "GisTransform.h"

#include "IbeoOpr.h"

using namespace std;

#if _DEBUG
#pragma comment(lib,"IbeoOprD.lib")
#else
#pragma comment(lib,"IbeoOpr.lib")
#endif

enum RadarType
{
	RADAR_TYPE_ESR = 9,
	RADAR_TYPE_RSDS_RL = 10,
	RADAR_TYPE_RSDS_RR = 11,
	RADAR_TYPE_RSDS_FL = 12,
	RADAR_TYPE_RSDS_FR = 13,
};

//CIbeoLidarOpr g_IbeoOpr;
hIbeoHandle g_pIbeoOpr = NULL;
CGpsManager g_PosData;
CLcmRevicer<LCM_NAVI_TO_SENSE_INFO> g_NaviRec(string("LCM_NAVI_TO_SENSE_INFO"));
CLcmRevicer<LCM_NAVI_REQUIRE_INFO> g_NaviReq(string("LCM_NAVI_REQUIRE_INFO"));
CLcmRevicer<LCM_SENSOR_FUSION_PACKAGE> g_SenseFusionSend(string("LCM_SENSOR_FUSION_PACKAGE"));
CLcmRevicer<LCM_IBEO_CLOUD_POINTS> g_IbeoExtern(string("LCM_IBEO_CLOUD_POINTS"));
CLcmRevicer<LIDAR_RADAR_INFO> g_ESR(string("LIDAR_RADAR_INFO_ESR"));
CLcmRevicer<LIDAR_RADAR_INFO> g_Rsds_RL(string("LIDAR_RADAR_INFO_DELPHI_RL"));
CLcmRevicer<LIDAR_RADAR_INFO> g_Rsds_RR(string("LIDAR_RADAR_INFO_DELPHI_RR"));
CLcmRevicer<LIDAR_RADAR_INFO> g_Rsds_FL(string("LIDAR_RADAR_INFO_DELPHI_FL"));
CLcmRevicer<LIDAR_RADAR_INFO> g_Rsds_FR(string("LIDAR_RADAR_INFO_DELPHI_FR"));
CLcmRevicer<LCM_TRAFIC_LIGHT_RESULT> g_TrafficLightRes(string("LCM_TRAFIC_LIGHT_RESULT"));
CLcmRevicer<LCM_MOBILEYE_INFO> g_MobileyeRec(string("LCM_MOBILEYE_INFO"));
CLidarDataOpr g_LidarLog("");
int AddIbeoObjInfoLite(LCM_SENSOR_FUSION_PACKAGE& Pack);
int InputTrafficLight();
int InputMobileyeData();
int InputEsrData();
int InputRsdsRLData();
int InputRsdsRRData();
int InputRsdsFLData();	//Front Left
int InputRsdsFRData();	//Front Right
int InputGpsData(LCM_GPS_DATA& gps);
void ResetPack(LCM_SENSOR_FUSION_PACKAGE& SensePack);
LCM_GPS_DATA PredictGpsData(LCM_GPS_DATA& gps, double dPreTime);

void __stdcall NaviDataRecFunc(void* pData, void* pUser);

LCM_SENSOR_FUSION_PACKAGE g_SensePack;
LCM_NAVI_REQUIRE_INFO g_Req;
CRITICAL_SECTION g_cs;

class SensorParams
{
public:
	char szLidarIp[32];
	int nLidarType;
	int nDataType;
	int nGpsType;
	bool bIslog;
	SensorParams()
	{
		strcpy(szLidarIp,"192.168.1.119");  //Lidar IP
		nLidarType = 8;						//1:Sick lms xxx  4\8:Ibeo 4\8L
		nDataType = 0;						//0:cloudpoint	 1:objects
		nGpsType = 2;						//0:xw3660 1:xw7660 2:mp-pos
		bIslog = false;						//0:NOT log  1:log
	}
};
SensorParams g_Param;
int GetParam(int argc, char* argv[], SensorParams& Param);

int g_DispCont = 0;
bool g_bIsDisp = false;
void __stdcall IbeoObjectsDataRecFunc(void* pData, void* pUser)
{
	LCM_IBEO_OBJECT_LIST* pObjects = (LCM_IBEO_OBJECT_LIST*)pData;
	if (g_Param.nDataType != 1)
		return;

	EnterCriticalSection(&g_cs);

	g_DispCont++;
	if (g_DispCont%12 == 0)
	{
		g_bIsDisp = true;
//		system("cls");
	}
	else
	{
		g_bIsDisp = false;
	}

	if (g_bIsDisp)
	{
		printf("Lidar object cont: %d\n", pObjects->IbeoObjects.size());
	}
	ResetPack(g_SensePack);
	int nRt = InputGpsData(g_SensePack.Gps);
	if (nRt == 0)
	{
		LeaveCriticalSection(&g_cs);
		return;
	}

	g_SensePack.IbeoObjList = *pObjects;

	InputMobileyeData();
	InputEsrData();
	InputRsdsRLData();
	InputRsdsRRData();
	InputRsdsFLData();
	InputRsdsFRData();

	g_Req.MsgInd = pObjects->FrameInd;
	g_Req.Gps = g_SensePack.Gps;

// 	LCM_NAVI_TO_SENSE_INFO NaviData;
// 	NaviData.MsgInd = g_Req.MsgInd;
// 	NaviDataRecFunc(&NaviData, 0);
// 
// 	return;

	g_NaviReq.Send(string("LCM_NAVI_REQUIRE_INFO"),g_Req);
	if (g_bIsDisp)
	{
		printf("Navi require send! #%d\n",g_Req.MsgInd);
	}
	LeaveCriticalSection(&g_cs);
}

void __stdcall IbeoPointsDataRecFunc(void* pData, void* pUser)
{
	LCM_IBEO_CLOUD_POINTS* pPoints = (LCM_IBEO_CLOUD_POINTS*)pData;

	if (g_Param.nDataType != 0)
		return;

	EnterCriticalSection(&g_cs);

	g_DispCont++;
	if (g_DispCont%12 == 0)
	{
		g_bIsDisp = true;
//		system("cls");
	}
	else
	{
		g_bIsDisp = false;
	}

	if (g_bIsDisp)
	{
		printf("Lidar points cont: %d\n", pPoints->IbeoPoints.size());
	}
	ResetPack(g_SensePack);

	int nRt = InputGpsData(g_SensePack.Gps);
	if (nRt == 0)
	{
		LeaveCriticalSection(&g_cs);
		return;
	}
	g_SensePack.IbeoCloudPoints = *pPoints;

	InputMobileyeData();
	InputEsrData();
	InputRsdsRLData();
	InputRsdsRRData();
	InputRsdsFLData();
	InputRsdsFRData();

	g_Req.MsgInd = pPoints->FrameInd;
	g_Req.Gps = g_SensePack.Gps;
	g_NaviReq.Send(string("LCM_NAVI_REQUIRE_INFO"),g_Req);
	if (g_bIsDisp)
	{
		printf("Navi require send! #%d\n",g_Req.MsgInd);
	}
	LeaveCriticalSection(&g_cs);
}

void __stdcall NaviDataRecFunc(void* pData, void* pUser)
{
	EnterCriticalSection(&g_cs);
	LCM_NAVI_TO_SENSE_INFO* pSenseData = (LCM_NAVI_TO_SENSE_INFO*)pData;
	if (pSenseData->MsgInd != g_Req.MsgInd)
	{
		cout << "data not match!" << pSenseData->MsgInd << ":" << g_Req.MsgInd << endl;
		LeaveCriticalSection(&g_cs);
		return;
	}
	if (g_bIsDisp)
	{
		printf("Navi data acquired! #%d\n",pSenseData->MsgInd);
	}

	g_SensePack.PackInd = pSenseData->MsgInd;
	g_SensePack.NaviInfo = *pSenseData;

	InputTrafficLight();

	g_SenseFusionSend.Send("LCM_SENSOR_FUSION_PACKAGE", g_SensePack);
	printf("Pack data size:%d Bytes\n",g_SensePack.getEncodedSize());

	if (g_Param.bIslog)
	{
		g_LidarLog.WriteLog(&g_SensePack);
	}
	if (g_bIsDisp)
	{
		printf("Is logging: %d   Ind:%d\n",g_Param.bIslog,g_LidarLog.GetInd());
	}
	LeaveCriticalSection(&g_cs);
}

void Help()
{
	printf("5 params needed!\n");
	printf("[Lidar IP adrass] [Lidar type] [Data type] [Gps type] [Is log]\n");
	printf("[Lidar IP adrass] 192.168.1.119\n");
	printf("[Lidar type] 1:Ibeo lcm CloudPoint data pack  4/8:Ibeo-4/8L\n");
	printf("[Data type]  0:CloudPoint  1:Objects\n");
	printf("[Gps type] 0:XW3660 1:XW7660 2:MP-POS\n");
	printf("[Is log] 0:NOT log  1:log");
}

int main(int argc, char* argv[])
{
	InitializeCriticalSection(&g_cs);

	GetParam(argc,argv,g_Param);

	g_ESR.Start();
	g_Rsds_RL.Start();
	g_Rsds_RR.Start();
	g_Rsds_FL.Start();
	g_Rsds_FR.Start();
	g_TrafficLightRes.Start();
	g_MobileyeRec.Start();

	g_PosData.Init((CGpsManager::GPS_MANAGER_TYPE)g_Param.nGpsType);
	g_PosData.Start();

	g_NaviRec.SetCallBack(NaviDataRecFunc, NULL);
	g_NaviRec.Start();

	g_pIbeoOpr = CreateIbeoHandle(g_Param.szLidarIp, (IBEO_DATA_TYPE)g_Param.nLidarType);
	g_pIbeoOpr->SetCallBackPoints(IbeoPointsDataRecFunc);
	g_pIbeoOpr->SetCallBackObjects(IbeoObjectsDataRecFunc);
	g_pIbeoOpr->Start();
// 	if (g_Param.nLidarType == 4 || g_Param.nLidarType == 8)
// 	{
// 		g_IbeoOpr.Init(g_Param.szLidarIp,g_Param.nLidarType);
// 		g_IbeoOpr.SetCallBackPoints(IbeoPointsDataRecFunc, NULL);
// 		g_IbeoOpr.SetCallBackObjects(IbeoObjectsDataRecFunc, NULL);
// 		g_IbeoOpr.Start();
// 	}
// 	else if (g_Param.nLidarType == 1)
// 	{
// 		g_IbeoExtern.SetCallBack(IbeoPointsDataRecFunc,NULL);
// 		g_IbeoExtern.Start();
// 	}
// 	else
// 	{
// 		printf("Lidar type not suppoted!!\n");
// 	}

	Sleep(INFINITE);

	/*
	long nCont = 0;
	while(1)
	{
		nCont++;
		Sleep(1000);
		system("cls");
		printf("%d\n",nCont);
		//////////////////////////////////////////////////////////////////////////
		LCM_IBEO_CLOUD_POINTS CloudPoints;
		long nRt = g_IbeoOpr.GetPointsData(CloudPoints);
		if (nRt == 0)
			printf("Get Points errer!\n");
		else
			printf("Lidar cloud points cont: %d\n",CloudPoints.IbeoPoints.size());
		//////////////////////////////////////////////////////////////////////////
		LCM_IBEO_OBJECT_LIST Objects;
		nRt = g_IbeoOpr.GetObjectsData(Objects);
		if (nRt == 0)
			printf("Get Objects errer!\n");
		else
			printf("Lidar object cont: %d\n",Objects.IbeoObjects.size());
		//////////////////////////////////////////////////////////////////////////
		LIDAR_RADAR_INFO EsrRadarData;
		nRt = g_ESR.GetData(EsrRadarData);
		if (nRt == 0)
			printf("Get ESR Radar Objects errer!\n");
		else
		{
			int nObjCont = 0;
			for (int i = 0; i < 64; i++)
			{
				if (EsrRadarData.Objects[i].bValid)
					nObjCont++;
			}
			printf("ESR Radar object cont: %d\n",nObjCont);
		}
		//////////////////////////////////////////////////////////////////////////
		LCM_GPS_DATA GpsData;
		nRt = g_PosData.GetData(GpsData);
		if (nRt == 0)
			printf("Get Gps data errer!\n");
		else
		{
			printf("%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.2x,%d\n",
				GpsData.GPS_TIME,
				GpsData.GPS_HEADING,
				GpsData.GPS_PITCH,
				GpsData.GPS_ROLL,
				GpsData.GPS_LATITUDE,
				GpsData.GPS_LONGITUDE,
				GpsData.GPS_ALTITUDE,
				GpsData.GPS_VE,
				GpsData.GPS_VN,
				GpsData.GPS_VU,
				GpsData.GPS_BASELINE,
				GpsData.GPS_NSV1,
				GpsData.GPS_NSV2,
				GpsData.GPS_STATE,
				GpsData.GPS_DATATYPE);
		}
		//////////////////////////////////////////////////////////////////////////
		LCM_NAVI_TO_SENSE_INFO NaviData;
		nRt = g_NaviRec.GetData(NaviData);
		if (nRt == 0)
			printf("Get Navi data errer!\n");
		else
			printf("Lane cont: %d\n",g_SensePack.NaviInfo.Paths.size());
		//////////////////////////////////////////////////////////////////////////
		printf("Is logging: %d   Ind:%d\n",g_Param.bIslog,g_LidarLog.GetInd());
	}

	Sleep(INFINITE);
	*/
}

int AddIbeoObjInfoLite(LCM_SENSOR_FUSION_PACKAGE& Pack)
{
	Pack.RadarList.clear();
	Pack.ContRadarObj = 0;
	LCM_IBEO_OBJECT_LIST ObjListT;
//	g_IbeoOpr.GetObjectsDataRepeat(ObjListT);
	g_pIbeoOpr->GetObjectsData(ObjListT);
	Pack.RadarList.resize(ObjListT.IbeoObjects.size());
	Pack.ContRadarObj = ObjListT.ContObjects;
	for (int i = 0; i < ObjListT.IbeoObjects.size(); i++)
	{
		Pack.RadarList[i].nObjectID = ObjListT.IbeoObjects[i].Id;
		Pack.RadarList[i].fObjLocX = ObjListT.IbeoObjects[i].ObjBoxCenter.x;
		Pack.RadarList[i].fObjLocY = ObjListT.IbeoObjects[i].ObjBoxCenter.y;
		Pack.RadarList[i].fObjSizeX = ObjListT.IbeoObjects[i].ObjBoxSize.x;
		Pack.RadarList[i].fObjSizeY = ObjListT.IbeoObjects[i].ObjBoxSize.y;
	}
	
	return Pack.RadarList.size();
}

int GetParam(int argc, char* argv[], SensorParams& Param)
{
	if (argc != 6)
	{
		Help();
		getchar();
		return 0;
	}
	strcpy(Param.szLidarIp,argv[1]);
	Param.nLidarType = atoi(argv[2]);
	Param.nDataType = atoi(argv[3]);
	Param.nGpsType = atoi(argv[4]);
	Param.bIslog = atoi(argv[5]);
	printf("Lidar Ip: %s\n", Param.szLidarIp);
	printf("Lidar type: %d\n", Param.nLidarType);
	printf("Lidar data type: %d\n", Param.nDataType);
	printf("Gps type: %d\n", Param.nGpsType);
	printf("Is log: %d\n", Param.bIslog);
	return 1;
}

list<LCM_TRAFIC_LIGHT_RESULT> g_TrafficLightList;
int g_ListLen = 12;
int g_GreenValidCount = 1;
int InputTrafficLight()
{
	LCM_TRAFIC_LIGHT_RESULT TLRes;
	int nRt = g_TrafficLightRes.GetData(TLRes);
	if (nRt <= 0 || nRt >= 12)
	{
		if (g_bIsDisp)
		{
			printf("Get Traffic Light Result Data error!!!\n");
		}
		g_TrafficLightList.clear();
		return 0;
	}
	else
	{
		g_TrafficLightList.push_front(TLRes);
		if (g_TrafficLightList.size() > g_ListLen)
			g_TrafficLightList.pop_back();
	}
	int GreenCont = 0;
	list<LCM_TRAFIC_LIGHT_RESULT>::iterator its = g_TrafficLightList.begin();
	list<LCM_TRAFIC_LIGHT_RESULT>::iterator ite = g_TrafficLightList.end();
	for (;its != ite;its++)
	{
		if (its->State == 0)
			GreenCont++;
	}
	if (GreenCont >= g_GreenValidCount)
		TLRes.State = 0;
	else
		TLRes.State = 2;

	for (unsigned int i = 0; i < g_SensePack.NaviInfo.Objs.size(); i++)
	{
		LCM_NAVI_OBJECT& ObjT = g_SensePack.NaviInfo.Objs[i];
//		if (ObjT.nMinorType == TLRes.Req.TraficLightId)
		{
			ObjT.nPlaceType = TLRes.State;
		}
	}
	if (g_bIsDisp)
	{
		printf("Traffic light state: %d\n", TLRes.State);
	}
	return 1;
}

int InputMobileyeData()
{
	LCM_MOBILEYE_INFO MobileyeData;
	int nRt = g_MobileyeRec.GetData(MobileyeData);
	if (nRt <= 0 || nRt >= 6)
	{
		if (g_bIsDisp)
		{
			printf("Get Mobileye Data error!!!\n");
		}
		return 0;
	}
	g_SensePack.MobileyeInfo = MobileyeData;
	if (g_bIsDisp)
	{
		printf("Mobileye object cont:%d, Left line:%d, Right line:%d\n",
			g_SensePack.MobileyeInfo.ObjList.size(),
			g_SensePack.MobileyeInfo.LeftLine.Quality,
			g_SensePack.MobileyeInfo.RightLine.Quality);
	}

	return 1;
}

int InputEsrData()
{
	LIDAR_RADAR_INFO EsrRadarData;
	int nRt = g_ESR.GetData(EsrRadarData);
	if (/*nRt != 1*/nRt <= 0 || nRt >=5)
	{
		if (g_bIsDisp)
		{
			printf("Get Esr Data error!!!\n");
		}
		return 0;
	}
	EsrRadarData.nLidarRadarType = RADAR_TYPE_ESR;
	int nValidCont = 0;
	for (int i = 0; i < 64; i++)
	{
		if (!EsrRadarData.Objects[i].bValid)
			continue;
		EsrRadarData.Objects[i].bValid = RADAR_TYPE_ESR;
		g_SensePack.RadarList.push_back(EsrRadarData.Objects[i]);
		nValidCont++;
	}
	g_SensePack.ContRadarObj = g_SensePack.RadarList.size();
	if (g_bIsDisp)
	{
		printf("Esr objects cont: %d\n", nValidCont);
	}
	return 1;
}

int InputRsdsRLData()
{
	LIDAR_RADAR_INFO RadarData;
	int nRt = g_Rsds_RL.GetData(RadarData);
	if (/*nRt != 1*/nRt <= 0 || nRt >=10)
	{
		if (g_bIsDisp)
		{
			printf("Get Rsds_RL Data error!!!\n");
		}
		return 0;
	}
	RadarData.nLidarRadarType = RADAR_TYPE_RSDS_RL;
	int nValidCont = 0;
	for (int i = 0; i < 64; i++)
	{
		if (!RadarData.Objects[i].bValid)
			continue;
		RadarData.Objects[i].bValid = RADAR_TYPE_RSDS_RL;
		g_SensePack.RadarList.push_back(RadarData.Objects[i]);
		nValidCont++;
	}
	g_SensePack.ContRadarObj = g_SensePack.RadarList.size();
	if (g_bIsDisp)
	{
		printf("Rsds_RL objects cont: %d\n", nValidCont);
	}
	return 1;
}

int InputRsdsRRData()
{
	LIDAR_RADAR_INFO RadarData;
	int nRt = g_Rsds_RR.GetData(RadarData);
	if (/*nRt != 1*/nRt <= 0 || nRt >=10)
	{
		if (g_bIsDisp)
		{
			printf("Get Rsds_RR Data error!!!\n");
		}
		return 0;
	}
	RadarData.nLidarRadarType = RADAR_TYPE_RSDS_RR;
	int nValidCont = 0;
	for (int i = 0; i < 64; i++)
	{
		if (!RadarData.Objects[i].bValid)
			continue;
		RadarData.Objects[i].bValid = RADAR_TYPE_RSDS_RR;
		g_SensePack.RadarList.push_back(RadarData.Objects[i]);
		nValidCont++;
	}
	g_SensePack.ContRadarObj = g_SensePack.RadarList.size();
	if (g_bIsDisp)
	{
		printf("Rsds_RR objects cont: %d\n", nValidCont);
	}
	return 1;
}

int InputRsdsFLData()
{
	LIDAR_RADAR_INFO RadarData;
	int nRt = g_Rsds_FL.GetData(RadarData);
	if (/*nRt != 1*/nRt <= 0 || nRt >=10)
	{
		if (g_bIsDisp)
		{
			printf("Get Rsds_FL Data error!!!\n");
		}
		return 0;
	}
	RadarData.nLidarRadarType = RADAR_TYPE_RSDS_FL;
	int nValidCont = 0;
	for (int i = 0; i < 64; i++)
	{
		if (!RadarData.Objects[i].bValid)
			continue;
		RadarData.Objects[i].bValid = RADAR_TYPE_RSDS_FL;
		g_SensePack.RadarList.push_back(RadarData.Objects[i]);
		nValidCont++;
	}
	g_SensePack.ContRadarObj = g_SensePack.RadarList.size();
	if (g_bIsDisp)
	{
		printf("Rsds_FL objects cont: %d\n", nValidCont);
	}
	return 1;
}

int InputRsdsFRData()
{
	LIDAR_RADAR_INFO RadarData;
	int nRt = g_Rsds_FR.GetData(RadarData);
	if (/*nRt != 1*/nRt <= 0 || nRt >=10)
	{
		if (g_bIsDisp)
		{
			printf("Get Rsds_FR Data error!!!\n");
		}
		return 0;
	}
	RadarData.nLidarRadarType = RADAR_TYPE_RSDS_FR;
	int nValidCont = 0;
	for (int i = 0; i < 64; i++)
	{
		if (!RadarData.Objects[i].bValid)
			continue;
		RadarData.Objects[i].bValid = RADAR_TYPE_RSDS_FR;
		g_SensePack.RadarList.push_back(RadarData.Objects[i]);
		nValidCont++;
	}
	g_SensePack.ContRadarObj = g_SensePack.RadarList.size();
	if (g_bIsDisp)
	{
		printf("Rsds_FR objects cont: %d\n", nValidCont);
	}
	return 1;
}

void ResetPack(LCM_SENSOR_FUSION_PACKAGE& SensePack)
{
	LCM_SENSOR_FUSION_PACKAGE SensePackTemp;
	SensePack = SensePackTemp;
}

int InputGpsData(LCM_GPS_DATA& gps)
{
	LCM_GPS_DATA GpsData;
	long nRt = g_PosData.GetData(GpsData);
	if (0 == nRt)
	{
		printf("Get Gps Data errer! quit!!\n"); 
		return 0;
	}
	gps = GpsData;
	if (g_bIsDisp)
	{
		printf("%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.2x,%d\n",
			GpsData.GPS_TIME,
			GpsData.GPS_HEADING,
			GpsData.GPS_PITCH,
			GpsData.GPS_ROLL,
			GpsData.GPS_LATITUDE,
			GpsData.GPS_LONGITUDE,
			GpsData.GPS_ALTITUDE,
			GpsData.GPS_VE,
			GpsData.GPS_VN,
			GpsData.GPS_VU,
			GpsData.GPS_BASELINE,
			GpsData.GPS_NSV1,
			GpsData.GPS_NSV2,
			GpsData.GPS_STATE,
			GpsData.GPS_DATATYPE);
	}

	return 1;
}

LCM_GPS_DATA PredictGpsData(LCM_GPS_DATA& gps, double dPreTime)
{
	LCM_GPS_DATA GpsOut = gps;
	Point2d PtOri(gps.GPS_LATITUDE, gps.GPS_LONGITUDE);
	Point2d PtDelta;
	PtDelta.x = gps.GPS_VE*dPreTime;
	PtDelta.y = gps.GPS_VN*dPreTime;
	Point2d PtPre = Point2GisWgsPoint(PtOri,PtDelta);
	GpsOut.GPS_LATITUDE = PtPre.x;
	GpsOut.GPS_LONGITUDE = PtPre.y;
	return GpsOut;
}