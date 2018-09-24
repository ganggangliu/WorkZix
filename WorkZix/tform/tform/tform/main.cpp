#include "windows.h"
#include <WinBase.h>
#include "variables.h"
#include "lcm/lcm-cpp.hpp"
#include "LcmReceiver.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <math.h>
#include "LCM_SENSOR_FUSION_PACKAGE.hpp"
#include "LCM_BEHAVIOR_PLAN.hpp"
#include "LCM_MOTION_PLAN_TRACE.hpp"
#include "LIDAR_RADAR_INFO.hpp"

using namespace std;
// data to recv
LCM_SENSOR_FUSION_PACKAGE			g_Package;
SIGNAL_BEHAVIOR_PLAN				g_BehaviorPlan;
TRAJECTORY_MOTION_PLAN				g_MotionPlan;

CRITICAL_SECTION					cs_Package;
CRITICAL_SECTION					cs_BehaviorPlan;
CRITICAL_SECTION					cs_MotionPlan;

std::string szURL					= "udpm://239.255.76.67:7667?ttl=1";
std::string PackageChannel			= "LCM_SENSOR_FUSION_PACKAGE";
std::string BehaviorPlanChannel		= "SIGNAL_BEHAVIOR_PLAN";
std::string MotionPlanChannel		= "TRAJECTORY_MOTION_PLAN";

CLcmRevicer<LCM_SENSOR_FUSION_PACKAGE>		lcmReceiver1(szURL, PackageChannel, LCM_TEMPLATE_TYPE);
CLcmRevicer<LCM_GEO_POINT_2D>				lcmReceiver2(szURL, BehaviorPlanChannel, LCM_BINARY_TYPE);
CLcmRevicer<LCM_GEO_POINT_2D>				lcmReceiver3(szURL, MotionPlanChannel, LCM_BINARY_TYPE);
lcm::LCM* g_LCMSender = new lcm::LCM("udpm://239.255.76.67:7667?ttl=1");


void WINAPI OnPackageCallBack(void* lp)
{
	EnterCriticalSection(&cs_Package);

	memcpy(&g_Package, (LCM_SENSOR_FUSION_PACKAGE*)lp, sizeof(LCM_SENSOR_FUSION_PACKAGE));

	printf("GPS ----- lon:%0.7f, lat:%0.7f  \n", g_Package.Gps.GPS_LATITUDE, g_Package.Gps.GPS_LONGITUDE);
	printf("Object ========= count:%d  \n", g_Package.IbeoObjList.IbeoObjects.size());

	int					count = 0;
	LIDAR_RADAR_INFO	Radar;
	for (int n = 0; n < 64; n++){
		Radar.Objects[n].bValid		= 0;
		Radar.Objects[n].fObjLocX	= 0;
		Radar.Objects[n].fObjLocY	= 0;
		Radar.Objects[n].fObjSizeX = 0;
		Radar.Objects[n].fObjSizeY = 0;
	}

	for (int i = 0; i < (int)g_Package.IbeoObjList.IbeoObjects.size(); i++){
		if (i == 64){
			break;
		}

		Radar.Objects[i].bValid		= 1;
		Radar.Objects[i].fObjLocX	= -g_Package.IbeoObjList.IbeoObjects[i].ObjBoxCenter.y/1000.0;
		Radar.Objects[i].fObjLocY	= g_Package.IbeoObjList.IbeoObjects[i].ObjBoxCenter.x/1000.0;
		Radar.Objects[i].fObjSizeX	= 2.0;
		Radar.Objects[i].fObjSizeY	= 2.0;

		count += 1;
	}

	g_LCMSender->publish("LIDAR_RADAR_INFO", &Radar);

	LeaveCriticalSection(&cs_Package);
}

void WINAPI OnBehaviorPlanCallBack(void* lp)
{
	EnterCriticalSection(&cs_BehaviorPlan);

	vector<char>* pVec = (vector<char>*)lp;
	memcpy(&g_BehaviorPlan, pVec->data(), sizeof(SIGNAL_BEHAVIOR_PLAN));

	LCM_BEHAVIOR_PLAN	BehaviorPlan;
	BehaviorPlan.valid			= 1;
	BehaviorPlan.braking		= g_BehaviorPlan.braking;
	BehaviorPlan.avoiding		= g_BehaviorPlan.avoiding;
	BehaviorPlan.turning		= g_BehaviorPlan.turning;
	BehaviorPlan.changing_lane	= g_BehaviorPlan.changing_lane;

	g_LCMSender->publish("LCM_BEHAVIOR_PLAN", &BehaviorPlan);
	printf("BehaviorPlan >>>>>>>> \n");

	LeaveCriticalSection(&cs_BehaviorPlan);
}

void WINAPI OnMotionPlanCallBack(void* lp)
{
	EnterCriticalSection(&cs_MotionPlan);

	vector<char>* pVec = (vector<char>*)lp;
	memcpy(&g_MotionPlan, pVec->data(), sizeof(TRAJECTORY_MOTION_PLAN));

	LCM_MOTION_PLAN_TRACE	MotionPlanTrace;
	MotionPlanTrace.m_longitude	= g_Package.Gps.GPS_LONGITUDE;
	MotionPlanTrace.m_latitude	= g_Package.Gps.GPS_LATITUDE;
	MotionPlanTrace.m_heading	= g_Package.Gps.GPS_HEADING;
	MotionPlanTrace.m_velocity	= sqrt(g_Package.Gps.GPS_VE * g_Package.Gps.GPS_VE + g_Package.Gps.GPS_VN * g_Package.Gps.GPS_VN + g_Package.Gps.GPS_VU * g_Package.Gps.GPS_VU);

	MotionPlanTrace.m_usTracePointNum = 0;
	for (int i = 0; i < g_MotionPlan.count; i++){
		if (i == 200){
			break;
		}
		MotionPlanTrace.m_tracePointArray[i].x = g_MotionPlan.trajectory[i].x;
		MotionPlanTrace.m_tracePointArray[i].y = g_MotionPlan.trajectory[i].y;
		MotionPlanTrace.m_usTracePointNum += 1;
	}

	g_LCMSender->publish("LCM_MOTION_PLAN_TRACE", &MotionPlanTrace);
	printf("MotionPlan >>>>>>>> \n");

	LeaveCriticalSection(&cs_MotionPlan);
}

void main()
{
	InitializeCriticalSection(&cs_Package);
	InitializeCriticalSection(&cs_BehaviorPlan);
	InitializeCriticalSection(&cs_MotionPlan);

	lcmReceiver1.SetCallBack((lpLcmRecFunc)OnPackageCallBack);
	lcmReceiver1.Start();

	lcmReceiver2.SetCallBack((lpLcmRecFunc)OnBehaviorPlanCallBack);
	lcmReceiver2.Start();

	lcmReceiver3.SetCallBack((lpLcmRecFunc)OnMotionPlanCallBack);
	lcmReceiver3.Start();

	printf("listening data...\n");
	printf("please do not press any key while this window is on top\n");

	Sleep(INFINITE);
	//system("pause");
	
}