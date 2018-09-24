#ifndef IBEO_OPR_H
#define IBEO_OPR_H

#include "LCM_IBEO_CLOUD_POINTS.hpp"
#include "LCM_IBEO_OBJECT_LIST.hpp"
#include "LCM_IBEO_VEHICLE_STATE.hpp"

using namespace std;

#ifdef IBEOOPR_EXPORTS
#define IBEOOPR_API __declspec(dllexport)
#else
#define IBEOOPR_API __declspec(dllimport)
#endif

typedef void(__stdcall *lpIbeoDataRecFunc)(void*,void*);

enum IBEOOPR_API IBEO_DATA_TYPE
{
	LCM_DATA_TYPE = 1,	//Get ibeo data from EtherNet by Lcm
	LUX_4_LINE = 4,		//Ibeo-4L
	LUX_8_LINE = 8,		//Ibeo-8L
	ECU_CONFUTION_SYSTEM = 16	//Ibeo fusion system(ECU)
};

class IBEOOPR_API CIbeoOpr 
{
public:
	//Set callback function to get ibeo data(Optional)
	virtual void SetCallBackPoints(lpIbeoDataRecFunc  hCallBack, void* pUser = NULL) = 0;
	virtual void SetCallBackObjects(lpIbeoDataRecFunc  hCallBack, void* pUser = NULL) = 0;
	virtual void SetCallBackVehicleState(lpIbeoDataRecFunc  hCallBack, void* pUser = NULL) = 0;

	//Start receive ibeo data
	virtual int Start() = 0;

	//Get ibeo data
 	virtual int GetPointsData(LCM_IBEO_CLOUD_POINTS& CloudPoints) = 0;
 	virtual int GetObjectsData(LCM_IBEO_OBJECT_LIST& Objects) = 0;
 	virtual int GetVehicleState(LCM_IBEO_VEHICLE_STATE& VehicleState) = 0;

	virtual void Release() = 0;
};

typedef CIbeoOpr* hIbeoHandle;

hIbeoHandle IBEOOPR_API CreateIbeoHandle(char* pszIp, IBEO_DATA_TYPE nType);
void IBEOOPR_API ReleaseIbeoHandle(hIbeoHandle& hHandle);

#endif