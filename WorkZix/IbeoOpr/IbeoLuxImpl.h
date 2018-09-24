#ifndef IBEO_LUX_IMPL_H
#define IBEO_LUX_IMPL_H

#include <ibeosdk/lux.hpp>
#include <ibeosdk/IpHelper.hpp>

#include <ibeosdk/datablocks/commands/CommandLuxReset.hpp>
#include <ibeosdk/datablocks/commands/CommandLuxGetStatus.hpp>
#include <ibeosdk/datablocks/commands/CommandLuxGetParameter.hpp>
#include <ibeosdk/datablocks/commands/CommandLuxSetParameter.hpp>
#include <ibeosdk/datablocks/commands/EmptyCommandReply.hpp>
#include <ibeosdk/datablocks/commands/ReplyLuxGetStatus.hpp>
#include <ibeosdk/datablocks/commands/ReplyLuxGetParameter.hpp>
#include <ibeosdk/datablocks/commands/CommandLuxSetNtpTimestampSync.hpp>


#include <iostream>
#include <cstdlib>
#include <string>
using namespace ibeosdk;
using namespace std;

#include "IbeoOpr.h"

class AllLuxListener : public ibeosdk::DataListener<ScanLux>,
	public ibeosdk::DataListener<ObjectListLux>,
	public ibeosdk::DataListener<VehicleStateBasicLux>
{
public:
	AllLuxListener();
	~AllLuxListener();
	void onData(const ScanLux* const scan);
	void onData(const ObjectListLux* const pObj);
	void onData(const VehicleStateBasicLux* const vsb);

	void Init(char* pszIp, IBEO_DATA_TYPE nType);
	void SetCallBackPoints(lpIbeoDataRecFunc hCallBack, void* pUser = NULL);
	void SetCallBackObjects(lpIbeoDataRecFunc hCallBack, void* pUser = NULL);
	void SetCallBackVehicleState(lpIbeoDataRecFunc hCallBack, void* pUser = NULL);
	int Start();
 	long GetPoints(LCM_IBEO_CLOUD_POINTS& points);
 	long GetObjects(LCM_IBEO_OBJECT_LIST& objects);
 	long GetVehicleState(LCM_IBEO_VEHICLE_STATE& VehicleState);

friend static DWORD WINAPI IbeothreadReadData(LPVOID pParam);

private:
	void runIbeoDataThread();
	string m_szIp;
	IBEO_DATA_TYPE m_nType;

	bool m_bIsNewPoints;
	LCM_IBEO_CLOUD_POINTS m_CurPoints;
	lpIbeoDataRecFunc m_CallBackPoints;
	void* m_pUserPoints;
	CRITICAL_SECTION m_mutexPoints;

	bool m_bIsNewObjects;
	LCM_IBEO_OBJECT_LIST m_CurObjects;
	lpIbeoDataRecFunc m_CallBackObjects;
	void* m_pUserObjects;
	CRITICAL_SECTION m_mutexObjects;

	bool m_bIsNewVS;
	LCM_IBEO_VEHICLE_STATE m_VehicleState;
	lpIbeoDataRecFunc m_CallBackVehicleState;
	void* m_pUserVehicleState;
	CRITICAL_SECTION m_mutexVehicleState;

	int m_nObjFrameInd;
	HANDLE m_hHandle;
};


#endif