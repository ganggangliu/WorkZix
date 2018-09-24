#ifndef IBEO_ECU_IMPL_H
#define IBEO_ECU_IMPL_H

#include <ibeosdk/ecu.hpp>
#include <ibeosdk/IpHelper.hpp>

#include <ibeosdk/datablocks/commands/CommandEcuAppBaseStatus.hpp>
#include <ibeosdk/datablocks/commands/ReplyEcuAppBaseStatus.hpp>
#include <ibeosdk/datablocks/commands/CommandEcuAppBaseCtrl.hpp>
#include <ibeosdk/datablocks/commands/EmptyCommandReply.hpp>


#include <ibeosdk/listener/DataListener.hpp>

#include <iostream>
#include <cstdlib>
#include <string>
using namespace ibeosdk;
using namespace std;

#include "IbeoOpr.h"

class AllEcuListener : public ibeosdk::DataListener<ibeosdk::ScanEcu>,
	public ibeosdk::DataListener<ObjectListEcu>,
	public ibeosdk::DataListener<ObjectListEcuEt>,
	public ibeosdk::DataListener<Image>,
	public ibeosdk::DataListener<PositionWgs84_2604>,
	public ibeosdk::DataListener<VehicleStateBasicEcu2806>,
	public ibeosdk::DataListener<VehicleStateBasicEcu>,
	public ibeosdk::DataListener<MeasurementList2821>,
	public ibeosdk::DataListener<DeviceStatus>,
	public ibeosdk::DataListener<DeviceStatus6303>,
	public ibeosdk::DataListener<LogMessageError>,
	public ibeosdk::DataListener<LogMessageDebug>,
	public ibeosdk::DataListener<LogMessageNote>,
	public ibeosdk::DataListener<LogMessageWarning> 
{
public:
	AllEcuListener();
	virtual ~AllEcuListener();

public:
	virtual void onData(const ScanEcu* const scan);
	virtual void onData(const ObjectListEcu* const objectList);
	virtual void onData(const ObjectListEcuEt* const objectList);
	virtual void onData(const Image* const image);
	void onData(const PositionWgs84_2604* const wgs84);
	virtual void onData(const VehicleStateBasicEcu2806* const vsb);
	virtual void onData(const VehicleStateBasicEcu* const vsb);
	void onData(const MeasurementList2821* const ml);
	virtual void onData(const DeviceStatus* const devStat);
	virtual void onData(const DeviceStatus6303* const devStat);
	virtual void onData(const LogMessageError* const logMsg);
	virtual void onData(const LogMessageWarning* const logMsg);
	virtual void onData(const LogMessageNote* const logMsg);
	virtual void onData(const LogMessageDebug* const logMsg);

public:
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

	int m_nObjCallBackInd;
	int m_nObjFrameInd;
	HANDLE m_hHandle;
};


#endif