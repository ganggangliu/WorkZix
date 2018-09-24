#include "IbeoLuxImpl.h"
#include "IbeoEcuImpl.h"

//Lcm(c++ version) head files must be placed below Ibeo(c version) head files
#include "IbeoOpr.h"
#include "LcmReceiver.h"
#include <string.h>
using namespace std;

//////////////////////////////////////////////////////////////////////////
//Ibeo data on lcm type
//////////////////////////////////////////////////////////////////////////
class CIbeoLcm: public CIbeoOpr
{
public:
	CIbeoLcm();
	~CIbeoLcm();

	void Init(char* pszIp, IBEO_DATA_TYPE nType);
	void SetCallBackPoints(lpIbeoDataRecFunc  hCallBack, void* pUser = NULL);
	void SetCallBackObjects(lpIbeoDataRecFunc  hCallBack, void* pUser = NULL);
	void SetCallBackVehicleState(lpIbeoDataRecFunc  hCallBack, void* pUser = NULL);

	int Start();
	int GetPointsData(LCM_IBEO_CLOUD_POINTS& CloudPoints);
	int GetObjectsData(LCM_IBEO_OBJECT_LIST& Objects);
	int GetVehicleState(LCM_IBEO_VEHICLE_STATE& VehicleState);
	void Release();

private:
	string m_szChannelNamePoint;
	string m_szChannelNameObject;
	string m_szChannelNameVS;
	CLcmRevicer<LCM_IBEO_CLOUD_POINTS> m_LcmPoint;
	CLcmRevicer<LCM_IBEO_OBJECT_LIST> m_LcmObject;
	CLcmRevicer<LCM_IBEO_VEHICLE_STATE> m_LcmVS;
};
CIbeoLcm::CIbeoLcm():
m_szChannelNamePoint("LCM_IBEO_CLOUD_POINTS"),
m_szChannelNameObject("LCM_IBEO_OBJECT_LIST"),
m_szChannelNameVS("LCM_IBEO_VEHICLE_STATE"),
m_LcmPoint(m_szChannelNamePoint),
m_LcmObject(m_szChannelNameObject),
m_LcmVS(m_szChannelNameVS)
{
	return;
}
CIbeoLcm::~CIbeoLcm()
{
	return;
}
void CIbeoLcm::Init(char* pszIp, IBEO_DATA_TYPE nType)
{
	return;
}
void CIbeoLcm::SetCallBackPoints(lpIbeoDataRecFunc  hCallBack, void* pUser)
{
	m_LcmPoint.SetCallBack(hCallBack, pUser);
	return;
}
void CIbeoLcm::SetCallBackObjects(lpIbeoDataRecFunc  hCallBack, void* pUser)
{
	m_LcmObject.SetCallBack(hCallBack, pUser);
	return;
}
void CIbeoLcm::SetCallBackVehicleState(lpIbeoDataRecFunc  hCallBack, void* pUser)
{
	m_LcmVS.SetCallBack(hCallBack, pUser);
	return;
}
int CIbeoLcm::Start()
{
	if (!m_LcmPoint.Start())
		return 0;
	if (!m_LcmObject.Start())
		return 0;
	if (!m_LcmVS.Start())
		return 0;
	
	return 1;
}
int CIbeoLcm::GetPointsData(LCM_IBEO_CLOUD_POINTS& CloudPoints)
{
	return m_LcmPoint.GetData(CloudPoints);
}
int CIbeoLcm::GetObjectsData(LCM_IBEO_OBJECT_LIST& Objects)
{
	return m_LcmObject.GetData(Objects);
}
int CIbeoLcm::GetVehicleState(LCM_IBEO_VEHICLE_STATE& VehicleState)
{
	return m_LcmVS.GetData(VehicleState);
}
void CIbeoLcm::Release()
{
	return;
}

//////////////////////////////////////////////////////////////////////////
//Ibeo data 4/8 line type
//////////////////////////////////////////////////////////////////////////
class CIbeoLux: public CIbeoOpr
{
public:
	CIbeoLux();
	~CIbeoLux();

	void Init(char* pszIp, IBEO_DATA_TYPE nType);
	void SetCallBackPoints(lpIbeoDataRecFunc hCallBack, void* pUser = NULL);
	void SetCallBackObjects(lpIbeoDataRecFunc hCallBack, void* pUser = NULL);
	void SetCallBackVehicleState(lpIbeoDataRecFunc hCallBack, void* pUser = NULL);

	int Start();
	int GetPointsData(LCM_IBEO_CLOUD_POINTS& CloudPoints);
	int GetObjectsData(LCM_IBEO_OBJECT_LIST& Objects);
	int GetVehicleState(LCM_IBEO_VEHICLE_STATE& VehicleState);
	void Release();

private:
	char m_szIp[256];
	IBEO_DATA_TYPE m_nType;
	AllLuxListener m_IbeoLux;
};
CIbeoLux::CIbeoLux()
{
	return;
}
CIbeoLux::~CIbeoLux()
{
	return;
}
void CIbeoLux::Init(char* pszIp, IBEO_DATA_TYPE nType)
{
	m_IbeoLux.Init(pszIp, nType);
}
void CIbeoLux::SetCallBackPoints(lpIbeoDataRecFunc hCallBack, void* pUser)
{
	m_IbeoLux.SetCallBackPoints(hCallBack, pUser);
}
void CIbeoLux::SetCallBackObjects(lpIbeoDataRecFunc hCallBack, void* pUser)
{
	m_IbeoLux.SetCallBackObjects(hCallBack, pUser);
}
void CIbeoLux::SetCallBackVehicleState(lpIbeoDataRecFunc hCallBack, void* pUser)
{
	m_IbeoLux.SetCallBackVehicleState(hCallBack, pUser);
}

int CIbeoLux::Start()
{
	return m_IbeoLux.Start();
}
int CIbeoLux::GetPointsData(LCM_IBEO_CLOUD_POINTS& CloudPoints)
{
	return m_IbeoLux.GetPoints(CloudPoints);
}
int CIbeoLux::GetObjectsData(LCM_IBEO_OBJECT_LIST& Objects)
{
	return m_IbeoLux.GetObjects(Objects);
}
int CIbeoLux::GetVehicleState(LCM_IBEO_VEHICLE_STATE& VehicleState)
{
	return m_IbeoLux.GetVehicleState(VehicleState);
}
void CIbeoLux::Release()
{
	return;
}

//////////////////////////////////////////////////////////////////////////
//Ibeo data form Ecu(ibeo confusin system)
//////////////////////////////////////////////////////////////////////////
class CIbeoEcu: public CIbeoOpr
{
public:
	CIbeoEcu();
	~CIbeoEcu();

	void Init(char* pszIp, IBEO_DATA_TYPE nType);
	void SetCallBackPoints(lpIbeoDataRecFunc hCallBack, void* pUser = NULL);
	void SetCallBackObjects(lpIbeoDataRecFunc hCallBack, void* pUser = NULL);
	void SetCallBackVehicleState(lpIbeoDataRecFunc hCallBack, void* pUser = NULL);

	int Start();
	int GetPointsData(LCM_IBEO_CLOUD_POINTS& CloudPoints);
	int GetObjectsData(LCM_IBEO_OBJECT_LIST& Objects);
	int GetVehicleState(LCM_IBEO_VEHICLE_STATE& VehicleState);
	void Release();

private:
	char m_szIp[256];
	IBEO_DATA_TYPE m_nType;
	AllEcuListener m_IbeoEcu;
};

CIbeoEcu::CIbeoEcu()
{
	return;
}
CIbeoEcu::~CIbeoEcu()
{
	return;
}
void CIbeoEcu::Init(char* pszIp, IBEO_DATA_TYPE nType)
{
	m_IbeoEcu.Init(pszIp, nType);
}
void CIbeoEcu::SetCallBackPoints(lpIbeoDataRecFunc hCallBack, void* pUser)
{
	m_IbeoEcu.SetCallBackPoints(hCallBack, pUser);
}
void CIbeoEcu::SetCallBackObjects(lpIbeoDataRecFunc hCallBack, void* pUser)
{
	m_IbeoEcu.SetCallBackObjects(hCallBack, pUser);
}
void CIbeoEcu::SetCallBackVehicleState(lpIbeoDataRecFunc hCallBack, void* pUser)
{
	m_IbeoEcu.SetCallBackVehicleState(hCallBack, pUser);
}

int CIbeoEcu::Start()
{
	return m_IbeoEcu.Start();
}
int CIbeoEcu::GetPointsData(LCM_IBEO_CLOUD_POINTS& CloudPoints)
{
	return m_IbeoEcu.GetPoints(CloudPoints);
}
int CIbeoEcu::GetObjectsData(LCM_IBEO_OBJECT_LIST& Objects)
{
	return m_IbeoEcu.GetObjects(Objects);
}
int CIbeoEcu::GetVehicleState(LCM_IBEO_VEHICLE_STATE& VehicleState)
{
	return m_IbeoEcu.GetVehicleState(VehicleState);
}
void CIbeoEcu::Release()
{
	return;
}

//////////////////////////////////////////////////////////////////////////

hIbeoHandle CreateIbeoHandle(char* pszIp, IBEO_DATA_TYPE nType)
{
	hIbeoHandle pIbeo = NULL;
	if (nType == LCM_DATA_TYPE)
	{
		pIbeo = new CIbeoLcm;
		((CIbeoLcm*)pIbeo)->Init(pszIp, nType);
	}
	else if (nType == LUX_4_LINE || nType == LUX_8_LINE)
	{
		pIbeo = new CIbeoLux;
		((CIbeoLux*)pIbeo)->Init(pszIp, nType);
	}
	else if (nType == ECU_CONFUTION_SYSTEM)
	{
		pIbeo = new CIbeoEcu;
		((CIbeoEcu*)pIbeo)->Init(pszIp, nType);
	}
	else
	{
		return NULL;
	}

	return pIbeo;
}

void ReleaseIbeoHandle(hIbeoHandle& hHandle)
{
	if (hHandle)
	{
		delete hHandle;
		hHandle = NULL;
	}

	return;
}