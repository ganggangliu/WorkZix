#ifndef IBEOLIDAROPR_H
#define IBEOLIDAROPR_H

#include "LCM_IBEO_CLOUD_POINTS.hpp"
#include "LCM_IBEO_OBJECT_LIST.hpp"
#include "LCM_IBEO_VEHICLE_STATE.hpp"

#include <vector>
using namespace std;


typedef void(__stdcall *lpIbeoDataRecFunc)(void*,void*);


class CIbeoLidarOpr
{
public:
    CIbeoLidarOpr();
    ~CIbeoLidarOpr();
	void Init(char* pszIp, long nType = 4);
    void SetCallBackPoints(lpIbeoDataRecFunc  hCallBack, void* pUser = NULL);
    void SetCallBackObjects(lpIbeoDataRecFunc  hCallBack, void* pUser = NULL);
	void SetCallBackVehicleState(lpIbeoDataRecFunc  hCallBack, void* pUser = NULL);
    long Start();
    long GetPointsData(LCM_IBEO_CLOUD_POINTS& CloudPoints);
    long GetObjectsData(LCM_IBEO_OBJECT_LIST& Objects);
	long GetObjectsDataRepeat(LCM_IBEO_OBJECT_LIST& Objects);
	long GetVehicleState(LCM_IBEO_VEHICLE_STATE& VehicleState);
    void runIbeoDataThread();
//    void onData(const ScanLux* const scan);

private:
	long m_nType;
    char m_szIp[256];
    void* m_pListener;
    void* m_id;
};

#endif // CIBEOLIDAROPR_H
