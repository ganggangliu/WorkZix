#include <Windows.h>
#include "GpsManager.h"
#include "LIDAR_CLOUD_POINTS.hpp"

class CLidarDataOpr 
{
public:
	CLidarDataOpr(char* pszPath);
	~CLidarDataOpr(void);
	long CreateDir();
	long WriteLog(LIDAR_CLOUD_POINTS* pLidarPoints, CGpsManager::GPS_IMU_INFO* pImuInfo);
	long ReadLog(LIDAR_CLOUD_POINTS* pLidarPoints, CGpsManager::GPS_IMU_INFO* pImuInfo);
	long LcmSendLog(LIDAR_CLOUD_POINTS* pLidarPoints, CGpsManager::GPS_IMU_INFO* pImuInfo);

	void SetInd(long nInd);

private:
	void CLidarDataOpr::AddTimeTail(char* psz);

	long m_nInd;
	char m_szPath[MAX_PATH];
	char m_szPathRead[MAX_PATH];
	FILE* m_fWrite;
	FILE* m_fRead;
};

