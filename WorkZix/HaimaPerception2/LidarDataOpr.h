#include <Windows.h>
#include "IBEO_OBJECTS_LIST.hpp"
#include "LIDAR_CLOUD_POINTS.hpp"
#include "GPS_DATA.hpp"
#include "LIDAR_RADAR_INFO.hpp"
using namespace std;

#define MAX_BUFFER_SIZE 10000000  //10MBytes

class CLidarDataOpr 
{
public:
	CLidarDataOpr(char* pszPath);
	~CLidarDataOpr(void);
	long CreateDir();
	long WriteLog(LIDAR_CLOUD_POINTS* pLidarPoints, GPS_DATA* pImuInfo);
	long WriteLog(IBEO_OBJECTS_LIST* pObjectList, GPS_DATA* pImuInfo);
	long ReadLog(LIDAR_CLOUD_POINTS* pLidarPoints, GPS_DATA* pImuInfo);
	long ReadLog(IBEO_OBJECTS_LIST* pObjectList, GPS_DATA* pImuInfo);
	long LcmSendLog(LIDAR_CLOUD_POINTS* pLidarPoints, GPS_DATA* pImuInfo);

private:
	void CLidarDataOpr::AddTimeTail(char* psz);

	long m_nInd;
	char m_szPath[MAX_PATH];
	char m_szPathRead[MAX_PATH];
	char* m_pBuffer;
};

