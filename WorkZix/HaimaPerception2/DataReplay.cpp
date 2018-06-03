#include <windows.h>
#include "OpenCVInc.h"
#include "LidarDataOpr.h"
#include "GpsManager.h"
#include <fstream> 
using namespace std;

#if _DEBUG
#pragma comment(lib,"GpsManagerD.lib")
#else
#pragma comment(lib,"GpsManager.lib")
#endif

CLidarDataOpr* g_pDataLog = NULL;
long ReadTrackLog(char* pszPath, vector<GPS_DATA>* pImu);
long ParseImuData(char* pBuffer, GPS_DATA* pImu);

int main(int argc, char* argv[])
{
	char szDataPath[MAX_PATH] = {0};
	char szGpsPath[MAX_PATH] = {0};
	strcpy(szDataPath,argv[1]);
	sprintf(szGpsPath,"%sTrack.txt",szDataPath);
	g_pDataLog = new CLidarDataOpr(szDataPath);

	int i = 0;
	while(1)
	{
//		LIDAR_CLOUD_POINTS
//		g_pDataLog->ReadLog();
	}

	return 0;
}

long ReadTrackLog(char* pszPath, vector<GPS_DATA>* pImu)
{
	pImu->clear();
	std::ifstream g_fCloudPt;
	g_fCloudPt.open(pszPath);
	char szLine[1024] = {0};
	while(g_fCloudPt.getline(szLine,1024))
	{
		GPS_DATA gpsT;
		ParseImuData(szLine,&gpsT);
		pImu->push_back(gpsT);
	}
	
	return pImu->size();
}

long ParseImuData(char* pBuffer, GPS_DATA* pImu)
{
	char* pszLocal = pBuffer;
	pImu->GPS_TIME = atof(pszLocal);

	pszLocal = strchr(pBuffer,',') + 1;
	pImu->GPS_HEADING = atof(pszLocal);

	pszLocal = strchr(pBuffer,',') + 1;
	pImu->GPS_PITCH = atof(pszLocal);

	pszLocal = strchr(pBuffer,',') + 1;
	pImu->GPS_ROLL = atof(pszLocal);

	pszLocal = strchr(pBuffer,',') + 1;
	pImu->GPS_LATITUDE = atof(pszLocal);

	pszLocal = strchr(pBuffer,',') + 1;
	pImu->GPS_LONGITUDE = atof(pszLocal);

	pszLocal = strchr(pBuffer,',') + 1;
	pImu->GPS_ALTITUDE = atof(pszLocal);

	pszLocal = strchr(pBuffer,',') + 1;
	pImu->GPS_VE = atof(pszLocal);

	pszLocal = strchr(pBuffer,',') + 1;
	pImu->GPS_VN = atof(pszLocal);

	pszLocal = strchr(pBuffer,',') + 1;
	pImu->GPS_VU = atof(pszLocal);

	pszLocal = strchr(pBuffer,',') + 1;
	pImu->GPS_BASELINE = atof(pszLocal);

	pszLocal = strchr(pBuffer,',') + 1;
	pImu->GPS_NSV1 = atoi(pszLocal);

	pszLocal = strchr(pBuffer,',') + 1;
	pImu->GPS_NSV2 = atoi(pszLocal);

	pszLocal = strchr(pBuffer,',') + 1;
	pImu->GPS_HDOP = atoi(pszLocal);

	return 0;
}