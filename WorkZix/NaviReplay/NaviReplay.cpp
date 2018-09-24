#include <Windows.h>
#include <vector>
#include <fstream>
#include "LCM_NAVI_REQUIRE_INFO.hpp"
#include "LcmReceiver.h"

using namespace std;

CLcmRevicer<LCM_NAVI_REQUIRE_INFO> g_Send(string("LCM_NAVI_REQUIRE_INFO"));
vector<LCM_GPS_DATA> ReadTrackLog(char* pszPath);
long ParseImuData(char* pBuffer, LCM_GPS_DATA* pImu);

int main(int argc, char* argv[])
{
	vector<LCM_GPS_DATA> VecGps;
	VecGps = ReadTrackLog("C:\\Users\\xinzi\\Desktop\\Tracks1015\\hm20161015.htrace");

	long nCont = 0;
	while(1)
	{
		for (int i = 50; i < 350; i++)
		{
			LCM_NAVI_REQUIRE_INFO Req;
			Req.MsgInd = 0;
			Req.Gps = VecGps[i];
			g_Send.Send(string("LCM_NAVI_REQUIRE_INFO"),Req);
			printf("%d\n",i);
			Sleep(100);
		}
	}
	

	return 0;
}

vector<LCM_GPS_DATA> ReadTrackLog(char* pszPath)
{
	vector<LCM_GPS_DATA> Imu0;
	std::ifstream fCloudPt;
	fCloudPt.open(pszPath);
	char szLine[1024] = {0};
	while(fCloudPt.getline(szLine,1024))
	{
		LCM_GPS_DATA gpsT;
		ParseImuData(szLine,&gpsT);
		Imu0.push_back(gpsT);
	}

	return Imu0;
}

long ParseImuData(char* pBuffer, LCM_GPS_DATA* pImu)
{
	char* pszLocal = strchr(pBuffer,',') + 1;

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_TIME = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_HEADING = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_PITCH = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_ROLL = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_LATITUDE = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_LONGITUDE = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_ALTITUDE = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_VE = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_VN = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_VU = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_BASELINE = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_NSV1 = atoi(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_NSV2 = atoi(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_STATE = atoi(pszLocal);

	return 0;
}