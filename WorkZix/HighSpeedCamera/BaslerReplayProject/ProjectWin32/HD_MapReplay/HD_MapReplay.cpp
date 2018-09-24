#include "OpenCVInc.h"
#include <string>
#include "LcmReceiver.h"
#include "GPSLCMDATA.hpp"
#include "UbloxReader.h"

using namespace std;
using namespace cv;

string g_szChanleVehicle("GPSLCMUb370");
string g_szChanleCompare("GPSLCMUblox");
string g_szChanleStart("GPSLCMSTART");

CLcmRevicer<gpslcmdata> g_Vehicle(g_szChanleVehicle);
CLcmRevicer<gpslcmdata> g_Compare(g_szChanleCompare);
CLcmRevicer<gpslcmdata> g_Start(g_szChanleStart);

CUbloxReader g_UbloxOpr;

//获取起始的回调函数
void WINAPI LcmStartCallBack(void* pData, void* pUser)
{
	gpslcmdata* pData_ = (gpslcmdata*)pData;
	double dLat = pData_->dLatitude/1024/3600;
	double dLon = pData_->dLongitude/1024/3600;
	double dHeading = pData_->fAngle;

	printf("Get start point:Lat:%.7f, Lon:%.7f, Heading:%.3f\n", dLat, dLon, dHeading);
}

int main(int argc, char* argv[])
{
	g_Start.SetCallBack(LcmStartCallBack, 0);
	g_Start.Start();

	if (argc == 2)
	{
		g_UbloxOpr.ReadLog(argv[1]);
	}

	vector<CUbloxData>& UbloxData = g_UbloxOpr.GetUbloxInfo();
	for (unsigned int i = 0; i < UbloxData.size(); i++)
	{
		gpslcmdata pos;
		memset(&pos, 0, sizeof(gpslcmdata));
		pos.status = 4;
		pos.dLatitude = UbloxData[i].nLatitude*1e-7;
		pos.dLongitude = UbloxData[i].nLongitude*1e-7;
		pos.fAngle = UbloxData[i].nHeadMot*1e-5;
		g_Vehicle.Send(g_szChanleVehicle, pos);
		printf("%.7f,%.7f,%.5f\n", pos.dLatitude, pos.dLongitude, pos.fAngle);
		Sleep(1000);
	}

	Sleep(INFINITE);

	return 0;
}

