#include <Windows.h>
#include "LcmReceiver.h"
#include "LCM_POINT2D_F.hpp"
#include "LCM_GPS_DATA.hpp"
#include "OpenCVInc.h"

using namespace std;

string szChannelRosPos("LIDAR_SLAM");
class RosPose
{
public:
	RosPose(){}
	RosPose(double x_, double y_, double z_, double w_)
	{
		x = x_;
		y = y_;
		z = z_;
		w = w_;
	}
	double x;
	double y;
	double z;
	double w;
};
CLcmRevicer<LCM_POINT2D_F> g_RosRec(szChannelRosPos,LCM_BINARY_TYPE);

string szChannelTrans("LCM_GPS_DATA");
CLcmRevicer<LCM_GPS_DATA> g_Send(szChannelTrans);

RosPose g_GpsBase(29.6737265, 106.4788911, 0, 312.907);

void WINAPI RosDataCallBack(void* pData, void* pUser)
{
	vector<char>* pData_ = (vector<char>*) pData;
	RosPose Pose;
	memcpy(&Pose, pData_->data(), sizeof(RosPose));

	RosPose GroundCart;
	double dAng = g_GpsBase.w/180.f*CV_PI;
	GroundCart.x = Pose.x*cos(dAng) - Pose.y*sin(dAng);
	GroundCart.y = Pose.x*sin(dAng) + Pose.y*cos(dAng);

	LCM_GPS_DATA GpsData;
	GpsData.GPS_LATITUDE = GroundCart.y / 111319.55 + g_GpsBase.x;
	GpsData.GPS_LONGITUDE = GroundCart.x / 111319.55 / cos(g_GpsBase.x/180.0*CV_PI) + g_GpsBase.y;
	GpsData.GPS_HEADING = g_GpsBase.w + Pose.w;
	while(GpsData.GPS_HEADING >= 360.0)
		GpsData.GPS_HEADING -= 360.0;
	while(GpsData.GPS_HEADING < 0.0)
		GpsData.GPS_HEADING += 360.0;
	g_Send.Send(szChannelTrans,GpsData);
}

int main(int argc, char* argv[])
{
	g_RosRec.SetCallBack(RosDataCallBack);
	g_RosRec.Start();

	Sleep(INFINITE);

	return 0;
}

