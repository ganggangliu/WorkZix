#include "ProjectOpr.h"
#include "LidarSlamOpr.h"
#include "LidarMappingOpr.h"

#if _DEBUG
#pragma comment(lib,"LidarMappingOprD.lib")
#else
#pragma comment(lib,"LidarMappingOpr.lib")
#endif

int main(int argc, char* argv[])
{
	CLidarDataOpr LidarOpr("D:\\测试资料\\激光雷达数据\\20160324113926\\");
	CLidarSlamOpr SlamOpr;
	SlamOpr.Init("D:\\GTWork\\01CODE\\ImageProcessing\\KoteiWorkExZix\\LidarSlamTest\\LidarTicks.yml",-0.03125/*1/32*/);

	double dWheelAngle = 0.0;
	while(1)
	{
		LCM_IBEO_CLOUD_POINTS LidarPoints;
		LCM_GPS_DATA ImuInfo;
		int nRt = LidarOpr.ReadLog(&LidarPoints, &ImuInfo);
		printf("%d\n",nRt);

TEMP:	SlamOpr.AlignLidarDataIbeo(&LidarPoints);
		cvNamedWindow("m_imgPoints",1);
		imshow("m_imgPoints",SlamOpr.m_imgPoints);
		//cvNamedWindow("m_imgPolyArea",1);
		//imshow("m_imgPolyArea",SlamOpr.m_imgPolyArea);
		cvNamedWindow("m_imgPolyAreaErode",1);
		imshow("m_imgPolyAreaErode",SlamOpr.m_imgPolyAreaErode);
  		cvNamedWindow("m_imgPointsErode",1);
  		imshow("m_imgPointsErode",SlamOpr.m_imgPointsErode);

		vector<Point2d> Track;
//		Mat aa = SlamOpr.GetTrackImageEx(SlamOpr.m_imgPointsErode,dWheelAngle,Track);
		Mat aa = SlamOpr.GetBestTrackEx(SlamOpr.m_imgPointsErode,dWheelAngle);
		cout << "dWheelAngle:" << dWheelAngle << endl;

		cvNamedWindow("aa",1);
		imshow("aa",aa);
		char c = waitKey(1);
		if (c == 'a')
		{
			dWheelAngle -= 1.0;
			goto TEMP;
		}
		else if (c == 'd')
		{
			dWheelAngle += 1.0;
			goto TEMP;
		}
		else if (c == 'w')
		{
		}
		else if (c == 's')
		{
		}
		else
		{
		}
	}

	return 0;
}

int main0(int argc, char* argv[])
{
	CLidarDataOpr LidarOpr("D:\\测试资料\\激光雷达数据\\20160324113926\\");
	CLidarSlamOpr SlamOpr;

	while(1)
	{
		LCM_IBEO_CLOUD_POINTS LidarPoints;
		LCM_GPS_DATA ImuInfo;
		int nRt = LidarOpr.ReadLog(&LidarPoints, &ImuInfo);
		printf("%d\n",nRt);
		if (nRt == 3000)
		{
			break;
		}
		SlamOpr.TestTick(&LidarPoints);
	}
	SlamOpr.SaveTick();

	return 0;
}
