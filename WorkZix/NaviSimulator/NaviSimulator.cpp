#include "LcmReceiver.h"
#include "LCM_NAVI_REQUIRE_INFO.hpp"
#include "LCM_NAVI_TO_SENSE_INFO.hpp"
#include "VirtualNavi.h"
#include "LibGuiOpenGL.h"
#include "ADM_VEHICLE_STATE.hpp"
#include "LcmReceiver.h"

#if _DEBUG
#pragma comment(lib,"LibGuiOpenGLD.lib")
#else
#pragma comment(lib,"LibGuiOpenGL.lib")
#endif

LIBGUIOPENGL_HANDLE g_hOpengl = GetLibGuiOpenGL();
CVirtualNavi g_Navi;
CLcmRevicer<LCM_NAVI_TO_SENSE_INFO> g_LcmSend(string("LCM_NAVI_TO_SENSE_INFO"));

bool g_bIsBusy = false;
void WINAPI LcmRecFuncEx(void* pData, void* pUser)
{
	if (g_bIsBusy)
	{
		printf("Virtual navy is busy! quit");
		return;
	}
	g_bIsBusy = true;
	LCM_NAVI_REQUIRE_INFO* pRequie = (LCM_NAVI_REQUIRE_INFO*)pData;
	LCM_NAVI_TO_SENSE_INFO Result = g_Navi.TransData(*pRequie);
	g_Navi.TrafficLightRequire(Result);
	Result.MsgInd = pRequie->MsgInd;
	g_LcmSend.Send(string("LCM_NAVI_TO_SENSE_INFO"),Result);
	cout << "Frame Send:" << Result.MsgInd << endl;
	cout << "Lane cont:" << Result.Paths.size() << endl;

	g_hOpengl->AddRefPoints(Result.RefLocale.x, Result.RefLocale.y, Result.RefHeading);
	g_bIsBusy = false;
}

int main(int argc, char* argv[])
{
//	g_Navi.Init("C:\\Users\\xinzi\\Desktop\\Tracks1015\\hm20161015.htrace");
//	g_Navi.AddBranch("C:\\Users\\xinzi\\Desktop\\Tracks1015\\hm20161015_1.htrace",-1);

	g_Navi.Init(argv[1]);
	if (argc > 2)
	{
		g_Navi.AddObjects(argv[2]);
	}
	if (argc > 3)
	{
		g_Navi.AddBranch(argv[3],atoi(argv[4]));
	}
	if (argc > 5)
	{
		g_Navi.AddBranch(argv[5],atoi(argv[6]));
	}
	
	for (int i = 0; i < g_Navi.m_Branches.size(); i++)
	{
		CVirtualLane& Path = g_Navi.m_Branches[i];
		for (int j = 0; j < Path.VecGps.size(); j++)
		{
			Mat RefPt = Mat::eye(4,4,CV_64F);
			RefPt.at<double>(0,3) = Path.VecGps[j].GPS_LATITUDE;
			RefPt.at<double>(1,3) = Path.VecGps[j].GPS_LONGITUDE;
			g_hOpengl->AddRefPoints(RefPt, (Mat_<char>(1,3)<<0,0,255));
			Mat PtL = (Mat_<double>(1,3) << Path.VecLineLeft[j].GPS_LATITUDE, Path.VecLineLeft[j].GPS_LONGITUDE, 0);
			g_hOpengl->AddPointsRelative(PtL,Mat(),(Mat_<char>(1,3) << 255,255,255));
			Mat PtR = (Mat_<double>(1,3) << Path.VecLineRight[j].GPS_LATITUDE, Path.VecLineRight[j].GPS_LONGITUDE, 0);
			g_hOpengl->AddPointsRelative(PtR,Mat(),(Mat_<char>(1,3) << 255,255,255));
		}

		for (int j = Path.nBranchStart; j <= Path.nBranchEnd; j++)
		{
			Mat RefPt = Mat::eye(4,4,CV_64F);
			RefPt.at<double>(0,3) = g_Navi.m_MainLane.VecGps[j].GPS_LATITUDE;
			RefPt.at<double>(1,3) = g_Navi.m_MainLane.VecGps[j].GPS_LONGITUDE;
			g_hOpengl->AddRefPoints(RefPt, (Mat_<char>(1,3)<<0,0,255));
		}
	}

	for (int i = 0; i < g_Navi.m_MainLane.VecGps.size(); i++)
	{
		Mat RefPt = Mat::eye(4,4,CV_64F);
		RefPt.at<double>(0,3) = g_Navi.m_MainLane.VecGps[i].GPS_LATITUDE;
		RefPt.at<double>(1,3) = g_Navi.m_MainLane.VecGps[i].GPS_LONGITUDE;
		g_hOpengl->AddRefPoints(RefPt, (Mat_<char>(1,3)<<0,255,0));
		Mat PtL = (Mat_<double>(1,3) << g_Navi.m_MainLane.VecLineLeft[i].GPS_LATITUDE, g_Navi.m_MainLane.VecLineLeft[i].GPS_LONGITUDE, 0);
		g_hOpengl->AddPointsRelative(PtL,Mat(),(Mat_<char>(1,3) << 255,255,255));
		Mat PtR = (Mat_<double>(1,3) << g_Navi.m_MainLane.VecLineRight[i].GPS_LATITUDE, g_Navi.m_MainLane.VecLineRight[i].GPS_LONGITUDE, 0);
		g_hOpengl->AddPointsRelative(PtR,Mat(),(Mat_<char>(1,3) << 255,255,255));
	}

	for (int i = 0; i < g_Navi.m_ObjectList.m_ObjListTrans.rows; i++)
	{
		Mat& ObjList = g_Navi.m_ObjectList.m_ObjListTrans;
		Point3d pt(ObjList.at<double>(i,1), ObjList.at<double>(i,2), 2);
		Mat color = (Mat_<char>(1,3) << 255,255,255);
		if ((int)ObjList.at<double>(i,4) == 6)
		{
			color = (Mat_<char>(1,3) << 255,0,0);
		}
		if ((int)ObjList.at<double>(i,4) == 7)
		{
			color = (Mat_<char>(1,3) << 255,255,0);
		}
		g_hOpengl->AddSphereRelative(pt, 3, Mat(), color);
	}

	CLcmRevicer<LCM_NAVI_REQUIRE_INFO> LcmReq(string("LCM_NAVI_REQUIRE_INFO"));
	LcmReq.SetCallBack(LcmRecFuncEx, NULL);
	LcmReq.Start();

	g_hOpengl->SetCameraMode(AUTO_FOCUS);
	g_hOpengl->SetScale(1.0);

	Sleep(INFINITE);

	return 0;
}

