#include "IDataStore.h"
#include "IMessageCodec.h"
#include "IMessageObserver.h"
#include "IMessageProvider.h"
#include "adm_kotei_api.h"
#include "LibGuiOpenGL.h"
#include "ADM_VEHICLE_STATE.hpp"
#include "LcmReceiver.h"
#include "LCM_NAVI_REQUIRE_INFO.hpp"
#include "LCM_NAVI_TO_SENSE_INFO.hpp"
#include "GpsManager.h"


#if _DEBUG
#pragma comment(lib,"LibGuiOpenGLD.lib")
#else
#pragma comment(lib,"LibGuiOpenGL.lib")
#endif

CLcmRevicer<LCM_NAVI_REQUIRE_INFO> g_NaviReq(string("LCM_NAVI_REQUIRE_INFO"));
CLcmRevicer<LCM_NAVI_TO_SENSE_INFO> g_NavRec(string("LCM_NAVI_TO_SENSE_INFO"));
CGpsManager g_GpsRec;

LIBGUIOPENGL_HANDLE g_Opengl = GetLibGuiOpenGL();

void DrawLaneLine(LCM_NAVI_TO_SENSE_INFO& NaviData);

void WINAPI NaviCallBack(void* pData, void* pUser)
{
	LCM_NAVI_TO_SENSE_INFO* pData_ = (LCM_NAVI_TO_SENSE_INFO*)pData;
	printf("Navi data recieved: %d\n",pData_->MsgInd);
	printf("Lane cont: %d, Line cont: %d\n",pData_->Paths.size(), pData_->Lines.size());
	DrawLaneLine(*pData_);
}

int SampleForADM_KOTEI_API();

long g_nCont = 0;
int main(int argc, char* argv[])
{
	g_NavRec.SetCallBack(NaviCallBack);
	g_NavRec.Start();
	SampleForADM_KOTEI_API();

	g_GpsRec.Init(CGpsManager::GPS_MANAGER_TYPE_IMU);
	g_GpsRec.Start();
	while(1)
	{
		Sleep(100);
		LCM_NAVI_REQUIRE_INFO Req;
		int nRt = g_GpsRec.GetData(Req.Gps);
		if (nRt <= 0)
		{
			Sleep(1);
			continue;
		}

		g_nCont++;
		Req.MsgInd = g_nCont;
		g_NaviReq.Send(string("LCM_NAVI_REQUIRE_INFO"), Req);
		printf("Navi require send: %d\n",Req.MsgInd);
	}

	return 0;
}

int SampleForADM_KOTEI_API()
{
	g_Opengl->SetCameraMode(1);
	g_Opengl->SetScale(1);
	g_Opengl->SetAxisLenth(1);
	Mat RefPoint = Mat::eye(4,4,CV_64F);
	g_Opengl->AddRefPoints(RefPoint);
	//³õÊ¼»¯ADMÄ£¿é
	if (0 != ADM_KOTEI_API_Initialize())
	{
		return -1;
	}

	int nRt = 0;
	long nCont = 0;
	while(1)
	{
		Av2HR_ADM_VEHICLE_STATE VS;

		Sleep(100);
		if (Av2HR_getVehicleState(&VS) < 0)
		{
			printf("Av2HR_getVehicleState error!\n");
			continue;
		}

		nCont++;

		LCM_NAVI_REQUIRE_INFO Req;
		Req.MsgInd = nCont;
		Req.Gps.GPS_LATITUDE = VS.m_iLatitude * 0.0000001;
		Req.Gps.GPS_LONGITUDE = VS.m_iLongitude * 0.0000001;
		Req.Gps.GPS_HEADING = 90.0 - (VS.m_iHeading * 0.1);
		Req.Gps.GPS_TIME = VS.m_tmSinceEpoch/1000.0;
		Req.Gps.GPS_VE = VS.m_iSpeed * 0.1;
		g_NaviReq.Send(string("LCM_NAVI_REQUIRE_INFO"), Req);
		printf("Navi require send: %d\n",Req.MsgInd);
	}



	return 1;
}

void DrawLaneLine(LCM_NAVI_TO_SENSE_INFO& NaviData)
{
	g_Opengl->ClearLines();
	g_Opengl->ClearBreakLines();
	g_Opengl->ClearSphere();

	for (int i = 0; i < NaviData.Lines.size(); i++)
	{
		LCM_NAVI_LINE& LineT = NaviData.Lines[i];
		vector<Vec3d> VecLine;
		for (int j = 0; j < LineT.Line.size(); j++)
		{
			LCM_POINT2D_F& ptT = LineT.Line[j];
			Vec3d pt(ptT.x,ptT.y,0);
			VecLine.push_back(pt);
		}
		Mat Lines(VecLine.size(),3,CV_64F,VecLine.data());
		if (LineT.LineType == 1)
		{
			g_Opengl->AddBreakLinesRelative(Lines.clone());
		}
		else
		{
			g_Opengl->AddLinesRelative(Lines.clone());
		}
	}

	for (int i = 0; i < NaviData.Paths.size(); i++)
	{
		LCM_NAVI_PATH& PathT = NaviData.Paths[i];
		vector<Vec3d> VecLine;
		Mat color;
		if (PathT.PathType == 1)
		{
			color = (Mat_<uchar>(1,3)<<0,0,255);//blue
		}
		else
		{
			color = (Mat_<uchar>(1,3)<<0,255,0);//green
		}
		for (int j = 0; j < PathT.Path.size(); j++)
		{
			LCM_POINT2D_F& ptT = PathT.Path[j];
			Vec3d pt(ptT.x,ptT.y,0);
			VecLine.push_back(pt);
		}
		Mat Lines(VecLine.size(),3,CV_64F,VecLine.data());
		g_Opengl->AddLinesRelative(Lines.clone(),Mat(),color);
	}

	for (unsigned int i = 0; i < NaviData.Objs.size(); i++)
	{
		LCM_NAVI_OBJECT& ObjRef = NaviData.Objs[i];
		Mat color = (Mat_<uchar>(1,3)<<255,255,255);
		if (ObjRef.nMajorType == 6)
			color = (Mat_<uchar>(1,3)<<255,0,0);
		if (ObjRef.nMajorType == 7)
			color = (Mat_<uchar>(1,3)<<255,255,0);
		if (ObjRef.nMajorType == 8)
			color = (Mat_<uchar>(1,3)<<0,255,0);
		float sz = 1.0;
		Point3d pt(ObjRef.points[0].x, ObjRef.points[0].y, ObjRef.points[0].z);	
		g_Opengl->AddSphereRelative(pt, sz, Mat(), color);
	}

}
