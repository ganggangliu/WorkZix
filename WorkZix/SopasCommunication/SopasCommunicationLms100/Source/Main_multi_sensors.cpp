#include <windows.h>
#include <process.h>
#include <tlhelp32.h>
#include "SopasOpr.h"
#include "SickLms511DataTransfer.h"
#include "OpenCVInc.h"
#include "LcmReceiver.h"
#include "LIDAR_RADAR_INFO.hpp"


using namespace cv;

int g_nGridFront = 50000;
int g_nGridSide = 30000;
int g_nGridBack = 30000;
int g_nGridSize = 200;
cv::Mat g_img;

string g_szChannleNameEsr = "ESR_TEST_CHANNEL";//"LIDAR_RADAR_INFO_ESR";
string g_szChannleNameRsdsRL = "RSDS_RL_TEST_CHANNEL";//"LIDAR_RADAR_INFO_ESR";
string g_szChannleNameRsdsRR = "RSDS_RR_TEST_CHANNEL";//"LIDAR_RADAR_INFO_ESR";
CLcmRevicer<LIDAR_RADAR_INFO> g_EsrRec(g_szChannleNameEsr);
CLcmRevicer<LIDAR_RADAR_INFO> g_RsdsRLRec(g_szChannleNameRsdsRL);
CLcmRevicer<LIDAR_RADAR_INFO> g_RsdsRRRec(g_szChannleNameRsdsRR);
string g_szChannleNameIbeoObj = "LCM_IBEO_OBJECT_LIST";
CLcmRevicer<LCM_IBEO_OBJECT_LIST> g_IbeoSend(g_szChannleNameIbeoObj);
CSickLms511DataTransfer g_SickLms511DataTransfer(-5.0,0.25);

CSopasParam g_ParamM, g_ParamL, g_ParamR;
CSopasOpr g_SopasM, g_SopasL, g_SopasR;

vector<cv::Point2f> g_BuffM, g_BuffL, g_BuffR ,g_BuffAll;

void DrawPoints(cv::Mat& img, vector<cv::Point2f>& Points, CvScalar color);
void DrawCar(cv::Mat& img, int nFront, int nRear, int nSide);
Point2d CarXY2ImageUV(Point2d CarXY);
void DrawLidar(Mat& img, CSopasParam ParamM, CSopasParam  ParamL, CSopasParam  ParamR);


DWORD GetProcessIDFromName(char* szName)
{
	DWORD id = 0;       // 进程ID
	PROCESSENTRY32 pe;  // 进程信息
	pe.dwSize = sizeof(PROCESSENTRY32);
	HANDLE hSnapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0); // 获取系统进程列表
	if(Process32First(hSnapshot, &pe))      // 返回系统中第一个进程的信息
	{
		do
		{
			char szCharStr[256];
			size_t converted = 0;
			wchar_t szWCahrStr[256];
			wcscpy(szWCahrStr, pe.szExeFile);
			wcstombs_s(&converted, szCharStr, 256, szWCahrStr, _TRUNCATE);
			if(0 == _stricmp(szCharStr, szName)) // 不区分大小写比较
			{
				id = pe.th32ProcessID;
				break;
			}
		}while(Process32Next(hSnapshot, &pe));      // 下一个进程
	}
	CloseHandle(hSnapshot);     // 删除快照
	return id;
}

long g_nCont = 0;
void __stdcall SopasDataCallBackMiddle(vector<cv::Point2f>* pData, void* pUser)
{
	g_nCont++;
	if (g_nCont%2 != 0)
	{
		return;
	}

	LIDAR_RADAR_INFO EsrData;
	int nRt = g_EsrRec.GetData(EsrData);
	if (nRt <= 0)
	{
		printf("Get Esr data failed!\n");
		return;
	}
	else
	{
		printf("Esr data done!\n");
	}

	LIDAR_RADAR_INFO RsdsRLData;
// 	nRt = g_RsdsRLRec.GetData(RsdsRLData);
// 	if (nRt <= 0)
// 	{
// 		printf("Get Rsds RL data failed!\n");
// 		return;
// 	}
// 	else
// 	{
// 		printf("Rsds RL data done!\n");
// 	}

	LIDAR_RADAR_INFO RsdsRRData;
// 	nRt = g_RsdsRRRec.GetData(RsdsRRData);
// 	if (nRt <= 0)
// 	{
// 		printf("Get Rsds RR data failed!\n");
// 		return;
// 	}
// 	else
// 	{
// 		printf("Rsds RR data done!\n");
// 	}

	g_BuffM = *pData;
	g_BuffL.clear();
	g_SopasL.GetData(g_BuffL);
	g_BuffR.clear();
	g_SopasR.GetData(g_BuffR);

	g_img = cv::Mat::zeros((g_nGridFront+g_nGridBack)/g_nGridSize+1,g_nGridSide*2/g_nGridSize+1,CV_8UC3);
	g_img.col((g_nGridSide/g_nGridSize)+1) = cvScalar(255,255,255);
	g_img.row((g_nGridFront/g_nGridSize)+1) = cvScalar(255,255,255);
	DrawCar(g_img, 4550, 1500, 1130);
	DrawLidar(g_img, g_ParamM, g_ParamL, g_ParamR);
	DrawPoints(g_img, g_BuffL, CV_RGB(0, 255, 0));
	DrawPoints(g_img, g_BuffR, CV_RGB(0, 0, 255));
	DrawPoints(g_img, g_BuffM, CV_RGB(255, 0, 0));

	cvNamedWindow("Lidar points");
	imshow("Lidar points", g_img);
	cv::waitKey(1);

	g_BuffAll.clear();
	g_BuffAll.insert(g_BuffAll.begin(), g_BuffL.begin(), g_BuffL.end());
	g_BuffAll.insert(g_BuffAll.begin(), g_BuffR.begin(), g_BuffR.end());
	g_BuffAll.insert(g_BuffAll.begin(), g_BuffM.begin(), g_BuffM.end());

	vector<LIDAR_RADAR_OBJECT_INFO> RadarAll;
	g_SickLms511DataTransfer.InputRadar(RadarAll, EsrData,
		g_SickLms511DataTransfer.m_EsrT,
		g_SickLms511DataTransfer.m_dEsrHeading);
//  	g_SickLms511DataTransfer.InputRadar(RadarAll, RsdsRLData,
//  		g_SickLms511DataTransfer.m_RsdsT,
//  		g_SickLms511DataTransfer.m_dRsdsHeading);
//  	g_SickLms511DataTransfer.InputRadar(RadarAll, RsdsRRData,
//  		g_SickLms511DataTransfer.m_RsdsT,
//  		g_SickLms511DataTransfer.m_dRsdsHeading);
	Mat FutionImg;
	LCM_IBEO_OBJECT_LIST IbeoData = g_SickLms511DataTransfer.ConfusionWithEsr(FutionImg, g_BuffAll, RadarAll);
	g_IbeoSend.Send(g_szChannleNameIbeoObj, IbeoData);
	cvNamedWindow("FutionImg");
	imshow("FutionImg", FutionImg);
	waitKey(1);

//	g_SickLms511DataTransfer.SendData("LCM_IBEO_CLOUD_POINTS", g_BuffAll);

	printf("Frame:%ld M:%d L:%d R:%d\n", g_nCont, g_BuffM.size(), g_BuffL.size(), g_BuffR.size());
}

int main(int argc, char* argv[])
{
// 	int nProId = GetProcessIDFromName("SopasCommunicationLms100D.exe");
// 
// 	HANDLE hHanle = OpenProcess(PROCESS_ALL_ACCESS, FALSE, nProId);
// 
// 	bool bRt = SetPriorityClass(hHanle, REALTIME_PRIORITY_CLASS);

	g_EsrRec.Start();
	g_RsdsRLRec.Start();
	g_RsdsRRRec.Start();

	g_img = cv::Mat::zeros((g_nGridFront+g_nGridBack)/g_nGridSize+1,g_nGridSide*2/g_nGridSize+1,CV_8UC3);
	g_img.col((g_nGridSide/g_nGridSize)+1) = cvScalar(255,255,255);
	g_img.row((g_nGridFront/g_nGridSize)+1) = cvScalar(255,255,255);

	strcpy(g_ParamM.szIp, "169.254.145.90");
	g_ParamM.dHeading = 0.0;
	g_ParamM.T = cv::Point2d(0,4550);
	strcpy(g_ParamL.szIp, "169.254.145.91");
	g_ParamL.dHeading = 90.0;
	g_ParamL.T = cv::Point2d(-1160/*-1130*/,1580/*1560*/);
	g_ParamL.dAngleMin = 90 - 60;
	g_ParamL.dAngleMax = 180;
	strcpy(g_ParamR.szIp, "169.254.145.92");
	g_ParamR.dHeading = -90.0;
	g_ParamR.T = cv::Point2d(1130,1560);
	g_ParamR.dAngleMin = 0;
	g_ParamR.dAngleMax = 90 + 60;

	g_SopasM.Init(g_ParamM);
	g_SopasL.Init(g_ParamL);
	g_SopasR.Init(g_ParamR);

	g_SopasM.SetCallBack(SopasDataCallBackMiddle, 0);

	int nRt = 0;
	nRt =g_SopasM.Start();
	if (nRt <= 0)
	{
		printf("Middle lidar start failed!\n");
		getchar();
	}
	else
	{
		printf("Middle lidar start finish!\n");
	}

	nRt =g_SopasL.Start();
	if (nRt <= 0)
	{
		printf("Left lidar start failed!\n");
	}
	else
	{
		printf("Left lidar start finish!\n");
	}

	nRt =g_SopasR.Start();
	if (nRt <= 0)
	{
		printf("Right lidar start failed!\n");
	}
	else
	{
		printf("Right lidar start finish!\n");
	}

	Sleep(INFINITE);
}

void DrawPoints(cv::Mat& img, vector<cv::Point2f>& Points, CvScalar color)
{
	cv::Vec<uchar,3> color_(color.val[0], color.val[1], color.val[2]);
	for (unsigned int i = 0; i < Points.size(); i++)
	{
		cv::Point2f pt = Points[i];
		double X = pt.x;
		double Y = pt.y;
		if (X>=-1*g_nGridSide && X<=g_nGridSide && Y<=g_nGridFront && Y>=-1.f*g_nGridBack)
		{
			X = g_nGridSide + X;
			Y = g_nGridFront - Y;
			double XX = floor(X/g_nGridSize);
			double YY = floor(Y/g_nGridSize);
			if (YY>=0 && YY < g_img.rows && XX>=0 && XX < g_img.cols)
			{
				g_img.at<cv::Vec<uchar,3>>(YY,XX) = color_;
//				circle(g_img,cv::Point(XX,YY),1,color,-1);
			}
		}
	}
}

void DrawCar(cv::Mat& img, int nFront, int nRear, int nSide)
{
	if (img.data == NULL)
	{
		img = Mat::zeros((g_nGridFront+g_nGridBack)/g_nGridSize+1,g_nGridSide*2/g_nGridSize+1,CV_8UC3);
	}
	img.col((g_nGridSide/g_nGridSize)+1) = CV_RGB(255,255,255);
	img.row((g_nGridFront/g_nGridSize)+1) = CV_RGB(255,255,255);

	Point2f Coners[4];
	Coners[0] = Point2f(-1.f*nSide,nFront);
	Coners[1] = Point2f(nSide,nFront);
	Coners[2] = Point2f(nSide,-1.f*nRear);
	Coners[3] = Point2f(-1.f*nSide,-1.f*nRear);

	Point2f Coners_[4];
	for (int i = 0; i < 4; i++)
	{
		Coners_[i] = CarXY2ImageUV(Coners[i]);
	}
	for (int i = 0; i < 4; i++)
	{
		line(img, Coners_[i], Coners_[(i+1)%4], CV_RGB(255,255,255),1);
	}

	for (int i = -1*g_nGridSide; i < g_nGridSide; i+=10000)
	{
		Point PtImg0 = CarXY2ImageUV(Point2f(i, 0));
		Point PtImg1(PtImg0.x, PtImg0.y+5);
		line(img, PtImg0, PtImg1, CV_RGB(255,255,255), 1);
	}
	for (int i = -1*g_nGridBack; i < g_nGridFront; i+=10000)
	{
		Point PtImg0 = CarXY2ImageUV(Point2f(0, i));
		Point PtImg1(PtImg0.x+5, PtImg0.y);
		line(img, PtImg0, PtImg1, CV_RGB(255,255,255), 1);
	}

}

Point2d CarXY2ImageUV(Point2d CarXY)
{
	Point2d out;
	double X = CarXY.x;
	double Y = CarXY.y;
	X = g_nGridSide + X;
	Y = g_nGridFront - Y;
	out.x = floor(X/g_nGridSize);
	out.y = floor(Y/g_nGridSize);

	return out;
}

void DrawLidar(Mat& img, CSopasParam ParamM, CSopasParam  ParamL, CSopasParam  ParamR)
{
	int nLidarSize = 100;
	int nLidarGridSize = (nLidarSize/g_nGridSize)+1;
	Point pt = CarXY2ImageUV(ParamM.T);
	circle(img, pt, nLidarGridSize, CV_RGB(255,0,0));
	pt = CarXY2ImageUV(ParamL.T);
	circle(img, pt, nLidarGridSize, CV_RGB(0,255,0));
	pt = CarXY2ImageUV(ParamR.T);
	circle(img, pt, nLidarGridSize, CV_RGB(0,0,255));
}

