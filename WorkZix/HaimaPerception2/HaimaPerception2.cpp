#include <windows.h>
#include "OpenCVInc.h"
#include "GpsManager.h"
#include "LcmReceiver.h"
#include "LIDAR_RADAR_INFO.hpp"
#include "LCM_IBEO_OBJECT_LIST.hpp"
#include "LCM_IBEO_CLOUD_POINTS.hpp"
using namespace std;


string g_szChNameRadar = "LIDAR_RADAR_INFO";
CLcmRevicer<LIDAR_RADAR_INFO> g_LcmRadar(g_szChNameRadar);
string g_szChNameIbeoObjectsList = "LCM_IBEO_OBJECT_LIST";
CLcmRevicer<LCM_IBEO_OBJECT_LIST> g_LcmIbeoObjects(g_szChNameIbeoObjectsList);
string g_szChNameIbeoCloudPoints = "LCM_IBEO_CLOUD_POINTS";
CLcmRevicer<LCM_IBEO_CLOUD_POINTS> g_LcmIbeoCloudPoints(g_szChNameIbeoCloudPoints);

long g_nGridFront = 50000;
long g_nGridBack = 20000;
long g_nGridSide = 20000;
long g_nGridSize = 100;

void DrawIbeoDataPoints(Mat& img, LCM_IBEO_CLOUD_POINTS* pIbeoPoints, double dHead, Point2d T, Scalar color);
void DrawIbeoDataObjects(Mat& img, LCM_IBEO_OBJECT_LIST* pIbeoObjects, double dHead, Point2d T, Scalar color = Scalar::all(-1));
void DrawDelphiData(Mat& img, LIDAR_RADAR_INFO* pRadarData, double dHead, Point2d T, Scalar color);
void DrawCar(Mat& img, double dFront, double dRear, double dSide);
Point2d CarXY2ImageUV(Point2d CarXY);
CGpsManager g_IMU;//(CGpsManager::GPS_MANAGER_TYPE_IMU);

//Front IBEO lidar
Scalar g_ColorList[10000];
double g_IbeoHeading = 0.f;
Point2d g_IbeoT = cvPoint2D32f(0,0);
Scalar g_IbeoColor = cvScalar(255,255,255);
LCM_IBEO_OBJECT_LIST g_IbeoObjects;
LCM_IBEO_CLOUD_POINTS g_IbeoPoints;

//Front ESR radar
double g_EsrHeading = 0.f;				
Point2d g_EstT = cvPoint2D32f(0,3.2);
Scalar g_EsrColor = cvScalar(0,255,0);
LIDAR_RADAR_INFO g_Esr;

//Front left radar
double g_FLRsdsHeading = 180.f;
Point2d g_FLRsdsT = cvPoint2D32f(0,-2.3);
Scalar g_FLColor = cvScalar(0,0,255);
LIDAR_RADAR_INFO g_FLRsds;

//Front right radar
double g_FRRsdsHeading = 180.f;
Point2d g_FRRsdsT = cvPoint2D32f(0,-2.3);
Scalar g_FRColor = cvScalar(255,255,0);
LIDAR_RADAR_INFO g_FRRsds;

//Rear left radar
double g_BLRsdsHeading = 0.f;
Point2d g_BLRsdsT = cvPoint2D32f(0,0);
Scalar g_BLColor = cvScalar(255,0,255);
LIDAR_RADAR_INFO g_BLRsds;

//Rear right radar
double g_BRRsdsHeading = 0.f;
Point2d g_BRRsdsT = cvPoint2D32f(0,0);
Scalar g_BRColor = cvScalar(0,255,255);
LIDAR_RADAR_INFO g_BRRsds;



void WINAPI RadarCallBack(void* pData, void* pUser)
{
	LIDAR_RADAR_INFO* pRadarData = (LIDAR_RADAR_INFO*) pData;

	if (pRadarData->nLidarRadarType == 9)	//Front ESR
	{
		g_Esr = *pRadarData;
	}
	else if (pRadarData->nLidarRadarType == 10)	//Rear left
	{
		g_BLRsds = *pRadarData;
	}
	else if(pRadarData->nLidarRadarType == 11)	//Rear right
	{
		g_BRRsds = *pRadarData;
	}
	else if(pRadarData->nLidarRadarType == 12)	//Front left
	{
		g_FLRsds = *pRadarData;
	}
	else if(pRadarData->nLidarRadarType == 13)	//Front right
	{
		g_FRRsds = *pRadarData;
	}
	else
	{
		printf("Unknow Radar type: %d\n", pRadarData->nLidarRadarType);
		return;
	}

	return;
}

void WINAPI IbeoObjectsListCallBack(void* pData, void* pUser)
{
	LCM_IBEO_OBJECT_LIST* pIbeoObjList = (LCM_IBEO_OBJECT_LIST*) pData;

	g_IbeoObjects = *pIbeoObjList;


	return;
}

void WINAPI IbeoCloudPointsCallBack(void* pData, void* pUser)
{
	LCM_IBEO_CLOUD_POINTS* pIbeoCloudPoints = (LCM_IBEO_CLOUD_POINTS*) pData;

	g_IbeoPoints = *pIbeoCloudPoints;
	
	return;
}

int main(int argc, char* argv[])
{
	RNG rng;
	for (int i = 0; i < 1000; i++)
	{
		g_ColorList[i].val[0] = rng.uniform(0,256);
		g_ColorList[i].val[1] = rng.uniform(0,256);
		g_ColorList[i].val[2] = rng.uniform(0,256);
	}

	g_IMU.Init(CGpsManager::GPS_MANAGER_TYPE_IMU);
	g_IMU.Start();

	g_LcmRadar.SetCallBack(RadarCallBack,NULL);
	g_LcmRadar.Start();

	g_LcmIbeoObjects.SetCallBack(IbeoObjectsListCallBack,NULL);
	g_LcmIbeoObjects.Start();

	g_LcmIbeoCloudPoints.SetCallBack(IbeoCloudPointsCallBack,NULL);
	g_LcmIbeoCloudPoints.Start();

	while(1)
	{
		Sleep(80);
		Mat image;
		//		DrawIbeoDataPoints(image,&g_IbeoPoints,g_IbeoHeading,g_IbeoT,g_IbeoColor);
		DrawIbeoDataObjects(image,&g_IbeoObjects,g_IbeoHeading,g_IbeoT,g_IbeoColor);
		DrawDelphiData(image,&g_Esr,g_EsrHeading,g_EstT,g_EsrColor);
		DrawDelphiData(image,&g_BLRsds,g_BLRsdsHeading,g_BLRsdsT,g_BLColor);
		DrawDelphiData(image,&g_BRRsds,g_BRRsdsHeading,g_BRRsdsT,g_BRColor);
		DrawDelphiData(image,&g_FLRsds,g_FLRsdsHeading,g_FLRsdsT,g_FLColor);
		DrawDelphiData(image,&g_FRRsds,g_FRRsdsHeading,g_FRRsdsT,g_FRColor);

		DrawCar(image,2.4,0.84,0.9);

		Mat image_;
		flip(image.t(),image_,1);
		cvNamedWindow("image");
		imshow("image",image);
		waitKey(1);
	}

	return 0;
}

void DrawIbeoDataPoints(Mat& img, LCM_IBEO_CLOUD_POINTS* pIbeoPoints, double dHead, Point2d T, Scalar color)
{
	if (img.data == NULL)
	{
		img = Mat::zeros((g_nGridFront+g_nGridBack)/g_nGridSize+1,g_nGridSide*2/g_nGridSize+1,CV_8UC3);
		img.col((g_nGridSide/g_nGridSize)+1) = cvScalar(255,255,255);
		img.row((g_nGridFront/g_nGridSize)+1) = cvScalar(255,255,255);
	}

	Vec<uchar,3> color_((uchar)color.val[0],(uchar)color.val[0],(uchar)color.val[0]);
	for (int i = 0; i < pIbeoPoints->IbeoPoints.size(); i++)
	{
		if (pIbeoPoints->IbeoPoints[i].layer <= -1)
		{
			continue;
		}
		double X = sin((-1.f*pIbeoPoints->IbeoPoints[i].angle/32.f+dHead)/180.f*CV_PI)*pIbeoPoints->IbeoPoints[i].distance + T.x * 1000.0;
		double Z = cos((-1.f*pIbeoPoints->IbeoPoints[i].angle/32.f+dHead)/180.f*CV_PI)*pIbeoPoints->IbeoPoints[i].distance + T.y * 1000.0;
		if (X>=-1*g_nGridSide && X<=g_nGridSide && Z<=g_nGridFront && Z>=-1.f*g_nGridBack)
		{
			X = g_nGridSide + X;
			Z = g_nGridFront - Z;
			double XX = floor(X/g_nGridSize);
			double ZZ = floor(Z/g_nGridSize);
			if (ZZ>=0 && ZZ < img.rows && XX>=0 && XX < img.cols)
			{
				img.at<Vec<uchar,3>>(ZZ,XX) = color_;
			}
		}
	}
}

void DrawIbeoDataObjects(Mat& img, LCM_IBEO_OBJECT_LIST* pIbeoObjects, double dHead, Point2d T, Scalar color)
{
	if (img.data == NULL)
	{
		img = Mat::zeros((g_nGridFront+g_nGridBack)/g_nGridSize+1,g_nGridSide*2/g_nGridSize+1,CV_8UC3);
		img.col((g_nGridSide/g_nGridSize)+1) = cvScalar(255,255,255);
		img.row((g_nGridFront/g_nGridSize)+1) = cvScalar(255,255,255);
	}

	for (int i = 0; i < pIbeoObjects->IbeoObjects.size(); i++)
	{
		LCM_IBEO_OBJECT& ObjRef = pIbeoObjects->IbeoObjects[i];
		//Create color
		bool isRandMatchColor = color == Scalar::all(-1);
		Scalar color_ = isRandMatchColor ? g_ColorList[ObjRef.Id] : color;
		//Draw counter points
		vector< LCM_POINT2D_F >& ContPointsRef = ObjRef.ContourPts;
		vector<double> VecXX,VecYY;
		for (int j = 0; j < ContPointsRef.size(); j++)
		{
			double X_ = ContPointsRef[j].x;
			double Y_ = ContPointsRef[j].y;
			double X = X_*cos(dHead/180.f*CV_PI) - Y_*sin(dHead/180.f*CV_PI) + T.x*1000.0;
			double Y = X_*sin(dHead/180.f*CV_PI) + Y_*cos(dHead/180.f*CV_PI) + T.y*1000.0;
			//           if (X>=-1*nGridSide && X<=nGridSide && Y<=nGridFront && Y>=-1.f*nGridBack)
			X = g_nGridSide + X;
			Y = g_nGridFront - Y;
			double XX = floor(X/g_nGridSize);
			double YY = floor(Y/g_nGridSize);
			circle(img,cvPoint(XX,YY),1,color_,-1);
			VecXX.push_back(XX);
			VecYY.push_back(YY);
		}

		//Draw counter lines
		for (int j = 1; j < VecXX.size(); j++)
		{
			line(img, cvPoint(VecXX[j-1],VecYY[j-1]), cvPoint(VecXX[j],VecYY[j]), color_, 1);
		}

		//Draw objects rect
		double X_ = ObjRef.ObjBoxCenter.x;
		double Y_ = ObjRef.ObjBoxCenter.y;
		double X = X_*cos(dHead/180.f*CV_PI) - Y_*sin(dHead/180.f*CV_PI) + T.x*1000.0;
		double Y = X_*sin(dHead/180.f*CV_PI) + Y_*cos(dHead/180.f*CV_PI) + T.y*1000.0;
		Point2f center(X, Y);
		Point2f center_((center.x + g_nGridSide) / g_nGridSize, (g_nGridFront - center.y) / g_nGridSize);
		Point2f size(ObjRef.ObjBoxSize.x, ObjRef.ObjBoxSize.y);
		Point2f size_(size.x / g_nGridSize, size.y / g_nGridSize);
		RotatedRect rectOrient(center_, size_, -1.f*ObjRef.ObjOrientation-dHead);
		Point2f vertices_[4];
		rectOrient.points(vertices_);
		for (int j = 0; j < 4; j++)
		{
			line(img, vertices_[j], vertices_[(j+1)%4], color_,1);
		}
	}

}

void DrawDelphiData(Mat& img, LIDAR_RADAR_INFO* pRadarData, double dHead, Point2d T, Scalar color)
{
	if (img.data == NULL)
	{
		img = Mat::zeros((g_nGridFront+g_nGridBack)/g_nGridSize+1,g_nGridSide*2/g_nGridSize+1,CV_8UC3);
		img.col((g_nGridSide/g_nGridSize)+1) = cvScalar(255,255,255);
		img.row((g_nGridFront/g_nGridSize)+1) = cvScalar(255,255,255);
	}

	Vec<uchar,3> color_((uchar)color.val[0],(uchar)color.val[0],(uchar)color.val[0]);
	for (int i = 0; i < 64; i++)
	{
		LIDAR_RADAR_OBJECT_INFO& ObjT = pRadarData->Objects[i];
		if (ObjT.bValid == 0)
		{
			continue;
		}
		double X = ((ObjT.fObjLocX*cos(dHead/180.f*CV_PI) - ObjT.fObjLocY*sin(dHead/180.f*CV_PI)) + T.x)*1000.f;
		double Z = ((ObjT.fObjLocX*sin(dHead/180.f*CV_PI) + ObjT.fObjLocY*cos(dHead/180.f*CV_PI)) + T.y)*1000.f;
		if (X>=-1*g_nGridSide && X<=g_nGridSide && Z<=g_nGridFront && Z>=-1.f*g_nGridBack)
		{
			X = g_nGridSide + X;
			Z = g_nGridFront - Z;
			double XX = floor(X/g_nGridSize);
			double ZZ = floor(Z/g_nGridSize);
			circle(img,cvPoint(XX,ZZ),3,color);
			//img.at<Vec<uchar,3>>(ZZ,XX) = color_;
		}
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

void DrawCar(Mat& img, double dFront, double dRear, double dSide)
{
	if (img.data == NULL)
	{
		img = Mat::zeros((g_nGridFront+g_nGridBack)/g_nGridSize+1,g_nGridSide*2/g_nGridSize+1,CV_8UC3);
		img.col((g_nGridSide/g_nGridSize)+1) = cvScalar(255,255,255);
		img.row((g_nGridFront/g_nGridSize)+1) = cvScalar(255,255,255);
	}

	Point2f Coners[4];
	Coners[0] = Point2f(-1.f*dSide*1000,dFront*1000);
	Coners[1] = Point2f(dSide*1000,dFront*1000);
	Coners[2] = Point2f(dSide*1000,-1.f*dRear*1000);
	Coners[3] = Point2f(-1.f*dSide*1000,-1.f*dRear*1000);

	Point2f Coners_[4];
	for (int i = 0; i < 4; i++)
	{
		Coners_[i] = CarXY2ImageUV(Coners[i]);
	}
	for (int i = 0; i < 4; i++)
	{
		line(img, Coners_[i], Coners_[(i+1)%4], CV_RGB(255,255,255),1);
	}
}