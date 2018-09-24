#ifndef SICK_LMS511_DATA_TRANSFER
#define SICK_LMS511_DATA_TRANSFER

#include "LCM_IBEO_OBJECT_LIST.hpp"
#include "LIDAR_RADAR_INFO.hpp"
#include "OpenCVInc.h"
#include "GpsManager.h"
#include <vector>
using namespace std;
using namespace cv;

class CSickLms511DataTransfer
{
public:
	CSickLms511DataTransfer(double dStartAngle, double dAngRes);
	~CSickLms511DataTransfer();

	void SendData(char* pszChannelName, unsigned short* pDist, int nLen);
	void SendData(char* pszChannelName, vector<cv::Point2f>& Points);
	void SendDataWithEsr(char* pszChannelName, vector<cv::Point2f>& Points);
	Mat Gridding(vector<cv::Point2f>& Points);
	Point2i CarXY2ImageUV(Point2d PtCar);
	Point2d ImageUV2CarXY(Point2i PtImg);
	LCM_IBEO_OBJECT_LIST ConfusionWithEsr(Mat& img, vector<cv::Point2f>& Points, vector<LIDAR_RADAR_OBJECT_INFO>& Esr);
	void RectifyEsrData(LIDAR_RADAR_INFO& Esr);
	void DrawCar(cv::Mat& img, int nFront, int nRear, int nSide);
	int InputRadar(vector<LIDAR_RADAR_OBJECT_INFO>& RadarAll, 
		LIDAR_RADAR_INFO& RadarData,
		Point2f T,
		double dHeading);

	void* m_pLcm;
	void* m_pLcmEsr;
	double m_dStartAngle;
	double m_dAngleRes;

	long m_nGridFront;
	long m_nGridBack;
	long m_nGridSide;
	long m_nGridSize;
	double m_dSegDist;//mm
	double m_dCurveFitEpsilon;
	double m_dMatchDist;
	int m_nLogInd;
	char m_FileBuffer[10000];
	Point2f m_EsrT;
	double m_dEsrHeading;
	Point2f m_RsdsT;
	double m_dRsdsHeading;
	CGpsManager m_GpsRec;
};


#endif //