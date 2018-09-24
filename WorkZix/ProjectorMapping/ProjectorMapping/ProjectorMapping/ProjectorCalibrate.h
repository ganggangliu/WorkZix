#ifndef _PROJECTOR_CALIBRATE_
#define _PROJECTOR_CALIBRATE_

#include "OpenCVInc.h"

typedef bool(__stdcall *pProjectorDisplayCallBack)(cv::Mat&, void* pUser);

class CProjectorParam
{
public:
	cv::Rect ProjectorRect;
	cv::Point CameraRes;	//x:width y:height
	int nRecifyType;		//0:Homography  1:Remap
	int nCameraInd;
	CProjectorParam()
	{
		ProjectorRect = cv::Rect(0, 0, 800, 600);
		CameraRes = cv::Point(1920, 1080);
		nRecifyType = 0;
		nCameraInd = 0;
	}
};

class CCalibResultCurve
{
public:
	cv::Mat Remap0;
	cv::Mat Remap1;

};

class CCalibResutPlanar
{
public:
	std::vector<cv::Point2f> PorjectorPoints;		//input
	std::vector<cv::Point2f> UnRectCameraPoints;	//input
	std::vector<cv::Point2f> RectCameraPoints;		//input
	cv::Mat HomoProj2Rec;	//Homography projector to rectify //output
	cv::Mat HomoRec2Cam;	//Homography rectify to camera
	cv::Mat HomoProj2Cam;	//Homography projector to camera
	int Calc();
	cv::Mat Rectify(cv::Mat& Image, CProjectorParam& Param);
};

class CProjectCalibrate
{
public:
	CProjectCalibrate();
	~CProjectCalibrate();
	void Init(CProjectorParam& Param);
	void SetProjectorDispCallBack(pProjectorDisplayCallBack pCallBack, void* pUser);
	bool RunCalib();
	bool RunCalibEx();
	cv::Mat Rectify(cv::Mat& Image);
	CProjectorParam m_Param;

friend DWORD WINAPI CameraThreadProc(LPVOID lpParameter);

private:
	cv::Rect GetMaxIntrinsicRect(std::vector<cv::Point2f> coutour, double dRatio);
	int StartCamera();
	int CameraCap(cv::Mat& image);
	int StopCamera();
	int DetectFlikerPoint(cv::Mat matObj, cv::Point2f& pt_out);
	cv::Mat Cvt8UC3_2_16SC1(cv::Mat img);
	cv::Mat GetGreyImage();
	bool WaitForCameraImageStable();
	pProjectorDisplayCallBack m_pProjectorDisp;
	void* m_pUser;
	
	cv::Mat m_matProjct;
	cv::VideoCapture m_cap;
	std::vector<cv::Point2f> m_CalibContour;
	bool m_bIsCameraRun;
	cv::Mat m_matCamBuf;
	HANDLE m_hCamera;
	CRITICAL_SECTION m_cs;
	uint64_t m_nCameraCount;
	CCalibResultCurve m_ResultCurve;
	CCalibResutPlanar m_ResultPlanar;
};

#endif