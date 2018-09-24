#ifndef TRAFIC_LIGHT_DETECTOR_H
#define TRAFIC_LIGHT_DETECTOR_H

#include "CameraOpr.h"
#include "LcmReceiver.h"
#include "LCM_TRAFIC_LIGHT_REQUIRE.hpp"
#include "LCM_TRAFIC_LIGHT_RESULT.hpp"
#include "TrafficLightsDetect.h"
#include "GpsManager.h"

struct TraficLightDetectParam
{
	CCameraParam CamParam;
	double dFocusLen;
	cv::Mat IntrinsicParam;
	cv::Mat DistortParam;
	double dPitch;//Unit:degree
	double dRoll;//Unit:degree
	double dYaw;//Unit:degree
	double dLateral;//Unit:m
	double dLongitude;//Unit:m
	double dHeight;//Unit:m
	double dRectSizeU;//Unit:m
	double dRectSizeV;//Unit:m
	double dDetRangeMin;//Unit:m
	double dDetRangeMax;//Unit:m
	double dTrafficLightSizeMin;//Unit:m
	double dTrafficLightSizeMax;//Unit:m
	bool bIsSaveImg;
	TraficLightDetectParam()
	{
		LoadParam();
		IntrinsicParam = (Mat_<float>(3,3) <<	dFocusLen,	0,			800,
												0,			dFocusLen,	600,
												0,			0,			1);
//		dPitch = 7.6;
// 		dPitch = 0.0;
// 		dRoll = 0.0;
// 		dYaw = -0.7;
// 		dLateral = 0.0/*0.2*/;
// 		dLongitude = 1.8;
// 		dHeight = 1.2;
// 		dRectSizeU = 2.0;
// 		dRectSizeV = 4.0;
// 		dDetRangeMin = 10.0;
// 		dDetRangeMax = 100.0;
	}
	int LoadParam();
	int WriteParam();
};

class CTraficLightDetector
{
public:
	CTraficLightDetector();
	~CTraficLightDetector();

	int Init(TraficLightDetectParam& Param);
	int Start();
	//0:Î´¼ì²âµ½ 1£ºÂÌ 2£º»Æ 3£ººì
	int TraficLightDetect(cv::Mat& Img, LCM_TRAFIC_LIGHT_REQUIRE& Req, cv::Mat& Img_out = cv::Mat());
	int TrafficLights_(Mat& imgsrc, Rect& roi,vector<light_pos_t> &plight,int size, double dDist);
	int Stop();

public:
	string m_szImgSaveDir;
	void AddTimeTail(string& sz);
	LCM_TRAFIC_LIGHT_RESULT Process(Mat& img_in, LCM_TRAFIC_LIGHT_REQUIRE& Req_in, Mat& img_out);
	void GetExtrinctParam();
	void CameraDataCallBack(Mat* pImg, void* pUser);
	int GetTraficLightLocate(LCM_TRAFIC_LIGHT_REQUIRE& Req, cv::Rect& Box);
	int Cartesian2Img(cv::Point3f& PtCart, cv::Point& PtImg);
	TraficLightDetectParam m_Param;
	cv::Mat m_ExtrinsicParam;
	hCameraHandle m_pCamera;
	CLcmRevicer<LCM_TRAFIC_LIGHT_REQUIRE> m_ReqRecieve;
	CLcmRevicer<LCM_TRAFIC_LIGHT_RESULT> m_ResultSend;
	CGpsManager m_ImuRec;
	int m_nLogInd;
	unsigned int m_Mode;
	unsigned char m_H_low;
	unsigned char m_H_up;
	unsigned char m_S_low;
	unsigned char m_S_up;
	unsigned char m_V_low;
	unsigned char m_V_up;
};


#endif