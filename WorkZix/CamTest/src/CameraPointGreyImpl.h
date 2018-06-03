#ifndef CAMERA_POINT_GREY_IMPL_H
#define CAMERA_POINT_GREY_IMPL_H

#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <list>
#include "CameraOpr.h"
#include "FlyCapture2.h"

class CCameraPointGreyImpl:public CCameraOpr
{
public:
	CCameraPointGreyImpl();
	~CCameraPointGreyImpl();

public:
	int Init(CCameraParam& Param);
	void SetCallBack(FunctionCameraData hCallBack, void* pUser);
	int Start();
	int GetData(cv::Mat& img);
	double GetCallBackRate();
	double GetFrameRate();
	double GetShutter();
	double GetGain();
	double GetExposure();
	int Stop();

private:
	int IsMacMatch(FlyCapture2::MACAddress* FlyMac, char* pszMac);
	friend void OnImageGrabbed(FlyCapture2::Image* pImage, const void* pCallbackData);
	CCameraParam m_Param;
	FunctionCameraData m_hCallBack;
	void* m_pUser;
	cv::Mat m_Data;
	int m_nDataState;
	boost::mutex m_mutex;
	FlyCapture2::PGRGuid m_GuId;
	int m_nDevInd;
	FlyCapture2::GigECamera m_cam;
	bool m_bRunning;
	unsigned int m_nCont;
	std::list<boost::posix_time::ptime>	m_FrameTimer;
};


#endif