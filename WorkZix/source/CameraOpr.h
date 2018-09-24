#ifndef CAMERA_OPR_H
#define CAMERA_OPR_H

#include <string>
#include <boost/function.hpp>
#include "OpenCVInc.h"

typedef boost::function<void(cv::Mat*,void*)> FunctionCameraData;

enum CAMERA_TYPE
{
	POINT_GREY_BFLY_PGE_23S6C_C = 0
};

enum CAMERA_STREAM_TYPE
{
	ORIGINAL_DATA_STREAM,
	COMPRESSED_DATA_STREAM
};

struct CCameraParam
{
	int nCameraType;
	string szMac;
	string szIp;
	unsigned int nOffSetX;
	unsigned int nOffSetY;
	unsigned int nHeight;
	unsigned int nWidth;
	CAMERA_STREAM_TYPE nDataStreamType;
	bool bIsTrigger;
	bool bIsAutoFrameRate;
	unsigned int nFrameRate;
	bool bIsAutoShutter;
	unsigned int nShutterTime;//MicrolliSeconds
	bool bIsAutoGain;
	CCameraParam()
	{
		nCameraType = 0;
		szMac = "MAC";
		szIp = "IP";
		nOffSetX = 0;
		nOffSetY = 0;
		nHeight = 1200;
		nWidth = 1600;
		nDataStreamType = COMPRESSED_DATA_STREAM;
		bIsTrigger = false;
		bIsAutoFrameRate = true;
		nFrameRate = 20;
		bIsAutoShutter = true;
		nShutterTime = 10000;
		bIsAutoGain = true;
	}
};

class CCameraOpr
{
public:
	virtual int Init(CCameraParam& Param) = 0;
	virtual void SetCallBack(FunctionCameraData hCallBack, void* pUser) = 0;
	virtual int Start() = 0;
	virtual double GetCallBackRate() = 0;
	virtual int GetData(Mat& img) = 0;
	virtual double GetFrameRate() = 0;
	virtual double GetShutter() = 0;
	virtual double GetGain() = 0;
	virtual double GetExposure() = 0;
	virtual int Stop() = 0;
};

typedef CCameraOpr* hCameraHandle;

hCameraHandle CreateCameraHandle(CCameraParam& Param);
void ReleaseCameraHandle(hCameraHandle& hHandle);

#endif