#pragma once
#include "CameraParam.h"
// #include <cv.h>
// using namespace cv;
#include "OpenCVInc.h"

#define MAX_DEV_NUM 2
#define MAX_FILE_LEN 256
#define LEFT 0
#define RIGHT 1

class CDataCapBase
{
public:
	virtual long InitParam(CCameraParam* pParam) = 0;
	virtual long Start() = 0;
	virtual long Stop() = 0;
	virtual long GetLatestImagePair(unsigned long* pnFrameInd, Mat* pMat, long nTryTimes) = 0;
	virtual long SetInteTime(long nTime) = 0;
	virtual CCameraParam& GetParam() = 0;
};
