#pragma once
#include "DataCapBase.h"
#include "MvcCam.h"
#include "AviReader.h"
#include "PicReader.h"
#include "PointGrey.h"

class CDataCapture
{
public:
	CDataCapture(long type);
	~CDataCapture(void);

	enum SourceType
	{
		CAMERA,
		VEDIO,
		PICTURE,
		POINT_GREY
	};

public:
	long InitParam(CCameraParam* pParam);
	long Start();
	long Stop();
	long GetLatestImagePair(unsigned long* pnFrameInd, Mat* pMat, long nTryTimes);
	long SetInteTime(long nTime);
	CCameraParam& GetParam();

private:
	CDataCapBase* m_pDataOpr;
};