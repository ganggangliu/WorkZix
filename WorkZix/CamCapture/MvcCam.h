#pragma once
#include "DataCapBase.h"
#include "MVC_Common.h"
#include "MVCAPI.h"
#pragma comment (lib, "MVC_API.lib")
using namespace cv;

class CMvcCamOpr:public CDataCapBase
{
public:
	CMvcCamOpr(void);
	~CMvcCamOpr(void);

public:
	long InitParam(CCameraParam* pParam);
	long Start();
	long Stop();
	long GetLatestImagePair(unsigned long* pnFrameInd, Mat* pMat, long nTryTimes);
	long GetLatestImagePairMono(unsigned long* pnFrameInd, Mat* pMat, long nTryTimes);
	long GetLatestImagePairStereo(unsigned long* pnFrameInd, Mat* pMat, long nTryTimes);
	long SetInteTime(long nTime);
	CCameraParam& GetParam();
	CCameraParam m_CameraParam;

private:
	//Ö¡»Øµ÷
	static UINT WINAPI cbMStreamProcL(WORD wCardNo,MVCFRAMEINFO FrameInfo,PVOID pUserData);
	static UINT WINAPI cbMStreamProcR(WORD wCardNo,MVCFRAMEINFO FrameInfo,PVOID pUserData);
	long GetDisparitIndex();
	long m_nIndex[MAX_DEV_NUM];
	bool m_bIsColr[MAX_DEV_NUM];
	long m_nRows[MAX_DEV_NUM];
	long m_nCols[MAX_DEV_NUM];
	Mat m_MatBuff[MAX_DEV_NUM];
	DWORD m_nStampFromCam[MAX_DEV_NUM];
	DWORD m_nStampFromCPU[MAX_DEV_NUM];
	bool m_bIsSynch;
	unsigned long m_nFrameId[MAX_DEV_NUM];
	CRITICAL_SECTION m_cs[MAX_DEV_NUM];
	unsigned long m_nFetchIndex;
	bool m_bRunning;
	long m_nDisparityIndex;
	double m_dDisparityIndex;

};

