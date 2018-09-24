#pragma once
#include "DataCapBase.h"
#include "FlyCapture2.h"
#pragma comment (lib, "FlyCapture2_v100.lib")
using namespace cv;
using namespace FlyCapture2;
using namespace std;

class CPointGreyOpr:public CDataCapBase
{
public:
	CPointGreyOpr(void);
	~CPointGreyOpr(void);

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
	friend void OnImageGrabbedL(Image* pImage, const void* pCallbackData);
	friend void OnImageGrabbedR(Image* pImage, const void* pCallbackData);
	long GetDisparitIndex();
	long IsMacMatch(MACAddress* FlyMac, char* pszMac);
	void PrintError( Error error );
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
	GigECamera m_camL;
	GigECamera m_camR;
	PGRGuid m_guidL;
	PGRGuid m_guidR;
	Image m_ImgBuf[MAX_DEV_NUM];
};

