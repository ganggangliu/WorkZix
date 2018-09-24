#pragma once
#include "DataCapBase.h"

using namespace cv;

class CAviReader:public CDataCapBase
{
public:
	CAviReader(void);
	~CAviReader(void);

	long InitParam(CCameraParam* pParam);
	long Start();
	long Stop();
	long GetLatestImagePair(unsigned long* pnFrameInd, Mat* pMat, long nTryTimes);
	long SetInteTime(long nTime);
	CCameraParam& GetParam();
	CCameraParam m_CameraParam;

private:
	static DWORD WINAPI AviRead(LPVOID wparam);
	char m_szPathLeft[MAX_PATH];
	char m_szPathRight[MAX_PATH];
	HANDLE m_hAviRead;
	bool m_bIsRuning;
	CRITICAL_SECTION m_cs;
	Mat m_MatBuff[MAX_DEV_NUM];
	long m_nIndex;
	bool m_bIsSynch;
	
};