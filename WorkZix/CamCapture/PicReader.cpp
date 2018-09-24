#include "PicReader.h"
#include <highgui.h>

CPicReader::CPicReader(void)
{
	::InitializeCriticalSection(&m_cs);
}
CPicReader::~CPicReader(void)
{
	::DeleteCriticalSection(&m_cs);
}

long CPicReader::InitParam(CCameraParam* pParam)
{
	m_CameraParam = *pParam;

	return 1;
}
long CPicReader::Start()
{
	m_bIsRuning = true;
	m_hAviRead = CreateThread(NULL,0,AviRead,this,NULL,NULL);
	m_bIsSynch = false;

	return 1;
}
DWORD CPicReader::AviRead(LPVOID wparam)
{
	CPicReader* pAvi = (CPicReader*)wparam;
// 	CvCapture *captureL=cvCreateFileCapture(pAvi->m_szPathLeft);//读取avi格式的影片
// 	CvCapture *captureR=cvCreateFileCapture(pAvi->m_szPathRight);//读取avi格式的影片
	IplImage* frameL;
	IplImage* frameR;
	pAvi->m_nIndex = 0;
	pAvi->m_bIsSynch = false;
	long nInd = 0;
	while(pAvi->m_bIsRuning)
	{
		nInd++;
//		pAvi->m_bIsSynch = false;
		::EnterCriticalSection(&pAvi->m_cs);
		char szPathL[MAX_PATH] = {0};
		sprintf(szPathL,"%s%d.png",pAvi->m_CameraParam.szMacLeft,nInd);
		frameL = cvLoadImage(szPathL);
		Mat matL(frameL);
		char szPathR[MAX_PATH] = {0};
		sprintf(szPathR,"%s%d.png",pAvi->m_CameraParam.szMacRight,nInd);
		frameR = cvLoadImage(szPathR);
		Mat matR(frameR);
		if(((pAvi->m_CameraParam.nCamCont == 2) && (frameL == 0 || frameR == 0))
			|| ((pAvi->m_CameraParam.nCamCont == 1) && (frameL == 0)))
		{
			::LeaveCriticalSection(&pAvi->m_cs);
			continue;
		}
		printf("%d\n",nInd);
		matL.copyTo(pAvi->m_MatBuff[LEFT]);
		if (pAvi->m_CameraParam.nCamCont == 2)
		{
			matR.copyTo(pAvi->m_MatBuff[RIGHT]);
		}
		pAvi->m_nIndex++;
		pAvi->m_bIsSynch = true;
		::LeaveCriticalSection(&pAvi->m_cs);
		while (pAvi->m_bIsSynch == true)
		{
			Sleep(1);
		}
		cvReleaseImage(&frameL);
		cvReleaseImage(&frameR);
	}

	return 1;
}

long CPicReader::Stop()
{
	m_bIsRuning = false;
	CloseHandle(m_hAviRead);

	return 1;
}
long CPicReader::GetLatestImagePair(unsigned long* pnFrameInd, Mat* pMat, long nTryTimes)
{
	for (int i = 0; i < nTryTimes; i++)
	{
		if (m_bIsSynch == true)
		{
			EnterCriticalSection(&m_cs);
			m_MatBuff[LEFT].copyTo(pMat[LEFT]);
			if (m_CameraParam.nCamCont == 2)
			{
				m_MatBuff[RIGHT].copyTo(pMat[RIGHT]);
			}
			*pnFrameInd = m_nIndex;
			m_bIsSynch = false;
			LeaveCriticalSection(&m_cs);
			return 1;
		}
		else
		{
			Sleep(1);
		}
	}

	return 0;
}
long CPicReader::SetInteTime(long nTime)
{
	return 1;
}
CCameraParam& CPicReader::GetParam()
{
	return m_CameraParam;
}