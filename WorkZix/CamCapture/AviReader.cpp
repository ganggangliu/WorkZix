#include "AviReader.h"
#include <highgui.h>

CAviReader::CAviReader(void)
{
	::InitializeCriticalSection(&m_cs);
}
CAviReader::~CAviReader(void)
{
	::DeleteCriticalSection(&m_cs);
}

long CAviReader::InitParam(CCameraParam* pParam)
{
	m_CameraParam = *pParam;
	return 1;
}
long CAviReader::Start()
{
	m_bIsRuning = true;
	m_hAviRead = CreateThread(NULL,0,AviRead,this,NULL,NULL);
	m_bIsSynch = false;

	return 1;
}
DWORD CAviReader::AviRead(LPVOID wparam)
{
	CAviReader* pAvi = (CAviReader*)wparam;
	CvCapture *captureL=cvCreateFileCapture(pAvi->m_CameraParam.szMacLeft);//读取avi格式的影片
	CvCapture *captureR = 0;
	if (pAvi->m_CameraParam.nCamCont == 2)
	{
		captureR=cvCreateFileCapture(pAvi->m_CameraParam.szMacRight);//读取avi格式的影片
	}
	IplImage* frameL;
	IplImage* frameR;
	pAvi->m_nIndex = 0;
	pAvi->m_bIsSynch = false;
	long nnn = 0;
	char ttt[MAX_PATH] = {};
	while(pAvi->m_bIsRuning)
	{
//		pAvi->m_bIsSynch = false;
		::EnterCriticalSection(&pAvi->m_cs);
		frameL = cvQueryFrame(captureL);
		Mat matL(frameL);
		frameR = cvQueryFrame(captureR);
		Mat matR(frameR);
		sprintf(ttt,"%d\n",nnn);
		printf(ttt);
		nnn++;
		if((pAvi->m_CameraParam.nCamCont == 2) && (frameL == 0 || frameR == 0))
		{
			::LeaveCriticalSection(&pAvi->m_cs);
			break;
		}
		if((pAvi->m_CameraParam.nCamCont == 1) && (frameL == 0))
		{
			::LeaveCriticalSection(&pAvi->m_cs);
			break;
		}
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

//		Sleep(50);
	}
	cvReleaseCapture(&captureL);
	if (pAvi->m_CameraParam.nCamCont == 2)
	{
		cvReleaseCapture(&captureR);
	}

	return 1;
}

long CAviReader::Stop()
{
	m_bIsRuning = false;
	CloseHandle(m_hAviRead);

	return 1;
}
long CAviReader::GetLatestImagePair(unsigned long* pnFrameInd, Mat* pMat, long nTryTimes)
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
long CAviReader::SetInteTime(long nTime)
{
	return 1;
}
CCameraParam& CAviReader::GetParam()
{
	return m_CameraParam;
}