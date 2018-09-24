#pragma once
#include "PointGrey.h"
#include <highgui.h>
#include "C/FlyCapture2_C.h"
#include <math.h>

CPointGreyOpr::CPointGreyOpr(void)
{
	m_nIndex[LEFT] = -1;
	m_nIndex[RIGHT] = -1;
	m_bIsColr[LEFT] = false;
	m_bIsColr[RIGHT] = false;
	memset(m_nStampFromCam,0,MAX_DEV_NUM*sizeof(WORD));
	memset(m_nStampFromCPU,0,MAX_DEV_NUM*sizeof(DWORD));
	m_bIsSynch = false;
	::InitializeCriticalSection(&m_cs[LEFT]);
	::InitializeCriticalSection(&m_cs[RIGHT]);
	m_nFrameId[LEFT] = 0;
	m_nFrameId[RIGHT] = 0;
	m_nFetchIndex = 0;
	m_bRunning = false;
	m_nDisparityIndex = 0;
	m_dDisparityIndex = 0.f;
	m_nRows[LEFT] = 1200;
	m_nCols[LEFT] = 1600;
	m_nRows[RIGHT] = 1200;
	m_nCols[RIGHT] = 1600;
}

CPointGreyOpr::~CPointGreyOpr(void)
{
	::DeleteCriticalSection(&m_cs[LEFT]);
	::DeleteCriticalSection(&m_cs[RIGHT]);
}

long CPointGreyOpr::InitParam(CCameraParam* pParam)
{
	m_CameraParam = *pParam;

	BusManager busMgr;
	unsigned int numCameras;
	Error error = busMgr.GetNumOfCameras(&numCameras);

	if (numCameras<pParam->nCamCont)
	{
		printf("%d cameras Requred, But %d fined!\n",pParam->nCamCont,numCameras);
		waitKey(1000);
	}
	
	CameraInfo camInfo;
	for (int i = 0; i < numCameras; i++)
	{
		PGRGuid guid;
		error = busMgr.GetCameraFromIndex(i, &guid);
		Camera cam;
		error = cam.Connect(&guid);
		CameraInfo camInfo;
		error = cam.GetCameraInfo(&camInfo);
		error = cam.Disconnect();

		if (IsMacMatch(&camInfo.macAddress,m_CameraParam.szMacLeft))
		{
			m_nIndex[LEFT] = i;
			m_guidL = guid;
		}
		if (IsMacMatch(&camInfo.macAddress,m_CameraParam.szMacRight))
		{
			m_nIndex[RIGHT] = i;
			m_guidR = guid;
		}
	}
	if (m_nIndex[LEFT] == -1)
	{
		printf("Can't find left camera:%s\n",pParam->szMacLeft);
		return 0;
	}
	if (m_nIndex[RIGHT] == -1)
	{
		printf("Can't find right camera:%s\n",pParam->szMacRight);
		return 0;
	}

	return 1;
}

long CPointGreyOpr::Start()
{
	m_nFetchIndex = 0;
	m_nFrameId[LEFT] = 0;
	if (m_CameraParam.nCamCont == 2)
	{
		m_nFrameId[RIGHT] = 0;
	}
	m_MatBuff[LEFT].create(m_nRows[LEFT],m_nCols[LEFT],CV_8UC3);

	if (m_CameraParam.nCamCont == 2)
	{
		m_MatBuff[RIGHT].create(m_nRows[RIGHT],m_nCols[RIGHT],CV_8UC3);
	}

	Error error;
	
	error = m_camL.Connect(&m_guidL);
	if (m_CameraParam.nCamCont == 2)
	{
		error = m_camR.Connect(&m_guidR);
	}

	GigEImageSettings imageSettings;		//设置图像尺寸
	error = m_camL.GetGigEImageSettings( &imageSettings );
	if ( error != PGRERROR_OK )
	{
		printf( "Error getting GigE image settings configuration" );
	}
	imageSettings.offsetX = 160;
	imageSettings.offsetY = 0;
	imageSettings.height = 1200;
	imageSettings.width = 1600;
//	imageSettings.pixelFormat = PIXEL_FORMAT_BGR;
	error = m_camL.SetGigEImageSettings( &imageSettings );
	if ( error != PGRERROR_OK )
	{
		printf( "Error getting GigE image settings configuration" );
	}
	if (m_CameraParam.nCamCont == 2)
	{
		error = m_camR.SetGigEImageSettings( &imageSettings );
		if ( error != PGRERROR_OK )
		{
			printf( "Error getting GigE image settings configuration" );
		}
	}

	TriggerMode TriMod;						//设置触发模式
	if (m_CameraParam.nTrigerMode == 0)
	{
		TriMod.onOff = false; 
	}
	else
	{
		TriMod.onOff = true; 
	}
	error = m_camL.SetTriggerMode( &TriMod );
	if (m_CameraParam.nCamCont == 2)
	{
		error = m_camR.GetTriggerMode( &TriMod );
	}

	Property prop;
	prop.type = FRAME_RATE;					//设置帧率，固定帧率20fps
	m_camL.GetProperty( &prop );
	prop.autoManualMode = false;
	prop.onOff = true;
	error = m_camL.SetProperty( &prop );
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}
	if (m_CameraParam.nCamCont == 2)
	{
		error = m_camR.SetProperty( &prop );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
	}
	prop.absValue = m_CameraParam.nFrameRate;
	error = m_camL.SetProperty( &prop );
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}
	if (m_CameraParam.nCamCont == 2)
	{
		error = m_camR.SetProperty( &prop );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
	}

	prop.type = SHUTTER;					//设置快门时间
	m_camL.GetProperty( &prop );
	prop.autoManualMode = false;
	error = m_camL.SetProperty( &prop );
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}
	if (m_CameraParam.nCamCont == 2)
	{
		error = m_camR.SetProperty( &prop );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
	}
	prop.absValue = (double)m_CameraParam.nInteTimeL/100.0;
	error = m_camL.SetProperty( &prop );
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}
	if (m_CameraParam.nCamCont == 2)
	{
		error = m_camR.SetProperty( &prop );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
	}

	prop.type = WHITE_BALANCE;				//Set auto white-balance
	m_camL.GetProperty( &prop );
	prop.onOff = true;
	prop.autoManualMode = true;
	error = m_camL.SetProperty( &prop );
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}
	if (m_CameraParam.nCamCont == 2)
	{
		error = m_camR.SetProperty( &prop );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
	}

	error = m_camL.StartCapture(OnImageGrabbedL,this);//设置回调函数
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}
	if (m_CameraParam.nCamCont == 2)
	{
		error = m_camR.StartCapture(OnImageGrabbedR,this);
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
	}

	Sleep(1000);
	printf("Detect Disparity Index...\n");
	GetDisparitIndex();
	printf("Disparity(double) index is %.6f\n",m_dDisparityIndex);
	printf("Disparity(int) index is %d\n",m_nDisparityIndex);
//	Sleep(1000);

	m_bRunning = true;

	return 1;
}

void OnImageGrabbedL(Image* pImage, const void* pCallbackData)
{
	CPointGreyOpr* pCam = (CPointGreyOpr*)pCallbackData;
	if (pCam->m_bRunning == false)
	{
		return;
	}

	if (pImage->GetRows() != pCam->m_CameraParam.nImageRows || pImage->GetCols() != pCam->m_CameraParam.nImageCols)
	{
		printf("Error: Left image is %d * %d\n",pImage->GetRows(),pImage->GetCols());
		return;
	}

	Image DestImage;
	pImage->Convert(PIXEL_FORMAT_BGR,&DestImage);
 	::EnterCriticalSection(&pCam->m_cs[LEFT]);
	pCam->m_nFrameId[LEFT]++;
	memcpy(pCam->m_MatBuff[LEFT].data,DestImage.GetData(),DestImage.GetDataSize());
	::LeaveCriticalSection(&pCam->m_cs[LEFT]);

	return;
}

void OnImageGrabbedR(Image* pImage, const void* pCallbackData)
{
	CPointGreyOpr* pCam = (CPointGreyOpr*)pCallbackData;
	if (pCam->m_bRunning == false)
	{
		return;
	}

	if (pImage->GetRows() != pCam->m_CameraParam.nImageRows || pImage->GetCols() != pCam->m_CameraParam.nImageCols)
	{
		printf("Error: Right image is %d * %d\n",pImage->GetRows(),pImage->GetCols());
		return;
	}

	Image DestImage;
	pImage->Convert(PIXEL_FORMAT_BGR,&DestImage);
	::EnterCriticalSection(&pCam->m_cs[RIGHT]);
	pCam->m_nFrameId[RIGHT]++;
	memcpy(pCam->m_MatBuff[RIGHT].data,DestImage.GetData(),DestImage.GetDataSize());
	::LeaveCriticalSection(&pCam->m_cs[RIGHT]);

	return;
}

long CPointGreyOpr::Stop()
{
	Error error;
	error = m_camL.StopCapture();
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}
	error = m_camL.Disconnect();
	{
		PrintError( error );
		return -1;
	}

	if (m_CameraParam.nCamCont == 2)
	{
		error = m_camR.StopCapture();
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
		error = m_camR.Disconnect();
		{
			PrintError( error );
			return -1;
		}
	}

	return 1;
}

long CPointGreyOpr::GetLatestImagePair(unsigned long* pnFrameInd, Mat* pMat, long nTryTimes)
{
	if (m_CameraParam.nCamCont == 2)
	{
		return(GetLatestImagePairStereo(pnFrameInd,pMat,nTryTimes));
	}
	else
	{
		return(GetLatestImagePairMono(pnFrameInd,pMat,nTryTimes));
	}

	return 1;
}

long CPointGreyOpr::GetLatestImagePairMono(unsigned long* pnFrameInd, Mat* pMat, long nTryTimes)
{
	for (int i = 0; i < nTryTimes; i++)
	{
		EnterCriticalSection(&m_cs[LEFT]);
		if (m_nFrameId[LEFT] > m_nFetchIndex)
		{
			m_MatBuff[LEFT].copyTo(pMat[LEFT]);
			*pnFrameInd = m_nFrameId[LEFT];
			m_nFetchIndex = m_nFrameId[LEFT];
			LeaveCriticalSection(&m_cs[LEFT]);
			return 1;
		}
		else
		{
			LeaveCriticalSection(&m_cs[LEFT]);
			Sleep(10);
		}
	}

	printf("Get Data Failed！LEFT:%d\n",m_nFrameId[LEFT]);
	
	return 0;

}

long CPointGreyOpr::GetLatestImagePairStereo(unsigned long* pnFrameInd, Mat* pMat, long nTryTimes)
{
	for (int i = 0; i < nTryTimes; i++)
	{
		EnterCriticalSection(&m_cs[LEFT]);
		EnterCriticalSection(&m_cs[RIGHT]);
		if (m_nFrameId[LEFT] == (m_nFrameId[RIGHT]+m_nDisparityIndex) && m_nFrameId[LEFT] > m_nFetchIndex)
		{
			m_MatBuff[LEFT].copyTo(pMat[LEFT]);
			m_MatBuff[RIGHT].copyTo(pMat[RIGHT]);
			*pnFrameInd = m_nFrameId[LEFT];
			m_nFetchIndex = m_nFrameId[LEFT];
			LeaveCriticalSection(&m_cs[LEFT]);
			LeaveCriticalSection(&m_cs[RIGHT]);
			return 1;
		}
		else
		{
			LeaveCriticalSection(&m_cs[LEFT]);
			LeaveCriticalSection(&m_cs[RIGHT]);
			Sleep(10);
		}
	}

	printf("Get Data Failed！LEFT:%d RIGHT:%d\n",m_nFrameId[LEFT],m_nFrameId[RIGHT]);
	GetDisparitIndex();
	printf("Recal disparity:%.2f\n",m_dDisparityIndex);

	return 0;

}

long CPointGreyOpr::SetInteTime(long nTime)
{
	Property prop;
	prop.type = SHUTTER;					//设置快门时间
	m_camL.GetProperty( &prop );
	prop.absValue = (double)nTime/100.0;
	Error error = m_camL.SetProperty( &prop );
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return -1;
	}
	if (m_CameraParam.nCamCont == 2)
	{
		error = m_camR.SetProperty( &prop );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return -1;
		}
	}

	return 1;
}

long CPointGreyOpr::GetDisparitIndex()
{
	m_dDisparityIndex = 0.f;
	long nCont = 0;
	for (int i = 0; i < 100; i++)
	{
		EnterCriticalSection(&m_cs[LEFT]);
		EnterCriticalSection(&m_cs[RIGHT]);
		long nTemp = (long)m_nFrameId[LEFT] - (long)m_nFrameId[RIGHT];
		if (nTemp < 0 || nTemp >10)
		{
			m_dDisparityIndex += 0.f;
		}
		else
		{
			m_dDisparityIndex += (double)(nTemp);
			nCont++;
		}
		LeaveCriticalSection(&m_cs[LEFT]);
		LeaveCriticalSection(&m_cs[RIGHT]);
		Sleep(10);
	}
	m_dDisparityIndex = m_dDisparityIndex/(double)nCont;
	m_nDisparityIndex = (long)(m_dDisparityIndex + 0.5f);

	return 1;
}

CCameraParam& CPointGreyOpr::GetParam()
{
	return m_CameraParam;
}

long CPointGreyOpr::IsMacMatch(MACAddress* FlyMac, char* pszMac)
{
	char szFlyMac[128];
	sprintf(szFlyMac,"%02X:%02X:%02X:%02X:%02X:%02X",FlyMac->octets[0],
		FlyMac->octets[1],FlyMac->octets[2],FlyMac->octets[3],FlyMac->octets[4],FlyMac->octets[5]);
	long nRt = strcmp(szFlyMac,pszMac);

	return (nRt == 0);
}

void CPointGreyOpr::PrintError( Error error )
{
	error.PrintErrorTrace();
}