#pragma once
#include "MvcCam.h"
#include <highgui.h>

CMvcCamOpr::CMvcCamOpr(void)
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
}

CMvcCamOpr::~CMvcCamOpr(void)
{
	::DeleteCriticalSection(&m_cs[LEFT]);
	::DeleteCriticalSection(&m_cs[RIGHT]);
}

long CMvcCamOpr::InitParam(CCameraParam* pParam)
{
	m_CameraParam = *pParam;
	long nCont = MVC_GetDeviceNumber();
	printf("%d Cameras found!\n",nCont);
	if (nCont<pParam->nCamCont)
	{
		printf("%d cameras Requred, But %d fined!\n",pParam->nCamCont,nCont);
		waitKey(1000);
	}
	
	MVCGE_DEVLIST DevList;
	for (int i = 0; i < nCont; i++)
	{
		MVC_GetDeviceInfo(i,&DevList);
		if (strcmp(DevList.DevMAC,pParam->szMacLeft) == 0)
		{
			m_nIndex[LEFT] = i;
			m_bIsColr[LEFT] = RS_A1300_GM60 == MVC_GetCameraType(m_nIndex[LEFT])?false:true;
		}
		if (strcmp(DevList.DevMAC,pParam->szMacRight) == 0)
		{
			m_nIndex[RIGHT] = i;
			m_bIsColr[RIGHT] = RS_A1300_GM60 == MVC_GetCameraType(m_nIndex[RIGHT])?false:true;
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

long CMvcCamOpr::Start()
{
	DWORD nResult= 0;
	//关闭巨帧
	MVC_SetNetPacketSize(m_nIndex[LEFT],1440);
	if (m_CameraParam.nCamCont == 2)
	{
		MVC_SetNetPacketSize(m_nIndex[RIGHT],1440);
	}
	//自动IP配置
	MVC_AutoIPConfig(m_nIndex[LEFT]);
	if (m_CameraParam.nCamCont == 2)
	{
		MVC_AutoIPConfig(m_nIndex[RIGHT]);
	}
	//打开设备
	MVC_OpenDevice(m_nIndex[LEFT]);
	printf("Open left camera!\n");
	if (m_CameraParam.nCamCont == 2)
	{
		MVC_OpenDevice(m_nIndex[RIGHT]); 
		printf("Open right camera!\n");
	}
	//关闭内部显示
	MVC_SetParameter(m_nIndex[LEFT], MVCDISP_ISDISP,0);
	if (m_CameraParam.nCamCont == 2)
	{
		MVC_SetParameter(m_nIndex[RIGHT], MVCDISP_ISDISP,0);
	}
	//设置曝光
	MVC_SetParameter(m_nIndex[LEFT],MVCADJ_INTTIME,m_CameraParam.nInteTimeL);
	if (m_CameraParam.nCamCont == 2)
	{
		MVC_SetParameter(m_nIndex[RIGHT],MVCADJ_INTTIME,m_CameraParam.nInteTimeR);
	}
	//设置模拟增益为0
	MVC_SetParameter(m_nIndex[LEFT],MVCADJ_OUTGAIN,0);
	if (m_CameraParam.nCamCont == 2)
	{
		MVC_SetParameter(m_nIndex[RIGHT],MVCADJ_OUTGAIN,0);
	}
	//设置白平衡
	MVC_AutoWhiteBalance(m_nIndex[LEFT]);
	if (m_CameraParam.nCamCont == 2)
	{
		MVC_AutoWhiteBalance(m_nIndex[RIGHT]);
	}
	//设置采集模式为触发采集模式（面阵相机）0：触发模式； 1：连续模式； 2：伪触发
	MVC_SetParameter(m_nIndex[LEFT],MVCADJ_SHUTTERTYPE,0);
	if (m_CameraParam.nCamCont == 2)
	{
		MVC_SetParameter(m_nIndex[RIGHT],MVCADJ_SHUTTERTYPE,0);
	}

	m_nRows[LEFT] = MVC_GetParameter(m_nIndex[LEFT],MVCADJ_HEIGHT);
	m_nCols[LEFT] = MVC_GetParameter(m_nIndex[LEFT],MVCADJ_WIDTH);
	if (m_CameraParam.nCamCont == 2)
	{
		m_nRows[RIGHT] = MVC_GetParameter(m_nIndex[RIGHT],MVCADJ_HEIGHT);
		m_nCols[RIGHT] = MVC_GetParameter(m_nIndex[RIGHT],MVCADJ_WIDTH);
	}

	if (m_bIsColr[LEFT])
		m_MatBuff[LEFT].create(m_nRows[LEFT],m_nCols[LEFT],CV_8UC3);
	else
		m_MatBuff[LEFT].create(m_nRows[LEFT],m_nCols[LEFT],CV_8UC1);
	if (m_CameraParam.nCamCont == 2)
	{
		if (m_bIsColr[RIGHT])
			m_MatBuff[RIGHT].create(m_nRows[RIGHT],m_nCols[RIGHT],CV_8UC3);
		else
			m_MatBuff[RIGHT].create(m_nRows[RIGHT],m_nCols[RIGHT],CV_8UC1);
	}

	MVC_SetStreamHOOK(m_nIndex[LEFT],cbMStreamProcL,this);
	if (m_CameraParam.nCamCont == 2)
	{
		MVC_SetStreamHOOK(m_nIndex[RIGHT],cbMStreamProcR,this);
	}

	m_nFetchIndex = 0;
	m_nFrameId[LEFT] = 0;
	if (m_CameraParam.nCamCont == 2)
	{
		m_nFrameId[RIGHT] = 0;
	}
	//开始采集
	MVC_EnableCapture(m_nIndex[LEFT]);
	if (m_CameraParam.nCamCont == 2)
	{
		MVC_EnableCapture(m_nIndex[RIGHT]);
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

UINT CMvcCamOpr::cbMStreamProcL(WORD wCardNo,MVCFRAMEINFO FrameInfo,PVOID pUserData)
{
	CMvcCamOpr* pCam = (CMvcCamOpr*)pUserData;
	BYTE* pTempBuf = FrameInfo.lBufPtr;

	if (FrameInfo.Height == 0 || FrameInfo.Width == 0 || FrameInfo.lBufPtr == NULL)
		return 1;

	//	pCam->m_nStampFromCPU[LEFT] = GetTickCount();

 	::EnterCriticalSection(&pCam->m_cs[LEFT]);
	if (pCam->m_bIsColr[LEFT] == true)
	{
		MVCFRAMEINFO FrameInfoColor;
		memset(&FrameInfoColor,0,sizeof(MVCFRAMEINFO));
		MVC_PixelConverter(wCardNo,&FrameInfo,&FrameInfoColor,0);
		memcpy(pCam->m_MatBuff[LEFT].data,FrameInfoColor.lBufPtr,FrameInfoColor.lBufSize);
	}
	else
	{
		memcpy(pCam->m_MatBuff[LEFT].data,FrameInfo.lBufPtr,FrameInfo.lBufSize);		
	}
	pCam->m_nFrameId[LEFT] = FrameInfo.FRAMEID;
	pCam->m_nStampFromCam[LEFT] = pTempBuf[3];
//	printf("L%d,%d\n",FrameInfo.FRAMEID,pCam->m_nStampFromCam[LEFT]);
	::LeaveCriticalSection(&pCam->m_cs[LEFT]);

	return 0;
}

UINT CMvcCamOpr::cbMStreamProcR(WORD wCardNo,MVCFRAMEINFO FrameInfo,PVOID pUserData)
{
	CMvcCamOpr* pCam = (CMvcCamOpr*)pUserData;
	BYTE* pTempBuf = FrameInfo.lBufPtr;

	if (FrameInfo.Height == 0 || FrameInfo.Width == 0 || FrameInfo.lBufPtr == NULL)
		return 1;

//	pCam->m_nStampFromCPU[RIGHT] = GetTickCount();

 	::EnterCriticalSection(&pCam->m_cs[RIGHT]);
	if (pCam->m_bIsColr[RIGHT] == true)
	{
		MVCFRAMEINFO FrameInfoColor;
		memset(&FrameInfoColor,0,sizeof(MVCFRAMEINFO));
		MVC_PixelConverter(wCardNo,&FrameInfo,&FrameInfoColor,0);
		memcpy(pCam->m_MatBuff[RIGHT].data,FrameInfoColor.lBufPtr,FrameInfoColor.lBufSize);
	}
	else
	{
		memcpy(pCam->m_MatBuff[RIGHT].data,FrameInfo.lBufPtr,FrameInfo.lBufSize);		
	}
	pCam->m_nFrameId[RIGHT] = FrameInfo.FRAMEID;
	pCam->m_nStampFromCam[RIGHT] = pTempBuf[3];
//	printf("R%d,%d\n",FrameInfo.FRAMEID,pCam->m_nStampFromCam[RIGHT]);
 	::LeaveCriticalSection(&pCam->m_cs[RIGHT]);

	return 0;
}

long CMvcCamOpr::Stop()
{
	MVC_DisableCapture(m_nIndex[LEFT]);
	if (m_CameraParam.nCamCont == 2)
	{
		MVC_DisableCapture(m_nIndex[RIGHT]);
	}

	return 1;
}

long CMvcCamOpr::GetLatestImagePair(unsigned long* pnFrameInd, Mat* pMat, long nTryTimes)
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

long CMvcCamOpr::GetLatestImagePairMono(unsigned long* pnFrameInd, Mat* pMat, long nTryTimes)
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

long CMvcCamOpr::GetLatestImagePairStereo(unsigned long* pnFrameInd, Mat* pMat, long nTryTimes)
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

long CMvcCamOpr::SetInteTime(long nTime)
{
	MVC_SetParameter(m_nIndex[LEFT],MVCADJ_INTTIME,nTime);
	if (m_CameraParam.nCamCont == 2)
	{
		MVC_SetParameter(m_nIndex[RIGHT],MVCADJ_INTTIME,nTime);
	}

	return 1;
}

long CMvcCamOpr::GetDisparitIndex()
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

CCameraParam& CMvcCamOpr::GetParam()
{
	return m_CameraParam;
}