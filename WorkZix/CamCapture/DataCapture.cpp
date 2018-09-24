#include "DataCapture.h"

CDataCapture::CDataCapture(long type)
{
	if (type == CAMERA)
		m_pDataOpr = new CMvcCamOpr();
	else if (type == VEDIO)
		m_pDataOpr = new CAviReader();
	else if (type == PICTURE)
		m_pDataOpr = new CPicReader();
	else if (type == POINT_GREY)
		m_pDataOpr = new CPointGreyOpr();
}

CDataCapture::~CDataCapture(void)
{
	if (m_pDataOpr)
	{
		delete m_pDataOpr;
		m_pDataOpr = NULL;
	}
}

long CDataCapture::InitParam(CCameraParam* pParam)
{
	return m_pDataOpr->InitParam(pParam);
}

long CDataCapture::Start()
{
	return m_pDataOpr->Start();
}

long CDataCapture::Stop()
{
	return m_pDataOpr->Stop();
}

long CDataCapture::GetLatestImagePair(unsigned long* pnFrameInd, Mat* pMat, long nTryTimes)
{
	return m_pDataOpr->GetLatestImagePair(pnFrameInd,pMat,nTryTimes);
}

long CDataCapture::SetInteTime(long nTime)
{
	return m_pDataOpr->SetInteTime(nTime);
}

CCameraParam& CDataCapture::GetParam()
{
	return m_pDataOpr->GetParam();
}