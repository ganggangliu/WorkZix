#include "SopasOpr.h"
#include "SopasImpl.h"

CSopasOpr::CSopasOpr()
{
	m_pSopas = new CSopasImpl();
}
CSopasOpr::~CSopasOpr()
{
	if (m_pSopas)
	{
		delete (CSopasImpl*)m_pSopas;
		m_pSopas = 0;
	}
}
int CSopasOpr::Init(CSopasParam& Param)
{
	return ((CSopasImpl*)m_pSopas)->Init(Param);
}
void CSopasOpr::SetCallBack(lpSopasDataCallBack hCallBack, void* pUser)
{
	return ((CSopasImpl*)m_pSopas)->SetCallBack(hCallBack, pUser);
}
int CSopasOpr::Start()
{
	return ((CSopasImpl*)m_pSopas)->Start();
}
int CSopasOpr::GetData(vector<cv::Point2f>& Points)
{
	return ((CSopasImpl*)m_pSopas)->GetData(Points);
}
int CSopasOpr::Stop()
{
	return ((CSopasImpl*)m_pSopas)->Stop();
}