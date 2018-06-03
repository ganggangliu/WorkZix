#include "DelphiRsdsOpr.h"
#include "DelphiRsdsImpl.h"

CDelphiRsdsOpr::CDelphiRsdsOpr()
{
	m_pDelphiOpr = new CDelphiRsdsImpl();
}

CDelphiRsdsOpr::~CDelphiRsdsOpr()
{
	if (m_pDelphiOpr)
	{
		delete ((CDelphiRsdsImpl*)m_pDelphiOpr);
		m_pDelphiOpr = 0;
	}
}

int CDelphiRsdsOpr::Init(CDelphiRsdsParam& Param)
{
	CDelphiRsdsImplParam ParamT;
	ParamT.PCanParam = Param.PCanParam;
	ParamT.XCanParam = Param.XCanParam;
	ParamT.szIpLeft = Param.szIpLeft;
	ParamT.szIpRight = Param.szIpRight;
	return ((CDelphiRsdsImpl*)m_pDelphiOpr)->Init(ParamT);
}

void CDelphiRsdsOpr::SetCallBackL(lpDelphiRsdsDataCallBack hCallBack, void* pUser)
{
	((CDelphiRsdsImpl*)m_pDelphiOpr)->SetCallBackL(hCallBack, pUser);
}

void CDelphiRsdsOpr::SetCallBackR(lpDelphiRsdsDataCallBack hCallBack, void* pUser)
{
	((CDelphiRsdsImpl*)m_pDelphiOpr)->SetCallBackR(hCallBack, pUser);
}

int CDelphiRsdsOpr::GetDataL(LIDAR_RADAR_INFO& Data)
{
	return ((CDelphiRsdsImpl*)m_pDelphiOpr)->GetDataL(Data);
}

int CDelphiRsdsOpr::GetDataR(LIDAR_RADAR_INFO& Data)
{
	return ((CDelphiRsdsImpl*)m_pDelphiOpr)->GetDataR(Data);
}

int CDelphiRsdsOpr::Start()
{
	return ((CDelphiRsdsImpl*)m_pDelphiOpr)->Start();
}

int CDelphiRsdsOpr::Stop()
{
	return ((CDelphiRsdsImpl*)m_pDelphiOpr)->Stop();
}