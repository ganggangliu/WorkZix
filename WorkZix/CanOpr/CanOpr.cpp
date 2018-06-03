#include "CanOpr.h"
#include "KvaserOpr.h"
#include "VCIOpr.h"
#include "CanParseImpl.h"

CCanOpr::CCanOpr()
{
	m_pCanOpr = NULL;
	return;
}


CCanOpr::~CCanOpr()
{
	if (m_pCanOpr)
	{
		delete m_pCanOpr;
		m_pCanOpr = NULL;
	}
	return;
}

int CCanOpr::Init(CCanParam& Param)
{
	if (m_pCanOpr)
	{
		delete ((CCanBase*)m_pCanOpr);
		m_pCanOpr = NULL;
	}
	m_Param = Param;
	if (m_Param.nDevType == CAN_KVASER)
	{
		m_pCanOpr = new CKaserOpr;
	}
	else if (m_Param.nDevType == CAN_ZHOULIGONG)
	{
		m_pCanOpr = new CVCIOpr;
	}
	else
	{
		printf("Unsupported Device type\n");
		return 0;
	}

	return ((CCanBase*)m_pCanOpr)->Init(Param);
}

void CCanOpr::SetCallBack(lpCanReadDataCallBack hCallBack, void* pUser)
{
	((CCanBase*)m_pCanOpr)->SetCallBack(hCallBack, pUser);
}

int CCanOpr::Start()
{
	return ((CCanBase*)m_pCanOpr)->Start();
}

int CCanOpr::Stop()
{
	return ((CCanBase*)m_pCanOpr)->Stop();
}

int CCanOpr::SendMsg(CCanMsgData& Msg)
{
	return ((CCanBase*)m_pCanOpr)->SendMsg(Msg);
}

int CCanOpr::GetMsg(CCanMsgData& Msg)
{
	return ((CCanBase*)m_pCanOpr)->GetMsg(Msg);
}

//////////////////////////////////////////////////////////////////////////

CCanParseOpr::CCanParseOpr()
{
	m_pCanParseOpr = new CCanParseImpl;
}

CCanParseOpr::~CCanParseOpr()
{
	if (m_pCanParseOpr)
	{
		delete (CCanParseImpl*)m_pCanParseOpr;
		m_pCanParseOpr = NULL;
	}
}

void CCanParseOpr::SetParsePattern(vector<CCanParseItem>& ParsePattern)
{
	((CCanParseImpl*)m_pCanParseOpr)->SetParsePattern(ParsePattern);
}

int CCanParseOpr::Init(CCanParam& Param)
{
	return ((CCanParseImpl*)m_pCanParseOpr)->Init(Param);
}

void CCanParseOpr::SetCallBack(lpCanReadDataCallBack hCallBack, void* pUser)
{
	((CCanParseImpl*)m_pCanParseOpr)->SetCallBack(hCallBack, pUser);
}

int CCanParseOpr::Start()
{
	return ((CCanParseImpl*)m_pCanParseOpr)->Start();
}

int CCanParseOpr::Stop()
{
	return ((CCanParseImpl*)m_pCanParseOpr)->Stop();
}

int CCanParseOpr::SendMsg(CCanMsgData& Msg)
{
	return ((CCanParseImpl*)m_pCanParseOpr)->SendMsg(Msg);
}

int CCanParseOpr::GetMsg(vector<CCanParseItem>& ParseList)
{
	return ((CCanParseImpl*)m_pCanParseOpr)->GetMsg(ParseList);
}
