#include "TcpIpBoostOpr.h"
#include "TcpIpBoostImpl.h"

CTcpIpBoostOpr::CTcpIpBoostOpr()
{
	return;
}

CClientBoostOpr::CClientBoostOpr()
{
	m_pClient = new CClientBoostImpl;

	return;
}

CClientBoostOpr::~CClientBoostOpr()
{
	if (m_pClient)
	{
		delete ((CClientBoostImpl*)m_pClient);
		m_pClient = 0;
	}
}

void CClientBoostOpr::SetCallBack(lpTcpIpDataRecFunc TcpIpDataCallBack, void* pUser)
{
	((CClientBoostImpl*)m_pClient)->SetCallBack(TcpIpDataCallBack, pUser);
}

int CClientBoostOpr::Start(int nPort, char* pAddress,char* pLocalAddress/* = 0*/)
{
	int nRt = ((CClientBoostImpl*)m_pClient)->Connect(nPort, pAddress, pLocalAddress);
	if (nRt <= 0)
		return nRt;
	return ((CClientBoostImpl*)m_pClient)->Start();
}

int CClientBoostOpr::ReceiveMsg(char* msg,int len/*, int nTimeOutMilis*/)
{
	return ((CClientBoostImpl*)m_pClient)->ReceiveMsg(msg, len/*, nTimeOutMilis*/);
}

int CClientBoostOpr::SendMsg(char* msg,int len/*, int nTimeOutMilis*/)
{
	return ((CClientBoostImpl*)m_pClient)->SendMsg(msg, len/*, nTimeOutMilis*/);
}

int CClientBoostOpr::Close()
{
	int nRt = ((CClientBoostImpl*)m_pClient)->Close();

	if (nRt <= 0)
		return nRt;
	return ((CClientBoostImpl*)m_pClient)->DisConnect();
}