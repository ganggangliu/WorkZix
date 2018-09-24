#ifndef TCPIP_BOOST_OPR_H
#define TCPIP_BOOST_OPR_H

typedef void(__stdcall *lpTcpIpDataRecFunc)(unsigned char*, int, void*);

class CTcpIpBoostOpr 
{
public:
	CTcpIpBoostOpr();
};

class CClientBoostOpr 
{
public:
	CClientBoostOpr();
	~CClientBoostOpr();

	void SetCallBack(lpTcpIpDataRecFunc TcpIpDataCallBack, void* pUser);
	int Start(int nPort, char* pAddress,char* pLocalAddress = 0);
	int ReceiveMsg(unsigned char* msg,int len/*, int nTimeOutMilis = 0*/);
	int SendMsg(unsigned char* msg,int len/*, int nTimeOutMilis = 0*/);
	int Close();

private:
	void* m_pClient;
};

#endif