#ifndef TCPIP_BOOST_OPR_H
#define TCPIP_BOOST_OPR_H


#ifdef TCPIPBOOSTOPR_EXPORTS
#define TCPIPBOOSTOPR_API __declspec(dllexport)
#else
#define TCPIPBOOSTOPR_API __declspec(dllimport)
#endif

typedef void(__stdcall *lpTcpIpDataRecFunc)(char*, int, void*);

class TCPIPBOOSTOPR_API CTcpIpBoostOpr 
{
public:
	CTcpIpBoostOpr();
};

class TCPIPBOOSTOPR_API CClientBoostOpr 
{
public:
	CClientBoostOpr();
	~CClientBoostOpr();

	void SetCallBack(lpTcpIpDataRecFunc TcpIpDataCallBack, void* pUser);
	int Start(int nPort, char* pAddress,char* pLocalAddress = 0);
	int ReceiveMsg(char* msg,int len/*, int nTimeOutMilis = 0*/);
	int SendMsg(char* msg,int len/*, int nTimeOutMilis = 0*/);
	int Close();

private:
	void* m_pClient;
};

#endif