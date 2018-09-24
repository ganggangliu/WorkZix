#include <WS2tcpip.h>
#include "TcpIpOpr.h"
#include "BinDataCallBack.h"


CTcpIpOpr::CTcpIpOpr()
{
	return;
}

CClientNet::CClientNet()
{
	memset(m_szHead,0,sizeof(m_szHead));
	m_nHeadLen = 0;
	m_TcpIpDataCallBack = NULL;
	m_nDataLen = 0;
	m_hThread = 0;
	m_nRecLen = 0;
	m_bIsRun = false;
	m_state = WAIT_HEAD;
	m_pUser = NULL;
	m_pBinDataOpr = new CBinDataCallBack;
}

CClientNet::~CClientNet()
{
	if (m_pBinDataOpr)
	{
		delete m_pBinDataOpr;
		m_pBinDataOpr = NULL;
	}
}

int CClientNet::Connect( int port,const char* address,const char* LocalAddress )  
{  
	int rlt = 0;  

	//用于记录错误信息并输出  
	int iErrMsg;  
	//启动WinSock  
	WSAData wsaData;  
	iErrMsg = WSAStartup(MAKEWORD(1,1),&wsaData);  
	if (iErrMsg != NO_ERROR)  
		//有错误  
	{  
		printf("failed with wsaStartup error : %d\n",iErrMsg);  

		rlt = 1;  
		return rlt;  
	}  

	//创建Socket  
	m_sock = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);  
	if (m_sock == INVALID_SOCKET)  
		//创建Socket失败  
	{  
		printf("socket failed with error : %d\n",WSAGetLastError());  

		rlt = 2;  
		return rlt;  
	}  

	//bind
	sockaddr_in sockaddrLocal;  
	sockaddrLocal.sin_family = AF_INET;  
	sockaddrLocal.sin_port = htons(0);  
	if (LocalAddress == NULL)
		sockaddrLocal.sin_addr.s_addr = htons(INADDR_ANY);
	else
		sockaddrLocal.sin_addr.s_addr = inet_addr(LocalAddress);
	if (bind(m_sock,(struct sockaddr*) &sockaddrLocal,sizeof(sockaddrLocal)) < 0)
	{
		printf("bind failed with error \n");  
		return 4;
	}

	//目标服务器数据  
	sockaddr_in sockaddrServer;
	sockaddrServer.sin_family = AF_INET;
	sockaddrServer.sin_port = htons(port);
	sockaddrServer.sin_addr.s_addr = inet_addr(address);

	//连接,sock与目标服务器连接  
	iErrMsg = connect(m_sock,(sockaddr*)&sockaddrServer,sizeof(sockaddrServer));  
	if (iErrMsg < 0)  
	{  
		printf("connect failed with error : %d\n",iErrMsg);  

		rlt = 3;  
		return rlt;  
	}  

	return rlt;  
}  

int CClientNet::ReceiveMsg(unsigned char* msg,int len)
{
	int rval; 
	rval = recv(m_sock,(char*)msg,len,0);  
	return rval;
}

int CClientNet::SendMsg(unsigned const char* msg,int len)  
{  
	int rlt = 0;  

	int iErrMsg = 0;  

	//发送消息，指定sock发送消息  
	iErrMsg = send(m_sock,(char*)msg,len,0);  
	if (iErrMsg < 0)  
		//发送失败  
	{  
		printf("send msg failed with error : %d\n",iErrMsg);  

		rlt = 1;  
		return rlt;  
	}  

	return rlt;  
}  

void CClientNet::Close()  
{   
	closesocket(m_sock);  
}  

void CClientNet::SetCallBack(lpTcpIpDataRecFunc TcpIpDataCallBack, void* pUser, unsigned char* pszHead, long nHeadLen, long nDataLen, long nRecLen)
{
	m_TcpIpDataCallBack = TcpIpDataCallBack;
	memcpy(m_szHead,pszHead,nHeadLen);
	m_nHeadLen = nHeadLen;
	m_nDataLen = nDataLen;
	m_nRecLen = nRecLen;
	m_pUser = pUser;
	((CBinDataCallBack*)m_pBinDataOpr)->SetDataPattern(pszHead,nHeadLen,nDataLen);
	((CBinDataCallBack*)m_pBinDataOpr)->SetCallBack(TcpIpDataCallBack,pUser);
}

long CClientNet::Start()
{
	if (m_hThread != 0)
	{
		printf("m_hThread != 0\n");
		return 0;
	}
	m_bIsRun = true;
	m_hThread = CreateThread(NULL,0,ThreadFunc,this,0,NULL);
	if (m_hThread == 0)
	{
		printf("CreateThread Failed \n");
		return 0;
	}

	return 1;
}

DWORD CClientNet::ThreadFunc(LPVOID pParam)
{
	CClientNet* pClientNet = (CClientNet*)pParam;
	unsigned char* pszBuff = new unsigned char[pClientNet->m_nRecLen];
	while(pClientNet->m_bIsRun)
	{
		long nRt = pClientNet->ReceiveMsg(pszBuff,pClientNet->m_nRecLen);
		printf("%d\n",nRt);
		((CBinDataCallBack*)pClientNet->m_pBinDataOpr)->FeedData(pszBuff,nRt);
	}
	delete [] pszBuff;

	return 1;
}

int CServerNet::Init( const char* address,int port )  
{  
	int rlt = 0;  

	//用于记录错误信息，并输出  
	int iErrorMsg;  

	//初始化WinSock  
	WSAData wsaData;  
	iErrorMsg = WSAStartup(MAKEWORD(1,1),&wsaData);  

	if (iErrorMsg != NO_ERROR)  
	{  
		//初始化WinSock失败  
		printf("wsastartup failed with error : %d\n",iErrorMsg);  

		rlt = 1;  
		return rlt;  
	}  

	//创建服务端Socket  
	m_sock = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);  
	if (m_sock == INVALID_SOCKET)  

	{  
		//创建Socket异常  
		printf("socket failed with error : %d\n",WSAGetLastError());  

		rlt = 2;  
		return rlt;  
	}  

	//声明信息  
	sockaddr_in serverAddr;  
	serverAddr.sin_family = AF_INET;  
	serverAddr.sin_port = htons(port);  
	serverAddr.sin_addr.s_addr = inet_addr(address);  

	//绑定  
	iErrorMsg = bind(m_sock,(sockaddr*)&serverAddr,sizeof(serverAddr));  
	if (iErrorMsg < 0)  
	{  
		//绑定失败  
		printf("bind failed with error : %d\n",iErrorMsg);  
		rlt = 3;  
		return rlt;  
	}  



	return rlt;  
}  

void CServerNet::Run()  
{  
	//公开连接  
	listen(m_sock,5);  

	sockaddr_in tcpAddr;  
	int len = sizeof(sockaddr);  
	SOCKET newSocket;  
	unsigned char buf[1024];
	int rval;  

	do   
	{  
		//接收信息  
		newSocket = accept(m_sock,(sockaddr*)&tcpAddr,&len);  


		if (newSocket == INVALID_SOCKET)  
		{  
			//非可用socket  

		}  
		else  
		{  
			//新socket连接  
			printf("new socket connect : %d\n",newSocket);  


			//消息处理  
			do  
			{  
				printf("process\n");  
				//接收数据  
				memset(buf,0,sizeof(buf));  
				rval = recv(newSocket,(char*)buf,1024,0);  


				if (rval == SOCKET_ERROR)  
					//这应该是个异常，当客户端没有调用closeSocket就直接退出游戏的时候，将会进入这里  
					printf("recv socket error\n");  



				if (rval == 0)  
					//recv返回0表示正常退出  
					printf("ending connection");  
				else  
					//显示接收到的数据  
					printf("recv %s\n",buf);  


			}while(rval != 0);  

			//关闭对应Accept的socket  
			closesocket(newSocket);  
		}  



	} while (1);  

	//关闭自身的Socket  
	closesocket(m_sock);  
}

CUDPReciever::CUDPReciever()
{
	m_pszHead = NULL;
	m_nHeadLen = 0;
	m_TcpIpDataCallBack = NULL;
	m_nDataLen = 0;
	m_hThread = 0;
	m_nRecLen = 0;
	m_bIsRun = false;
	m_pUser = NULL;
	m_pBinDataOpr = new CBinDataCallBack;
}
CUDPReciever::~CUDPReciever()
{
	if (m_pBinDataOpr)
	{
		delete m_pBinDataOpr;
		m_pBinDataOpr = NULL;
	}
}
//创建并连接监听
int CUDPReciever::Connect(int port,const char* address)
{
	int rlt = 0;
	int iErrMsg;  
	WSAData wsaData;  
	iErrMsg = WSAStartup(MAKEWORD(2,2),&wsaData);  
	if (iErrMsg != NO_ERROR)
	{  
		printf("failed with wsaStartup error : %d\n",iErrMsg);  
		rlt = 1;  
		return rlt;  
	}  

	//创建Socket  
	m_sock = socket(AF_INET,SOCK_DGRAM,0);  
	if (m_sock == INVALID_SOCKET)
	{  
		printf("socket failed with error : %d\n",WSAGetLastError());  
		rlt = 2;  
		return rlt;  
	}  

	//bind
	sockaddr_in sockaddrLocal;  
	sockaddrLocal.sin_family = AF_INET;  
	sockaddrLocal.sin_port = htons(port);
	sockaddrLocal.sin_addr.s_addr = inet_addr(address);

	if (bind(m_sock,(SOCKADDR*) &sockaddrLocal,sizeof(sockaddrLocal)) < 0)
	{
		printf("bind failed with error \n");  
		return 4;
	}



	return rlt;  
}
//收数据
int CUDPReciever::ReceiveMsg(unsigned char* msg,int len)
{
	int nRet = 0;
	sockaddr_in RemoteSock;
	int dwSendSize = sizeof(RemoteSock);
	nRet = recvfrom(m_sock,(char*)msg,len,0,(SOCKADDR*)&RemoteSock,&dwSendSize);
	return nRet;
}
//发送信息  
int CUDPReciever::SendMsg(char* pszDesAddr, int nDesPort, const unsigned char* msg,int len)
{
	sockaddr_in RemoteSock;
	RemoteSock.sin_family = AF_INET;  
	RemoteSock.sin_addr.s_addr = inet_addr(pszDesAddr);  
	RemoteSock.sin_port=htons(nDesPort);  
	int nRt = sendto(m_sock, (char*)msg, len, 0,  
		(struct sockaddr *)&RemoteSock, sizeof(RemoteSock));
	return nRt;
}
//关闭  
void CUDPReciever::Close()
{
	//关闭socket连接  
	closesocket(m_sock);  
	//清理  
	WSACleanup();  
}
//
void CUDPReciever::SetCallBack(lpTcpIpDataRecFunc TcpIpDataCallBack, void* pUser, unsigned char* pszHead, long nHeadLen, long nDataLen, long nRecLen)
{
	m_TcpIpDataCallBack = TcpIpDataCallBack;
	m_pszHead = new unsigned char[nHeadLen];
	memcpy(m_pszHead,pszHead,nHeadLen);
	m_nHeadLen = nHeadLen;
	m_nDataLen = nDataLen;
	m_nRecLen = nRecLen;
	m_pUser = pUser;
	((CBinDataCallBack*)m_pBinDataOpr)->SetDataPattern(pszHead,nHeadLen,nDataLen);
	((CBinDataCallBack*)m_pBinDataOpr)->SetCallBack(TcpIpDataCallBack,pUser);
}
//
long CUDPReciever::Start()
{
	if (m_hThread != 0)
	{
		printf("m_hThread != 0\n");
		return 0;
	}
	m_bIsRun = true;
	m_hThread = CreateThread(NULL,0,ThreadFunc,this,0,NULL);
	if (m_hThread == 0)
	{
		printf("CreateThread Failed \n");
		return 0;
	}

	return 1;
}

DWORD CUDPReciever::ThreadFunc(LPVOID pParam)
{
	CUDPReciever* pUdpNet = (CUDPReciever*)pParam;
	unsigned char* pszBuff = new unsigned char[pUdpNet->m_nRecLen];
	while(pUdpNet->m_bIsRun)
	{
		long nRt = pUdpNet->ReceiveMsg(pszBuff,pUdpNet->m_nRecLen);
		((CBinDataCallBack*)pUdpNet->m_pBinDataOpr)->FeedData(pszBuff,nRt);
	}
	delete [] pszBuff;

	return 1;
}