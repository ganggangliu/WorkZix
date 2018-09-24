#ifndef TCPIPOPR_H
#define TCPIPOPR_H

#include<windows.h>  
#include <stdio.h> 

#pragma comment(lib, "Ws2_32.lib")


class CTcpIpOpr 
{
public:
	CTcpIpOpr(void);
	
};

typedef void(WINAPI *lpTcpIpDataRecFunc)(unsigned char*, int, void*);

class CClientNet  
{  
public:  
	enum STATE
	{
		WAIT_HEAD,
		FILL_DATA,
		FINISH
	};
	CClientNet();
	~CClientNet();
	//连接上指定服务器  
	int Connect(int port,const char* address,const char* LocalAddress = NULL);  
	//收数据
	int ReceiveMsg(unsigned char* msg,int len);
	//发送信息  
	int SendMsg(const unsigned char* msg,int len);  
	//关闭  
	void Close();  
	//
	void SetCallBack(lpTcpIpDataRecFunc TcpIpDataCallBack, void* pUser, unsigned char* pszHead, long nHeadLen, long nDataLen, long nRecLen);
	//
	long Start();

private:  
	static DWORD WINAPI ThreadFunc(LPVOID pParam);
	HANDLE m_hThread;
	bool m_bIsRun;
	SOCKET m_sock;  
	unsigned char m_szHead[16];
	long m_nHeadLen;
	long m_nDataLen;
	long m_nRecLen;
	void* m_pUser;
	lpTcpIpDataRecFunc m_TcpIpDataCallBack;
	STATE m_state;
	void* m_pBinDataOpr;
};  

class CServerNet  
{  
public:  

	//初始化服务器,返回0表示成功  
	int Init(const char* address,int port);  

	//更新数据  
	void Run();  

private:  
	SOCKET m_sock;  
};  

class CUDPReciever
{
public:
	CUDPReciever();
	~CUDPReciever();
	//创建并连接监听
	int Connect(int port,const char* address);  
	//收数据
	int ReceiveMsg(unsigned char* msg,int len);
	//发送信息  
	int SendMsg(char* pszDesAddr, int nDesPort, const unsigned char* msg,int len);  
	//关闭  
	void Close();  
	//
	void SetCallBack(lpTcpIpDataRecFunc TcpIpDataCallBack, void* pUser, unsigned char* pszHead, long nHeadLen, long nDataLen, long nRecLen);
	//
	long Start();

private:  
	static DWORD WINAPI ThreadFunc(LPVOID pParam);
	HANDLE m_hThread;
	bool m_bIsRun;
	SOCKET m_sock; 
	SOCKET m_RemoteSock; 
	unsigned char* m_pszHead;
	long m_nHeadLen;
	long m_nDataLen;
	long m_nRecLen;
	void* m_pUser;
	lpTcpIpDataRecFunc m_TcpIpDataCallBack;
	void* m_pBinDataOpr;
};

#endif