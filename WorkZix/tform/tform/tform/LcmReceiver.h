#pragma once
#include <windows.h>
#include "lcm/lcm-cpp.hpp"

#pragma comment(lib,"ws2_32.lib")
typedef void(WINAPI *lpLcmRecFunc)(void*);
typedef void(WINAPI *lpLcmRecFuncEx)(void*,void*);

enum LCM_TYPE
{
	LCM_TEMPLATE_TYPE = 0,
	LCM_BINARY_TYPE = 1
};

template <class T>  
class CLcmRevicer
{
public:
	CLcmRevicer(std::string& szURL, std::string& szChanNmae, LCM_TYPE type = LCM_TEMPLATE_TYPE);
	CLcmRevicer(std::string& szChanNmae, LCM_TYPE type = LCM_TEMPLATE_TYPE);
	~CLcmRevicer();
	void SetCallBack(lpLcmRecFunc hLcmCallBack);
	void SetCallBack(lpLcmRecFuncEx hLcmCallBackEx, void* pUser = NULL);
 	long Start();
 	long GetData(T& Msg);
	long GetDataEx(std::vector<char>& BinBuf);
	long Send(std::string szChannle, T& Msg);
private:
	void Init(std::string& szURL,std::string& szChanNmae, LCM_TYPE type);
	static DWORD WINAPI ThreadFunc(LPVOID pParam);
	void BeginThread();
	void HandleRouteMap(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,const T* pRouteLink);
	void HandleRouteMapEx(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan);
	lcm::LCM* m_plcmIpc;
	T m_Msg;
	void* m_pUser;
	CRITICAL_SECTION cs;
	long m_RepeatGetCont;
	lpLcmRecFunc m_hLcmCallBack;
	lpLcmRecFuncEx m_hLcmCallBackEx;
	HANDLE m_hThread;
	long nTest;
	long m_nLcmType;
	std::vector<char> m_BinBuf;
};

template<class T>
DWORD WINAPI CLcmRevicer<T>::ThreadFunc(LPVOID pParam)
{
	CLcmRevicer<T>* pLcm = (CLcmRevicer<T>*)pParam;
	pLcm->BeginThread();
	return 1;
}

template<class T>
void CLcmRevicer<T>::Init(std::string& szURL,std::string& szChanNmae, LCM_TYPE type)
{ 
	m_plcmIpc = new lcm::LCM(szURL);
	if(type == LCM_TEMPLATE_TYPE)
		m_plcmIpc->subscribe(szChanNmae, &CLcmRevicer::HandleRouteMap, this);
	else
		m_plcmIpc->subscribe(szChanNmae, &CLcmRevicer::HandleRouteMapEx, this);
	InitializeCriticalSection(&cs);
	m_RepeatGetCont = 0;
	m_hLcmCallBack = 0;
	m_hLcmCallBackEx = 0;
	m_hThread = 0;
//	memset(&m_Msg,0,sizeof(T));
	m_pUser = NULL;
	m_nLcmType = type;
}

template<class T>
CLcmRevicer<T>::CLcmRevicer(std::string& szURL,std::string& szChanNmae, LCM_TYPE type)
{ 
	Init(szURL,szChanNmae,type);
}

template<class T>
CLcmRevicer<T>::CLcmRevicer(std::string& szChanNmae, LCM_TYPE type)
{ 
	std::string szURL = "udpm://239.255.76.67:7667?ttl=1";
	Init(szURL,szChanNmae,type);
}

template<class T>
CLcmRevicer<T>::~CLcmRevicer()
{
	DeleteCriticalSection(&cs);
}

template<class T>
void CLcmRevicer<T>::HandleRouteMap(const lcm::ReceiveBuffer* rbuf,
	const std::string& chan,const T* pRouteLink)
{
	EnterCriticalSection(&cs);
	m_Msg = *pRouteLink;
	m_RepeatGetCont = 0;
	LeaveCriticalSection(&cs);
	if (m_hLcmCallBack)
	{
		(*m_hLcmCallBack)(&m_Msg);
	}
	if (m_hLcmCallBackEx)
	{
		(*m_hLcmCallBackEx)(&m_Msg,m_pUser);
	}
}

template<class T>
void CLcmRevicer<T>::HandleRouteMapEx(const lcm::ReceiveBuffer* rbuf,
	const std::string& chan)
{
	EnterCriticalSection(&cs);
	m_BinBuf.resize(rbuf->data_size);
	memcpy(m_BinBuf.data(), rbuf->data, rbuf->data_size);
// 	for(int i = 0; i < rbuf->data_size; i++)
// 	{
// 		m_BinBuf[i] = (((char*)(rbuf->data))[i]);
// 	}
	m_RepeatGetCont = 0;
	LeaveCriticalSection(&cs);
	if (m_hLcmCallBack)
	{
		(*m_hLcmCallBack)(&m_BinBuf);
	}
	if (m_hLcmCallBackEx)
	{
		(*m_hLcmCallBackEx)(&m_BinBuf,m_pUser);
	}
}

template<class T>
long CLcmRevicer<T>::GetData(T& Msg)
{
	EnterCriticalSection(&cs);
	Msg = m_Msg;
	m_RepeatGetCont++;
	LeaveCriticalSection(&cs);
	return m_RepeatGetCont;
}

template<class T>
long CLcmRevicer<T>::GetDataEx(std::vector<char>& BinBuf)
{
	EnterCriticalSection(&cs);
	BinBuf.resize(m_BinBuf.size());
	memcpy(BinBuf.data(), m_BinBuf.data(), m_BinBuf.size());
	m_RepeatGetCont++;
	LeaveCriticalSection(&cs);
	return m_RepeatGetCont;
}

template<class T>
void CLcmRevicer<T>::BeginThread()
{
	struct timeval timeout = { 3, 0 }; 
	fd_set readfds;
	int fd = m_plcmIpc->getFileno();
	while (1) {
		FD_ZERO (&readfds);
		FD_SET (fd,&readfds);
		int status = select (fd + 1,&readfds,0,0,&timeout);  
		if(-1 == status) {
			if(EINTR == errno) 
			{
				Sleep(10);
			}
			else 
			{
				break;
			}
		}
		else if(0 == status) 
		{			
		}
		else 
		{
			if(FD_ISSET (fd,&readfds)) 
			{			
				m_plcmIpc->handle();
			}
		}
	}

}

template<class T>
void CLcmRevicer<T>::SetCallBack(lpLcmRecFunc hLcmCallBack)
{
	m_hLcmCallBack = hLcmCallBack;
}

template<class T>
void CLcmRevicer<T>::SetCallBack(lpLcmRecFuncEx hLcmCallBackEx, void* pUser)
{
	m_hLcmCallBackEx = hLcmCallBackEx;
	m_pUser = pUser;
}

template<class T>
long CLcmRevicer<T>::Start()
{
	if (m_hThread != 0)
	{
		printf("m_hThread != 0\n");
		return 0;
	}

	m_hThread = CreateThread(NULL,0,ThreadFunc,this,0,NULL);
	if (m_hThread == 0)
	{
		printf("CreateThread Failed \n");
		return 0;
	}

	return 1;
	
}

template<class T>
long CLcmRevicer<T>::Send(std::string szChannle, T& Msg)
{
	return m_plcmIpc->publish(szChannle, &Msg);
}

