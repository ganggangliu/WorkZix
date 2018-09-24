#ifndef DATA_BUFFER_H
#define DATA_BUFFER_H

#include <windows.h>
#include <vector>
using namespace std;

template <class T>  
class CDataBuffer
{
public:
	CDataBuffer();
	~CDataBuffer();
	
	void Init(int nBufSize);

	int AddData(T& Data);
	
	int GetDatas(vector<T>& VecData);

	int GetDropCont(){return m_nDropCont;};

private:
	int m_nBufSize;
	vector<T> m_Buffer;
	int m_nDropCont;
	CRITICAL_SECTION m_cs;
};

template <class T>
CDataBuffer<T>::CDataBuffer()
{
}

template <class T>
CDataBuffer<T>::~CDataBuffer()
{
	DeleteCriticalSection(&m_cs);
}

template <class T>
void CDataBuffer<T>::Init(int nBufSize)
{
	m_nBufSize = nBufSize;
	m_Buffer.reserve(nBufSize);
	m_nDropCont = 0;
	InitializeCriticalSection(&m_cs);
}

template <class T>
int CDataBuffer<T>::AddData(T& Data)
{
	EnterCriticalSection(&m_cs);
	if (m_Buffer.size()>=m_nBufSize)
	{
		m_Buffer.clear();
		m_nDropCont+=m_nBufSize;
		m_Buffer.push_back(Data);
		LeaveCriticalSection(&m_cs);
		return 0;
	}
	m_Buffer.push_back(Data);
	LeaveCriticalSection(&m_cs);
	return 1;
}

template <class T>
int CDataBuffer<T>::GetDatas(vector<T>& VecData)
{
	EnterCriticalSection(&m_cs);
	VecData = m_Buffer;
	m_Buffer.clear();
	LeaveCriticalSection(&m_cs);
	return VecData.size();
}










#endif