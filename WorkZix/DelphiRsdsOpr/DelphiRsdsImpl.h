#ifndef DELPHI_RSDS_IMPL_H
#define DELPHI_RSDS_IMPL_H

#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>
#include <string>
#include "TcpIpBoostImpl.h"
#include "LIDAR_RADAR_INFO.hpp"
#include "BinDataCallBackBoost.h"
#include "CanOpr.h"

typedef boost::function<void(LIDAR_RADAR_INFO*,void*)> FunctionRsdsData;

struct CDelphiRsdsImplParam
{
	CCanParam XCanParam;
	CCanParam PCanParam;
	string szIpLeft;
	string szIpRight;
};

class CDelphiRsdsImpl
{
public:
	CDelphiRsdsImpl(void);

	int Init(CDelphiRsdsImplParam& Param);
	void SetCallBackL(FunctionRsdsData hCallBack, void* pUser);
	void SetCallBackR(FunctionRsdsData hCallBack, void* pUser);
	int GetDataL(LIDAR_RADAR_INFO& Data);
	int GetDataR(LIDAR_RADAR_INFO& Data);
	int Start();
	int Stop();

private:
	void TcpIpDataCallBackL(unsigned char* pData, int nLen, void* pUser);
	void TcpIpDataCallBackR(unsigned char* pData, int nLen, void* pUser);
	void BinDataCallBackL(unsigned char* pData, int nLen, void* pUser);
	void BinDataCallBackR(unsigned char* pData, int nLen, void* pUser);
	int TcpIpParse(unsigned char* psz, LIDAR_RADAR_INFO* pRsdsData);
	int PCanInit();
	int XCanInit();
	CDelphiRsdsImplParam m_Param;
	CCanOpr m_XCanOpr;
	CCanOpr m_PCanOpr;
	FunctionRsdsData m_pCallBackL;
	FunctionRsdsData m_pCallBackR;
	void* m_pUserL;
	void* m_pUserR;
	LIDAR_RADAR_INFO m_DataL;
	LIDAR_RADAR_INFO m_DataR;
	int m_nDataStateL;
	int m_nDataStateR;
	CClientBoostImpl m_TcpIpOprL;
	CClientBoostImpl m_TcpIpOprR;
	boost::mutex m_mutexL;
	boost::mutex m_mutexR;
	CBinDataCallBackBoost m_BinDataOprL;
	CBinDataCallBackBoost m_BinDataOprR;
};

#endif