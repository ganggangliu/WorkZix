#ifndef DELPHI_RSDS_OPR_H
#define DELPHI_RSDS_OPR_H

#include "CanOpr.h"
#include "LIDAR_RADAR_INFO.hpp"

#ifdef DELPHIRSDSOPR_EXPORTS
#define DELPHIRSDSOPR_API __declspec(dllexport)
#else
#define DELPHIRSDSOPR_API __declspec(dllimport)
#endif

typedef void(__stdcall *lpDelphiRsdsDataCallBack)(LIDAR_RADAR_INFO*, void*);

struct CDelphiRsdsParam
{
	CCanParam XCanParam;
	CCanParam PCanParam;
	string szIpLeft;
	string szIpRight;
};

class DELPHIRSDSOPR_API CDelphiRsdsOpr 
{
public:
	CDelphiRsdsOpr();
	~CDelphiRsdsOpr();
	int Init(CDelphiRsdsParam& Param);
	void SetCallBackL(lpDelphiRsdsDataCallBack hCallBack, void* pUser);
	void SetCallBackR(lpDelphiRsdsDataCallBack hCallBack, void* pUser);
	int GetDataL(LIDAR_RADAR_INFO& Data);
	int GetDataR(LIDAR_RADAR_INFO& Data);
	int Start();
	int Stop();

private:
	void* m_pDelphiOpr;
};


#endif