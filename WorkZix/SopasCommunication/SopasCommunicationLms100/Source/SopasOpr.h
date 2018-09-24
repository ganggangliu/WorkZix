#ifndef SOPAS_OPR_H
#define SOPAS_OPR_H

#ifdef SOPASCOMMUNICATIONLMS100_EXPORTS
#define SOPASOPR_API __declspec(dllexport)
#else
#define SOPASOPR_API __declspec(dllimport)
#endif

#include "OpenCVInc.h"
#include <vector>

using namespace std;

typedef void(__stdcall *lpSopasDataCallBack)(vector<cv::Point2f>*, void*);

class CSopasParam
{
public:
	char szIp[16];
	unsigned int nPort;
	double dAngleMin;
	double dAngleMax;
	double dHeading;
	cv::Point2d T;
	CSopasParam::CSopasParam()
	{
		memset(szIp, 0, 16);
		nPort = 2111;
		dAngleMin = 0.0;
		dAngleMax = 180.0;
		dHeading = 0.0;
		T.x = 0;
		T.y = 0;
	}
};

class CSopasOpr
{
public:
	CSopasOpr();
	~CSopasOpr();

	int Init(CSopasParam& Param);
	void SetCallBack(lpSopasDataCallBack hCallBack, void* pUser);
	int Start();
	int GetData(vector<cv::Point2f>& Points);
	int Stop();

private:
	void* m_pSopas;
};


#endif
