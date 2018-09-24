#ifndef CONFUTION_H
#define CONFUTION_H


#include "OpenCVInc.h"

class CConfution
{
public:
	CConfution();
	~CConfution();

	int Init(CSopasParam& Param);
	void SetCallBack(lpSopasDataCallBack hCallBack, void* pUser);
	int Start();
	int GetData(vector<cv::Point2f>& Points);
	int Stop();

private:
	void* m_pSopas;
};


#endif
