#ifndef GPSMANAGER_H
#define GPSMANAGER_H

#include <Windows.h>
#include <string>
#include <vector>
#include "LCM_GPFPD_BIN_DATA.hpp"
#include "LCM_GPS_DATA.hpp"
#include "LCM_POS_BIN_DATA.hpp"
using namespace std;

typedef void(WINAPI *lpGpsManagerRecFunc)(void*,void*);

class CGpsManager 
{
public:
	enum GPS_MANAGER_TYPE
	{
		GPS_MANAGER_TYPE_GPS = 0,
		GPS_MANAGER_TYPE_IMU = 1,
		GPS_MANAGER_TYPE_POS = 2,
		GPS_MANAGER_TYPE_HMI = 3
	};

public:
//	CGpsManager(GPS_MANAGER_TYPE nType, double dTrigerLen = 0);
	CGpsManager();
	~CGpsManager(void);

public:
	void Init(GPS_MANAGER_TYPE nType, double dTrigerLen = 0);
	void SetCallBack(lpGpsManagerRecFunc hLcmCallBack, void* pUser = NULL);
	long Start();
	long EnablePrintLog(bool bPrint);
	long GetData(LCM_GPS_DATA& ImuInfo);
	long CheckMile(LCM_GPS_DATA& ImuInfo, double& dDeltaDist);
	bool ParseBinData(LCM_GPS_DATA& gpsDataOut, LCM_GPFPD_BIN_DATA& BinData);
	bool ParseBinDataPos(LCM_GPS_DATA& gpsDataOut, LCM_POS_BIN_DATA& BinData);
	bool ParseBinDataPos(LCM_GPS_DATA& gpsDataOut, vector<char>* pBinData);
	lpGpsManagerRecFunc m_hLcmCallBack;
	void* m_pUser;
	void PrintLog(LCM_GPS_DATA& ImuInfo);
	bool m_bIsPrintLog;

private:
	bool CRCheck(unsigned char* pdata, int datalen);
	double GetDist(LCM_GPS_DATA& ImuInfo0, LCM_GPS_DATA& ImuInfo1);
	double rad(double d);
	LCM_GPS_DATA m_ImuInfoBefor;
//	CLcmRevicer<GPFPD_BIN_DATA>* m_pLcmGPFPD;
//	CLcmRevicer<GPS_DATA>* m_pLcmGPS;
	void* m_pLcmGPFPD;
	void* m_pLcmGPS;
	void* m_pLcmPOS;
	void* m_pLcmHMI;
	long m_nType;
	double m_dTrigerLen;
	string m_szURL;
	string m_szChannleNameIMU;
	string m_szChannleNameGPS;
	string m_szChannleNamePOS;
	string m_szChannleNameHMI;
};

#endif