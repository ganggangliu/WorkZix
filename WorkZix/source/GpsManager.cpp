#include "GpsManager.h"
#include "LcmReceiver.h"
#include <math.h>

void WINAPI IMUDataCallBack(void* pData,void* pUser)
{
	CGpsManager* GpsMgr = (CGpsManager*)pUser;
	LCM_GPFPD_BIN_DATA* pBinData = (LCM_GPFPD_BIN_DATA*)pData;
	LCM_GPS_DATA GpsImuT;
	if (GpsMgr->ParseBinData(GpsImuT,*pBinData) == 0)
		return ;
	GpsImuT.GPS_DATATYPE = 1;
	if (GpsMgr->m_hLcmCallBack)
		(*GpsMgr->m_hLcmCallBack)(&GpsImuT,GpsMgr->m_pUser);
	if ((long)(GpsImuT.GPS_TIME*1000)%1000 == 0 && GpsMgr->m_bIsPrintLog == true)
	{
		GpsMgr->PrintLog(GpsImuT);
	}
}

void WINAPI GpsDataCallBack(void* pData,void* pUser)
{
	CGpsManager* GpsMgr = (CGpsManager*)pUser;
	LCM_GPS_DATA* pGpsData = (LCM_GPS_DATA*)pData;
	if (GpsMgr->m_hLcmCallBack)
		(*GpsMgr->m_hLcmCallBack)(pGpsData,GpsMgr->m_pUser);
	if ((long)(pGpsData->GPS_TIME*1000)%1000 == 0 && GpsMgr->m_bIsPrintLog == true)
	{
		GpsMgr->PrintLog(*pGpsData);
	}
}

void WINAPI PosDataCallBack(void* pData,void* pUser)
{
	CGpsManager* GpsMgr = (CGpsManager*)pUser;
	vector<char>* pBinData = (vector<char>*)pData;
	LCM_GPS_DATA GpsImuT;
	LCM_POS_BIN_DATA BinDataT;
//	memcpy(&BinDataT.data, pBinData->data(), pBinData->size());
//	if (GpsMgr->ParseBinDataPos(GpsImuT,BinDataT) == 0)
	if (GpsMgr->ParseBinDataPos(GpsImuT,pBinData) == 0)
		return ;
	GpsImuT.GPS_DATATYPE = 2;
	if (GpsMgr->m_hLcmCallBack)
		(*GpsMgr->m_hLcmCallBack)(&GpsImuT,GpsMgr->m_pUser);
	double nnn = (GpsImuT.GPS_TIME - floor(GpsImuT.GPS_TIME));
	long nB = nnn * 1000;
//	printf("%.3f\n",GpsImuT.GPS_TIME);
	if (nB <= 10 && GpsMgr->m_bIsPrintLog == true)
	{
		GpsMgr->PrintLog(GpsImuT);
	}
}

void WINAPI HmiDataCallBack(void* pData,void* pUser)
{
	CGpsManager* GpsMgr = (CGpsManager*)pUser;
	vector<char>* pBinData = (vector<char>*)pData;
	LCM_GPS_DATA GpsImuT;
	if (GpsMgr->ParseBinDataPos(GpsImuT,pBinData) == 0)
		return ;
	GpsImuT.GPS_DATATYPE = CGpsManager::GPS_MANAGER_TYPE_HMI;
	if (GpsMgr->m_hLcmCallBack)
		(*GpsMgr->m_hLcmCallBack)(&GpsImuT,GpsMgr->m_pUser);
	double nnn = (GpsImuT.GPS_TIME - floor(GpsImuT.GPS_TIME));
	long nB = nnn * 1000;
	//	printf("%.3f\n",GpsImuT.GPS_TIME);
	if (nB <= 10 && GpsMgr->m_bIsPrintLog == true)
	{
		GpsMgr->PrintLog(GpsImuT);
	}
}

CGpsManager::CGpsManager(/*GPS_MANAGER_TYPE nType, double dTrigerLen*/)
{
// 	m_szURL = "udpm://239.255.76.67:7667?ttl=1";
// 	m_szChannleNameIMU = "LCM_GPFPD_BIN_DATA";
// 	m_szChannleNameGPS = "LCM_GPS_DATA";
// 	m_szChannleNamePOS = "GPS_POSITION_FPS_100";
// 
// 	m_hLcmCallBack = 0;
// 	m_pUser = NULL;
// 	m_nType = nType;
// 	m_dTrigerLen = dTrigerLen;
// 	m_ImuInfoBefor.GPS_LATITUDE = 0;
// 	m_ImuInfoBefor.GPS_LONGITUDE = 0;
// 	m_bIsPrintLog = false;
}

// CGpsManager::CGpsManager()
// {
// 	
// }

CGpsManager::~CGpsManager(void)
{

}

void CGpsManager::Init(GPS_MANAGER_TYPE nType, double dTrigerLen/* = 0*/)
{
	m_szURL = "udpm://239.255.76.67:7667?ttl=1";
	m_szChannleNameIMU = "LCM_GPFPD_BIN_DATA";
	m_szChannleNameGPS = "LCM_GPS_DATA";
	m_szChannleNamePOS = "GPS_POSITION_FPS_100";
	m_szChannleNameHMI = "KOTEI_DF_GNSS_DATA";
	m_hLcmCallBack = 0;
	m_pUser = NULL;
	m_nType = nType;
	m_dTrigerLen = dTrigerLen;
	m_ImuInfoBefor.GPS_LATITUDE = 0;
	m_ImuInfoBefor.GPS_LONGITUDE = 0;
	m_bIsPrintLog = false;
}

long CGpsManager::CheckMile(LCM_GPS_DATA& ImuInfo, double& dDeltaDist)
{
	LCM_GPS_DATA ImuInfoT;
	long nRt = GetData(ImuInfoT);

	dDeltaDist = GetDist(m_ImuInfoBefor,ImuInfoT);
	if (dDeltaDist >= m_dTrigerLen)
	{
		m_ImuInfoBefor = ImuInfoT;
		ImuInfo = ImuInfoT;
		return 1;
	}
	else
	{
		ImuInfo = ImuInfoT;
		return 0;
	}

	return nRt;
}

long CGpsManager::GetData(LCM_GPS_DATA& ImuInfo)
{
	if (m_nType == GPS_MANAGER_TYPE_GPS)
	{
		LCM_GPS_DATA GpsData;
		long nRt = ((CLcmRevicer<LCM_GPS_DATA>*)m_pLcmGPS)->GetData(GpsData);
		ImuInfo = GpsData;
		return nRt;
	}
	if (m_nType == GPS_MANAGER_TYPE_IMU)
	{
		LCM_GPFPD_BIN_DATA BinData;
		long nRt = ((CLcmRevicer<LCM_GPFPD_BIN_DATA>*)m_pLcmGPFPD)->GetData(BinData);
		if (nRt != 1)
		{
			return 0;
		}
		if (ParseBinData(ImuInfo,BinData) == 0)
		{
			printf("Parse error!\n");
			return 0;
		}
		ImuInfo.GPS_DATATYPE = 1;
		return 1;
	}
	if (m_nType == GPS_MANAGER_TYPE_POS)
	{
		LCM_POS_BIN_DATA BinData;
		vector<char> Temp;
		long nRt = ((CLcmRevicer<LCM_POS_BIN_DATA>*)m_pLcmPOS)->GetDataEx(Temp);
		if (nRt != 1)
		{
			return 0;
		}
		memcpy(BinData.data, Temp.data(), 61);
		if (ParseBinDataPos(ImuInfo,BinData) == 0)
			return 0;
		ImuInfo.GPS_DATATYPE = 2;
		return 1;
	}

	if (m_nType == GPS_MANAGER_TYPE_HMI)
	{
		LCM_POS_BIN_DATA BinData;
		vector<char> Temp;
		long nRt = ((CLcmRevicer<LCM_POS_BIN_DATA>*)m_pLcmPOS)->GetDataEx(Temp);
		if (nRt != 1)
		{
			return 0;
		}
		memcpy(BinData.data, Temp.data(), 61);
		if (ParseBinDataPos(ImuInfo,BinData) == 0)
			return 0;
		ImuInfo.GPS_DATATYPE = CGpsManager::GPS_MANAGER_TYPE_HMI;
		return 1;
	}

	printf("Gps type not supported!\n");
	return 0;
}

bool CGpsManager::ParseBinData(LCM_GPS_DATA& gpsDataOut, LCM_GPFPD_BIN_DATA& BinData)
{
	if (CRCheck((unsigned char*)BinData.data,53) == false)
		return false;

	float fTemp = 0;
	unsigned int nTemp = 0;
	unsigned char cTemp = 0;

	memcpy(&nTemp,BinData.data+3+2,sizeof(unsigned int));
	gpsDataOut.GPS_TIME = (double)nTemp/1000.f;

	memcpy(&fTemp,BinData.data+3+6,sizeof(float));
	gpsDataOut.GPS_HEADING = fTemp;

	memcpy(&fTemp,BinData.data+3+10,sizeof(float));
	gpsDataOut.GPS_PITCH = fTemp;

	memcpy(&fTemp,BinData.data+3+14,sizeof(float));
	gpsDataOut.GPS_ROLL = fTemp;

	memcpy(&nTemp,BinData.data+3+18,sizeof(unsigned int));
	gpsDataOut.GPS_LATITUDE = (double)nTemp*1e-7;

	memcpy(&nTemp,BinData.data+3+22,sizeof(unsigned int));
	gpsDataOut.GPS_LONGITUDE = (double)nTemp*1e-7;

	memcpy(&nTemp,BinData.data+3+26,sizeof(unsigned int));
	gpsDataOut.GPS_ALTITUDE = (double)nTemp*0.001f;

	memcpy(&fTemp,BinData.data+3+30,sizeof(float));
	gpsDataOut.GPS_VE = fTemp;

	memcpy(&fTemp,BinData.data+3+34,sizeof(float));
	gpsDataOut.GPS_VN = fTemp;

	memcpy(&fTemp,BinData.data+3+38,sizeof(float));
	gpsDataOut.GPS_VU = fTemp;

	memcpy(&fTemp,BinData.data+3+42,sizeof(float));
	gpsDataOut.GPS_BASELINE = fTemp;

	memcpy(&cTemp,BinData.data+3+46,sizeof(unsigned char));
	gpsDataOut.GPS_NSV1 = cTemp;

	memcpy(&cTemp,BinData.data+3+47,sizeof(unsigned char));
	gpsDataOut.GPS_NSV2 = cTemp;

	memcpy(&cTemp,BinData.data+3+48,sizeof(unsigned char));
	gpsDataOut.GPS_STATE = cTemp;

	return true;
}

bool CGpsManager::ParseBinDataPos(LCM_GPS_DATA& gpsDataOut, LCM_POS_BIN_DATA& BinData)
{
	if (CRCheck((unsigned char*)(BinData.data+2),61-2) == false)
		return false;

	double dTemp = 0.f;
	unsigned char cTemp = 0;
	long lTemp = 0;
	unsigned long ulTemp = 0;
	short sTemp = 0;
	unsigned short usTemp = 0;


	// 	memcpy(&dTemp,BinData.data+3,sizeof(double));
	// 	gpsDataOut.GPSTime = dTemp;

	memcpy(&usTemp,BinData.data+4,sizeof(usTemp));
	unsigned short usWeek = usTemp;
	memcpy(&lTemp,BinData.data+6,sizeof(lTemp));
	long nMiliSecend = lTemp;
	gpsDataOut.GPS_TIME = usWeek*7.0*24.0*60.0*60.0 + lTemp/1000.0;

	memcpy(&cTemp,BinData.data+10,sizeof(cTemp));
	gpsDataOut.GPS_NSV1 = cTemp;
	//////////////////////////////////////////////////////////////////////////

	memcpy(&lTemp,BinData.data+11,sizeof(long));
	gpsDataOut.GPS_LATITUDE = (double)lTemp*1e-7;

	memcpy(&lTemp,BinData.data+15,sizeof(long));
	gpsDataOut.GPS_LONGITUDE = (double)lTemp*1e-7;

	memcpy(&lTemp,BinData.data+19,sizeof(long));
	gpsDataOut.GPS_ALTITUDE = (double)lTemp*1e-3;

	memcpy(&lTemp,BinData.data+23,sizeof(long));
	gpsDataOut.GPS_VN = (double)lTemp*1e-3;

	memcpy(&lTemp,BinData.data+27,sizeof(long));
	gpsDataOut.GPS_VE = (double)lTemp*1e-3;

	memcpy(&lTemp,BinData.data+31,sizeof(long));
	gpsDataOut.GPS_VU = (double)lTemp*1e-3;

	memcpy(&lTemp,BinData.data+35,sizeof(long));
	gpsDataOut.GPS_ROLL = (double)lTemp*1e-3;

	memcpy(&lTemp,BinData.data+39,sizeof(long));
	gpsDataOut.GPS_PITCH = (double)lTemp*1e-3;

	memcpy(&ulTemp,BinData.data+43,sizeof(unsigned long));
	gpsDataOut.GPS_HEADING = (double)ulTemp*1e-3;

	memcpy(&sTemp,BinData.data+47,sizeof(short));
	double dAccn = (double)lTemp*1e-3;//北向加速度，m/s^2

	memcpy(&sTemp,BinData.data+49,sizeof(short));
	double dAcce = (double)lTemp*1e-3;//东向加速度，m/s^2

	memcpy(&sTemp,BinData.data+51,sizeof(short));
	double dAccu = (double)lTemp*1e-3;//地向加速度，m/s^2

	memcpy(&sTemp,BinData.data+53,sizeof(short));
	double dAccRoll = (double)lTemp*1e-3;//横滚角速度，degree/s

	memcpy(&sTemp,BinData.data+55,sizeof(short));
	double dAccePitch = (double)lTemp*1e-3;//俯仰角速度，degree/s

	memcpy(&sTemp,BinData.data+57,sizeof(short));
	double dAccuHeading = (double)lTemp*1e-3;//航向角速度，degree/s

	memcpy(&cTemp,BinData.data+59,sizeof(unsigned char));
	gpsDataOut.GPS_STATE = cTemp;

	double dVx = gpsDataOut.GPS_VE;
	double dVy = gpsDataOut.GPS_VN;
	double dHeading = gpsDataOut.GPS_HEADING;
	gpsDataOut.GPS_VN = cos(dHeading/180.0*3.141592654)*dVy + cos((dHeading+90.0)/180.0*3.141592654)*dVx;
	gpsDataOut.GPS_VE = sin(dHeading/180.0*3.141592654)*dVy + sin((dHeading+90.0)/180.0*3.141592654)*dVx;

	return true;
}

bool CGpsManager::ParseBinDataPos(LCM_GPS_DATA& gpsDataOut, vector<char>* pBinData)
{
	if (CRCheck((unsigned char*)(pBinData->data()+2),61-2) == false)
		return false;

	double dTemp = 0.f;
	unsigned char cTemp = 0;
	long lTemp = 0;
	unsigned long ulTemp = 0;
	short sTemp = 0;
	unsigned short usTemp = 0;


	// 	memcpy(&dTemp,BinData.data+3,sizeof(double));
	// 	gpsDataOut.GPSTime = dTemp;

	memcpy(&usTemp,pBinData->data()+4,sizeof(usTemp));
	unsigned short usWeek = usTemp;
	memcpy(&lTemp,pBinData->data()+6,sizeof(lTemp));
	long nMiliSecend = lTemp;
	gpsDataOut.GPS_TIME = usWeek*7.0*24.0*60.0*60.0 + lTemp/1000.0;

	memcpy(&cTemp,pBinData->data()+10,sizeof(cTemp));
	gpsDataOut.GPS_NSV1 = cTemp;

// 	return true;
	//////////////////////////////////////////////////////////////////////////

	memcpy(&lTemp,pBinData->data()+11,sizeof(long));
	gpsDataOut.GPS_LATITUDE = (double)lTemp*1e-7;

	memcpy(&lTemp,pBinData->data()+15,sizeof(long));
	gpsDataOut.GPS_LONGITUDE = (double)lTemp*1e-7;

	memcpy(&lTemp,pBinData->data()+19,sizeof(long));
	gpsDataOut.GPS_ALTITUDE = (double)lTemp*1e-3;

	memcpy(&lTemp,pBinData->data()+23,sizeof(long));
	gpsDataOut.GPS_VN = (double)lTemp*1e-3;

	memcpy(&lTemp,pBinData->data()+27,sizeof(long));
	gpsDataOut.GPS_VE = (double)lTemp*1e-3;

	memcpy(&lTemp,pBinData->data()+31,sizeof(long));
	gpsDataOut.GPS_VU = (double)lTemp*1e-3;

	memcpy(&lTemp,pBinData->data()+35,sizeof(long));
	gpsDataOut.GPS_ROLL = (double)lTemp*1e-3;

	memcpy(&lTemp,pBinData->data()+39,sizeof(long));
	gpsDataOut.GPS_PITCH = (double)lTemp*1e-3;

	memcpy(&ulTemp,pBinData->data()+43,sizeof(unsigned long));
	gpsDataOut.GPS_HEADING = (double)ulTemp*1e-3;

	memcpy(&sTemp,pBinData->data()+47,sizeof(short));
	double dAccn = (double)lTemp*1e-3;//北向加速度，m/s^2

	memcpy(&sTemp,pBinData->data()+49,sizeof(short));
	double dAcce = (double)lTemp*1e-3;//东向加速度，m/s^2

	memcpy(&sTemp,pBinData->data()+51,sizeof(short));
	double dAccu = (double)lTemp*1e-3;//地向加速度，m/s^2

	memcpy(&sTemp,pBinData->data()+53,sizeof(short));
	double dAccRoll = (double)lTemp*1e-3;//横滚角速度，degree/s

	memcpy(&sTemp,pBinData->data()+55,sizeof(short));
	double dAccePitch = (double)lTemp*1e-3;//俯仰角速度，degree/s

	memcpy(&sTemp,pBinData->data()+57,sizeof(short));
	double dAccuHeading = (double)lTemp*1e-3;//航向角速度，degree/s

	memcpy(&cTemp,pBinData->data()+59,sizeof(unsigned char));
	gpsDataOut.GPS_STATE = cTemp;

	double dVx = gpsDataOut.GPS_VE;
	double dVy = gpsDataOut.GPS_VN;
	double dHeading = gpsDataOut.GPS_HEADING;
	gpsDataOut.GPS_VN = cos(dHeading/180.0*3.141592654)*dVy + cos((dHeading+90.0)/180.0*3.141592654)*dVx;
	gpsDataOut.GPS_VE = sin(dHeading/180.0*3.141592654)*dVy + sin((dHeading+90.0)/180.0*3.141592654)*dVx;

	return true;
}

bool CGpsManager::CRCheck(unsigned char* pdata, int datalen)
{
	unsigned char CS = 0; 
	for (int i=0; i< datalen-1; i++) 
	{ 
		CS += pdata[i]; 
	}

	return (CS == pdata[datalen-1]);
}

double CGpsManager::GetDist(LCM_GPS_DATA& ImuInfo0, LCM_GPS_DATA& ImuInfo1)
{
	double dX = 111319.55*(ImuInfo0.GPS_LATITUDE - ImuInfo1.GPS_LATITUDE);
	double dY = 111319.55*(ImuInfo0.GPS_LONGITUDE - ImuInfo1.GPS_LONGITUDE)*cos(rad(ImuInfo1.GPS_LATITUDE));
	double dDist = sqrt(pow(dX,2) + pow(dY,2));

	return dDist;
}

double CGpsManager::rad(double d)
{
	return d * 3.141592654 / 180.0;
} 

void CGpsManager::SetCallBack(lpLcmRecFuncEx hLcmCallBack, void* pUser)
{
	m_hLcmCallBack = hLcmCallBack;
	m_pUser = pUser;
}

long CGpsManager::Start()
{
	m_pLcmGPFPD = new CLcmRevicer<LCM_GPFPD_BIN_DATA>(m_szURL,m_szChannleNameIMU);
	m_pLcmGPS = new CLcmRevicer<LCM_GPS_DATA>(m_szURL,m_szChannleNameGPS);
	m_pLcmPOS = new CLcmRevicer<LCM_POS_BIN_DATA>(m_szURL,m_szChannleNamePOS,LCM_BINARY_TYPE);
	m_pLcmHMI = new CLcmRevicer<LCM_POS_BIN_DATA>(m_szURL,m_szChannleNameHMI,LCM_BINARY_TYPE);
	if (m_nType == GPS_MANAGER_TYPE_IMU)
	{
		((CLcmRevicer<LCM_GPFPD_BIN_DATA>*)m_pLcmGPFPD)->SetCallBack(IMUDataCallBack,this);
		((CLcmRevicer<LCM_GPFPD_BIN_DATA>*)m_pLcmGPFPD)->Start();
	}
	if (m_nType == GPS_MANAGER_TYPE_GPS)
	{
		((CLcmRevicer<LCM_GPS_DATA>*)m_pLcmGPS)->SetCallBack(GpsDataCallBack,this);
		((CLcmRevicer<LCM_GPS_DATA>*)m_pLcmGPS)->Start();
	}
	if (m_nType == GPS_MANAGER_TYPE_POS)
	{
		((CLcmRevicer<LCM_POS_BIN_DATA>*)m_pLcmPOS)->SetCallBack(PosDataCallBack,this);
		((CLcmRevicer<LCM_POS_BIN_DATA>*)m_pLcmPOS)->Start();
	}
	if (m_nType == GPS_MANAGER_TYPE_HMI)
	{
		((CLcmRevicer<LCM_POS_BIN_DATA>*)m_pLcmHMI)->SetCallBack(HmiDataCallBack,this);
		((CLcmRevicer<LCM_POS_BIN_DATA>*)m_pLcmHMI)->Start();
	}
	return 1;
}

long CGpsManager::EnablePrintLog(bool bPrint)
{
	m_bIsPrintLog = bPrint;
	return 1;
}

void CGpsManager::PrintLog(LCM_GPS_DATA& ImuInfo)
{
	printf("\n%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.2x,%d\n",
		ImuInfo.GPS_TIME,
		ImuInfo.GPS_HEADING,
		ImuInfo.GPS_PITCH,
		ImuInfo.GPS_ROLL,
		ImuInfo.GPS_LATITUDE,
		ImuInfo.GPS_LONGITUDE,
		ImuInfo.GPS_ALTITUDE,
		ImuInfo.GPS_VE,
		ImuInfo.GPS_VN,
		ImuInfo.GPS_VU,
		ImuInfo.GPS_BASELINE,
		ImuInfo.GPS_NSV1,
		ImuInfo.GPS_NSV2,
		ImuInfo.GPS_STATE,
		ImuInfo.GPS_DATATYPE);
}