#include <fstream> 
#include <time.h>
#include "LidarDataOpr.h"

CLidarDataOpr::CLidarDataOpr(char* pszPath)
{
	m_nInd = 0;
	strcpy_s(m_szPath,pszPath);
	strcpy_s(m_szPathRead,pszPath);
	m_fWrite = NULL;
	m_fRead = NULL;
}
CLidarDataOpr::~CLidarDataOpr(void)
{
	if (m_fWrite)
	{
		fclose(m_fWrite);
		m_fWrite = NULL;
	}
	if (m_fRead)
	{
		fclose(m_fRead);
		m_fRead = NULL;
	}
}

long CLidarDataOpr::CreateDir()
{
	AddTimeTail(m_szPath);
	strcat_s(m_szPath,"\\");
	CreateDirectoryA(m_szPath,NULL);

	return 1;
}

long CLidarDataOpr::WriteLog(LIDAR_CLOUD_POINTS* pLidarPoints, CGpsManager::GPS_IMU_INFO* pImuInfo)
{
	char szPathCloudPt[MAX_PATH] = {0};
	char szPathImu[MAX_PATH] = {0};
	sprintf_s(szPathCloudPt,"%s%06d.lidar",m_szPath,m_nInd);
	sprintf_s(szPathImu,"%s%06d.imu",m_szPath,m_nInd);
	m_fWrite = fopen(szPathCloudPt,"wt");
	fprintf(m_fWrite,"%d\t%d\t%d\t%d\t%d\t%d\t\n",
		(long)pLidarPoints->nCont,
		(long)pLidarPoints->nDevInd,
		(long)pLidarPoints->nDevType,
		(long)pLidarPoints->nFrameInd,
		(long)pLidarPoints->nTimeStamp,
		(long)pLidarPoints->points.size());

	for (int i = 0; i < pLidarPoints->points.size(); i++)
	{
		fprintf(m_fWrite,"%d\t%d\t%d\t%.2f\t%.2f\t%.2f\n",
			(long)pLidarPoints->points[i].layer,
			(long)pLidarPoints->points[i].flag,
			(long)pLidarPoints->points[i].echo,
			(double)pLidarPoints->points[i].angle,
			(double)pLidarPoints->points[i].distance,
			(double)pLidarPoints->points[i].epw);
	}
	fclose(m_fWrite);

	m_fWrite = fopen(szPathImu,"wt");
	fprintf(m_fWrite,"%.3f\t%.3f\t%.3f\t%.3f\t%.7f\t%.7f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d\t%d\t%d\t%d\n",
		pImuInfo->GPSTime,
		pImuInfo->Heading,
		pImuInfo->Pitch,
		pImuInfo->Roll,
		pImuInfo->Lattitude,
		pImuInfo->Longitude,
		pImuInfo->Altitude,
		pImuInfo->Ve,
		pImuInfo->Vn,
		pImuInfo->Vu,
		pImuInfo->Baseline,
		pImuInfo->NSV1,
		pImuInfo->NSV2,
		pImuInfo->Status,
		pImuInfo->Type);
	fclose(m_fWrite);
	//ofstream fCloudPt(szPathCloudPt,ios::trunc);
	//ofstream fImu(szPathImu,ios::trunc);

	//fCloudPt<<(long)pLidarPoints->nCont<<"\t"<<(long)pLidarPoints->nDevInd<<"\t"
	//							 <<(long)pLidarPoints->nDevType<<"\t"<<(long)pLidarPoints->nFrameInd<<"\t"
	//							 <<(long)pLidarPoints->nTimeStamp<<"\t"<<(long)pLidarPoints->points.size()<<endl<<endl;
	//for (int i = 0; i < pLidarPoints->points.size(); i++)
	//{
	//	fCloudPt<<(long)pLidarPoints->points[i].layer<<"\t";
	//	fCloudPt<<(long)pLidarPoints->points[i].flag<<"\t";
	//	fCloudPt<<(long)pLidarPoints->points[i].echo<<"\t";
	//	fCloudPt<<(double)pLidarPoints->points[i].angle<<"\t";
	//	fCloudPt<<(double)pLidarPoints->points[i].distance<<"\t";
	//	fCloudPt<<(double)pLidarPoints->points[i].epw<<endl;
	//}
	//fCloudPt.close();

	//fImu<<(double)pImuInfo->GPSTime<<"\t"
	//	<<(double)pImuInfo->Heading<<"\t"
	//	<<(double)pImuInfo->Pitch<<"\t"
	//	<<(double)pImuInfo->Roll<<"\t"
	//	<<(double)pImuInfo->Lattitude<<"\t"
	//	<<(double)pImuInfo->Longitude<<"\t"
	//	<<(double)pImuInfo->Altitude<<"\t"
	//	<<(double)pImuInfo->Ve<<"\t"
	//	<<(double)pImuInfo->Vn<<"\t"
	//	<<(double)pImuInfo->Vu<<"\t"
	//	<<(double)pImuInfo->Baseline<<"\t"
	//	<<(long)pImuInfo->NSV1<<"\t"
	//	<<(long)pImuInfo->NSV2<<"\t"
	//	<<(long)pImuInfo->Status<<"\t"
	//	<<(long)pImuInfo->Type<<endl;

	//fImu.close();

	m_nInd++;

	return m_nInd;
}

long CLidarDataOpr::ReadLog(LIDAR_CLOUD_POINTS* pLidarPoints, CGpsManager::GPS_IMU_INFO* pImuInfo)
{
	char szPathCloudPt[MAX_PATH] = {0};
	char szPathImu[MAX_PATH] = {0};
	sprintf_s(szPathCloudPt,"%s%06d.lidar",m_szPathRead,m_nInd);
	sprintf_s(szPathImu,"%s%06d.imu",m_szPathRead,m_nInd);
	ifstream fCloudPt(szPathCloudPt);
 	if (!fCloudPt.is_open())
 	{
		return -1;
 	}
	ifstream fImu(szPathImu);
	if (!fImu.is_open())
	{
		return -1;
	}

	long nLongT;
	double dDoubleT;
	fCloudPt>>nLongT;
	pLidarPoints->nCont = nLongT;
	fCloudPt>>nLongT;
	pLidarPoints->nDevInd = nLongT;
	fCloudPt>>nLongT;
	pLidarPoints->nDevType = nLongT;
	fCloudPt>>nLongT;
	pLidarPoints->nFrameInd = nLongT;
	fCloudPt>>nLongT;
	pLidarPoints->nTimeStamp = nLongT;
	fCloudPt>>nLongT;
	pLidarPoints->points.resize(nLongT);
	for (int i = 0; i < pLidarPoints->points.size(); i++)
	{
		fCloudPt>>nLongT;
		pLidarPoints->points[i].layer = nLongT;
		fCloudPt>>nLongT;
		pLidarPoints->points[i].flag = nLongT;
		fCloudPt>>nLongT;
		pLidarPoints->points[i].echo = nLongT;
		fCloudPt>>pLidarPoints->points[i].angle;
		fCloudPt>>pLidarPoints->points[i].distance;
		fCloudPt>>pLidarPoints->points[i].epw;
	}
	fCloudPt.close();

	//fImu>>dDoubleT;
	//pImuInfo->GPSTime = dDoubleT;
	//fImu>>dDoubleT;
	//pImuInfo->Pitch = dDoubleT;
	//fImu>>dDoubleT;
	//pImuInfo->Roll = dDoubleT;
	//fImu>>dDoubleT;
	//pImuInfo->Lattitude = dDoubleT;
	//fImu>>dDoubleT;
	//pImuInfo->Longitude = dDoubleT;
	//fImu>>dDoubleT;
	//pImuInfo->Altitude = dDoubleT;
	//fImu>>dDoubleT;
	//pImuInfo->Ve = dDoubleT;
	//fImu>>dDoubleT;
	//pImuInfo->Vn = dDoubleT;
	//fImu>>dDoubleT;
	//pImuInfo->Vu = dDoubleT;
	//fImu>>dDoubleT;
	//pImuInfo->Baseline = dDoubleT;
	//fImu>>dDoubleT;
	//pImuInfo->NSV1 = dDoubleT;
	//fImu>>dDoubleT;
	//pImuInfo->NSV2 = dDoubleT;
	//fImu>>dDoubleT;
	//pImuInfo->Status = dDoubleT;
	//fImu>>dDoubleT;
	//pImuInfo->Type = dDoubleT;

	fImu>>pImuInfo->GPSTime;
		fImu>>pImuInfo->Heading;
		fImu>>pImuInfo->Pitch;
		fImu>>pImuInfo->Roll;
		fImu>>pImuInfo->Lattitude;
		fImu>>pImuInfo->Longitude;
		fImu>>pImuInfo->Altitude;
		fImu>>pImuInfo->Ve;
		fImu>>pImuInfo->Vn;
		fImu>>pImuInfo->Vu;
		fImu>>pImuInfo->Baseline;
		fImu>>pImuInfo->NSV1;
		fImu>>pImuInfo->NSV2;
		fImu>>nLongT;
		pImuInfo->Status=nLongT;
		fImu>>pImuInfo->Type;

	fImu.close();

	m_nInd++;

	return m_nInd;
}

void CLidarDataOpr::AddTimeTail(char* psz)
{
	time_t t = time(0); 
	char tmp[64]; 
	strftime( tmp, sizeof(tmp), "%Y%m%d%H%M%S",localtime(&t) );
	strcat(psz,tmp);
}

long CLidarDataOpr::LcmSendLog(LIDAR_CLOUD_POINTS* pLidarPoints, CGpsManager::GPS_IMU_INFO* pImuInfo)
{
	return 1;
}

void CLidarDataOpr::SetInd(long nInd)
{
	m_nInd = nInd;
}