#include "LidarDataOpr.h"
#include <fstream> 
#include <time.h>

CLidarDataOpr::CLidarDataOpr(char* pszPath)
{
	m_nInd = 0;
	strcpy_s(m_szPath,pszPath);
	strcpy_s(m_szPathRead,pszPath);
	m_pBuffer = new char[MAX_BUFFER_SIZE];
}
CLidarDataOpr::~CLidarDataOpr(void)
{
	if (m_pBuffer)
	{
		delete [] m_pBuffer;
		m_pBuffer = NULL;
	}
}

long CLidarDataOpr::CreateDir()
{
	AddTimeTail(m_szPath);
	strcat_s(m_szPath,"\\");
	CreateDirectoryA(m_szPath,NULL);

	return 1;
}

long CLidarDataOpr::WriteLog(LIDAR_CLOUD_POINTS* pLidarPoints, GPS_DATA* pImuInfo)
{
	char szPathCloudPt[MAX_PATH] = {0};
	char szPathImu[MAX_PATH] = {0};
	sprintf_s(szPathCloudPt,"%sCloudPoints%06d.bin",m_szPath,m_nInd);
	sprintf_s(szPathImu,"%sCloudPointsIMU%06d.bin",m_szPath,m_nInd);

	std::ofstream fCloudPt(szPathCloudPt, std::ios::binary);
	long nDataLen = pLidarPoints->getEncodedSize();
	pLidarPoints->encode(m_pBuffer, 0, nDataLen);
	fCloudPt.write(m_pBuffer,nDataLen);
	fCloudPt.close();

	std::ofstream fImu(szPathImu, std::ios::binary);
	nDataLen = pImuInfo->getEncodedSize();
	pImuInfo->encode(m_pBuffer, 0, nDataLen);
	fImu.write(m_pBuffer,nDataLen);
	fImu.close();

	m_nInd++;

	return m_nInd;
}

long CLidarDataOpr::WriteLog(IBEO_OBJECTS_LIST* pObjectList, GPS_DATA* pImuInfo)
{
	char szPathObjectList[MAX_PATH] = {0};
	char szPathImu[MAX_PATH] = {0};
	sprintf_s(szPathObjectList,"%sObjectList%06d.bin",m_szPath,m_nInd);
	sprintf_s(szPathImu,"%sObjectListIMU%06d.bin",m_szPath,m_nInd);

	std::ofstream fCloudPt(szPathObjectList, std::ios::binary);
	long nDataLen = pObjectList->getEncodedSize();
	pObjectList->encode(m_pBuffer, 0, nDataLen);
	fCloudPt.write(m_pBuffer,nDataLen);
	fCloudPt.close();

	std::ofstream fImu(szPathImu, std::ios::binary);
	nDataLen = pImuInfo->getEncodedSize();
	pImuInfo->encode(m_pBuffer, 0, nDataLen);
	fImu.write(m_pBuffer,nDataLen);
	fImu.close();

	m_nInd++;

	return m_nInd;
}

long CLidarDataOpr::ReadLog(LIDAR_CLOUD_POINTS* pLidarPoints, GPS_DATA* pImuInfo)
{
	char szPathCloudPt[MAX_PATH] = {0};
	char szPathImu[MAX_PATH] = {0};
	sprintf_s(szPathCloudPt,"%sCloudPoints%06d.bin",m_szPathRead,m_nInd);
	sprintf_s(szPathImu,"%sCloudPointsIMU%06d.bin",m_szPathRead,m_nInd);

	ifstream fCloudPt(szPathCloudPt, std::ios::binary);
	filebuf* pbuf = fCloudPt.rdbuf();
	long nDataLen = pbuf->pubseekoff(0,ios::end,ios::in);  
	pbuf->pubseekpos (0,ios::in);
	pbuf->sgetn(m_pBuffer, nDataLen);
	fCloudPt.close();
	pLidarPoints->decode(m_pBuffer, 0, nDataLen);

	ifstream fImu(szPathImu, std::ios::binary);
	pbuf = fImu.rdbuf();
	nDataLen = pbuf->pubseekoff(0,ios::end,ios::in);  
	pbuf->pubseekpos (0,ios::in);
	pbuf->sgetn(m_pBuffer, nDataLen);
	fImu.close();
	pImuInfo->decode(m_pBuffer, 0, nDataLen);

	m_nInd++;

	return m_nInd;
}

long CLidarDataOpr::ReadLog(IBEO_OBJECTS_LIST* pObjectList, GPS_DATA* pImuInfo)
{
	char szPathObjectList[MAX_PATH] = {0};
	char szPathImu[MAX_PATH] = {0};
	sprintf_s(szPathObjectList,"%sObjectList%06d.bin",m_szPathRead,m_nInd);
	sprintf_s(szPathImu,"%sObjectListIMU%06d.bin",m_szPathRead,m_nInd);

	ifstream fObjectList(szPathObjectList, std::ios::binary);
	filebuf* pbuf = fObjectList.rdbuf();
	long nDataLen = pbuf->pubseekoff(0,ios::end,ios::in);  
	pbuf->pubseekpos (0,ios::in);
	pbuf->sgetn(m_pBuffer, nDataLen);
	fObjectList.close();
	pObjectList->decode(m_pBuffer, 0, nDataLen);

	ifstream fImu(szPathImu, std::ios::binary);
	pbuf = fImu.rdbuf();
	nDataLen = pbuf->pubseekoff(0,ios::end,ios::in);  
	pbuf->pubseekpos (0,ios::in);
	pbuf->sgetn(m_pBuffer, nDataLen);
	fImu.close();
	pImuInfo->decode(m_pBuffer, 0, nDataLen);

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

long CLidarDataOpr::LcmSendLog(LIDAR_CLOUD_POINTS* pLidarPoints, GPS_DATA* pImuInfo)
{
	return 1;
}