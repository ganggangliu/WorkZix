#include "gpscontroler.h"
#include <lcm/lcm-cpp.hpp>
#include "LcmReceiver.h"

long GetParams(int argc, char** argv, GPS_PORT& gpsPort);
void TransData(GPS_INFO& gpsInfo, LCM_GPS_DATA& GpsSend);
void RunGpsRec();
void RunIMURec();
void RunPOSRec();

std::string szURLLink = "udpm://239.255.76.67:7667?ttl=1";
std::string szChannleIMU = "LCM_GPFPD_BIN_DATA";
std::string szChannleGPS = "LCM_GPS_DATA";
std::string szChannlePOS = "GPS_POSITION_FPS_100";
lcm::LCM	m_lcm(szURLLink);
long nDataType = 0;
GPSControler PSControler;
long nCont = 0;

void WINAPI IMUDataCallBack(void* pData)
{
	LCM_GPFPD_BIN_DATA* pGpsData = (LCM_GPFPD_BIN_DATA*)pData;
	m_lcm.publish(szChannleIMU,pGpsData/*,sizeof(GPFPD_BIN_DATA)*/);

	nCont++;
	if (/*(long)(gpsInfo.GPSTime*1000.f)%1000 == 0*/nCont==100)
	{
		nCont = 0;
		LCM_GPS_DATA gpsInfo;
		PSControler.ParseBinData(gpsInfo,*pGpsData);
		char sz[256] = {0};
		sprintf(sz,"%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.2x\n",
			gpsInfo.GPS_TIME,
			gpsInfo.GPS_HEADING,
			gpsInfo.GPS_PITCH,
			gpsInfo.GPS_ROLL,
			gpsInfo.GPS_LATITUDE,
			gpsInfo.GPS_LONGITUDE,
			gpsInfo.GPS_ALTITUDE,
			gpsInfo.GPS_VE,
			gpsInfo.GPS_VN,
			gpsInfo.GPS_VU,
			gpsInfo.GPS_BASELINE,
			gpsInfo.GPS_NSV1,
			gpsInfo.GPS_NSV2,
			gpsInfo.GPS_STATE);
		printf(sz);
	}

	return;
}

void WINAPI IMUDataCallBackPos(void* pData)
{
	LCM_POS_BIN_DATA* pGpsData = (LCM_POS_BIN_DATA*)pData;
//	m_lcm.publish(szChannlePOS,pGpsData/*,sizeof(GPFPD_BIN_DATA)*/);
	m_lcm.publish(szChannlePOS,pGpsData->data,61);

	nCont++;
	if (/*(long)(gpsInfo.GPSTime*1000.f)%1000 == 0*/nCont==100)
	{
		nCont = 0;
		LCM_GPS_DATA gpsInfo;
		PSControler.ParseBinDataPos(gpsInfo,*pGpsData);
		printf("\n%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.2x\n",
			gpsInfo.GPS_TIME,
			gpsInfo.GPS_HEADING,
			gpsInfo.GPS_PITCH,
			gpsInfo.GPS_ROLL,
			gpsInfo.GPS_LATITUDE,
			gpsInfo.GPS_LONGITUDE,
			gpsInfo.GPS_ALTITUDE,
			gpsInfo.GPS_VE,
			gpsInfo.GPS_VN,
			gpsInfo.GPS_VU,
			gpsInfo.GPS_BASELINE,
			gpsInfo.GPS_NSV1,
			gpsInfo.GPS_NSV2,
			gpsInfo.GPS_STATE);

	}

	return;
}

int main(int argc, char* argv[])
{
	if (!GetParams(argc,argv,PSControler.m_gpsPortParam))
	{
		printf("Com params error!\n");
		getchar();
		return 0;
	}

	if (!PSControler.openGPSPort(PSControler.m_gpsPortParam))
	{
		printf("Open Com Failed!\n");
		getchar();
		return 0;
	}
	PSControler.enableRecvGps();

	if (nDataType == 0)		//接受GPS数据
	{
		RunGpsRec();
	}
	else if(nDataType == 1)	//接受IMU数据
	{
		RunIMURec();
	}
	else					//接受POS数据
	{
		RunPOSRec();
	}

	return 0;
}

long GetParams(int argc, char** argv, GPS_PORT& gpsPort)
{
	gpsPort.parity = NOPARITY;
	gpsPort.byteSize = 8;
	gpsPort.stopBits = ONESTOPBIT;
	gpsPort.baudRate = 0;
	memset(gpsPort.portName,0,sizeof(gpsPort.portName));
	for (int i = 0; i < argc; i++)
	{
		char* psz = NULL;
		psz = strstr(argv[i],"-p");
		if (psz == NULL)
		{
			psz = strstr(argv[i],"-P");
		}
		if (psz != NULL)
		{
			psz+=2;
			int nTemp = atoi(psz);
			sprintf(gpsPort.portName,"%s%d","COM",nTemp);
		}

		psz = NULL;
		psz = strstr(argv[i],"-b");
		if (psz == NULL)
		{
			psz = strstr(argv[i],"-B");
		}
		if (psz != NULL)
		{
			psz+=2;
			gpsPort.baudRate = atoi(psz);
		}

		psz = NULL;
		psz = strstr(argv[i],"-d");
		if (psz == NULL)
		{
			psz = strstr(argv[i],"-D");
		}
		if (psz != NULL)
		{
			psz+=2;
			nDataType = atoi(psz);
		}
	}

	printf("Port:%s\n",gpsPort.portName);
	printf("BaudRate:%d\n",gpsPort.baudRate);
	printf("DataType:%d\n",nDataType);

	if (gpsPort.baudRate == 0 || strcmp(gpsPort.portName,"") == 0)
	{
		return 0;
	}

	return 1;
}

void TransData(GPS_INFO& gpsInfo, LCM_GPS_DATA& GpsSend)
{
	GpsSend.GPS_TIME       = gpsInfo.GPSTime;   // 时间
	GpsSend.GPS_HEADING    = gpsInfo.Heading; // 方向角
	GpsSend.GPS_LATITUDE   = gpsInfo.Lattitude; // 纬度
	GpsSend.GPS_LONGITUDE  = gpsInfo.Longitude; // 经度
	GpsSend.GPS_ALTITUDE   = gpsInfo.Altitude;  // 高度
	GpsSend.GPS_VE         = gpsInfo.Ve;  // 东向速度
	GpsSend.GPS_VN         = gpsInfo.Vn;  // 北向速度
	GpsSend.GPS_VU         = gpsInfo.Vu;  // 天向速度
	GpsSend.GPS_BASELINE   = gpsInfo.Baseline; // 基线长度
	GpsSend.GPS_NSV1       = gpsInfo.NSV1;   // 主天线星数
	GpsSend.GPS_NSV2       = gpsInfo.NSV2;   // 辅天线星数

	if (gpsInfo.Status == 'B')
		GpsSend.GPS_STATE = 4.0f;   // GPS差分状态标识： 4 代表最好的差分
	else
		GpsSend.GPS_STATE = 1.0f;

}

void RunGpsRec()
{
	char serialInfo[2048] = { 0 };
	GPS_INFO gpsInfo;
	bool bRet = false;
	while (1) 
	{
		memset(serialInfo, 0, sizeof(serialInfo));
		bRet = PSControler.recvGPSInfo(gpsInfo, serialInfo, 2047);
		if (true == bRet) 
		{
			LCM_GPS_DATA GpsSend;
			TransData(gpsInfo,GpsSend);
			m_lcm.publish(szChannleGPS,&GpsSend);

			printf("\n");
			printf(serialInfo);
		}
	}
}

void RunIMURec()
{
	PSControler.SetCallBack(IMUDataCallBack);
	PSControler.BeginRecvBinData();
	Sleep(INFINITE);
}

void RunPOSRec()
{
	PSControler.SetCallBack(IMUDataCallBackPos);
	PSControler.BeginRecvBinDataPos();
	Sleep(INFINITE);
}