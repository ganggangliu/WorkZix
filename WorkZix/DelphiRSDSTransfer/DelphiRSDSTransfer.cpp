#include <windows.h>
#include "LIDAR_RADAR_INFO.hpp"
#include "LcmReceiver.h"
#include "TcpIpOpr.h"


std::string g_szLocalIpL;
std::string g_szLocalIpR;
int g_nDataIndL = 0;
int g_nDataIndR = 0;

long TcpIpParse(unsigned char* psz, LIDAR_RADAR_INFO* pRsdsData);

std::string g_szChannleNameRL = "LIDAR_RADAR_INFO_DELPHI_RL";
std::string g_szChannleNameRR = "LIDAR_RADAR_INFO_DELPHI_RR";
CLcmRevicer<LIDAR_RADAR_INFO> LcmRSDS_RL(g_szChannleNameRL);
CLcmRevicer<LIDAR_RADAR_INFO> LcmRSDS_RR(g_szChannleNameRR);

long g_nContL = 0;
void WINAPI TcpIpDataRecFuncL(unsigned char* pData ,int nLen, void* pUser)
{
	g_nContL++;
	LIDAR_RADAR_INFO RsdsData;
	RsdsData.nLidarRadarType = g_nDataIndL;
	long nCont = TcpIpParse(pData, &RsdsData);
	printf("Left:%d  Ind:%d\n", nCont, g_nContL);
	LcmRSDS_RL.Send(g_szChannleNameRL,RsdsData);
}

long g_nContR = 0;
void WINAPI TcpIpDataRecFuncR(unsigned char* pData ,int nLen, void* pUser)
{
	g_nContR++;
	LIDAR_RADAR_INFO RsdsData;
	RsdsData.nLidarRadarType = g_nDataIndR;
	long nCont = TcpIpParse(pData, &RsdsData);
	printf("Right:%d Ind:%d\n", nCont, g_nContR);
	LcmRSDS_RR.Send(g_szChannleNameRR,RsdsData);
}

void main(int argc, char* argv[])
{
	if (argc != 5)
	{
		printf("4 Params needed! [Local IP left] [Local IP right] [left channel name] [right channel name] ");
		getchar();
		return;
	}
	else
	{
		g_szLocalIpL = std::string(argv[1]);
		g_szLocalIpR = std::string(argv[2]);
		g_szChannleNameRL = (argv[3]);
		g_szChannleNameRR = (argv[4]);
	}

	CClientNet ClientL;
	std::string szIpL = "169.254.145.71";
	long nPortL = 55555;
	printf("Connect Left Radar.......\n");
	int nRt = ClientL.Connect(nPortL,szIpL.c_str(),g_szLocalIpL.c_str());
	if (nRt != 0)
	{
		printf("Left RSDS Radar Tcp Data Open failed     IP:%s PORT:%d LocalIP:%s  Open failed!\n",szIpL.c_str(),nPortL,g_szLocalIpL.c_str());
	}
	else
	{
		unsigned char szHead[] = {0xAA, 0X55, 0x33, 0XCC, 0x00, 0XFF, 0x11, 0XEE, 0x22, 0XDD, 0x44, 0XBB, 0x01, 0X12, 0x45, 0X67};
		ClientL.SetCallBack(TcpIpDataRecFuncL,NULL,szHead,16,964-16,964);
		ClientL.Start();
	}
	

	CClientNet ClientR;
	std::string szIpR = "169.254.145.72";
	long nPortR = 55555;
	printf("Connect Right Radar.......\n");
	nRt = ClientR.Connect(nPortR,szIpR.c_str(),g_szLocalIpR.c_str());
	if (nRt != 0)
	{
		printf("Right RSDS Radar Tcp Data Open failed     IP:%s PORT:%d LocalIP:%s  Open failed!\n",szIpR.c_str(),nPortR,g_szLocalIpR.c_str());
	}
	else
	{
		unsigned char szHead[] = {0xAA, 0X55, 0x33, 0XCC, 0x00, 0XFF, 0x11, 0XEE, 0x22, 0XDD, 0x44, 0XBB, 0x01, 0X12, 0x45, 0X67};
		ClientR.SetCallBack(TcpIpDataRecFuncR,NULL,szHead,16,964,964/2);
		ClientR.Start();
	}

	Sleep(INFINITE);

}

long TcpIpParse(unsigned char* psz, LIDAR_RADAR_INFO* pRsdsData)
{
	int nValid = 0;
	for (int i = 0; i < 64; i++)
	{
		LIDAR_RADAR_OBJECT_INFO& ObjInfo = pRsdsData->Objects[i];
		long nStart = 68 + 14*i;

		short T;
		unsigned char Tc;
		memcpy(&T, psz + nStart + 0, sizeof(short));
		double dLonPos = (double)T / 128.f;
		memcpy(&T, psz + nStart + 2, sizeof(short));
		double dLonVel = (double)T / 128.f;
		memcpy(&T, psz + nStart + 4, sizeof(short));
		double dLonAcc = (double)T / 1024.f;
		memcpy(&T, psz + nStart + 6, sizeof(short));
		double dLatPos = (double)T / 128.f;
		memcpy(&T, psz + nStart + 8, sizeof(short));
		double dLatVel = (double)T / 128.f;
		memcpy(&T, psz + nStart + 10, sizeof(short));
		double dLatAcc = (double)T / 1024.f;
		memcpy(&Tc, psz + nStart + 12, sizeof(unsigned char));
		unsigned int Status = (unsigned int)Tc;
		memcpy(&Tc, psz + nStart + 13, sizeof(unsigned char));
		bool Stationary = (bool)Tc;

		ObjInfo.nObjectID = i;
		ObjInfo.fObjLocX = dLatPos;
		ObjInfo.fObjLocY = dLonPos;
		ObjInfo.fObjSizeX = 0.0/*dLatVel*/;
		ObjInfo.fObjSizeY = dLonVel;
		if (Status == 0 || Status == 1)
			ObjInfo.bValid = 0;
		else
		{
			ObjInfo.bValid = 1;
			nValid++;
		}

	}

//	printf("%d\n",nValid);

	return nValid;
}