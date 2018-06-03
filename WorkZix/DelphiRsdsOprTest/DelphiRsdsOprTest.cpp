#include <windows.h>
#include "DelphiRsdsOpr.h"

#if _DEBUG
#pragma comment(lib,"DelphiRsdsOprD.lib")
#else
#pragma comment(lib,"DelphiRsdsOpr.lib")
#endif

unsigned int g_nContL = 0;
void __stdcall RsdsDataCallBackL(LIDAR_RADAR_INFO* pData, void* pUser)
{
	g_nContL++;
	unsigned int nObjCont = 0;
	for (unsigned int i = 0; i < 64; i++)
	{
		if (pData->Objects[i].bValid)
			nObjCont++;
	}
	printf("L:%d:%d\n", g_nContL, nObjCont);
}

unsigned int g_nContR = 0;
void __stdcall RsdsDataCallBackR(LIDAR_RADAR_INFO* pData, void* pUser)
{
	g_nContR++;
	unsigned int nObjCont = 0;
	for (unsigned int i = 0; i < 64; i++)
	{
		if (pData->Objects[i].bValid)
			nObjCont++;
	}
	printf("R:%d:%d\n", g_nContR, nObjCont);
}

int main(int argc, char* argv[])
{
	CDelphiRsdsParam Param;
	Param.szIpLeft = "169.254.145.13";		//连接左雷达网线的本机ip
	Param.szIpRight = "169.254.145.14";		//连接右雷达网线的本机ip
	Param.XCanParam.nDevType = CAN_ZHOULIGONG;	
	Param.XCanParam.nDevInd = 0;
	Param.XCanParam.sChannel[0].nBaudRate = CAN_BAUDRATE_125K;
	Param.XCanParam.sChannel[0].nChannleInd = 1;	//rsds车身can连接的通道id
	Param.PCanParam.nDevType = CAN_ZHOULIGONG;
	Param.PCanParam.nDevInd = 0;
	Param.PCanParam.sChannel[0].nBaudRate = CAN_BAUDRATE_500K;
	Param.PCanParam.sChannel[0].nChannleInd = 0;	//rsds数据can连接的通道id
	CDelphiRsdsOpr Opr;
	Opr.Init(Param);
	Opr.SetCallBackL(RsdsDataCallBackL,0);
	Opr.SetCallBackR(RsdsDataCallBackR,0);

	Opr.Start();

	bool bIsRunning = true;
	unsigned int nTestInd = 0;
	while(1)
	{
		printf("+++++++++++++++++++++++++++++++++++++++++++\n");
		printf("Test id:%d\n", nTestInd++);
		int nKey = 10/*getchar()*/;
		Sleep(2000);
		if (nKey == 10)
		{
			bIsRunning = !bIsRunning;
			if (bIsRunning)
			{
				printf("User start\n");
				Opr.Start();
				printf("User start done\n");
			}
			else
			{
				printf("User stop\n");
				Opr.Stop();
				printf("User stop done\n");
			}
		}
	}

	Sleep(INFINITE);

	return 0;
}

