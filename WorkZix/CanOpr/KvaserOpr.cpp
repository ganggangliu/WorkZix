#include "KvaserOpr.h"

#include <fstream>
using namespace std;

CKaserOpr::CKaserOpr()
{
	m_pnHanlde = NULL;
	m_pCallBack = NULL;
	m_pUser = 0;
	m_hThread = NULL;
	return;
}

CKaserOpr::~CKaserOpr()
{
	if (m_pnHanlde)
	{
		delete [] m_pnHanlde;
		m_pnHanlde = NULL;
	}
}

int CKaserOpr::Init(CCanParam& Param)
{
	m_Param = Param;
	m_pnHanlde = new int(m_Param.sChannel.size());
	return 1;
}

void CKaserOpr::SetCallBack(lpCanReadDataCallBack hCallBack, void* pUser)
{
	m_pCallBack = hCallBack;
	m_pUser = pUser;
}

DWORD WINAPI CKaserOpr::ThreadFunc(LPVOID pParam)
{
	CKaserOpr* pKvaserOpr = (CKaserOpr*)pParam;

	canStatus   stat;
	CCanMsgData CanMsg;

	while(1)
	{
		bool bIsBusy = false;
		for (int i = 0; i < pKvaserOpr->m_Param.sChannel.size(); i++)
		{
			int Handle = pKvaserOpr->m_pnHanlde[i];
			stat = canRead(Handle, &CanMsg.id, CanMsg.msg, NULL, NULL, NULL);
			if (stat == canOK)
			{
				bIsBusy = true;
				CanMsg.channelInd = Handle;
				(*(pKvaserOpr->m_pCallBack))(&CanMsg, pKvaserOpr->m_pUser);
			}
		}
		if (!bIsBusy)
		{
			Sleep(1);
		}
	
	}

}

int CKaserOpr::Start()
{
	canInitializeLibrary();

	for (int i = 0; i < m_Param.sChannel.size(); i++)
	{
		CCanChannel& channel = m_Param.sChannel[i];
		m_pnHanlde[i] = InitCtrl(channel.nChannleInd, channel.nBaudRate);
		if (m_pnHanlde[i] < 0)
		{
			printf("Kvaser channel %d open error!\n", i);
			return 0;
		}
	}

	if (m_pCallBack)
	{
		m_hThread = CreateThread(NULL,0,ThreadFunc,this,0,NULL);
		if (m_hThread == 0)
		{
			printf("CreateThread Failed \n");
			return 0;
		}
	}

	return 1;
}

int CKaserOpr::Stop()
{
	for (int i = 0; i < m_Param.sChannel.size(); i++)
	{
		(void)canBusOff(m_pnHanlde[i]);
		canClose(m_pnHanlde[i]);
	}
	
	DWORD nRt = 0;
	TerminateThread(m_hThread, nRt);

	return 1;
}

int CKaserOpr::InitCtrl(int ChannelId, CanBaudRate BaudRate)
{
	int stat;
	int hnd;

	printf("canOpenChannel, channel %d... ", ChannelId);
	hnd = canOpenChannel(ChannelId, canOPEN_EXCLUSIVE);
	if (hnd < 0) 
	{
		return hnd;
	}

	int bitrate = 0;
	if (BaudRate == CAN_BAUDRATE_125K)
	{
		bitrate = canBITRATE_125K;
	}
	else if (BaudRate == CAN_BAUDRATE_500K)
	{
		bitrate = canBITRATE_500K;
	}else
	{
		printf("Kvaser error: Unsupport baudrate!\n");
		return -1;
	}
	stat = canSetBusParams(hnd, bitrate, 0, 0, 0, 0, 0);
	if (stat < 0) 
	{
		printf("canSetBusParams failed, stat=%d\n", stat);
		return -1;
	}

	stat = canBusOn(hnd);
	if (stat < 0) 
	{
		printf("canBusOn failed, stat=%d\n", stat);
		return -1;
	}

	return hnd;
}

int CKaserOpr::SendMsg(CCanMsgData& Msg)
{
	int stat = canWrite(Msg.channelInd, Msg.id, &Msg.msg, 8, 0);
	if (stat != canOK)
	{
		return 0;
	}
	stat = canWriteSync(Msg.channelInd, 100);
	if (stat != canOK)
	{
		return 0;
	}
	return 1;
}

int CKaserOpr::GetMsg(CCanMsgData& Msg)
{
	if (m_pCallBack)
	{
		printf("Data call back mode, can't GetMsg\n");
		return 0;
	}
	canStatus stat = canRead(Msg.channelInd, &Msg.id, Msg.msg, NULL, NULL, NULL);
	if (stat != canOK)
	{
		printf("canRead error\n");
		return 0;
	}

	return 1;
}