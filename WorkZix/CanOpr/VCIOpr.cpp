#include "VCIOpr.h"
#include <fstream>
#include "ControlCAN.h"
using namespace std;


CVCIOpr::CVCIOpr()
{
	m_pnHanlde = NULL;
	m_pCallBack = NULL;
	m_pUser = 0;
	m_hThread = NULL;
	return;
}

CVCIOpr::~CVCIOpr()
{
	if (m_pnHanlde)
	{
		delete [] m_pnHanlde;
		m_pnHanlde = NULL;
	}
}

int CVCIOpr::Init(CCanParam& Param)
{
	m_Param = Param;
	m_pnHanlde = new int(m_Param.sChannel.size());
	return 1;
}

void CVCIOpr::SetCallBack(lpCanReadDataCallBack hCallBack, void* pUser)
{
	m_pCallBack = hCallBack;
	m_pUser = pUser;
}

DWORD WINAPI CVCIOpr::ThreadFunc(LPVOID pParam)
{
	CVCIOpr* pVCIOpr = (CVCIOpr*)pParam;

	DWORD stat = 0;
	CCanMsgData CanMsg;

	while(1)
	{
		bool bIsBusy = false;
		for (int i = 0; i < pVCIOpr->m_Param.sChannel.size(); i++)
		{
			CCanChannel& Channel = pVCIOpr->m_Param.sChannel[i];
			VCI_CAN_OBJ VciMsg;
			stat = VCI_Receive(4 , pVCIOpr->m_Param.nDevInd, Channel.nChannleInd, &VciMsg, 1, 100);
			if (stat > 0)
			{
				bIsBusy = true;
				CanMsg.channelInd = Channel.nChannleInd;
				CanMsg.id = VciMsg.ID;
				memcpy(CanMsg.msg, VciMsg.Data, sizeof(unsigned char)*8);
				(*(pVCIOpr->m_pCallBack))(&CanMsg, pVCIOpr->m_pUser);
			}
		}
		if (!bIsBusy)
		{
			Sleep(1);
		}
	
	}

}

int CVCIOpr::Start()
{
	DWORD nRt = VCI_OpenDevice(4, m_Param.nDevInd, 0);
	if (nRt != STATUS_OK)
	{
		printf("VCI_OpenDevice faild!\n");
		return 0;
	}

	for (int i = 0; i < m_Param.sChannel.size(); i++)
	{
		CCanChannel& channel = m_Param.sChannel[i];
		VCI_INIT_CONFIG conf;
		conf.AccCode = 0x00000000;
		conf.AccMask = 0xFFFFFFFF;
		conf.Reserved = 0;
		conf.Filter = 1;
		conf.Timing0 = 0x00;
		conf.Timing1 = 0x1C;
		conf.Mode = 0;
		if (channel.nBaudRate == CAN_BAUDRATE_125K)
		{
			conf.Timing0 = 0x03;
			conf.Timing1 = 0x1C;
		}
		nRt = VCI_InitCAN(4, m_Param.nDevInd, channel.nChannleInd, &conf);
		if (nRt != STATUS_OK)
		{
			printf("VCI_OpenDevice faild!\n");
			return 0;
		}
		nRt = VCI_StartCAN(4, m_Param.nDevInd, channel.nChannleInd);
		if (nRt != STATUS_OK)
		{
			printf("VCI_StartCAN faild!\n");
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

int CVCIOpr::Stop()
{
	DWORD nRt = 0;
	for (int i = 0; i < m_Param.sChannel.size(); i++)
	{
		nRt = VCI_ResetCAN(4, m_Param.nDevInd, m_Param.sChannel[i].nChannleInd);
		if (nRt != STATUS_OK)
		{
			printf("VCI_ResetCAN faild\n");
		}
	}

	VCI_CloseDevice(4, m_Param.nDevInd);
	Sleep(1000);

	TerminateThread(m_hThread, nRt);
	
	return 1;
}

int CVCIOpr::SendMsg(CCanMsgData& Msg)
{
	VCI_CAN_OBJ VciMsg;
	VciMsg.DataLen = 8;
	VciMsg.SendType = 0;
	VciMsg.RemoteFlag = 0;
	VciMsg.ExternFlag = 0;
	VciMsg.ID = Msg.id;
	memcpy(VciMsg.Data, Msg.msg, sizeof(unsigned char)*8);
	DWORD nRt = VCI_Transmit(4, m_Param.nDevInd, Msg.channelInd, &VciMsg, 1);
	if (nRt != STATUS_OK)
	{
		printf("VCI_Transmit\n");
		return 0;
	}

	return 1;
}

int CVCIOpr::GetMsg(CCanMsgData& Msg)
{
	if (m_pCallBack)
	{
		printf("Data call back mode, can't GetMsg\n");
		return 0;
	}
	VCI_CAN_OBJ VciMsg;
	DWORD stat = VCI_Receive(4 , m_Param.nDevInd, Msg.channelInd, &VciMsg, 1, 100);
	if (stat <= 0)
	{
		printf("canRead error\n");
		return 0;
	}
	Msg.id = VciMsg.ID;
	memcpy(Msg.msg, VciMsg.Data, sizeof(unsigned char)*8);

	return 1;
}