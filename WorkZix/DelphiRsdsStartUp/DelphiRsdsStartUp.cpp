#include <Windows.h>
#include "CanOpr.h"

#if _DEBUG
#pragma comment(lib,"CanOprD.lib")
#else
#pragma comment(lib,"CanOpr.lib")
#endif

unsigned char PCanBuffer0[8] = {0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char PCanBuffer1[8] = {0xf1,0x03,0x02,0x00,0x00,0x00,0x00,0x00};

unsigned char XCanBuffer0[8] = {0x00,0x08,0x00,0x00,0x3b,0x0a,0x8c,0x70};
unsigned char XCanBuffer1[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char XCanBuffer2[8] = {0x80,0x20,0x00,0xa7,0x50,0x80,0xdf,0xfe};
unsigned char XCanBuffer3[8] = {0x16,0x00,0x80,0x00,0x00,0x00,0x37,0xff};
unsigned char XCanBuffer4[8] = {0x01,0x61,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char XCanBuffer5[8] = {0x01,0x00,0x00,0x00,0x00,0x00,0x09,0xc6};
unsigned char XCanBuffer6[8] = {0x66,0x02,0x00,0x01,0x00,0x00,0x01,0x00};

CCanParam g_LeftPCan;
CCanParam g_LeftXCan;
CCanParam g_RightPCan;
CCanParam g_RightXCan;

bool GetParams(int argc, char* argv[]);

int main(int argc, char* argv[])
{
	if (!GetParams(argc, argv))
	{
		getchar();
		return 0;
	}

	CCanOpr CanOprLeftPCan;
	CanOprLeftPCan.Init(g_LeftPCan);
	CanOprLeftPCan.Start();
	CCanMsgData Msg;
	Msg.channelInd = 0;
	Msg.id = 0x762;
	memcpy(Msg.msg, PCanBuffer0, 8);
	CanOprLeftPCan.SendMsg(Msg);
	Sleep(1000);
	memcpy(Msg.msg, PCanBuffer1, 8);
	CanOprLeftPCan.SendMsg(Msg);
	Sleep(1000);
	CanOprLeftPCan.Stop();
	Sleep(10000);

	CCanOpr CanOprRightPCan;
	CanOprRightPCan.Init(g_RightPCan);
	CanOprRightPCan.Start();
	Msg.channelInd = 0;
	Msg.id = 0x764;
	memcpy(Msg.msg, PCanBuffer0, 8);
	CanOprRightPCan.SendMsg(Msg);
	Sleep(1000);
	memcpy(Msg.msg, PCanBuffer1, 8);
	CanOprRightPCan.SendMsg(Msg);
	Sleep(1000);
	CanOprRightPCan.Stop();
	Sleep(1000);

	CCanOpr CanOprLeftXCan;
	CanOprLeftXCan.Init(g_LeftXCan);
	CanOprLeftXCan.Start();
	Msg.channelInd = 0;
	for (unsigned int i = 0; i < 5; i++)
	{
		Msg.id = 0x20;
		memcpy(Msg.msg, XCanBuffer0, 8);
		CanOprLeftXCan.SendMsg(Msg);
		Sleep(100);
		Msg.id = 0x65;
		memcpy(Msg.msg, XCanBuffer1, 8);
		CanOprLeftXCan.SendMsg(Msg);
		Sleep(100);
		Msg.id = 0x100;
		memcpy(Msg.msg, XCanBuffer2, 8);
		CanOprLeftXCan.SendMsg(Msg);
		Sleep(100);
		Msg.id = 0x145;
		memcpy(Msg.msg, XCanBuffer3, 8);
		CanOprLeftXCan.SendMsg(Msg);
		Sleep(100);
		Msg.id = 0x400;
		memcpy(Msg.msg, XCanBuffer4, 8);
		CanOprLeftXCan.SendMsg(Msg);
		Sleep(100);
		Msg.id = 0x405;
		memcpy(Msg.msg, XCanBuffer5, 8);
		CanOprLeftXCan.SendMsg(Msg);
		Sleep(100);
		Msg.id = 0x500;
		memcpy(Msg.msg, XCanBuffer6, 8);
		CanOprLeftXCan.SendMsg(Msg);
		Sleep(100);

		printf("XCan send %d\n",i);
	}

	CanOprRightPCan.Stop();

	return 0;
}

bool GetParams(int argc, char* argv[])
{
	printf("6 Params needed!\n");
	printf("[Left PCan channel ind] [Left XCan channel ind] [Right PCan channel ind] [Right XCan channel ind]\n");
	if (argc < 5)
	{
		printf("Miss param\n");
		return false;
	}
	g_LeftPCan.nDevType = CAN_KVASER;
	g_LeftPCan.nDevInd = 0;
	g_LeftPCan.sChannel[0].nBaudRate = CAN_BAUDRATE_500K;
	g_LeftPCan.sChannel[0].nChannleInd = atoi(argv[1]);
	g_LeftXCan.nDevType = CAN_KVASER;
	g_LeftXCan.nDevInd = 0;
	g_LeftXCan.sChannel[0].nBaudRate = CAN_BAUDRATE_125K;
	g_LeftXCan.sChannel[0].nChannleInd = atoi(argv[2]);
	g_RightPCan.nDevType = CAN_KVASER;
	g_RightPCan.nDevInd = 0;
	g_RightPCan.sChannel[0].nBaudRate = CAN_BAUDRATE_500K;
	g_RightPCan.sChannel[0].nChannleInd = atoi(argv[3]);
	g_RightXCan.nDevType = CAN_KVASER;
	g_RightXCan.nDevInd = 0;
	g_RightXCan.sChannel[0].nBaudRate = CAN_BAUDRATE_125K;
	g_RightXCan.sChannel[0].nChannleInd = atoi(argv[4]);

	return true;
}