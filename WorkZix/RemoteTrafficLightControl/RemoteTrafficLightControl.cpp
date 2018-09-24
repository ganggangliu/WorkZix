#include "LCM_TRAFIC_LIGHT_REQUIRE.hpp"
#include "LCM_TRAFIC_LIGHT_RESULT.hpp"
#include "LcmReceiver.h"

using namespace std;

CLcmRevicer<LCM_TRAFIC_LIGHT_REQUIRE> g_LcmTLRecReq(string("LCM_TRAFIC_LIGHT_REQUIRE"));
CLcmRevicer<LCM_TRAFIC_LIGHT_RESULT> g_LcmSend(string("LCM_TRAFIC_LIGHT_RESULT"));
CLcmRevicer<LCM_TRAFIC_LIGHT_RESULT> g_RemoteRec(string("REMOTE_CTL_CMD"), LCM_BINARY_TYPE);

typedef struct RemoteCtlCmd
{
	int cmd;
}REMOTE_CTL_CMD;

REMOTE_CTL_CMD g_RemoteStateBefor;
bool g_bStateChanged = false;
int g_nTotalState = -1;

void WINAPI TraffiLightRecCallBack(void* pData, void* pUser)
{
	LCM_TRAFIC_LIGHT_REQUIRE* pData_ = (LCM_TRAFIC_LIGHT_REQUIRE*)pData;

	printf("TraffiLightRecCallBack\n");

	LCM_TRAFIC_LIGHT_RESULT Res;
	Res.Req = *pData_;
	REMOTE_CTL_CMD RemoteCmd;
	RemoteCmd.cmd = 0;
	vector<char> RemoteData;
	int nRt = g_RemoteRec.GetDataEx(RemoteData);
	printf("g_RemoteRec.GetDataEx = %d\n", nRt);
	if (nRt < 0 || nRt >= 12)
	{
		RemoteCmd.cmd = g_RemoteStateBefor.cmd;
	}
	else
	{
		memcpy(&RemoteCmd, RemoteData.data(), sizeof(REMOTE_CTL_CMD));
	}
	if (RemoteCmd.cmd != g_RemoteStateBefor.cmd &&//检测信号变化
		RemoteCmd.cmd == 2)
	{
		g_bStateChanged = true;
	}
	else
	{
		g_bStateChanged = false;
	}
	if (g_bStateChanged == true)
	{
		g_nTotalState = 0;
	}
	if (g_nTotalState >= 0)
	{
		g_nTotalState++;
	}
	if (g_nTotalState >= 125)//20秒延时
	{
		g_nTotalState = -1;
	}
	if (g_nTotalState < 0)
	{
		Res.State = 2;
	}
	else
	{
		Res.State = 0;
	}
	printf("RemoteCmd:%d,g_bStateChanged:%d,g_nTotalState:%d,Res.State:%d\n",
		RemoteCmd.cmd,g_bStateChanged,g_nTotalState,Res.State);

	g_LcmSend.Send(string("LCM_TRAFIC_LIGHT_RESULT"), Res);

	g_RemoteStateBefor = RemoteCmd;
}

int main(int argc, char* argv[])
{
	g_RemoteStateBefor.cmd = 0;
	g_LcmTLRecReq.SetCallBack(TraffiLightRecCallBack, 0);
	g_LcmTLRecReq.Start();

	g_RemoteRec.Start();

	Sleep(INFINITE);

	return 0;
}

