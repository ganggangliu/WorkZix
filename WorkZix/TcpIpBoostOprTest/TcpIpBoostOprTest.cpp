#include <windows.h>
#include <stdio.h>
#include "TcpIpBoostOpr.h"

int g_nCont = 0;
void __stdcall TcpIpDataRecFunc(unsigned char* pData, int nLen, void* pUser)
{
	unsigned char* pBuff = (unsigned char*)pData;

//	printf("%d:%d\n", g_nCont++, nLen);
	for (int i = 0; i < 10; i++)
	{
		printf("%0x",(unsigned char)(pData[i]));
	}
	printf("\n");

}

int main(int argc, char* argv[])
{
	CClientBoostOpr ClientOpr;
	unsigned char szHead[2] = {0xAA, 0xCC};
	ClientOpr.SetCallBack(TcpIpDataRecFunc, 0);
	int nRt = ClientOpr.Start(8002, "169.254.145.71", "169.254.145.13");
// 	char szHead_[10000];
// 	unsigned int nCont = 0;
// 	while(1)
// 	{
// 		nRt = ClientOpr.ReceiveMsg(szHead_,10000,10);
// 		printf("%d:%d\n", nCont++, nRt);
// 	}
	Sleep(INFINITE);
	nRt = ClientOpr.ReceiveMsg(szHead,2);
	nRt = ClientOpr.ReceiveMsg(szHead,2);
	if (nRt)
	{
		printf("Open finish!\n");
	}
	else
	{
		printf("Open faild!\n");
	}

	bool bState = false;
	while (1)
	{
		int nRt = getchar();
		bState = !bState;
		if(bState)
		{
			int nRt = ClientOpr.Close();
			if (nRt)
			{
				printf("Close finish!\n");
			}
			else
			{
				printf("Close faild!\n");
			}
		}
		else
		{
			int nRt = ClientOpr.Start(55555, "192.168.1.10");
			if (nRt)
			{
				printf("Open finish!\n");
			}
			else
			{
				printf("Open faild!\n");
			}
		}
		
	}

	Sleep(INFINITE);

	return 0;
}

