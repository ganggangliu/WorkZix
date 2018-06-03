#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory.h>

#include <canlib.h>

#include "CanBase.h"

class CKaserOpr:CCanBase
{
public:
	CKaserOpr();
	~CKaserOpr();

public:
	int Init(CCanParam& Param);
	void SetCallBack(lpCanReadDataCallBack hCallBack, void* pUser);
	int Start();
	int Stop();
	int SendMsg(CCanMsgData& Msg);
	int GetMsg(CCanMsgData& Msg);

private:
	static DWORD WINAPI ThreadFunc(LPVOID pParam);
	int InitCtrl(int ChannelId, CanBaudRate BaudRate);
	CCanParam m_Param;
	int* m_pnHanlde;
	lpCanReadDataCallBack m_pCallBack;
	void* m_pUser;
	HANDLE m_hThread;
};