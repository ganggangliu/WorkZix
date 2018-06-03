#include <windows.h>

#include "CanBase.h"

class CVCIOpr:CCanBase
{
public:
	CVCIOpr();
	~CVCIOpr();

public:
	int Init(CCanParam& Param);
	void SetCallBack(lpCanReadDataCallBack hCallBack, void* pUser);
	int Start();
	int Stop();
	int SendMsg(CCanMsgData& Msg);
	int GetMsg(CCanMsgData& Msg);

private:
	static DWORD WINAPI ThreadFunc(LPVOID pParam);
	CCanParam m_Param;
	int* m_pnHanlde;
	lpCanReadDataCallBack m_pCallBack;
	void* m_pUser;
	HANDLE m_hThread;
};