#include "CanOpr.h"
#include <vector>
#include <Windows.h>

using namespace std;

class CCanParseImpl
{
public:
	CCanParseImpl();
	~CCanParseImpl();

public:
	int Init(CCanParam& Param);
	void SetParsePattern(vector<CCanParseItem>& ParsePattern);
	void SetCallBack(lpCanReadDataCallBack hCallBack, void* pUser);
	int Start();
	int Stop();
	int SendMsg(CCanMsgData& Msg);
	//return < 0: no data; return == 1: new data; return > 1: old data
	int GetMsg(vector<CCanParseItem>& ParseList);

	friend void __stdcall CanDataCallBack4Parse(void* pData, void* pUser);

private:
	int FeedData(CCanMsgData& CanMsg);	//0: NOT complete   1:complete, new data got
	int CanSegmentParseIntel(CCanMsgData& Msg, CCanParseItem& Item);
	CCanOpr m_CanOpr;
	lpCanReadDataCallBack m_pCallBack;
	void* m_pUser;
	vector<CCanParseItem> m_ParsePatternConst;
	vector<CCanParseItem> m_ParsePattern;
	vector<CCanParseItem> m_ParseOut;
	CRITICAL_SECTION m_cs;
	int m_nGetRepeatCont;
	unsigned int m_nMatchInd;
};
