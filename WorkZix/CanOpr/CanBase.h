#ifndef CAN_BASE_H
#define CAN_BASE_H

#include "CanOpr.h"

class CCanBase
{
public:
	virtual int Init(CCanParam& Param) = 0;
	virtual void SetCallBack(lpCanReadDataCallBack hCallBack, void* pUser) = 0;
	virtual int Start() = 0;
	virtual int Stop() = 0;
	virtual int SendMsg(CCanMsgData& Msg) = 0;
	virtual int GetMsg(CCanMsgData& Msg) = 0;
};


#endif