#include "CanParseImpl.h"

void __stdcall CanDataCallBack4Parse(void* pData, void* pUser)
{
	CCanMsgData* pMsg = (CCanMsgData*)pData;
	//	printf("%ld\n",pMsg->id);
	CCanParseImpl* pCanParse = (CCanParseImpl*)pUser;
	pCanParse->FeedData(*pMsg);
}

CCanParseImpl::CCanParseImpl()
{
	m_pCallBack = NULL;
	m_pUser = NULL;
	InitializeCriticalSection(&m_cs);
	m_nGetRepeatCont = -1;
	m_nMatchInd = 0;
}

CCanParseImpl::~CCanParseImpl()
{
	DeleteCriticalSection(&m_cs);
}

void CCanParseImpl::SetParsePattern(vector<CCanParseItem>& ParsePattern)
{
	m_ParsePatternConst = ParsePattern;
	m_ParsePattern = ParsePattern;
}

int CCanParseImpl::Init(CCanParam& Param)
{
	m_nGetRepeatCont = -1;
	m_nMatchInd = 0;
	int nRt = m_CanOpr.Init(Param);
	m_CanOpr.SetCallBack(CanDataCallBack4Parse, this);
	return nRt;
}

void CCanParseImpl::SetCallBack(lpCanReadDataCallBack hCallBack, void* pUser)
{
	m_pCallBack = hCallBack;
	m_pUser = pUser;
}

int CCanParseImpl::Start()
{
	return m_CanOpr.Start();
}

int CCanParseImpl::Stop()
{
	return m_CanOpr.Stop();
}

int CCanParseImpl::SendMsg(CCanMsgData& Msg)
{
	return m_CanOpr.SendMsg(Msg);
}

int CCanParseImpl::GetMsg(vector<CCanParseItem>& ParseList)
{
	if (m_nGetRepeatCont < 0)
	{
		return m_nGetRepeatCont;
	}
	EnterCriticalSection(&m_cs);
	m_nGetRepeatCont++;
	ParseList = m_ParseOut;
	LeaveCriticalSection(&m_cs);

	return m_nGetRepeatCont;
}

int CCanParseImpl::FeedData(CCanMsgData& CanMsg)
{
	if (m_nMatchInd >= m_ParsePattern.size())
	{
		printf("m_nMatchInd >= m_ParsePattern.size()\n");
		return 0;
	}

	//如果Can.id超过了解析范围，同时解析尚未完成，则进入回调函数
	if ((CanMsg.id < m_ParsePattern.front().nMsgInd || 
		CanMsg.id > m_ParsePattern.back().nMsgInd) &&
		m_nMatchInd != 0)
	{
		EnterCriticalSection(&m_cs);
		m_ParseOut = m_ParsePattern;
		m_nGetRepeatCont = 0;
		LeaveCriticalSection(&m_cs);
		m_ParsePattern = m_ParsePatternConst;
		m_nMatchInd = 0;
		if (m_pCallBack)
		{
			(*m_pCallBack)(&m_ParseOut, m_pUser);
		}
		return 1;
	}

	while(CanMsg.id > m_ParsePattern[m_nMatchInd].nMsgInd && m_nMatchInd+1 < m_ParsePattern.size() && m_nMatchInd != 0)
	{
		m_nMatchInd++;
	}

	while(CanMsg.id == m_ParsePattern[m_nMatchInd].nMsgInd)
	{
		CanSegmentParseIntel(CanMsg, m_ParsePattern[m_nMatchInd]);
		m_nMatchInd++;
		if (m_nMatchInd >= m_ParsePattern.size())
		{
			m_nMatchInd = m_ParsePattern.size()-1;
			break;
		}
	}
	//如果解析列表已经到末尾，则解析完成，进入回调
	if (m_nMatchInd >= m_ParsePattern.size() - 1)
	{
		EnterCriticalSection(&m_cs);
		m_ParseOut = m_ParsePattern;
		m_nGetRepeatCont = 0;
		LeaveCriticalSection(&m_cs);
		m_ParsePattern = m_ParsePatternConst;
		m_nMatchInd = 0;
		if (m_pCallBack)
		{
			(*m_pCallBack)(&m_ParseOut, m_pUser);
		}
		return 1;
	}

	return 0;
}

int CCanParseImpl::CanSegmentParseIntel(CCanMsgData& Msg, CCanParseItem& Item)
{
	if (Msg.id != Item.nMsgInd)
		return 0;

	Item.bIsProcessed = true;

	int nStartByte = Item.nStartBit/8;
	int nStartBitInByte = Item.nStartBit%8;
	int nEndBit = Item.nStartBit + Item.nBitWidth - 1;
	int nEndByte = nEndBit/8;
	int nEndBitInByte_ = 7 - (nEndBit%8);

	if (!Item.bIsSigned)//unsigned case
	{
		unsigned short Buff = 0;
		memset(&Buff, 0, sizeof(unsigned short));
		unsigned char* pBuff0 = (unsigned char*)(&Buff);
		unsigned char* pBuff1 = pBuff0 + 1;
		*pBuff0 = Msg.msg[nStartByte];
		if (nStartByte != nEndByte)//multi byte case
		{
			*pBuff1 = Msg.msg[nEndByte];
			Buff <<= nEndBitInByte_;
			Buff >>= (nEndBitInByte_ + nStartBitInByte);
		}
		else						//single byte case
		{
			Buff <<= 8 + nEndBitInByte_;
			Buff >>= (8 + nEndBitInByte_ + nStartBitInByte);
		}
		Item.fValue = (double)Buff * Item.dScale + Item.dBias;
		Item.nValue = Item.fValue;
	}
	else//signed case
	{
		short Buff = 0;
		memset(&Buff, 0, sizeof(short));
		unsigned char* pBuff0 = (unsigned char*)(&Buff);
		unsigned char* pBuff1 = pBuff0 + 1; 

		*pBuff0 = Msg.msg[nStartByte];
		if (nStartByte != nEndByte)//multi byte case
		{
			*pBuff1 = Msg.msg[nEndByte];
			Buff <<= nEndBitInByte_;
			Buff >>= (nEndBitInByte_ + nStartBitInByte);
		}
		else						//single byte case
		{
			Buff <<= 8 + nEndBitInByte_;
			Buff >>= (8 + nEndBitInByte_ + nStartBitInByte);
		}

		int nSignedByte = Item.nSignedBit/8;
		int nSignedBit = Item.nSignedBit%8;
		unsigned char nT = 1;
		nT <<= nSignedBit;
		bool bIsNagative = ((Msg.msg[nSignedByte] & nT) != 0);
		if (bIsNagative)
		{
			Buff |= 0x1000000000000000;
		}
		Item.fValue = (double)Buff * Item.dScale + Item.dBias;
		Item.nValue = Item.fValue;
	}

	return 1;
}