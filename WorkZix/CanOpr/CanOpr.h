#ifndef CAN_OPR_H
#define CAN_OPR_H

#ifdef CANOPR_EXPORTS
#define CANOPR_API __declspec(dllexport)
#else
#define CANOPR_API __declspec(dllimport)
#endif

#include <vector>
using namespace std;

typedef void(__stdcall *lpCanReadDataCallBack)(void*, void*);

//////////////////////////////////////////////////////////////////////////
//CCanOpr
//Get data from CAN device without any processing
//////////////////////////////////////////////////////////////////////////
enum CanDeviceType
{
	CAN_KVASER,
	CAN_ZHOULIGONG
};

enum CanBaudRate
{
	CAN_BAUDRATE_125K,
	CAN_BAUDRATE_500K
};

typedef struct tag_CanChannel
{
	int nChannleInd;			//Channel index of the device
	CanBaudRate nBaudRate;		//Channel baudrate
	tag_CanChannel::tag_CanChannel()
	{
		nChannleInd = 0;
		nBaudRate = CAN_BAUDRATE_500K;
	};
}CCanChannel;

typedef struct tag_CanParam
{
	CanDeviceType nDevType;		//Device type
	int nDevInd;				//Device id in the OS
	vector<CCanChannel> sChannel;//Channels' info of the device
	tag_CanParam::tag_CanParam()
	{
		nDevType = CAN_KVASER;
		nDevInd = 0;
		sChannel.resize(1);
	};
}CCanParam;

typedef struct tag_CanMsgData
{
	int channelInd;				//The channel id of the msg
	long id;					//The head id of the msg
	unsigned char msg[8];		//msg
	tag_CanMsgData::tag_CanMsgData()
	{
		channelInd = 0;
		id = 0;
		memset(msg, 0, sizeof(unsigned char)*8);
	};
}CCanMsgData;

class CANOPR_API CCanOpr 
{
public:
	CCanOpr(void);
	~CCanOpr();

public:
	int Init(CCanParam& Param);
	//Once SetCallBack() called, GetMsg() will NOT work
	void SetCallBack(lpCanReadDataCallBack hCallBack, void* pUser);
	int Start();
	int Stop();
	int SendMsg(CCanMsgData& Msg);
	int GetMsg(CCanMsgData& Msg);

private:
	CCanParam m_Param;
	void* m_pCanOpr;
};

//////////////////////////////////////////////////////////////////////////
//CCanParseOpr
//Get data from CAN device and parse them by specific pattern
//////////////////////////////////////////////////////////////////////////
typedef struct tag_CanParseItem
{
	bool bIsProcessed;			//out
	int nValue;					//out
	float fValue;				//out
	char szName[256];			//in
	bool bIsIntelEncode;		//in
	long nMsgInd;				//in
	int nStartBit;				//in
	int nBitWidth;				//in
	bool bIsSigned;				//in
	int nSignedBit;				//in
	double dScale;				//in
	double dBias;				//in
	tag_CanParseItem()
	{
		bIsProcessed = false;
		memset(szName, 0, sizeof(char)*256);
		nValue = 0;
		fValue = 0.f;
		bIsIntelEncode = true;
		nMsgInd = 0;
		nStartBit = 0;
		nBitWidth = 0;
		bIsSigned = false;
		nSignedBit = 0;
		dScale = 1.0;
		dBias = 0.0;
	}
}CCanParseItem;

class CANOPR_API CCanParseOpr 
{
public:
	CCanParseOpr();
	~CCanParseOpr();

public:
	int Init(CCanParam& Param);
	void SetParsePattern(vector<CCanParseItem>& ParsePattern);
	void SetCallBack(lpCanReadDataCallBack hCallBack, void* pUser);
	int Start();
	int Stop();
	int SendMsg(CCanMsgData& Msg);
	//return < 0: no data; return == 1: new data; return > 1: old data
	int GetMsg(vector<CCanParseItem>& ParseList);

private:
	void* m_pCanParseOpr;
};

#endif
