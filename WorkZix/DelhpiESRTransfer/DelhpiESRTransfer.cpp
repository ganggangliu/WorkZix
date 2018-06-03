#ifdef WIN32
    #include <windows.h>
#endif
#include <string>
#include <memory.h>

#include "lcm/lcm-cpp.hpp"
#include "LIDAR_RADAR_INFO.hpp"
#include "LcmReceiver.h"

#include "transplant.h"
#include "candrive.h"
#include "debugfunction.h"

#define MAX_SEND_OBJECTS		64

typedef struct ESR_RADAR_OBJECT_INFO_
{
	unsigned short	objectID;	//目标序号（0-63）
	bool	RollCount;			//RollCount标志位（0、1）
	float	Range;				//目标相对距离（m）
	float	RangeRate;			//目标相对速度（m/s）
	float	RangeAccel;			//目标相对加速度(m/s/s)
	float	Angle;				//目标相对角度(度，顺时针)
	float	Width;				//目标宽度（m）
	bool	GroupingChanged;	//目标物数量发生改变
	bool	OnComing;			//正在靠近
	float	LateralRate;		//侧向速度（m/s，逆时针）
	unsigned short	MedRangeMode;
	unsigned short	Status;		//0：无目标 1-7：各种目标
	bool	BridgeObject;		//
	ESR_RADAR_OBJECT_INFO_::ESR_RADAR_OBJECT_INFO_()
	{
		memset(this,0,sizeof(ESR_RADAR_OBJECT_INFO_));
	}
}ESR_RADAR_OBJECT_INFO;

typedef struct ESR_RADAR_INFO_
{
	enum { MAX_OBJECTS = 64 };
	ESR_RADAR_OBJECT_INFO ESRObjects[MAX_OBJECTS];
}ESR_RADAR_INFO;

long DataParse(ESR_RADAR_OBJECT_INFO& ObjInfo, unsigned char* buffer);
float BitToFloat(WORD Buff, long nLen, bool bIsSign, float fScale);
long TransData(ESR_RADAR_INFO& EsrRadarInfo, LIDAR_RADAR_INFO& RadarInfoSend);
bool IsInVector(std::vector<int>& Vec, int nIndex);

std::string g_szChannleName = "LIDAR_RADAR_INFO_ESR";
CLcmRevicer<LIDAR_RADAR_INFO> LcmESR(g_szChannleName);

int g_canDeviceType = 4;   // 设备类型号(USBCAN2)
int g_bitrateP = 500 * 1000; //canBITRATE_500K;   //PCAND波特率
int g_nDevPortP = 0;             //PCAN端口号
int g_nDeviceLabel = 0;         // PCAN端口号
int g_nCanDevInd = 0;


void PerformTestP(CanDrive& canChannel);
int InitCtrl_(CanDrive &canChannel, CAN_CONFIG& config, int nDevInd, int nCanInd);

// 返回值
// 0表示成功，1表示失败
int main(int argc, char* argv[])
{
	if (argc < 4)
	{
		printf("Params miss!\n"
            "Four Params needed:[Can Device index] [PCan channle] [Device index] [Channel name(option)]\n'");
		getchar();
        return 0;
	}
	else
	{
		g_nCanDevInd = atoi(argv[1]);
		g_nDevPortP = atoi(argv[2]);
		g_nDeviceLabel = atoi(argv[3]);
	}
	if (argc >= 5)
	{
		g_szChannleName = argv[4];
	}
    printf("[Can Device index] [PCan channle] [Device index] \n",
           g_nCanDevInd, g_nDevPortP, g_nDeviceLabel);

    g_bitrateP = 500 * 1000; // canBITRATE_500K;

    CanDrive canChannel;
	printf("Starting...\n");

    PerformTestP(canChannel);

	printf("\nThat's all for today!\n");
    return 1;
}

// Private Can 私有Can，雷达自己定义的数据借口
void PerformTestP(CanDrive& canChannel)
{
    int         iOpenFlag = 0;

    long        id;
//    canStatus   stat;
    BYTE        msg[8];
    int         i;
    CAN_CONFIG canConfig;
    canConfig.baud_rate = g_bitrateP;
	canConfig.acc_code = 0x00000000;
	canConfig.acc_mask = 0xFFFFFFFF;
	canConfig.acc_code = 0x00000000;
	canConfig.filter = 1;
	canConfig.mode = 0;

    iOpenFlag = InitCtrl_(canChannel, canConfig,  g_nCanDevInd, g_nDevPortP);
    if (iOpenFlag < 1)
    {
        return;
    }

//	stat = canSetAcceptanceFilter(handle1, 0x500, 0x53F, FALSE);

    printf("\n");

	id = 1280;
	
	unsigned char buffer[20] = {0};
	unsigned int nLen;
	unsigned int nFlag;
	unsigned long ntime;
	
	long nCont = 0;
	unsigned long nTime = 100;
	ESR_RADAR_INFO EsrRadarInfo;
	ESR_RADAR_OBJECT_INFO* pObjInfo = EsrRadarInfo.ESRObjects;
    CAN_OBJ data;
    bool bRet = false;

	while(1)
	{
		//获取结构体
        //stat = canReadWait(handle1, &id, buffer, &nLen, &nFlag, &ntime, 100);
        memset(&data, 0, sizeof(CAN_OBJ));
        bRet = canChannel.Receive(data, 1, 100);
		if (bRet == false)
		{
			Sleep(1);
			continue;
		}
		
//		printf("%d\n",data.id);

		id = (long)(data.id);
		memcpy(buffer,data.data,8);

		if (id < 1280 || id > 1343)
			continue;
		long nIndex = id - 1280;
		pObjInfo[nIndex].objectID = nIndex;
		DataParse(pObjInfo[nIndex],buffer);
		if (id != 1343)
			continue;

		//成功获取一帧数据后发送结构体
		LIDAR_RADAR_INFO RadarInfoSend;
		RadarInfoSend.nLidarRadarType = g_nDeviceLabel;
		//数据转换，过滤掉无效障碍物，返回有效障碍物个数
		long nContValid = TransData(EsrRadarInfo, RadarInfoSend);
		printf("%d\n",nContValid);
		double dRangeMin = 100000;
		int nRangeMinInd = -1;
		for (int i = 0; i < 64; i++)
		{
			if ((EsrRadarInfo.ESRObjects[i].Status == 3 ||
				EsrRadarInfo.ESRObjects[i].Status == 4) &&
				abs(EsrRadarInfo.ESRObjects[i].Angle) <= 5)
			{
				if (abs(EsrRadarInfo.ESRObjects[i].Range) < dRangeMin)
				{
					dRangeMin = abs(EsrRadarInfo.ESRObjects[i].Range);
					nRangeMinInd = i;
				}
			}
		}
		if (nRangeMinInd != -1)
		{
			printf("Id:%d, Range:%.2f, Angle:%.2f, RangeRate:%.2f, LatRate:%.2f, Mode:%d\n",
				EsrRadarInfo.ESRObjects[nRangeMinInd].objectID,
				EsrRadarInfo.ESRObjects[nRangeMinInd].Range,
				EsrRadarInfo.ESRObjects[nRangeMinInd].Angle,
				EsrRadarInfo.ESRObjects[nRangeMinInd].RangeRate,
				EsrRadarInfo.ESRObjects[nRangeMinInd].LateralRate,
				EsrRadarInfo.ESRObjects[nRangeMinInd].MedRangeMode);
		}
		else
		{
			printf("No data!\n");
		}

		LcmESR.Send(g_szChannleName, RadarInfoSend);

		//初始化结构体
		memset(&EsrRadarInfo,0,sizeof(ESR_RADAR_INFO));
	}

    //(void)canBusOff(handle1);
    canChannel.Stop();

//    (void)canBusOff(handle2);
    
    //canClose(handle1);
    canChannel.CloseDevice(g_canDeviceType, g_nDevPortP);
//    canClose(handle2);
}


long DataParse(ESR_RADAR_OBJECT_INFO& ObjInfo, unsigned char* buffer)
{
	WORD temp;
	UCHAR* p0 = (UCHAR*)(&temp);
	UCHAR* p1 = p0 + 1;

	memset(&temp,0,sizeof(WORD));
	memcpy(p0,&buffer[1],1);
	*p0 &= 0xe0;//11100000
	*p0 >>= 5;
	ObjInfo.Status = temp;

// 	if (ObjInfo.Status == 0)
// 	{
// 		return 1;
// 	}

	memset(&temp,0,sizeof(WORD));
	memcpy(p0,&buffer[4],1);
	*p0 &= 0x40;//0100 0000
	*p0 >>= 6;
	ObjInfo.RollCount = *p0;

	memset(&temp,0,sizeof(WORD));
	memcpy(p0,&buffer[0],1);
	*p0 &= 0xfc;//11111100
	*p0 >>= 2;
	//	ObjInfo.LateralRate = (float)temp * 0.25 - 8.0;
	ObjInfo.LateralRate = BitToFloat(temp,6,true,0.25);

	memset(&temp,0,sizeof(WORD));
	memcpy(p0,&buffer[0],1);
	*p0 &= 0x02;//00000010
	*p0 >>= 1;
	ObjInfo.GroupingChanged = *p0;

	memset(&temp,0,sizeof(WORD));
	memcpy(p0,&buffer[0],1);
	*p0 &= 0x01;//00000001
	//	*p0 >>= 1;
	ObjInfo.OnComing = *p0;

	memset(&temp,0,sizeof(WORD));
	memcpy(p1,&buffer[1],1);
	memcpy(p0,&buffer[2],1);
	temp &= 0x1ff8;//0001 1111 1111 1000
	temp >>= 3;
	//	ObjInfo.Angle = (float)temp * 0.1 - 51.2;
	ObjInfo.Angle = BitToFloat(temp,10,true,0.1);

	memset(&temp,0,sizeof(WORD));
	memcpy(p1,&buffer[2],1);
	memcpy(p0,&buffer[3],1);
	temp &= 0x07ff;//0000 0111 1111 1111
	//	temp >>= 3;
	//	ObjInfo.Range = (float)temp * 0.1;
	ObjInfo.Range = BitToFloat(temp,11,false,0.1);

	memset(&temp,0,sizeof(WORD));
	memcpy(p0,&buffer[4],1);
	*p0 &= 0x80;//1000 0000
	*p0 >>= 7;
	ObjInfo.BridgeObject = temp;

	memset(&temp,0,sizeof(WORD));
	memcpy(p0,&buffer[4],1);
	*p0 &= 0x3c;//0011 1100
	*p0 >>= 2;
	//	ObjInfo.Width = (float)temp * 0.5;
	ObjInfo.Width = BitToFloat(temp,4,false,0.5);

	memset(&temp,0,sizeof(WORD));
	memcpy(p1,&buffer[4],1);
	memcpy(p0,&buffer[5],1);
	temp &= 0x03ff;//0000 0011 1111 1111
	//	temp >>= 3;
	//	ObjInfo.RangeAccel = (float)temp * 0.05 - 25.6;
	ObjInfo.RangeAccel = BitToFloat(temp,10,true,0.05);

	memset(&temp,0,sizeof(WORD));
	memcpy(p0,&buffer[6],1);
	*p0 &= 0xc0;//1100 0000
	*p0 >>= 6;
	ObjInfo.MedRangeMode = temp;

	memset(&temp,0,sizeof(WORD));
	memcpy(p1,&buffer[6],1);
	memcpy(p0,&buffer[7],1);
	temp &= 0x3fff;//0011 1111 1111 1111
	//	temp >>= 3;
	//	ObjInfo.RangeRate = (float)temp * 0.01 - 81.92;
	ObjInfo.RangeRate = BitToFloat(temp,14,true,0.01);

	long sss = 0;

	return 1;
}

float BitToFloat(WORD Buff, long nLen, bool bIsSign, float fScale)
{
	float fOut;

	if (bIsSign)
	{
		bool bIsNeg = Buff & (0x0001<<(nLen-1));
		if (bIsNeg)
		{
			WORD valid = ~(0xffff<<(nLen-1));
			WORD Buff1 = (~Buff)&valid;
			Buff1++;
			fOut = (float)Buff1 * fScale * -1.0;
		}
		else
			fOut = (float)Buff * fScale;
	}
	else
	{
		fOut = (float)Buff * fScale;
	}



	return fOut;
}

long TransData(ESR_RADAR_INFO& EsrRadarInfo, LIDAR_RADAR_INFO& RadarInfoSend)
{
	long nCont = 0;
	ESR_RADAR_OBJECT_INFO* pObj = EsrRadarInfo.ESRObjects;
	LIDAR_RADAR_OBJECT_INFO* pObjSend = RadarInfoSend.Objects;
	for (int i=0; i < MAX_SEND_OBJECTS; i++)
	{
		pObjSend[i].nObjectID = i;
		//		if (pObj[i].Status == 0 || pObj[i].Status == 1 || pObj[i].Status == 6)
		if (pObj[i].Status != 3 && pObj[i].Status != 4)
		{
			pObjSend[i].bValid = false;
			continue;
		}
		else
		{
			pObjSend[i].bValid = true;
		}
		pObjSend[i].fObjLocX = pObj[i].Range*sin(pObj[i].Angle/180.f*3.141592654);
		pObjSend[i].fObjLocY = pObj[i].Range*cos(pObj[i].Angle/180.f*3.141592654);
		pObjSend[i].fObjSizeX = 0/*pObj[i].LateralRate*/;
		pObjSend[i].fObjSizeY = pObj[i].RangeRate;
		nCont++;
	}

	return nCont;
}

bool IsInVector(std::vector<int>& Vec, int nIndex)
{
	for (int i = 0; i < Vec.size(); i++)
	{
		if (nIndex == Vec[i])
		{
			return true;
		}
	}
	return false;
}


// 返回值 0 表示失败，1表示成功。
int InitCtrl_(CanDrive &canChannel, CAN_CONFIG& config, int nDevInd, int nCanInd)
{
	int iReturn = 1;

	//
	// First, open a handle to the CAN circuit. Specifying
	// canOPEN_EXCLUSIVE ensures we get a circuit that noone else
	// is using.
	//
	printf("canOpenChannel, channel %d... \n", nDevInd);
	// hnd = canOpenChannel(ctrl, canOPEN_EXCLUSIVE);
	bool bRet = canChannel.OpenDevice(g_canDeviceType, nDevInd);
	if (!bRet)
	{
		COM_ERR("%s, OpenDevice error\n", __FUNCTION__);
		//iReturn = 0;
		//goto ERR_InitCtrl;
	}
	printf("OK.\n");

	//
	// Using our new shiny handle, we specify the baud rate
	// using one of the convenient canBITRATE_xxx constants.

	printf("Setting the bus speed...");
	bRet = canChannel.Connect(g_canDeviceType, nDevInd, nCanInd, config);
	//stat = canSetBusParams(hnd, bitrate, 0, 0, 0, 0, 0);
	if (!bRet)
	{
		COM_ERR("%s, Connect CAN0 error\n", __FUNCTION__);
		iReturn = 0;
		goto ERR_InitCtrl;
	}
	printf("OK.\n");

	//
	// Then we start the ball rolling.
	// 
	printf("Go bus-on...");
	// 开启CAN0
	bRet = canChannel.Start();
	if (!bRet)
	{
		COM_ERR("%s, Start CAN0 error\n", __FUNCTION__);
		iReturn = 0;
		goto ERR_InitCtrl;
	}

	//    stat = canBusOn(hnd);
	//    if (stat < 0) {
	//        printf("canBusOn failed, stat=%d\n", stat);
	//    }
	printf("OK.\n");

ERR_InitCtrl:
	return iReturn;
}