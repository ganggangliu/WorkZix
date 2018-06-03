#include <windows.h>
#include <canlib.h>
#include "LIDAR_RADAR_INFO.hpp"
#include "LcmReceiver.h"

#define MAX_SEND_OBJECTS		64

int g_bitrateP = canBITRATE_500K;   //PCAND波特率
int g_nDevPortP = 1;             //PCAN端口号
int g_bitrateV = canBITRATE_125K;   //VCAN波特率
int g_nDevPortV = 0;             // PCAN端口号

int g_nDevIndL = 10;             // 设备编号左侧
int g_nDevIndR = 11;             // 设备编号右侧

typedef struct RSDS_RADAR_OBJECT_INFO_
{
	int		counters;
	unsigned short	objectID;	//目标序号（0-63）
	int		f_valid_detection;	//检测合法标记位
	bool	detect_state;		//检测状态
	float	det_amplitude;		//信号强度
	float	det_angle;			//角度
	float	det_range;			//距离
	float	det_range_rate;		//加速度
	RSDS_RADAR_OBJECT_INFO_::RSDS_RADAR_OBJECT_INFO_()
	{
		memset(this,0,sizeof(RSDS_RADAR_OBJECT_INFO_));
	}
}RSDS_RADAR_OBJECT_INFO;

void PerformTestP();
void PerformTestV();
long DataParse(RSDS_RADAR_OBJECT_INFO& ObjInfo, unsigned char* buffer);
float BitToFloat(WORD Buff, long nLen, bool bIsSign, float fScale);
long TransData(RSDS_RADAR_OBJECT_INFO& ObjInfo, LIDAR_RADAR_OBJECT_INFO& Obj);

std::string szChannleName = "LIDAR_RADAR_INFO";
CLcmRevicer<LIDAR_RADAR_INFO> LcmRSDS(szChannleName);

void main(int argc, char* argv[])
{
	if (argc != 3)
	{
		printf("Params miss!\nFour Params needed:[PCan channle] [Device index]\n'");
		getchar();
	}
	else
	{
		g_nDevPortP = atoi(argv[1]);
		g_nDevIndL = atoi(argv[2]);
	}
	printf("[PCan channle %d]  [ Device index %d]\n",
		g_nDevPortP,g_nDevIndL);

	g_bitrateP = canBITRATE_500K;
//	g_bitrateV = canBITRATE_125K;

	printf("Starting...\n");

	canInitializeLibrary();

//	PerformTestV();
	PerformTestP();

	printf("\nThat's all for today!\n");

}


void Check(char* id, canStatus stat)
{
    char buf[50];
    if (stat != canOK) {
        buf[0] = '\0';
        canGetErrorText(stat, buf, sizeof(buf));
        printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
    }
}

int InitCtrl(int ctrl, long bitrate)
{
    int stat;
    int hnd;

    //
    // First, open a handle to the CAN circuit. Specifying
    // canOPEN_EXCLUSIVE ensures we get a circuit that noone else
    // is using.
    //
    printf("canOpenChannel, channel %d... ", ctrl);
    hnd = canOpenChannel(ctrl, canOPEN_EXCLUSIVE);
    if (hnd < 0) {
        Check("canOpenChannel", (canStatus)hnd);
        exit(1);
    }
    printf("OK.\n");

    //
    // Using our new shiny handle, we specify the baud rate
    // using one of the convenient canBITRATE_xxx constants.

    printf("Setting the bus speed...");
    stat = canSetBusParams(hnd, bitrate, 0, 0, 0, 0, 0);
    if (stat < 0) {
        printf("canSetBusParams failed, stat=%d\n", stat);
    }
    printf("OK.\n");

    //
    // Then we start the ball rolling.
    // 
    printf("Go bus-on...");
    stat = canBusOn(hnd);
    if (stat < 0) {
        printf("canBusOn failed, stat=%d\n", stat);
    }
    printf("OK.\n");

    // Return the handle; our caller will need it for
    // further exercising the CAN controller.
    return hnd;
}

void PerformTestP(void)
{
    int         handle1;

    long        id;
    canStatus   stat;
    BYTE        msg[8];
    int         i;

    handle1 = InitCtrl(g_nDevPortP,g_bitrateP);

//	stat = canSetAcceptanceFilter(handle1, 0x500, 0x53F, FALSE);

    printf("\n");

	id = 1024; //400h
	
	unsigned char buffer[20] = {0};
	unsigned int nLen;
	unsigned int nFlag;
	unsigned long ntime;
	
	long nCont = 0;
	unsigned long nTime = 100;
	LIDAR_RADAR_INFO RadarInfoL;
	LIDAR_RADAR_OBJECT_INFO* pObjInfoL = RadarInfoL.Objects;
	LIDAR_RADAR_INFO RadarInfoR;
	LIDAR_RADAR_OBJECT_INFO* pObjInfoR = RadarInfoR.Objects;

	while(1)
	{
		//获取结构体
		stat = canReadWait(handle1, &id, buffer, &nLen, &nFlag, &ntime, 100);
//		Check("canReadWait", stat);
//		printf("%d\n",id);
		if (stat != canOK)
		{
			printf("Reconnecting!...\n");
			canBusOff(handle1);
			canClose(handle1);
			canOpenChannel(g_nDevPortP, canOPEN_EXCLUSIVE);
			canBusOn(g_nDevPortP);
			continue;
		}

		if (id >= 1024 /*400h*/ && id <= 1043 /*413h*/)
		{
			long nIndex = id - 1024;
			pObjInfoL[nIndex].nObjectID = nIndex;
			RSDS_RADAR_OBJECT_INFO T;
			DataParse(T,buffer);
			TransData(T,pObjInfoL[nIndex]);
		}

		if (id != 1043)
			continue;
		//////////////////////////////////////////////////////////////////////////
		//测试代码，查看所有返回的标记
		/*
		std::vector<int> VecType(0);
		for (int i = 0; i < 64; i++)
		{
			if (!IsInVector(VecType,EsrRadarInfo.ESRObjects[i].Status) && EsrRadarInfo.ESRObjects[i].Status!=0&&EsrRadarInfo.ESRObjects[i].Status!=1)
			{
				VecType.push_back(EsrRadarInfo.ESRObjects[i].Status);
				printf("%d,",EsrRadarInfo.ESRObjects[i].Status);
			}
		}
		printf("\n");
		*/
		//////////////////////////////////////////////////////////////////////////
		RadarInfoL.nLidarRadarType = 10;
		LcmRSDS.Send(szChannleName,RadarInfoL);
// 		RadarInfoR.nLidarRadarType = 11;
// 		LcmRSDS.Send(szChannleName,RadarInfoR);
		
		long nObjContL = 0;
		long nObjContR = 0;
		for (int i = 0; i < 64; i++)
		{
			if (RadarInfoL.Objects[i].bValid == 1)
			{
				nObjContL++;
			}
		}
		for (int i = 0; i < 64; i++)
		{
			if (RadarInfoR.Objects[i].bValid == 1)
			{
				nObjContR++;
			}
		}
		printf("left:%d, right:%d\n",nObjContL,nObjContR);

		//初始化结构体
		memset(&RadarInfoL,0,sizeof(LIDAR_RADAR_INFO));
		memset(&RadarInfoR,0,sizeof(LIDAR_RADAR_INFO));
	}

    (void)canBusOff(handle1);
//    (void)canBusOff(handle2);
    
    canClose(handle1);
//    canClose(handle2);
}

void PerformTestV(void)
{
    int         handle1;

    long        id;
    canStatus   stat;
    BYTE        msg[8];
    int         i;

    handle1 = InitCtrl(g_nDevPortV,g_bitrateV);

	unsigned int nLen;
	unsigned long ntime;

	long nCont = 0;
	unsigned long nTime = 100;

	nLen = 8;

	id = 0x20; 
	unsigned char buffer0[8] = {0x00,0x08,0x00,0x00,0x3b,0x0a,0x8c,0x70}; 
	stat = canWrite(handle1, id, buffer0, nLen, canMSG_STD);
	id = 0x65; 
	unsigned char buffer1[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	stat = canWrite(handle1, id, buffer1, nLen, canMSG_STD);
	id = 0x100; 
	unsigned char buffer2[8] = {0x80,0x20,0x00,0xa7,0x50,0x80,0xdf,0xfe};
	stat = canWrite(handle1, id, buffer2, nLen, canMSG_STD);
	id = 0x145; 
	unsigned char buffer3[8] = {0x16,0x00,0x80,0x00,0x00,0x00,0x37,0xff}; 
	stat = canWrite(handle1, id, buffer3, nLen, canMSG_STD);
	id = 0x400; 
	unsigned char buffer4[8] = {0x01,0x61,0x00,0x00,0x00,0x00,0x00,0x00};
	stat = canWrite(handle1, id, buffer4, nLen, canMSG_STD);
	id = 0x405; 
	unsigned char buffer5[8] = {0x01,0x00,0x00,0x00,0x00,0x00,0x09,0xc6}; 
	stat = canWrite(handle1, id, buffer5, nLen, canMSG_STD);
	id = 0x500; 
	unsigned char buffer6[8] = {0x66,0x02,0x00,0x01,0x00,0x00,0x01,0x00}; 
	stat = canWrite(handle1, id, buffer6, nLen, canMSG_STD);

	stat = canWriteSync(handle1, 100);

    (void)canBusOff(handle1);
//    (void)canBusOff(handle2);
    
    canClose(handle1);
//    canClose(handle2);
}

long DataParse(RSDS_RADAR_OBJECT_INFO& ObjInfo, unsigned char* buffer)
{
	WORD temp;
	UCHAR* p0 = (UCHAR*)(&temp);
	UCHAR* p1 = p0 + 1;

	memset(&temp,0,sizeof(WORD));
	memcpy(p0,&buffer[0],1);
	ObjInfo.counters = temp;

	memset(&temp,0,sizeof(WORD));
	memcpy(p1,&buffer[1],1);
	memcpy(p0,&buffer[2],1);
	temp &= 0x7fff;//01111111 11111111
	ObjInfo.det_range = BitToFloat(temp,16,false,0.1);

	if (temp == 0x7FFF)
		ObjInfo.detect_state = false;
	else
		ObjInfo.detect_state = true;

	memset(&temp,0,sizeof(WORD));
	memcpy(p1,&buffer[3],1);
	memcpy(p0,&buffer[4],1);
	ObjInfo.det_range_rate = BitToFloat(temp,16,true,0.1);

	memset(&temp,0,sizeof(WORD));
	memcpy(p0,&buffer[5],1);
	memcpy(p1,&buffer[1],1);
	(*p1) >>= 7; 
	ObjInfo.det_angle = BitToFloat(temp,16,true,0.1);

	memset(&temp,0,sizeof(WORD));
	memcpy(p0,&buffer[7],1);
	ObjInfo.f_valid_detection = temp;

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

long TransData(RSDS_RADAR_OBJECT_INFO& ObjInfo, LIDAR_RADAR_OBJECT_INFO& Obj)
{
	if (ObjInfo.detect_state == true)
	{
		Obj.bValid = 1;
	}
	else
	{
		Obj.bValid = 0;
	}

	Obj.nObjectID = ObjInfo.objectID;
	Obj.fObjLocX =	ObjInfo.det_range*sin(ObjInfo.det_angle/180.f*3.141592654);
	Obj.fObjLocY =	ObjInfo.det_range*cos(ObjInfo.det_angle/180.f*3.141592654);
	Obj.fObjSizeX = 0;
	Obj.fObjSizeY = 0;


	return 1;
}