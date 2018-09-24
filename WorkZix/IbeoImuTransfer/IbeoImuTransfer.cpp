#include "LcmReceiver.h"
#include "LCM_NAVI_OBJECT.hpp"
#include "CanOpr.h"
#include "GpsManager.h"


#if _DEBUG
#pragma comment(lib,"CanOprD.lib")
#else
#pragma comment(lib,"CanOpr.lib")
#endif

CCanOpr g_CanOpr;
CGpsManager g_PosData;
LCM_GPS_DATA g_ImuDataBefor;

int g_nCont = 0;
bool g_bIsFirstData = true;
void __stdcall ImuRecCallBack(void* pData, void* pUser)
{
	g_nCont++;
	printf("id:%d  ", g_nCont);
	LCM_GPS_DATA* pImuData_ = (LCM_GPS_DATA*)pData;
	if (g_bIsFirstData)
	{
		g_bIsFirstData = false;
		g_ImuDataBefor = *pImuData_;
		return;
	}

	float fVehicleVelocity = sqrt(pow(pImuData_->GPS_VE,2)+pow(pImuData_->GPS_VN,2));//m/s
	float fDeltaHeading = pImuData_->GPS_HEADING - g_ImuDataBefor.GPS_HEADING;//degree
	if (fDeltaHeading < -180.0)
	{
		fDeltaHeading += 360;
	}
	if (fDeltaHeading > 180.0)
	{
		fDeltaHeading -= 360;
	}
	float fVehicleYawRate = fDeltaHeading*100.0;//degree/s

	unsigned short usSpeed = 0;
	unsigned char  ucSpeedSign = 0;
	if (fVehicleVelocity >= 0.0f)
	{
		usSpeed = (unsigned short)(fVehicleVelocity * 3.6 * 100.0);//0.01km/h
		ucSpeedSign = 0;
	}
	else
	{
		usSpeed = (unsigned short)(-fVehicleVelocity * 3.6 * 100.0);
		ucSpeedSign = 1;
	}
	unsigned short usYawRate = 0;
	unsigned char  ucYawRateSign = 0;
	if (fVehicleYawRate >= 0.0f)
	{
		usYawRate = (unsigned short)(fVehicleYawRate * 100);//0.01degree/s
		ucYawRateSign = 1;
	}
	else
	{
		usYawRate = (unsigned short)(-fVehicleYawRate * 100);
		ucYawRateSign = 0;
	}
	// Êý¾Ý
	CCanMsgData CanData;
	CanData.channelInd = 0;
	CanData.id = 0x4f8;
	CanData.msg[0] = usSpeed & 0xFF;
	CanData.msg[1] = (usSpeed & 0xFF00) >> 8;
//	CanData.msg[1] |= (ucSpeedSign << 7);
	CanData.msg[2] = usYawRate & 0xFF;
	CanData.msg[3] = (usYawRate & 0x7F00) >> 8;
	CanData.msg[3] |= (ucYawRateSign << 7);
	CanData.msg[4] = 0x00;
	CanData.msg[5] = 0x00;
	CanData.msg[6] = 0x00;
	CanData.msg[7] = 0x00;

	int nRt = g_CanOpr.SendMsg(CanData);
	if (nRt > 0)
	{
		printf("Transfer data: Speed: %.2f m/s,%.2fkm/h  YawRate: %.2f degree/s\n", fVehicleVelocity, fVehicleVelocity*3.6, fVehicleYawRate);
	}
	else
	{
		printf("Data transfer error! data: Speed: %.2f m/s  YawRate: %.2f degree/s\n", fVehicleVelocity, fVehicleYawRate);
	}

	g_ImuDataBefor = *pImuData_;
}

int main(int argc, char* argv[])
{
	g_PosData.Init((CGpsManager::GPS_MANAGER_TYPE)(2));
	g_PosData.SetCallBack(ImuRecCallBack);
	g_PosData.Start();

	CCanParam Param;
	Param.nDevType = CAN_KVASER;
	Param.nDevInd = 0;
	Param.sChannel.resize(1);
	Param.sChannel[0].nChannleInd = 0;
	Param.sChannel[0].nBaudRate = CAN_BAUDRATE_500K;
	if (argc > 1)
	{
		Param.sChannel[0].nChannleInd = atoi(argv[1]);
	}
	g_CanOpr.Init(Param);
	int nRt = g_CanOpr.Start();
	if (nRt <= 0)
	{
		printf("Open can errer!\n");
	}

// 	IMU_INFORMATION data;
// 	data.v_x = 1.f;
// 	data.v_y = 2.f;
// 	data.yaw_rate = 3.f;
// 	vector<unsigned char> VecData;
// 	VecData.resize(sizeof(IMU_INFORMATION));
// 	memcpy(VecData.data(), &data, VecData.size());
// 	while(1)
// 	{
// 		ImuRecCallBack(&VecData,0);
// 	}

	Sleep(INFINITE);

	return 0;
}

