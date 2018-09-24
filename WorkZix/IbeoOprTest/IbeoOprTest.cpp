#include <Windows.h>
#include "IbeoOpr.h"

#include "LCM_SENSOR_FUSION_PACKAGE.hpp"
#include "LcmReceiver.h"

#if _DEBUG
#pragma comment(lib,"IbeoOprD.lib")
#else
#pragma comment(lib,"IbeoOpr.lib")
#endif

CLcmRevicer<LCM_SENSOR_FUSION_PACKAGE> g_PackSend(string("LCM_SENSOR_FUSION_PACKAGE"));
CLcmRevicer<LCM_IBEO_CLOUD_POINTS> g_CloudPointsSend(string("LCM_IBEO_CLOUD_POINTS"));
CLcmRevicer<LCM_IBEO_OBJECT_LIST> g_ObjectListSend(string("LCM_IBEO_OBJECT_LIST"));

long g_nPointCont = 0;
void __stdcall IbeoDataCallBackPt(void* pData, void* pUser)
{
	g_nPointCont++;
	LCM_IBEO_CLOUD_POINTS* pIbeoPt = (LCM_IBEO_CLOUD_POINTS*)pData;
	printf("Ibeo point cont:%d, %d\n", pIbeoPt->IbeoPoints.size(), g_nPointCont);
	g_CloudPointsSend.Send(string("LCM_IBEO_CLOUD_POINTS"),*pIbeoPt);
}

long g_nObjectCont = 0;
void __stdcall IbeoDataCallBackObj(void* pData, void* pUser)
{
	g_nObjectCont++;
	LCM_IBEO_OBJECT_LIST* pIbeoObj = (LCM_IBEO_OBJECT_LIST*)pData;
	printf("Ibeo Object cont:%d, %d\n", pIbeoObj->IbeoObjects.size(), g_nObjectCont);
	g_ObjectListSend.Send(string("LCM_IBEO_OBJECT_LIST"),*pIbeoObj);
// 	LCM_SENSOR_FUSION_PACKAGE Pack;
// 	Pack.IbeoObjList = *pIbeoObj;
// 	g_PackSend.Send(string("LCM_SENSOR_FUSION_PACKAGE"), Pack);
}

int main(int argc, char* argv[])
{
	hIbeoHandle pHandle = CreateIbeoHandle("192.168.0.100", ECU_CONFUTION_SYSTEM);
	pHandle->SetCallBackPoints(IbeoDataCallBackPt);
	pHandle->SetCallBackObjects(IbeoDataCallBackObj);
	pHandle->Start();

	Sleep(INFINITE);

	return 0;
}
