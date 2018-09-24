#include <Windows.h>
#include "OpenCVInc.h"
#include "ProjectOpr.h"
#include "LidarMappingOpr.h"

#include "LCM_IBEO_CLOUD_POINTS.hpp"
#include "LcmReceiver.h"

#if _DEBUG
#pragma comment(lib,"LidarMappingOprD.lib")
#else
#pragma comment(lib,"LidarMappingOpr.lib")
#endif

std::string g_szChannleName = "LCM_SENSOR_FUSION_PACKAGE";
CLcmRevicer<LCM_SENSOR_FUSION_PACKAGE> g_LcmPackData(g_szChannleName);

//long SendLidarCloudPtByLcm(Mat* pMat, CLidarMappingOpr::GPS_IMU_INFO* pGps);

CLidarMappingOpr* g_pOpr = NULL;

void WINAPI PackDataCallBack(void* pData)
{
//	system("cls");
	printf("+++++++++++++++++++++++++++++++++++++\n");
	CLidarMappingOpr::DATA_PACKAGE* pData_ = (CLidarMappingOpr::DATA_PACKAGE*)pData;

	printf("Long:%.7f, Lat:%.7f, Head:%.2f, VE:%.2f, VN:%.2f, State:%d\n",
		pData_->ImuData.Longitude,
		pData_->ImuData.Lattitude,
		pData_->ImuData.Heading,
		pData_->ImuData.Ve,
		pData_->ImuData.Vn,
		pData_->ImuData.Status);

	if (pData_->nPackType == 0)
	{
		if (pData_->LidarMapping.data == NULL)
		{
			return;
		}

		printf("\nLane cont: %d\n",pData_->Lanes.size());
		for (int i = 0; i < pData_->Lanes.size(); i++)
		{
			printf("Lane_%d Id:%d Type:%d\n",i,pData_->Lanes[i].nInd,pData_->Lanes[i].nType);
		}

		Mat nn = (pData_->LidarMapping) >= 1;
		Mat nn_;
		cvtColor(nn,nn_,COLOR_GRAY2RGB);
		g_pOpr->DrawCar(nn_,2400,840,900);
		g_pOpr->DrawNavi(nn_,*pData_);
		g_pOpr->DrawRadar(nn_,*pData_);

		Mat disp = Grey2FateColor(pData_->LidarMapping);
		cvNamedWindow("1234");
		imshow("1234",disp);
		cvNamedWindow("12345");
		imshow("12345",nn_);
		waitKey(1);

		return;
	}
	else if (pData_->nPackType == 1)
	{
		printf("Objects cont: %d\n", pData_->ObjectList.objects.size());
		Mat nn_;
		g_pOpr->DrawCar(nn_,2400,840,900);
		g_pOpr->DrawNavi(nn_,*pData_);
		g_pOpr->DrawRadar(nn_,*pData_);
		g_pOpr->DrawIbeoObjects(nn_,*pData_);
		int nNearestObjId = -1;
		double fNearestDist = 100000.0;
		for (int i = 0; i < pData_->ObjectList.objects.size(); i++)
		{
			if (pData_->ObjectList.objects[i].lane_loc != 0)
			{
				continue;
			}
			if (pData_->ObjectList.objects[i].ref_point.y <= 0.0)
			{
				continue;
			}
			double dDist = sqrt(pow(pData_->ObjectList.objects[i].obj_loc_x,2) 
				+ pow(pData_->ObjectList.objects[i].obj_loc_y,2));
			if (dDist < fNearestDist)
			{
				nNearestObjId = i;
				fNearestDist = dDist;
			}
		}
		if (nNearestObjId == -1)
		{
			printf("No obj in the main path\n");
		}
		else
		{
			CLidarMappingOpr::SENSOR_OBJECT& objT = pData_->ObjectList.objects[nNearestObjId];
			double dVelRel_m_s = objT.v_y / 1000.0/*sqrt(pow(objT.v_x,2) + pow(objT.v_y,2)) / 1000.0*/;
			double dVelRel_km_s = dVelRel_m_s * 3.6;
			double dAbsRel_m_s = sqrt(pow(objT.v_x_abs,2) + pow(objT.v_y_abs,2)) / 1000.0;
			double dAbsRel_km_s = dAbsRel_m_s * 3.6;
			double dEgoVel_m_s = sqrt(pow(pData_->ImuData.Ve,2) + pow(pData_->ImuData.Vn,2));
			double dEgoVel_km_s = dEgoVel_m_s * 3.6;
			char szDisp[256] = {0};
			printf("frame:%d, id:%02d, age:%02d, Class:%02d, Rel_Vel:%.2fm/s(%.2fkm/s), Abs_Vel:%.2fm/s(%.2fkm/s), Eog_Vel:%.2fm/s(%.2fkm/h)\n",
				pData_->ObjectList.frame_ind, objT.obj_id, objT.age, objT.classify,
				dVelRel_m_s, dVelRel_km_s,
				dAbsRel_m_s, dAbsRel_km_s,
				dEgoVel_m_s, dEgoVel_km_s);
			printf("Vn:%.2f, Ve:%.2f, Heading:%.2f\n",pData_->ImuData.Vn,pData_->ImuData.Ve,pData_->ImuData.Heading);
		}
		//Mat disp = Grey2FateColor(pData_->LidarMapping);
		//cvNamedWindow("1234");
		//imshow("1234",disp);
		cvNamedWindow("12345");
		imshow("12345",nn_);
		waitKey(1);
	}
	else
	{
		printf("Unknow Pack type!\n");
	}

	return;
	
}

int main(int argc, char* argv[])
{
	CLidarMappingParam Param;
	Param.nFrameCont = 5;
	Param.bIsMapping = true;
	Param.dCalibHeadingOffset = 0.0;
	Param.dCalibLongOffset = 3500;
	Param.nGroundLine = -1/*3*/;
	Param.nSkyLine = 10/*5*/;
	Param.nValidCont = 1;
	Param.LoadParam();
	CLidarMappingOpr LidarMappingOpr(Param);
	LidarMappingOpr.SetScale(80000,50000,50000,150);
	LidarMappingOpr.SetCallBack(PackDataCallBack);
	LidarMappingOpr.Start();
	g_pOpr = &LidarMappingOpr;
	Sleep(INFINITE);

	return 0;
}

