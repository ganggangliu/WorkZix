#include "CameraOpr.h"
#include "CameraPointGreyImpl.h"

hCameraHandle CreateCameraHandle(CCameraParam& Param)
{
	hCameraHandle hCam = 0;
	if (Param.nCameraType == POINT_GREY_BFLY_PGE_23S6C_C)
	{
		hCam = new CCameraPointGreyImpl;
	}

	if (hCam)
	{
		hCam->Init(Param);
	}

	return hCam;
}

void ReleaseCameraHandle(hCameraHandle& hHandle)
{
	if (hHandle)
	{
		delete hHandle;
		hHandle = 0;
	}
}