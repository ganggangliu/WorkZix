#include "TraficLightDetector.h"

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
	TraficLightDetectParam Param;
//	Param.WriteParam();
	Param.CamParam.nDataStreamType = COMPRESSED_DATA_STREAM;
	Param.CamParam.bIsAutoGain = false;
	Param.CamParam.bIsAutoFrameRate = false;
	Param.CamParam.nFrameRate = 10;
	Param.CamParam.bIsAutoShutter = false;
	Param.CamParam.nShutterTime = 15000;
//	Param.CamParam.szMac = "00:B0:9D:EE:A9:93";
	CTraficLightDetector TLDetecot;
	TLDetecot.Init(Param);

// 	TLDetecot.m_ReqRecieve.Start();
// 	while(1)
// 	{
// 		Mat aaa;
// 		TLDetecot.CameraDataCallBack(&aaa, &TLDetecot);
// 	}

	TLDetecot.Start();

	Sleep(INFINITE);

	return 1;
}

