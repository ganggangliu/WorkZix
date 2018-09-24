#include "ProjectOpr.h"
#include "LcmReceiver.h"

CLcmRevicer<LCM_SENSOR_FUSION_PACKAGE> PackSend(std::string("LCM_SENSOR_FUSION_PACKAGE"));

int main(int argc, char* argv[])
{
	CLidarDataOpr Lidar(argv[1]);

	int nSleepTime = 80;
	long nStartInd = 0;
	long nEndInd = LONG_MAX;
	if (argc>=3)
	{
		nSleepTime = atoi(argv[2]);
	}
	if (argc>=4)
	{
		nStartInd = atoi(argv[3]);
	}
	if (argc>=5)
	{
		nEndInd = atoi(argv[4]);
	}
	Lidar.SetInd(nStartInd);

	while (1)
	{
		Sleep(nSleepTime);
		LCM_SENSOR_FUSION_PACKAGE Pack;
		long nRt = Lidar.ReadLog(&Pack);
		if (nRt < 0)
		{
			Lidar.SetInd(nStartInd);
		}
		if (nRt > nEndInd)
		{
			Lidar.SetInd(nStartInd);
		}
		PackSend.Send(std::string("LCM_SENSOR_FUSION_PACKAGE"), Pack);
		cout << nRt << endl;
	}

	return 0;
}

