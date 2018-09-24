#include "ProjectOpr.h"
#include "LcmReceiver4L.h"

string szChannelName("LCM_SENSOR_FUSION_PACKAGE");
CLcmReceive<LCM_SENSOR_FUSION_PACKAGE> PackSend(szChannelName);

int main(int argc, char* argv[])
{
    CLidarDataOpr Lidar(argv[1]);

    long nStartInd = 0;
    long nEndInd = LONG_MAX;
    if (argc>=3)
    {
        nStartInd = atoi(argv[2]);
    }
    if (argc>=4)
    {
        nEndInd = atoi(argv[3]);
    }
    Lidar.SetInd(nStartInd);

    while (1)
    {
        usleep(80000);
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
