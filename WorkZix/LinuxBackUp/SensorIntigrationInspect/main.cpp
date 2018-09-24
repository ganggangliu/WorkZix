#include "LcmReceiver4L.h"
#include "LCM_SENSOR_FUSION_PACKAGE.hpp"
#include "SensorDisp.h"

string g_szChannelName("LCM_SENSOR_FUSION_PACKAGE");
CLcmReceive<LCM_SENSOR_FUSION_PACKAGE> g_LcmSend(g_szChannelName);
CSensorDisp g_Disp;


void DataCallBackFunc(void* pData, void* pUser)
{
    LCM_SENSOR_FUSION_PACKAGE* pData_ = (LCM_SENSOR_FUSION_PACKAGE*)pData;
//    printf("Receive Pack data %d\n", pData_->NaviInfo.MsgInd);
//    g_Disp.ShowPackData(*pData_);
}

int main(int argc, char ** argv)
{
    g_LcmSend.SetCallBack(DataCallBackFunc);
    g_LcmSend.Start();

    while(1)
    {
        LCM_SENSOR_FUSION_PACKAGE PackData;
        int nRt = g_LcmSend.GetData(PackData);
        if(nRt != 1)
        {
            printf("Get data error!\n");
            usleep(1000);
        }
        g_Disp.ShowPackData(PackData);
        printf("Frame: %d\n", PackData.IbeoObjList.FrameInd);
    }


    sleep(INFINITY);

    return 0;
}

