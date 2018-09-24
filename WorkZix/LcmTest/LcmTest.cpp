#include <windows.h>
#include <vector>
#include "lcm/lcm-cpp.hpp"
#include "LCM_GPS_DATA.hpp"
#include <iostream>
#include <math.h>
using namespace std;

#pragma comment(lib,"ws2_32.lib")

bool ParseBinDataPos(LCM_GPS_DATA& gpsDataOut, char* pBinData)
{
// 	if (CRCheck((unsigned char*)(pBinData+2),61-2) == false)
// 		return false;

	double dTemp = 0.f;
	unsigned char cTemp = 0;
	long lTemp = 0;
	unsigned long ulTemp = 0;
	short sTemp = 0;
	unsigned short usTemp = 0;


	// 	memcpy(&dTemp,BinData.data+3,sizeof(double));
	// 	gpsDataOut.GPSTime = dTemp;

	memcpy(&usTemp,pBinData+4,sizeof(usTemp));
	unsigned short usWeek = usTemp;
	memcpy(&lTemp,pBinData+6,sizeof(lTemp));
	long nMiliSecend = lTemp;
	gpsDataOut.GPS_TIME = usWeek*7.0*24.0*60.0*60.0 + lTemp/1000.0;

	memcpy(&cTemp,pBinData+10,sizeof(cTemp));
	gpsDataOut.GPS_NSV1 = cTemp;
	//////////////////////////////////////////////////////////////////////////

	return true;

	memcpy(&lTemp,pBinData+11,sizeof(long));
	gpsDataOut.GPS_LATITUDE = (double)lTemp*1e-7;

	memcpy(&lTemp,pBinData+15,sizeof(long));
	gpsDataOut.GPS_LONGITUDE = (double)lTemp*1e-7;

	memcpy(&lTemp,pBinData+19,sizeof(long));
	gpsDataOut.GPS_ALTITUDE = (double)lTemp*1e-3;

	memcpy(&lTemp,pBinData+23,sizeof(long));
	gpsDataOut.GPS_VN = (double)lTemp*1e-3;

	memcpy(&lTemp,pBinData+27,sizeof(long));
	gpsDataOut.GPS_VE = (double)lTemp*1e-3;

	memcpy(&lTemp,pBinData+31,sizeof(long));
	gpsDataOut.GPS_VU = (double)lTemp*1e-3;

	memcpy(&lTemp,pBinData+35,sizeof(long));
	gpsDataOut.GPS_ROLL = (double)lTemp*1e-3;

	memcpy(&lTemp,pBinData+39,sizeof(long));
	gpsDataOut.GPS_PITCH = (double)lTemp*1e-3;

	memcpy(&ulTemp,pBinData+43,sizeof(unsigned long));
	gpsDataOut.GPS_HEADING = (double)ulTemp*1e-3;

	memcpy(&sTemp,pBinData+47,sizeof(short));
	double dAccn = (double)lTemp*1e-3;//北向加速度，m/s^2

	memcpy(&sTemp,pBinData+49,sizeof(short));
	double dAcce = (double)lTemp*1e-3;//东向加速度，m/s^2

	memcpy(&sTemp,pBinData+51,sizeof(short));
	double dAccu = (double)lTemp*1e-3;//地向加速度，m/s^2

	memcpy(&sTemp,pBinData+53,sizeof(short));
	double dAccRoll = (double)lTemp*1e-3;//横滚角速度，degree/s

	memcpy(&sTemp,pBinData+55,sizeof(short));
	double dAccePitch = (double)lTemp*1e-3;//俯仰角速度，degree/s

	memcpy(&sTemp,pBinData+57,sizeof(short));
	double dAccuHeading = (double)lTemp*1e-3;//航向角速度，degree/s

	memcpy(&cTemp,pBinData+59,sizeof(unsigned char));
	gpsDataOut.GPS_STATE = cTemp;

	return true;
}

double g_TimeBefor = 0.0;

class Handler 
{
public:
	~Handler() {}

	void handleMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan)
	{
		long nnn = 0;
		LCM_GPS_DATA gpsData;
		ParseBinDataPos(gpsData, (char*)(rbuf->data));
		long aaa = (long)((gpsData.GPS_TIME - g_TimeBefor + 0.0005)*1000);
		//printf("%d\n",aaa);
//		if ( aaa != 10)
		{
			printf("%02d   %.3f\n",aaa,gpsData.GPS_TIME);
		}
		
		g_TimeBefor = gpsData.GPS_TIME;
//		printf("%.3f\n",gpsData.GPS_TIME);
//		std::cout << (long)((gpsData.GPS_TIME - floor(gpsData.GPS_TIME))*1000) << endl;
	}
};

int main(int argc, char** argv)
{
	lcm::LCM lcm;

	if(!lcm.good())
		return 1;

	Handler handlerObject;
	lcm.subscribe("GPS_POSITION_FPS_100", &Handler::handleMessage, &handlerObject);

	while(0 == lcm.handle());

	return 0;
}
