#include "SickLms511DataTransfer.h"
#include "LcmReceiver.h"
#include "LCM_IBEO_CLOUD_POINTS.hpp"
#include "LIDAR_RADAR_INFO.hpp"
#include "LCM_IBEO_OBJECT_LIST.hpp"

int main(int argc, char* argv[])
{
	CSickLms511DataTransfer Opr(0,0);
	while(1)
	{
		vector<cv::Point2f> Points;
		LIDAR_RADAR_INFO Esr;
		Opr
	}

}