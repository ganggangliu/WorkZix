#include "CameraPointGreyImpl.h"
#include <boost/timer.hpp>
using namespace boost;
using namespace cv;
using namespace FlyCapture2;

//#pragma comment (lib, "FlyCapture2_v100.lib")

void OnImageGrabbed(Image* pImage, const void* pCallbackData)
{
	CCameraPointGreyImpl* pCam = (CCameraPointGreyImpl*)pCallbackData;
	if (!pCam->m_bRunning)
		return;

	if (pImage->GetRows() != pCam->m_Param.nHeight || pImage->GetCols() != pCam->m_Param.nWidth)
	{
		printf("Image size not compact!\n");
		return;
	}

	Image DestImage;
	pImage->Convert(PIXEL_FORMAT_BGR,&DestImage);
	pCam->m_mutex.lock();
	pCam->m_nCont++;
	pCam->m_nDataState = 0;
	if (DestImage.GetRows() != pCam->m_Data.rows || DestImage.GetCols() != pCam->m_Data.cols)
	{
		pCam->m_Data = Mat::zeros(DestImage.GetRows(), DestImage.GetCols(), CV_8UC3);
	}
	memcpy(pCam->m_Data.data, DestImage.GetData(), DestImage.GetDataSize());
// 	if (pImage->GetRows() != pCam->m_Data.rows || pImage->GetCols() != pCam->m_Data.cols)
// 	{
// 		pCam->m_Data = Mat::zeros(pImage->GetRows(), pImage->GetCols(), CV_8UC3);
// 	}
// 	memcpy(pCam->m_Data.data, pImage->GetData(), pImage->GetDataSize());
	pCam->m_mutex.unlock();
	if (!pCam->m_hCallBack.empty())
	{
		(pCam->m_hCallBack)(&(pCam->m_Data), pCam->m_pUser);
	}

	posix_time::ptime time_now = posix_time::microsec_clock::universal_time();
	pCam->m_FrameTimer.push_back(time_now);
	std::list<posix_time::ptime>::iterator Itor;
	for ( Itor = pCam->m_FrameTimer.begin(); Itor != pCam->m_FrameTimer.end(); )
	{
		posix_time::millisec_posix_time_system_config::time_duration_type time_elapse;
		posix_time::ptime time_now = posix_time::microsec_clock::universal_time();
		time_elapse = time_now - *Itor;
		int ticks = time_elapse.ticks();//us
		if ( ticks >= 1000000 - 10000 )
		{
			Itor = pCam->m_FrameTimer.erase(Itor);
		}
		else
		{
			Itor++;
		}
	}

//	std::cout << std::setprecision(4) << pCam->GetFrameRate() << ":" <<  pCam->m_nCont << std::endl;
//	printf("Frame Rate:%.2ffps  Shutter Time:%.2fms  Gain:%.2fdb  Exposure:%.2fEV\n",
//		pCam->GetFrameRate(),
//		pCam->GetShutter(),
//		pCam->GetGain(),
//		pCam->GetExposure());

	return;
}

CCameraPointGreyImpl::CCameraPointGreyImpl()
{
	m_pUser = 0;
	m_nDataState = -1;
	m_nDevInd = -1;
	m_bRunning = false;
	m_nCont = 0;
}

CCameraPointGreyImpl::~CCameraPointGreyImpl()
{

}

int CCameraPointGreyImpl::Init(CCameraParam& Param)
{
	m_Param = Param;
	return 1;
}

void CCameraPointGreyImpl::SetCallBack(FunctionCameraData hCallBack, void* pUser)
{
	m_hCallBack = hCallBack;
	m_pUser = pUser;
}

int CCameraPointGreyImpl::Start()
{
	BusManager busMgr;
	unsigned int numCameras;
	Error error = busMgr.GetNumOfCameras(&numCameras);
	if (numCameras <= 0)
	{
		printf("No PointGrey device!\n");
		return 0;
	}

	m_nDevInd = -1;
	CameraInfo camInfo;
	for (int i = 0; i < numCameras; i++)
	{
		PGRGuid guid;
		error = busMgr.GetCameraFromIndex(i, &guid);
		Camera cam;
		error = cam.Connect(&guid);
		CameraInfo camInfo;
		error = cam.GetCameraInfo(&camInfo);
		error = cam.Disconnect();
		if (IsMacMatch(&camInfo.macAddress, (char*)m_Param.szMac.c_str()))
		{
			m_nDevInd = i;
			m_GuId = guid;
		}
	}
	if (m_nDevInd < 0)
	{
		printf("MAC[%s] device not found!\n", m_Param.szMac);
		return 0;
	}

	error = m_cam.Connect(&m_GuId);
	if ( error != PGRERROR_OK )
	{
		printf("Camera connect failed!\n");
		return 0;
	}

	GigEImageSettings imageSettings;		//设置图像尺寸
	imageSettings.offsetX = 160;
	imageSettings.offsetY = 0;
	imageSettings.height = 1200;
	imageSettings.width = 1600;
	if (m_Param.nDataStreamType == ORIGINAL_DATA_STREAM)
		imageSettings.pixelFormat = PIXEL_FORMAT_RGB8;//PIXEL_FORMAT_RAW8 PIXEL_FORMAT_RGB8
	else
		imageSettings.pixelFormat = PIXEL_FORMAT_RAW8;//PIXEL_FORMAT_RAW8 PIXEL_FORMAT_RGB8
	error = m_cam.SetGigEImageSettings(&imageSettings);
	if ( error != PGRERROR_OK )
	{
		printf( "Camera param set failed!\n" );
		return 0;
	}

	TriggerMode TriMod;						//设置触发模式
	if (m_Param.bIsTrigger == false)
		TriMod.onOff = false; 
	else
		TriMod.onOff = true; 
	error = m_cam.SetTriggerMode(&TriMod);
	if ( error != PGRERROR_OK )
	{
		printf( "Camera TriMod set failed!\n" );
		return 0;
	}

	Property prop;
	prop.type = FRAME_RATE;					//设置帧率，固定帧率20fps
	m_cam.GetProperty(&prop);
	prop.autoManualMode = m_Param.bIsAutoFrameRate;
	prop.onOff = true;
	error = m_cam.SetProperty(&prop);
	if (error != PGRERROR_OK)
	{
		printf( "Camera FRAME_RATE(0) set failed!\n" );
		return 0;
	}
	prop.absValue = m_Param.nFrameRate;
	error = m_cam.SetProperty(&prop);
	if (error != PGRERROR_OK)
	{
		printf( "Camera FRAME_RATE(1) set failed!\n" );
		return 0;
	}

	prop.type = SHUTTER;					//设置快门时间
	m_cam.GetProperty(&prop);
	prop.autoManualMode = m_Param.bIsAutoShutter;
	error = m_cam.SetProperty(&prop);
	if (error != PGRERROR_OK)
	{
		printf( "Camera SHUTTER(0) set failed!\n" );
		return 0;
	}
	prop.absValue = (double)m_Param.nShutterTime/1000.0;
	error = m_cam.SetProperty(&prop);
	if (error != PGRERROR_OK)
	{
		printf( "Camera SHUTTER(1) set failed!\n" );
		return 0;
	}

	prop.type = WHITE_BALANCE;				//Set auto white-balance
	m_cam.GetProperty(&prop);
	prop.onOff = true;
	prop.autoManualMode = false;			//758 669
 	prop.valueA = 758;
 	prop.valueB = 669;
	error = m_cam.SetProperty(&prop);
	if (error != PGRERROR_OK)
	{
		printf( "Camera WHITE_BALANCE set failed!\n" );
		return 0;
	}

	prop.type = GAIN;
	m_cam.GetProperty(&prop);
	prop.onOff = true;
	prop.autoManualMode = m_Param.bIsAutoGain;
	error = m_cam.SetProperty(&prop);
	if (error != PGRERROR_OK)
	{
		printf( "Camera GAIN(0) set failed!\n" );
		return 0;
	}
	prop.absValue = 0.0;
	error = m_cam.SetProperty(&prop);
	if (error != PGRERROR_OK)
	{
		printf( "Camera GAIN(1) set failed!\n" );
		return 0;
	}

	error = m_cam.StartCapture(OnImageGrabbed,this);//设置回调函数
	if (error != PGRERROR_OK)
	{
		printf( "Camera callback set failed!\n" );
		return 0;
	}

	m_bRunning = true;
	
}

double CCameraPointGreyImpl::GetCallBackRate()
{
	return m_FrameTimer.size();
}

double CCameraPointGreyImpl::GetFrameRate()
{
//	return (double)m_FrameTimer.size();
	Property prop;
	prop.type = FRAME_RATE;
	m_cam.GetProperty(&prop);
	return prop.absValue;
}

double CCameraPointGreyImpl::GetShutter()
{
	Property prop;
	prop.type = SHUTTER;
	m_cam.GetProperty(&prop);
	return prop.absValue;
}

double CCameraPointGreyImpl::GetGain()
{
	Property prop;
	prop.type = GAIN;
	m_cam.GetProperty(&prop);
	return prop.absValue;
}

double CCameraPointGreyImpl::GetExposure()
{
	Property prop;
	prop.type = AUTO_EXPOSURE;
	m_cam.GetProperty(&prop);
	return prop.absValue;
}

int CCameraPointGreyImpl::GetData(Mat& img)
{
	m_mutex.lock();
	if (m_nDataState < 0)
	{
		m_mutex.unlock();
		return m_nDataState;
	}
	m_Data.copyTo(img);
	m_nDataState++;
	m_mutex.unlock();
	return m_nDataState;
}

int CCameraPointGreyImpl::Stop()
{
	Error error;
	error = m_cam.StopCapture();
	if (error != PGRERROR_OK)
	{
		printf("Stop capture failed!\n");
		return 0;
	}
	error = m_cam.Disconnect();
	{
		printf("Disconnect capture failed!\n");
		return 0;
	}

	return 1;
}

int CCameraPointGreyImpl::IsMacMatch(FlyCapture2::MACAddress* FlyMac, char* pszMac)
{
	char szFlyMac[128];
	sprintf(szFlyMac,"%02X:%02X:%02X:%02X:%02X:%02X",FlyMac->octets[0],
		FlyMac->octets[1],FlyMac->octets[2],FlyMac->octets[3],FlyMac->octets[4],FlyMac->octets[5]);
	int nRt = strcmp(szFlyMac,pszMac);

	return (nRt == 0);
}