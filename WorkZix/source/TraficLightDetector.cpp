#include <boost/bind.hpp>
#include "TraficLightDetector.h"
#include "LCM_NAVI_REQUIRE_INFO.hpp"

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/typeof/typeof.hpp>
using namespace std;  
using namespace boost::property_tree;

using namespace cv;
using namespace boost;

int TraficLightDetectParam::LoadParam()
{
	char szFilePath[MAX_PATH] = {0};
	GetModuleFileNameA(NULL,szFilePath,MAX_PATH);
	char* p = strrchr(szFilePath,'\\');
	*p = 0x00;
	strcat(szFilePath,"\\TraficLightDetect.xml");

	if (boost::filesystem::exists(szFilePath))
	{
		ptree pt;
		read_xml(szFilePath, pt);
 		dFocusLen= pt.get<double>("TraficLightDetectParam.dFocusLen", 4310.0);
		dPitch= pt.get<double>("TraficLightDetectParam.dPitch", 0.0);
		dRoll= pt.get<double>("TraficLightDetectParam.dRoll", 0.0);
		dYaw= pt.get<double>("TraficLightDetectParam.dYaw", 0.0);
		dLateral= pt.get<double>("TraficLightDetectParam.dLateral", 0.0);
		dLongitude= pt.get<double>("TraficLightDetectParam.dLongitude", 0.0);
		dHeight= pt.get<double>("TraficLightDetectParam.dHeight", 0.0);
		dRectSizeU= pt.get<double>("TraficLightDetectParam.dRectSizeU", 2.0);
		dRectSizeV= pt.get<double>("TraficLightDetectParam.dRectSizeV", 4.0);
		dDetRangeMin= pt.get<double>("TraficLightDetectParam.dDetRangeMin", 10.0);
		dDetRangeMax= pt.get<double>("TraficLightDetectParam.dDetRangeMax", 100.0);
		CamParam.szMac= pt.get<string>("TraficLightDetectParam.szMac");
		dTrafficLightSizeMin = pt.get<double>("TraficLightDetectParam.dTrafficLightSizeMin", 0.1);
		dTrafficLightSizeMax = pt.get<double>("TraficLightDetectParam.dTrafficLightSizeMax", 0.5);
		bIsSaveImg = pt.get<bool>("TraficLightDetectParam.bIsSaveImg", false);

		return 1;
	}
	else
	{
		printf("%s NOT exists!\n", szFilePath);

		return 0;
	}

	return 0;
}

int TraficLightDetectParam::WriteParam()
{
	char szFilePath[MAX_PATH] = {0};
	GetModuleFileNameA(NULL,szFilePath,MAX_PATH);
	char* p = strrchr(szFilePath,'\\');
	*p = 0x00;
	strcat(szFilePath,"\\TraficLightDetect.xml");

	ptree pt;
	pt.put<double>("TraficLightDetectParam.dFocusLen", dFocusLen);
	pt.put<double>("TraficLightDetectParam.dPitch", dPitch);
	pt.put<double>("TraficLightDetectParam.dRoll", dRoll);
	pt.put<double>("TraficLightDetectParam.dYaw", dYaw);
	pt.put<double>("TraficLightDetectParam.dLateral", dLateral);
	pt.put<double>("TraficLightDetectParam.dLongitude", dLongitude);
	pt.put<double>("TraficLightDetectParam.dHeight", dHeight);
	pt.put<double>("TraficLightDetectParam.dRectSizeU", dRectSizeU);
	pt.put<double>("TraficLightDetectParam.dRectSizeV", dRectSizeV);
	pt.put<double>("TraficLightDetectParam.dDetRangeMin", dDetRangeMin);
	pt.put<double>("TraficLightDetectParam.dDetRangeMax", dDetRangeMax);
	pt.put<string>("TraficLightDetectParam.szMac", CamParam.szMac);
	pt.put<double>("TraficLightDetectParam.dTrafficLightSizeMin",dTrafficLightSizeMin);
	pt.put<double>("TraficLightDetectParam.dTrafficLightSizeMax",dTrafficLightSizeMax);
	pt.put<bool>("TraficLightDetectParam.bIsSaveImg", bIsSaveImg);

	write_xml(szFilePath, pt);

	return 1;
}

unsigned int nCont = 0;
void CTraficLightDetector::CameraDataCallBack(Mat* pImg, void* pUser)
{
	CTraficLightDetector* pTLD = (CTraficLightDetector*)pUser;
/*
//	*pImg = imread("C:\\Users\\Sense\\Desktop\\15640979-2017-04-08-150202.bmp");
//	*pImg = imread("C:\\Users\\Sense\\Desktop\\15640979-2017-04-11-173128.bmp");
//	*pImg = imread("C:\\Users\\Sense\\Desktop\\15640979-2017-04-11-164216.bmp");
//	*pImg = imread("C:\\Users\\Sense\\Desktop\\15640979-2017-04-11-164948.bmp");
//	*pImg = imread("C:\\Users\\Sense\\Desktop\\15640979-2017-04-11-173316.bmp");
//	*pImg = imread("C:\\Users\\Sense\\Desktop\\15640979-2017-04-11-173431.bmp");
//	*pImg = imread("C:\\Users\\Sense\\Desktop\\15640979-2017-04-11-174027.bmp");
//	*pImg = imread("C:\\Users\\Sense\\Desktop\\15640979-2017-04-11-175101.bmp");
//	*pImg = imread("C:\\Users\\Sense\\Desktop\\15640979-2017-04-12-110754.bmp");
	*pImg = imread("C:\\Users\\Sense\\Desktop\\15640979-2017-04-13-141720.bmp");

	vector<light_pos_t> LightPos;
	LightPos.resize(20);
//	int nContnn = TrafficLights(Img, CvRect(Rect(0,0,1600,1200)), LightPos, 20);
	int nContnn = TrafficLights_(*pImg, (Rect(0,0,1600,1200)), LightPos, 20);
	char szDisp[256] = {0};
	sprintf(szDisp, "%d %d %d %d %d %d", m_H_low, m_H_up, m_S_low, m_S_up, m_V_low, m_V_up);
 	putText(*pImg, szDisp, Point(100,100),0,1,CV_RGB(255,255,255),3);
 	putText(*pImg, szDisp, Point(100,200),0,1,CV_RGB(0,0,0),3);
	cvNamedWindow("1234",0);
	imshow("1234",*pImg);
	int c = waitKey(1);

	if (c == '1')
		m_H_up++;
	if (c == 'q')
		m_H_up--;
	if (c == 'a')
		m_H_low++;
	if (c == 'z')
		m_H_low--;

	if (c == '2')
		m_S_up++;
	if (c == 'w')
		m_S_up--;
	if (c == 's')
		m_S_low++;
	if (c == 'x')
		m_S_low--;

	if (c == '3')
		m_V_up++;
	if (c == 'e')
		m_V_up--;
	if (c == 'd')
		m_V_low++;
	if (c == 'c')
		m_V_low--;

	return;
*/	

// 	vector<string> VecImgPath;
// 	VecImgPath.push_back("C:\\Users\\Sense\\Desktop\\TrafficLog\\img0328\\5.bmp");
// 	VecImgPath.push_back("C:\\Users\\Sense\\Desktop\\TrafficLog\\img0328\\6.bmp");
// 	VecImgPath.push_back("C:\\Users\\Sense\\Desktop\\TrafficLog\\img0328\\7.bmp");
// 	VecImgPath.push_back("C:\\Users\\Sense\\Desktop\\TrafficLog\\img0328\\0.bmp");
// 	VecImgPath.push_back("C:\\Users\\Sense\\Desktop\\TrafficLog\\img0328\\1.bmp");
// 	VecImgPath.push_back("C:\\Users\\Sense\\Desktop\\TrafficLog\\img0328\\2.bmp");
// 	VecImgPath.push_back("C:\\Users\\Sense\\Desktop\\TrafficLog\\img0328\\3.bmp");
// 	VecImgPath.push_back("C:\\Users\\Sense\\Desktop\\TrafficLog\\img0328\\4.bmp");
// 
// 	vector<LCM_NAVI_REQUIRE_INFO> VecReq;
// 	LCM_NAVI_REQUIRE_INFO NaviReq;
// 	NaviReq.MsgInd = 5;
// 	NaviReq.Gps.GPS_HEADING = 83.399; NaviReq.Gps.GPS_LATITUDE = 39.6452345; NaviReq.Gps.GPS_LONGITUDE = 116.6695640; 
// 	VecReq.push_back(NaviReq);
// 	NaviReq.MsgInd = 6;
// 	NaviReq.Gps.GPS_HEADING = 77.112; NaviReq.Gps.GPS_LATITUDE = 39.6452350; NaviReq.Gps.GPS_LONGITUDE = 116.6695685; 
// 	VecReq.push_back(NaviReq);
// 	NaviReq.MsgInd = 7;
// 	NaviReq.Gps.GPS_HEADING = 90.911; NaviReq.Gps.GPS_LATITUDE = 39.6452363; NaviReq.Gps.GPS_LONGITUDE = 116.6695801; 
// 	VecReq.push_back(NaviReq);
//  	NaviReq.MsgInd = 0;
//  	NaviReq.Gps.GPS_HEADING = 82.422; NaviReq.Gps.GPS_LATITUDE = 39.6451005; NaviReq.Gps.GPS_LONGITUDE = 116.6682364; 
//  	VecReq.push_back(NaviReq);
//  	NaviReq.MsgInd = 1;
//  	NaviReq.Gps.GPS_HEADING = 83.214; NaviReq.Gps.GPS_LATITUDE = 39.6451305; NaviReq.Gps.GPS_LONGITUDE = 116.6685324; 
//  	VecReq.push_back(NaviReq);
//  	NaviReq.MsgInd = 2;
//  	NaviReq.Gps.GPS_HEADING = 82.410; NaviReq.Gps.GPS_LATITUDE = 39.6451607; NaviReq.Gps.GPS_LONGITUDE = 116.6688232; 
//  	VecReq.push_back(NaviReq);
//  	NaviReq.MsgInd = 3;
//  	NaviReq.Gps.GPS_HEADING = 82.346; NaviReq.Gps.GPS_LATITUDE = 39.6452006; NaviReq.Gps.GPS_LONGITUDE = 116.6692098; 
// 	VecReq.push_back(NaviReq);
// 	NaviReq.MsgInd = 4;
// 	NaviReq.Gps.GPS_HEADING = 83.160; NaviReq.Gps.GPS_LATITUDE = 39.6452171; NaviReq.Gps.GPS_LONGITUDE = 116.6693750; 
// 	VecReq.push_back(NaviReq);
// 
// 	*pImg = imread(VecImgPath[m_nLogInd]);
// 	CLcmRevicer<LCM_NAVI_REQUIRE_INFO> LcmSend(string("LCM_NAVI_REQUIRE_INFO"));
// 	LcmSend.Send("LCM_NAVI_REQUIRE_INFO",VecReq[m_nLogInd]);
// 	Sleep(100);

// 	LCM_TRAFIC_LIGHT_REQUIRE Req;
// 	Req.TraficLightLocate.x = 0;
// 	Req.TraficLightLocate.y = 80.000;
// 	Req.TraficLightLocate.z = 0.0;
// 	int nRt = 1;

  	LCM_TRAFIC_LIGHT_REQUIRE Req;
  	int nRt = pTLD->m_ReqRecieve.GetData(Req);

 	if (nRt !=  1)
 	{
		Mat ImgDrawed;
		pImg->copyTo(ImgDrawed);
		ImgDrawed.rowRange(ImgDrawed.rows-48, ImgDrawed.rows) = Scalar(0,0,0);
		char szDisp[256] = {0};
		sprintf(szDisp, "%.2f -2", m_pCamera->GetCallBackRate());
		putText(ImgDrawed, szDisp, Point(0,ImgDrawed.rows-1),0,2,CV_RGB(255,255,255),3);
 		cvNamedWindow("ImgDrawed",0);
 		imshow("ImgDrawed", ImgDrawed);
 		waitKey(1);
 		return;
 	}
// 	printf("Traffic Light Req received!\n");

	Mat ImgDisp;
	Process(*pImg, Req, ImgDisp);
	cvNamedWindow("ImgDrawed",0);
	imshow("ImgDrawed", ImgDisp);
	int nKey = waitKey(1);
	if (nKey == 'w' || nKey == 'W')
	{
		m_Param.dPitch += 0.1;
	}
	else if (nKey == 's' || nKey == 'S')
	{
		m_Param.dPitch -= 0.1;
	}
	else if (nKey == 'a' || nKey == 'A')
	{
		m_Param.dYaw += 0.1;
	}
	else if (nKey == 'd' || nKey == 'D')
	{
		m_Param.dYaw -= 0.1;
	}
	else if (nKey == 'j' || nKey == 'J')
	{
		m_Param.dHeight += 0.1;	
	}
	else if (nKey == 'k' || nKey == 'K')
	{
		m_Param.dHeight -= 0.1;
	}
	else if (nKey == 32)
	{
		m_nLogInd++;
		m_nLogInd = m_nLogInd>=8?0:m_nLogInd;
	}
	if (nKey != -1)
	{
		printf("dPitch:%.2f dRoll:%.2f dYaw:%.2f dHeight:%.2f\n",
			m_Param.dPitch,
			m_Param.dRoll,
			m_Param.dYaw,
			m_Param.dHeight);
		printf("x:%.2f, y:%.2f, z:%.2f\n", Req.TraficLightLocate.x,Req.TraficLightLocate.y,Req.TraficLightLocate.z);;
		GetExtrinctParam();
		waitKey(1);
	}
}

void CTraficLightDetector::AddTimeTail(string& sz)
{
	time_t t = time(0); 
	char tmp[64]; 
	strftime( tmp, sizeof(tmp), "%Y%m%d%H%M%S",localtime(&t) );
	sz += string(tmp);
}

LCM_TRAFIC_LIGHT_RESULT CTraficLightDetector::Process(Mat& img_in, LCM_TRAFIC_LIGHT_REQUIRE& Req_in, Mat& img_out)
{
	GetExtrinctParam();
	Mat ImgTarget, ImgDrawed;
	img_in.copyTo(ImgDrawed);
	int nRtRes = TraficLightDetect(ImgDrawed, Req_in, ImgTarget);
	LCM_TRAFIC_LIGHT_RESULT Result;
	Result.Req = Req_in;
	Result.State = nRtRes;
	m_ResultSend.Send(string("LCM_TRAFIC_LIGHT_RESULT"),  Result);
	ImgDrawed.rowRange(ImgDrawed.rows-48, ImgDrawed.rows) = Scalar(0,0,0);
	char szDisp[256] = {0};
	sprintf(szDisp, "%.2f %d", m_pCamera->GetCallBackRate(), nRtRes);
	putText(ImgDrawed, szDisp, Point(0,ImgDrawed.rows-1),0,2,CV_RGB(255,255,255),3);
	ImgDrawed.copyTo(img_out);

	return Result;
}

void CTraficLightDetector::GetExtrinctParam()
{
	LCM_GPS_DATA ImuData;
	m_ImuRec.GetData(ImuData);
	m_ExtrinsicParam = Mat::eye(3,4,CV_32F);
	float cx = cos(-1.0*(m_Param.dPitch+ImuData.GPS_PITCH)/180.0*CV_PI);
	float sx = sin(-1.0*(m_Param.dPitch+ImuData.GPS_PITCH)/180.0*CV_PI);
	float cy = cos(m_Param.dYaw/180.0*CV_PI);
	float sy = sin(m_Param.dYaw/180.0*CV_PI);
	float cz = cos(-1.0*m_Param.dRoll/180.0*CV_PI);
	float sz = sin(-1.0*m_Param.dRoll/180.0*CV_PI);
	Mat Rx = (Mat_<float>(3,3) <<	1,	0,	0,
		0,	cx,	-sx,
		0,	sx,	cx);
	Mat Ry = (Mat_<float>(3,3) <<	cy,	0,	sy,
		0,	1,	0,
		-sy,0,	cy);
	Mat Rz = (Mat_<float>(3,3) <<	cz,	-sz,0,
		sz,	cz,	0,
		0,	0,	1);
	Mat R = Rx*Ry*Rz;
	R.copyTo(m_ExtrinsicParam(Range(0,3),Range(0,3)));
	m_ExtrinsicParam.at<float>(0,3) = -1.0*m_Param.dLateral*1000.0;
	m_ExtrinsicParam.at<float>(1,3) = m_Param.dHeight*1000.0;
	m_ExtrinsicParam.at<float>(2,3) = -1.0*m_Param.dLongitude*1000.0;
}

CTraficLightDetector::CTraficLightDetector():
m_ReqRecieve(string("LCM_TRAFIC_LIGHT_REQUIRE")),
m_ResultSend(string("LCM_TRAFIC_LIGHT_RESULT"))
{
	using namespace boost::filesystem;
	m_szImgSaveDir = current_path().string();
	m_szImgSaveDir += (string("\\") + "TrafficLightSave\\");
	if (m_Param.bIsSaveImg)
	{
		create_directory(m_szImgSaveDir);
		AddTimeTail(m_szImgSaveDir);
		m_szImgSaveDir += "\\";
		create_directory(m_szImgSaveDir);
	}

	m_nLogInd = 0;

	m_Mode = 0;
	m_H_low = 25;//20
	m_H_up = 80;
	m_S_low = 50;//43
	m_S_up = 255;
	m_V_low = 25;//40
	m_V_up = 255;

	m_ImuRec.Init(CGpsManager::GPS_MANAGER_TYPE_POS);
	m_ImuRec.Start();

	return;
}

CTraficLightDetector::~CTraficLightDetector()
{
	ReleaseCameraHandle(m_pCamera);
}

int CTraficLightDetector::Init(TraficLightDetectParam& Param)
{
	m_Param = Param;

	GetExtrinctParam();

	m_pCamera = CreateCameraHandle(m_Param.CamParam);
	if (m_pCamera == 0)
	{
		printf("CreateCameraHandle failed!\n");
		return 0;
	}

	return 1;
}

int CTraficLightDetector::Start()
{
	m_pCamera->SetCallBack(boost::bind(&CTraficLightDetector::CameraDataCallBack,this,_1,_2), this);
	int nRt = m_pCamera->Start();
	if (nRt <= 0)
	{
		printf("m_pCamera->Start() failed!\n");
		return 0;
	}

	nRt = m_ReqRecieve.Start();
	if (nRt <= 0)
	{
		printf("m_ReqRecieve.Start() failed!\n");
		return 0;
	}

	return 1;
}

int CTraficLightDetector::Stop()
{
	int nRt = m_pCamera->Stop();
	if (nRt <= 0)
	{
		printf("m_pCamera->Stop() failed!\n");
		return 0;
	}

	return 1;
}

int CTraficLightDetector::GetTraficLightLocate(LCM_TRAFIC_LIGHT_REQUIRE& Req, cv::Rect& Box)
{
	Point3f luConer(Req.TraficLightLocate.x - m_Param.dRectSizeU/2, 
					Req.TraficLightLocate.y,
					Req.TraficLightLocate.z + m_Param.dRectSizeV/2);
	Point3f rdConer(Req.TraficLightLocate.x + m_Param.dRectSizeU/2, 
					Req.TraficLightLocate.y,
					Req.TraficLightLocate.z - m_Param.dRectSizeV/2);
	Point luConerImg;
	int nRt = Cartesian2Img(luConer, luConerImg);
	if (nRt <= 0)
		return 0;
	Point rdConerImg;
	nRt = Cartesian2Img(rdConer, rdConerImg);
	if (nRt <= 0)
		return 0;

	Box.x = luConerImg.x;
	Box.y = luConerImg.y;
	Box.height = rdConerImg.y - luConerImg.y;
	Box.width = rdConerImg.x - luConerImg.x;

	return 1;
}

int CTraficLightDetector::TrafficLights_(Mat& imgsrc, Rect& roi,vector<light_pos_t> &plight,int size, double dDist)
{
	int nSizePixelMin = INT_MAX;
	int nSizePixelMax = INT_MIN;
	if (dDist >= 1.0)
	{
		nSizePixelMin = m_Param.dFocusLen*m_Param.dTrafficLightSizeMin/dDist;
		nSizePixelMax = m_Param.dFocusLen*m_Param.dTrafficLightSizeMax/dDist;
	}

	Mat ImgRoi = imgsrc(roi);
	Mat ImgRoiBlur = ImgRoi.clone();
	blur(ImgRoiBlur, ImgRoiBlur, Size(5,5), Point(2,2));
	Mat ImgRoiHsv;
	cvtColor(ImgRoiBlur, ImgRoiHsv, CV_RGB2HSV);
	Mat GreenArea;
	inRange(ImgRoiHsv,Scalar(m_H_low,m_S_low,m_V_low),Scalar(m_H_up,m_S_up,m_V_up),GreenArea);
	Mat GreenAreaDilate;
	erode(GreenArea, GreenAreaDilate, Mat::ones(5,5,CV_8U),cvPoint(2,2));

	vector<Vec4i> hierarchy;
	vector<vector<Point>> contours;
	findContours(GreenAreaDilate.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	int nMaxAreaInd = -1;
	double dMaxArea = -1.0;
	Rect MaxRect;
	for (unsigned int i = 0; i < contours.size(); i++)
	{
		Rect rect_ = boundingRect(contours[i]);
		double dContourArea = contourArea(contours[i]);
		if (dContourArea <= 20)
		{
			rectangle(ImgRoi, rect_, CV_RGB(255,0,0),2);
			continue;
		}
		double dPer = (double)rect_.width/(double)rect_.height;
		if (dPer < 0.5 || dPer > 2.0)
		{
			rectangle(ImgRoi, rect_, CV_RGB(0,0,255),2);
			continue;
		}
		if (dContourArea/(rect_.width*rect_.height) < 0.5)
		{
			rectangle(ImgRoi, rect_, CV_RGB(255,255,0),2);
			continue;
		}
		if (rect_.width < nSizePixelMin || rect_.width > nSizePixelMax ||
			rect_.height < nSizePixelMin || rect_.height < nSizePixelMax)
		{
			rectangle(ImgRoi, rect_, CV_RGB(255,255,0),2);
			continue;
		}
		rectangle(ImgRoi, rect_, CV_RGB(0,255,0),2);
		if (dContourArea > dMaxArea)
		{
			dMaxArea = dContourArea;
			nMaxAreaInd = i;
			MaxRect = rect_;
		}
	}
	if (dMaxArea >= 20)
	{
		plight[0].type = 0;
		plight[0].pos = MaxRect;
		plight[0].pos.x += roi.x;
		plight[0].pos.y += roi.y;
		return 1;
	}

	return 0;
}

int CTraficLightDetector::TraficLightDetect(cv::Mat& Img, LCM_TRAFIC_LIGHT_REQUIRE& Req, cv::Mat& Img_out)
{
	Img.copyTo(Img_out);
	Rect Box;
	int nRt = GetTraficLightLocate(Req, Box);
	if (nRt <= 0)
		return -1;
// 	if (Box.tl().x < 0 || Box.tl().x >= Img_out.cols ||  Box.tl().y < 0 ||  Box.tl().y >= Img_out.rows)
// 		return -1;
// 	if (Box.br().x < 0 || Box.br().x >= Img_out.cols ||  Box.br().y < 0 ||  Box.br().y >= Img_out.rows)
// 		return 0;
	if (Box.tl().x >= Img_out.cols - 10 || Box.tl().y >= Img_out.rows - 10)
		return -1;
	if (Box.br().x <= 10 || Box.br().y <= 10)
		return -1;
	if (Box.x <= 0)
	{
		Box.width += Box.x;
		Box.x = 0;	}
	if (Box.y <= 0)
	{
		Box.height += Box.y;
		Box.y = 0;
	}
	if (Box.x + Box.width >= Img.cols)
	{
		Box.width = Img.cols - Box.x;
	}
	if (Box.y + Box.height >= Img.rows)
	{
		Box.height = Img.rows - Box.y;
	}
	if (Box.tl().x < 0 || Box.tl().y < 0 || Box.br().x >= Img.cols || Box.br().y >= Img.rows)
	{
		return -1;
	}

	if (m_Param.bIsSaveImg)
	{
		string szTimeTail;
		AddTimeTail(szTimeTail);
		char szSaveFileName[256] = {0};
		sprintf(szSaveFileName, "%06_%s.png", Req.MsgInd, szTimeTail.c_str());
		imwrite(m_szImgSaveDir + szSaveFileName, Img(Box));
		printf("%s Saved\n", szSaveFileName);
	}

	vector<light_pos_t> LightPos;
	LightPos.resize(20);
//	int nCont =TrafficLights(Img_out, CvRect(Box), LightPos, 20);
	int nCont =TrafficLights_(Img_out, Box, LightPos, 20, Req.TraficLightLocate.y);
//	printf("Traffic light cont:%d\n", nCont);
	rectangle(Img, Box, CV_RGB(0,0,0), 2);
	Rect BoxWhite = Box;
	BoxWhite.x -= 2;
	BoxWhite.y -= 2;
	BoxWhite.width += 4;
	BoxWhite.height += 4;
	rectangle(Img, BoxWhite, CV_RGB(255,255,255), 2);
	if (LightPos[0].type == 0)
		rectangle(Img, LightPos[0].pos, CV_RGB(0,255,0), 2);
	if (LightPos[0].type == 1)
		rectangle(Img, LightPos[0].pos, CV_RGB(255,255,0), 2);
	if (LightPos[0].type == 2)
		rectangle(Img, LightPos[0].pos, CV_RGB(255,0,0), 2);

	if (nCont == 0)
		return -1;
	return LightPos[0].type;
}

int CTraficLightDetector::Cartesian2Img(cv::Point3f& PtCart, cv::Point& PtImg)
{
	Mat PtCartMat = (Mat_<float>(4,1) << PtCart.x*1000.0, -1.0*PtCart.z*1000.0, PtCart.y*1000.0, 1.f);
	Mat PtImgMat = m_Param.IntrinsicParam * m_ExtrinsicParam * PtCartMat;
	if (PtImgMat.at<float>(2,0) <= 0.f)
		return 0;
	
	PtImg.x = PtImgMat.at<float>(0,0)/PtImgMat.at<float>(2,0);
	PtImg.y = PtImgMat.at<float>(1,0)/PtImgMat.at<float>(2,0);

	return 1;
}