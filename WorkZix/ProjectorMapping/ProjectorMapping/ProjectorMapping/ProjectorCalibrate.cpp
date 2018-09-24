#include "stdafx.h"
#include "ProjectorCalibrate.h"
#include "GetMaxInnerRectYaoP.h"

using namespace cv;

DWORD WINAPI CameraThreadProc(LPVOID lpParameter)
{
	CProjectCalibrate* pOpr = (CProjectCalibrate*)lpParameter;
	uint64_t nInd = 0;
	while (pOpr->m_bIsCameraRun)
	{
		namedWindow("CameraOri", 0);
		Mat Temp;
		pOpr->m_cap >> Temp;
//		flip(Temp,Temp,0);
//		bool bRt = pOpr->m_cap.read(Temp);
		if (Temp.data == 0)
		{
// 			pOpr->m_bIsCameraRun = false;
// 			break;
			Sleep(1);
			continue;
		}
		nInd++;
		EnterCriticalSection(&(pOpr->m_cs));
		Temp.copyTo(pOpr->m_matCamBuf);
		pOpr->m_nCameraCount++;
		LeaveCriticalSection(&(pOpr->m_cs));
		if (Temp.data != 0)
		{
			std::stringstream ss;
			ss << nInd;
			putText(Temp, ss.str(), Point(0,100), 0, 2, CV_RGB(255,0,0), 3);
			imshow("CameraOri", Temp);
			waitKey(1);
		}
	}
	

	return 1;
}


int CCalibResutPlanar::Calc()
{
	return 1;
}

cv::Mat CCalibResutPlanar::Rectify(cv::Mat& Image, CProjectorParam& Param)
{
	if (Image.data == 0 || HomoProj2Rec.data == 0)
	{
		return Mat();
	}
	
	Mat matProject = Mat::zeros(Param.ProjectorRect.height, Param.ProjectorRect.width, CV_8UC3);
	double dRatioProjector = (double)matProject.cols/(double)matProject.rows;
	double dRatioImage = (double)Image.cols/(double)Image.rows;
	int nResizeRows, nResizeCols;
	if (dRatioProjector >= dRatioImage)
	{
		nResizeRows = matProject.rows;
		nResizeCols = nResizeRows*dRatioImage;
	}
	else
	{
		nResizeCols = matProject.cols;
		nResizeRows = nResizeCols/dRatioImage;
	}
	Mat matResizedImage;
	resize(Image, matResizedImage, Size(nResizeCols,nResizeRows));
	matResizedImage.copyTo(matProject(Range(0,matResizedImage.rows),Range(0,matResizedImage.cols)));
// 	namedWindow("Image", 1);
// 	imshow("Image", Image);
// 	waitKey();
	
	Mat matRectified;
	warpPerspective(matProject, matRectified, HomoProj2Rec, matProject.size(), CV_INTER_LINEAR);
	return matRectified;
}

CProjectCalibrate::CProjectCalibrate()
{
	m_pProjectorDisp = 0;
	m_pUser = 0;
	m_hCamera = 0;
	InitializeCriticalSection(&m_cs);
	m_bIsCameraRun = false;
}

CProjectCalibrate::~CProjectCalibrate()
{
	StopCamera();
	m_cap.release();

	DeleteCriticalSection(&m_cs);
}

void CProjectCalibrate::Init(CProjectorParam& Param)
{
	m_Param = Param;
}

void CProjectCalibrate::SetProjectorDispCallBack(pProjectorDisplayCallBack pCallBack, void* pUser)
{
	m_pProjectorDisp = pCallBack;
	m_pUser = pUser;
}

bool CProjectCalibrate::RunCalib()
{
	namedWindow("Camera", 0);
	if (m_pProjectorDisp == 0)
	{
		return false;
	}

	if (m_matProjct.data == 0)
	{
		m_matProjct = Mat::zeros(m_Param.ProjectorRect.height, 
			m_Param.ProjectorRect.width, 3);
	}

	StartCamera();

//	namedWindow("Marks", 0);
	int nBlockSize = 4;

//	WaitForCameraImageStable();
	Mat matOri;
	Mat matPattren;
	m_matProjct.copyTo(matPattren);
	matPattren(Range(0,nBlockSize),Range(0,nBlockSize)) = Scalar(255,255,255);
	matPattren(Range(matPattren.rows-4,matPattren.rows),Range(0,nBlockSize)) = Scalar(255,255,255);
	matPattren(Range(0,nBlockSize),Range(matPattren.cols-4,matPattren.cols)) = Scalar(255,255,255);
	matPattren(Range(matPattren.rows-4,matPattren.rows),
		Range(matPattren.cols-4,matPattren.cols)) = Scalar(255,255,255);
	for (unsigned int i = 0; i < 10000000; i++)
	{
		m_matProjct = Scalar(0,0,0);
		(*m_pProjectorDisp)(m_matProjct, m_pUser);
		waitKey(1);				//第一个waitKey后，m_pProjectorDisp的回调才会起作用
		waitKey(500);			//等待500ms后牌照
//		m_cap >> matOri;
		int nRt = CameraCap(matOri);
		if (nRt <= 0)
		{
			return false;
		}
		imshow("Camera", matOri);
		waitKey(1);				//刷新Camera界面
		Mat matBg = Cvt8UC3_2_16SC1(matOri);

		matPattren.copyTo(m_matProjct);
		(*m_pProjectorDisp)(m_matProjct, m_pUser);

		waitKey(1);
		waitKey(500);
//		m_cap >> matOri;
		nRt = CameraCap(matOri);
		if (nRt <= 0)
		{
			return false;
		}
		imshow("Camera", matOri);
		waitKey(1);

//		continue;
		
		Mat matNow = Cvt8UC3_2_16SC1(matOri);
//		imshow("Marks", matNow);
//		waitKey(1);
		Mat matDelta = /*abs*/(matNow - matBg) >= 30;
		Mat matDeltaErode/* = matDelta*/;
		erode(matDelta, matDeltaErode, Mat::ones(3,3,matDelta.type()), Point(1,1));
// 		imshow("Marks", matDeltaErode);
// 		waitKey(1);
		vector<Vec4i> hierarchy;
		vector<vector<Point>> contours;
		findContours(matDeltaErode.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		for (unsigned int i = 0; i < contours.size(); i++)
		{
			Rect rect_ = boundingRect(contours[i]);
			rectangle(matOri, rect_, CV_RGB(255,0,0), 3);
		}
		imshow("Camera", matOri);
		waitKey(1);
		if (contours.size() != 4)
		{
			continue;
		}
		m_CalibContour.clear();
		for (unsigned int i = 0; i < contours.size(); i++)
		{
			Rect rect_ = boundingRect(contours[i]);
			m_CalibContour.push_back(
				Point2f(rect_.x+rect_.width/2,rect_.y+rect_.height/2));
		}
		for (unsigned int i = 0; i < 4; i++)
		{
			line(matOri, m_CalibContour[i%4], m_CalibContour[(i+1)%4], 
				CV_RGB(255,255,255), 1);
		}
		std::stringstream ss;
		ss << "Corners found! Press 'Y' to admit, or 'N' to retry.";
		putText(matOri, ss.str(), Point(0,10), 0, 1.5, CV_RGB(255,255,255),3);
		imshow("Camera", matOri);
		nRt = waitKey(1);
// 		if (nRt == 'y' || nRt == 'Y')
// 		{
// 			break;
// 		}
	}
	

	return true;
}

bool CProjectCalibrate::RunCalibEx()
{
	namedWindow("Camera", 0);
	if (m_pProjectorDisp == 0)
	{
		return false;
	}

	if (m_matProjct.data == 0)
	{
		m_matProjct = Mat::zeros(m_Param.ProjectorRect.height, 
			m_Param.ProjectorRect.width, 3);
	}

	StartCamera();

	//	namedWindow("Marks", 0);
	int nBlockSize = 4;

	//	WaitForCameraImageStable();
	Mat matOri;
	vector<Mat> matPattrens(4);

	m_matProjct.copyTo(matPattrens[0]);
	matPattrens[0] = Scalar(0,0,0);
	matPattrens[0](Range(0,nBlockSize),Range(0,nBlockSize)) = Scalar(255,255,255);

	m_matProjct.copyTo(matPattrens[1]);
	matPattrens[1] = Scalar(0,0,0);
	matPattrens[1](Range(0,nBlockSize),Range(m_matProjct.cols-4,m_matProjct.cols)) = Scalar(255,255,255);

	m_matProjct.copyTo(matPattrens[2]);
	matPattrens[2] = Scalar(0,0,0);
	matPattrens[2](Range(m_matProjct.rows-4,m_matProjct.rows),
		Range(m_matProjct.cols-4,m_matProjct.cols)) = Scalar(255,255,255);

	m_matProjct.copyTo(matPattrens[3]);
	matPattrens[3] = Scalar(0,0,0);
	matPattrens[3](Range(m_matProjct.rows-4,m_matProjct.rows),Range(0,nBlockSize)) = Scalar(255,255,255);

	vector<Point2f> VecCornors;
	for (unsigned int i = 0; i < matPattrens.size(); i++)
	{
		Point2f pt;
		int nRt = DetectFlikerPoint(matPattrens[i], pt);
		if (nRt <= 0)
		{
			return false;
		}
		VecCornors.push_back(pt);

		Mat matDisp;
		nRt = CameraCap(matDisp);
		if (nRt <= 0 || matDisp.data == 0)
		{
			continue;
		}
		for (unsigned int j = 0; j < VecCornors.size(); j++)
		{
			Point2f pt0 = VecCornors[j%VecCornors.size()];
			Point2f pt1 = VecCornors[(j+1)%VecCornors.size()];
			circle(matDisp, pt0, 5, CV_RGB(255,0,0), 3);
			line(matDisp, pt0, pt1, CV_RGB(255,255,255), 2);
			namedWindow("Camera", 0);
			imshow("Camera", matDisp);
			waitKey(1);
		}
	}

	Mat matDisp;
	int nRt = CameraCap(matDisp);
	if (nRt <= 0)
	{
		return false;
	}
	for (int i = 0; i < 4; i++)
	{
		line(matDisp, VecCornors[i%4], VecCornors[(i+1)%4], CV_RGB(255,255,255), 3);
		circle(matDisp, VecCornors[i], 3, CV_RGB(255,0,0), -1);
	}

	m_ResultPlanar.PorjectorPoints.clear();
	m_ResultPlanar.PorjectorPoints.push_back(Point2f(0, 0));
	m_ResultPlanar.PorjectorPoints.push_back(Point2f(m_Param.ProjectorRect.width, 0));
	m_ResultPlanar.PorjectorPoints.push_back(Point2f(m_Param.ProjectorRect.width, m_Param.ProjectorRect.height));
	m_ResultPlanar.PorjectorPoints.push_back(Point2f(0, m_Param.ProjectorRect.height));

	m_ResultPlanar.UnRectCameraPoints.clear();
	m_ResultPlanar.UnRectCameraPoints = VecCornors;

// 	Rect RectArea;// = GetMaxIntrinsicRect(m_ResultPlanar.UnRectCameraPoints);
// 	RectArea.x = 493;
// 	RectArea.y = 275;
// 	RectArea.width = 840;
// 	RectArea.height = 500;
	vector<Point> VecCornorsInt;
	VecCornorsInt.push_back(VecCornors[0]);
	VecCornorsInt.push_back(VecCornors[1]);
	VecCornorsInt.push_back(VecCornors[2]);
	VecCornorsInt.push_back(VecCornors[3]);
// 	Rect RectArea = getMaxInscribedRectangle(VecCornorsInt,
// 		(double)m_Param.ProjectorRect.width/(double)m_Param.ProjectorRect.height);
	Rect RectArea = GetMaxIntrinsicRect(VecCornors,
		(double)m_Param.ProjectorRect.width/(double)m_Param.ProjectorRect.height);
	m_ResultPlanar.RectCameraPoints.clear();
	m_ResultPlanar.RectCameraPoints.push_back(RectArea.tl());
	m_ResultPlanar.RectCameraPoints.push_back(Point2f(RectArea.x+RectArea.width, RectArea.y));
	m_ResultPlanar.RectCameraPoints.push_back(RectArea.br());
	m_ResultPlanar.RectCameraPoints.push_back(Point2f(RectArea.x, RectArea.y+RectArea.height));

	m_ResultPlanar.HomoRec2Cam = findHomography(m_ResultPlanar.PorjectorPoints,
		m_ResultPlanar.UnRectCameraPoints);
	m_ResultPlanar.HomoProj2Cam = findHomography(m_ResultPlanar.PorjectorPoints,
		m_ResultPlanar.RectCameraPoints);
	m_ResultPlanar.HomoProj2Rec = m_ResultPlanar.HomoRec2Cam.inv()*m_ResultPlanar.HomoProj2Cam;

	for (int i = 0; i < 4; i++)
	{
		line(matDisp, m_ResultPlanar.RectCameraPoints[i%4], m_ResultPlanar.RectCameraPoints[(i+1)%4], CV_RGB(0,255,0), 3);
		circle(matDisp, m_ResultPlanar.RectCameraPoints[i], 3, CV_RGB(0,255,0), -1);
	}
	imshow("Camera", matDisp);
	waitKey(1);
//	return m_ResultPlanar.Calc();

	return true;
}

bool CProjectCalibrate::WaitForCameraImageStable()
{
	if (!m_cap.isOpened())
	{
		return false;
	}
	
	namedWindow("Camera", 0);
//	namedWindow("dDeltaValue", 0);
	Mat image_befor, image_now, image_now_grey, image_now_float;
	for (int i = 0; i < 10000; i++)
	{
		m_cap >> image_now;
		if (image_now.data == 0)
		{
			return false;
		}

		cvtColor(image_now, image_now_grey, CV_RGB2GRAY);
		image_now_grey.convertTo(image_now_float, CV_32F);
		if (image_befor.data == 0)
		{
			image_now_float.copyTo(image_befor);
			continue;
		}
		

		Mat deltaImage = abs(image_now_float - image_befor)>=10;
		Mat deltaImageErode;
		erode(deltaImage, deltaImageErode, Mat::ones(3,3,deltaImage.type()), Point(1,1));
// 		int nType = image_now.type();
// 		int nChan = image_now.channels();
// 		int nType_ = deltaImage.type();
// 		int nChan_ = deltaImage.channels();
// 		double dMin, dMax;
// 		minMaxLoc(deltaImage, &dMin, &dMax);
		int nCont = countNonZero(deltaImageErode);

//		double dDeltaValue = max(abs(dMin), abs(dMax));
		char szDisp[256] = {0};
		sprintf(szDisp, "%d/%d %d", i, 100, nCont);
		putText(image_now, szDisp, Point(100,100), 0, 1.0, CV_RGB(255,255,255));
		
		imshow("Camera", image_now);
//		imshow("dDeltaValue", deltaImageErode);
		waitKey(1);

		if (nCont < 5)
		{
//			destroyWindow("dDeltaValue");
			return true;
		}

		image_now_float.copyTo(image_befor);
	}

	return true;
}

cv::Mat CProjectCalibrate::Cvt8UC3_2_16SC1(cv::Mat img)
{
	Mat out, out_grey;
	cvtColor(img, out_grey, CV_RGB2GRAY);
	out_grey.convertTo(out, CV_16S);

	return out;
}

cv::Mat CProjectCalibrate::GetGreyImage()
{
	if (!m_cap.isOpened())
	{
		return Mat();
	}

	Mat out, out_grey, out_float;
	m_cap >> out;
	cvtColor(out, out_grey, CV_RGB2GRAY);
	out_grey.convertTo(out_float, CV_32F);

	return out_float;
}

int CProjectCalibrate::StartCamera()
{
	m_nCameraCount = 0;
	if (!m_cap.open(m_Param.nCameraInd))
	{
		return -1;
	}
	m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_Param.CameraRes.x);
	m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_Param.CameraRes.y);
	if(!m_cap.isOpened())
		return -1;

//	Sleep(1000);

	m_bIsCameraRun = true;
	m_hCamera = CreateThread(NULL,0,CameraThreadProc,this,0,NULL);
	if(NULL == m_hCamera)
	{
		m_bIsCameraRun = false;
		printf("Camera thread start failed!");
		return -1;
	}

	return 1;
}

int CProjectCalibrate::CameraCap(cv::Mat& image)
{
	if (!m_bIsCameraRun)
	{
		return -1;
	}

	EnterCriticalSection(&m_cs);
	m_matCamBuf.copyTo(image);
	LeaveCriticalSection(&m_cs);

	return 1;
}

int CProjectCalibrate::StopCamera()
{
	m_bIsCameraRun = false;
	WaitForSingleObject( m_hCamera, INFINITE );

	CloseHandle(m_hCamera);

	return 1;
}

int CProjectCalibrate::DetectFlikerPoint(cv::Mat matObj, cv::Point2f& pt_out)
{
	Mat matOri;
	for (unsigned int i = 0; i < 100; i++)
	{
		m_matProjct = Scalar(0,0,0);
		(*m_pProjectorDisp)(m_matProjct, m_pUser);
		waitKey(1);				//第一个waitKey后，m_pProjectorDisp的回调才会起作用
		waitKey(500);			//等待500ms后牌照
		int nRt = CameraCap(matOri);
		if (nRt <= 0)
		{
			return -1;
		}
		imshow("Camera", matOri);
		waitKey(1);				//刷新Camera界面
		Mat matBg = Cvt8UC3_2_16SC1(matOri);

		matObj.copyTo(m_matProjct);
		(*m_pProjectorDisp)(m_matProjct, m_pUser);

		waitKey(1);
		waitKey(500);
		//		m_cap >> matOri;
		nRt = CameraCap(matOri);
		if (nRt <= 0)
		{
			return -1;
		}
		imshow("Camera", matOri);
		waitKey(1);

		Mat matNow = Cvt8UC3_2_16SC1(matOri);
		Mat matDelta = (matNow - matBg) >= 30;
		Mat matDeltaErode/* = matDelta*/;
		erode(matDelta, matDeltaErode, Mat::ones(3,3,matDelta.type()), Point(1,1));
		// 		imshow("Marks", matDeltaErode);
		// 		waitKey(1);
		vector<Vec4i> hierarchy;
		vector<vector<Point>> contours;
		findContours(matDeltaErode.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		for (unsigned int i = 0; i < contours.size(); i++)
		{
			Rect rect_ = boundingRect(contours[i]);
			rectangle(matOri, rect_, CV_RGB(255,0,0), 3);
			pt_out = Point(rect_.x+rect_.width/2,rect_.y+rect_.height/2);
		}
		imshow("Camera", matOri);
		waitKey(1);
		if (contours.size() != 1)
		{
			continue;
		}

		std::stringstream ss;
		ss << "Corners found! Press 'Y' to admit, or 'N' to retry.";
		putText(matOri, ss.str(), Point(0,100), 0, 1.5, CV_RGB(0,255,255),3);
		imshow("Camera", matOri);
		nRt = waitKey(1);
//		if (nRt == 'y' || nRt == 'Y')
		{
			m_matProjct = Scalar(0,0,0);
			(*m_pProjectorDisp)(m_matProjct, m_pUser);
		 	return 1;
		}
	}

	return -1;
}

cv::Rect CProjectCalibrate::GetMaxIntrinsicRect(std::vector<Point2f> coutour, double dRatio)
{
	Rect out(0,0,0,0);
	if (coutour.size() <= 0)
	{
		return out;
	}
	int nMaxX = 0;
	int nMaxY = 0;
	for (unsigned int i = 0; i < coutour.size(); i++)
	{
		if (coutour[i].x >= nMaxX)
		{
			nMaxX = coutour[i].x;
		}
		if (coutour[i].y >= nMaxY)
		{
			nMaxY = coutour[i].y;
		}
	}

	Mat Temp = Mat::zeros(nMaxY+100, nMaxX+100, CV_8UC1);

	vector<Point> VecPtInt;
	for (unsigned int i = 0; i < coutour.size(); i++)
	{
		VecPtInt.push_back(coutour[i]);
	}
	vector<vector<Point> > VecContour;
	VecContour.push_back(VecPtInt);
	drawContours(Temp, VecContour, 0, Scalar(255,255,255), -1);
	Mat TempDilate;
	dilate(Temp, TempDilate, Mat::ones(5,5,CV_8UC1), Point(2,2));
	Temp = TempDilate;
	double dDeltaDist = 10;
	vector<Rect> VecRect;
	for (unsigned int i = 0; i < coutour.size(); i++)
	{
		Point pt0 = coutour[i%coutour.size()];
		Point pt1 = coutour[(i+1)%coutour.size()];
		double dDist = sqrt(pow((double)pt0.x-pt1.x,2)+pow((double)pt0.y-pt1.y,2));
		for (double j = 0.0; j < dDist; j+=dDeltaDist)
		{
			Point ptLT;
			ptLT.x = pt0.x + (pt1.x-pt0.x)/dDist*j;
			ptLT.y = pt0.y + (pt1.y-pt0.y)/dDist*j;
			if (ptLT.x < 0 || ptLT.y < 0 || ptLT.x >= Temp.cols || ptLT.y >= Temp.rows )
			{
				continue;
			}

			Point ptRT, ptLB, ptRB;
			for (unsigned k = 0; k < Temp.cols; k+=dDeltaDist)
			{
				ptRT = Point(ptLT.x+k, ptLT.y);
				if (ptRT.x >= Temp.cols-1)
					break;
				ptLB = Point(ptLT.x, ptLT.y+k/dRatio);
				if (ptLB.y >= Temp.rows-1)
					break;
				ptRB = Point(ptLT.x+k, ptLT.y+k/dRatio);
				if (ptRB.x >= Temp.cols-1 || ptRB.y >= Temp.rows-1)
					break;
				if (Temp.at<uchar>(ptRT) == 0 || 
					Temp.at<uchar>(ptLB) == 0 || 
					Temp.at<uchar>(ptRB) == 0)
				{
					break;
				}
				Rect rect_inner(ptLT, ptRB);
				VecRect.push_back(rect_inner);
			}

// 			if (VecRect.size() > 0)
// 			{
// 				Rect& rc = VecRect.back();
// 				Mat aaa = Temp/2;
// 				rectangle(aaa, rc.tl(), rc.br(), CV_RGB(255,255,255), 2);
// 				circle(aaa, rc.tl(), 5, CV_RGB(255,255,255), -1);
// 				namedWindow("MaxInnerRect", 0);
// 				imshow("MaxInnerRect", aaa);
// 				waitKey(1);
// 			}
			
			Point ptRT_, ptLB_, ptRB_;
			for (unsigned k = 0; k < Temp.cols; k+=dDeltaDist)
			{
				ptRT_ = Point(ptLT.x+k, ptLT.y);
				if (ptRT_.x >= Temp.cols-1)
					break;
				ptLB_ = Point(ptLT.x, ptLT.y-k/dRatio);
				if (ptLB_.y <= 0)
					break;
				ptRB_ = Point(ptLT.x+k, ptLT.y-k/dRatio);
				if (ptRB.x >= Temp.cols-1 || ptRB.y <= 0)
					break;
				if (Temp.at<uchar>(ptRT_) == 0 || 
					Temp.at<uchar>(ptLB_) == 0 || 
					Temp.at<uchar>(ptRB_) == 0)
				{
					break;
				}
				Rect rect_inner(ptLB_, ptRT_);
				VecRect.push_back(rect_inner);
			}

// 			if (VecRect.size() > 0)
// 			{
// 				Rect& rc = VecRect.back();
// 				Mat aaa = Temp/2;
// 				rectangle(aaa, rc.tl(), rc.br(), CV_RGB(255,255,255), 2);
// 				circle(aaa, rc.tl(), 5, CV_RGB(255,255,255), -1);
// 				namedWindow("MaxInnerRect", 0);
// 				imshow("MaxInnerRect", aaa);
// 				waitKey(1);
// 			}
		}
	}

	for (unsigned int i = 0; i < VecRect.size(); i++)
	{
		if (VecRect[i].width >= out.width)
		{
			out = VecRect[i];
		}
	}

	destroyWindow("MaxInnerRect");

	return out;
}

cv::Mat CProjectCalibrate::Rectify(cv::Mat& Image)
{
	return m_ResultPlanar.Rectify(Image, m_Param);
}