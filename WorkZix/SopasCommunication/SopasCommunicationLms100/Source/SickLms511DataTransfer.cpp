#include <Windows.h>
#include "SickLms511DataTransfer.h"
#include "LcmReceiver.h"
#include "LCM_IBEO_CLOUD_POINTS.hpp"
#include "LIDAR_RADAR_INFO.hpp"
#include "LCM_IBEO_OBJECT_LIST.hpp"

Mat Grey2FateColor(Mat& mat_in)
{
	Mat temp(mat_in.rows,mat_in.cols,CV_8UC1);
	cvNormalize( &CvMat(mat_in), &CvMat(temp), 255, 0, CV_MINMAX );

	CvMat* red = cvCreateMat(temp.rows, temp.cols, CV_8U);
	CvMat* green = cvCreateMat(temp.rows, temp.cols, CV_8U);
	CvMat* blue = cvCreateMat(temp.rows, temp.cols, CV_8U);
	CvMat* mask = cvCreateMat(temp.rows, temp.cols, CV_8U);

	Mat color_mat(temp.rows,temp.cols,CV_8UC3);
	// 计算各彩色通道的像素值
	cvSubRS(&CvMat(temp), cvScalar(255), blue);	// blue(I) = 255 - gray(I)
	cvCopy(&CvMat(temp), red);			// red(I) = gray(I)
	cvCopy(&CvMat(temp), green);			// green(I) = gray(I),if gray(I) < 128
	cvCmpS(green, 128, mask, CV_CMP_GE );	// green(I) = 255 - gray(I), if gray(I) >= 128
	cvSubRS(green, cvScalar(255), green, mask);
	cvConvertScale(green, green, 2.0, 0.0);

	// 合成伪彩色图
	cvMerge(blue, green, red, NULL, &CvMat(color_mat));

	cvReleaseMat( &red );
	cvReleaseMat( &green );
	cvReleaseMat( &blue );
	cvReleaseMat( &mask );

	return color_mat;
}

CSickLms511DataTransfer::CSickLms511DataTransfer(double dStartAngle, double dAngRes)
{
	m_dAngleRes = dAngRes;
	m_dStartAngle = dStartAngle;
	m_pLcm = new CLcmRevicer<LCM_IBEO_CLOUD_POINTS>(std::string("LCM_IBEO_CLOUD_POINTS"));
	m_pLcmEsr = new CLcmRevicer<LIDAR_RADAR_INFO>(std::string("LIDAR_RADAR_INFO_ESR"));
	((CLcmRevicer<LIDAR_RADAR_INFO>*)m_pLcmEsr)->Start();

	m_nGridFront = 50000;
	m_nGridBack = 20000;
	m_nGridSide = 20000;
	m_nGridSize = 200;

	m_dSegDist = 500;
	m_dCurveFitEpsilon = 100;
	m_dMatchDist = 3000;
	m_nLogInd = 0;

	m_EsrT.x = 0.0;
	m_EsrT.y = 4500;
	m_dEsrHeading = 0.0;

	m_RsdsT.x = 0.0;
	m_RsdsT.y = 3800.0;
	m_dRsdsHeading = 0.0;

	m_GpsRec.Init(CGpsManager::GPS_MANAGER_TYPE_POS);
	m_GpsRec.Start();
}

CSickLms511DataTransfer::~CSickLms511DataTransfer()
{

}

void CSickLms511DataTransfer::SendData(char* pszChannelName, unsigned short* pDist, int nLen)
{
	LCM_IBEO_CLOUD_POINTS IbeoPts;
	IbeoPts.IbeoPoints.resize(nLen);
	for (int i = 0; i < nLen; i++)
	{
		double dAng = m_dStartAngle+i*m_dAngleRes;
		IbeoPts.IbeoPoints[i].angle = (dAng-90.0)*32;
		IbeoPts.IbeoPoints[i].distance = (float)(pDist[i])*2.0;
		IbeoPts.IbeoPoints[i].layer = 0;
		IbeoPts.IbeoPoints[i].echo = 0;
	}
	IbeoPts.ContPoints = nLen;
	((CLcmRevicer<LCM_IBEO_CLOUD_POINTS>*)m_pLcm)->Send(std::string(pszChannelName/*"LCM_IBEO_CLOUD_POINTS"*/),IbeoPts);
}

void CSickLms511DataTransfer::SendData(char* pszChannelName, vector<cv::Point2f>& Points)
{
	LCM_IBEO_CLOUD_POINTS IbeoPts;
	IbeoPts.IbeoPoints.resize(Points.size());
	for (unsigned int i = 0; i < Points.size(); i++)
	{
		cv::Point2f pt = Points[i];
		double dAng = atan2(pt.y, pt.x)/CV_PI*180.0;
		IbeoPts.IbeoPoints[i].angle = (dAng-90.0)*32;
		IbeoPts.IbeoPoints[i].distance = sqrt(pow(pt.x,2) + pow(pt.y,2));
		IbeoPts.IbeoPoints[i].layer = 0;
		IbeoPts.IbeoPoints[i].echo = 0;
	}
	IbeoPts.ContPoints = IbeoPts.IbeoPoints.size();
	((CLcmRevicer<LCM_IBEO_CLOUD_POINTS>*)m_pLcm)->Send(std::string(pszChannelName/*"LCM_IBEO_CLOUD_POINTS"*/),IbeoPts);
}

void CSickLms511DataTransfer::SendDataWithEsr(char* pszChannelName, vector<cv::Point2f>& Points)
{
	LIDAR_RADAR_INFO EsrData;
	int nRt = ((CLcmRevicer<LIDAR_RADAR_INFO>*)m_pLcmEsr)->GetData(EsrData);
	if (nRt <= 0)
	{
		printf("Get Esr data failed!\n");
		return;
	}
}

LCM_IBEO_OBJECT_LIST CSickLms511DataTransfer::ConfusionWithEsr(Mat& img, vector<cv::Point2f>& Points, vector<LIDAR_RADAR_OBJECT_INFO>& Esr)
{
	DWORD t1 = ::GetTickCount();
//	RectifyEsrData(Esr);
	LCM_GPS_DATA gps;
	m_GpsRec.GetData(gps);

	if (img.data == NULL)
	{
		img = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8UC3);
	}

	m_nLogInd++;
	LCM_IBEO_OBJECT_LIST out;

	double dMin, dMax;
	Mat Img = Gridding(Points);
	DrawCar(img, 4550, 1500, 1130);

	//Save Log
	//////////////////////////////////////////////////////////////////////////
// 	char szImgPath[256];
// 	sprintf(szImgPath,"%06d.png",m_nLogInd);
// 	imwrite(szImgPath,Img);
// 	char szImgEsr[256];
// 	sprintf(szImgEsr,"%06d.esr",m_nLogInd);
// 	ofstream fout(szImgEsr, ios::binary);
// 	int nDataLen = Esr.getEncodedSize();
// 	Esr.encode((void*)m_FileBuffer, 0, 10000);
// 	fout.write(m_FileBuffer, nDataLen);
// 	fout.close();
	//////////////////////////////////////////////////////////////////////////

// 	minMaxLoc(Img, &dMin, &dMax);
// 	printf("Img %.2f - %.2f\n", dMin, dMax);
// 	imshow("Img",Img);
// 	waitKey();
	Img.row(0) = 0;
	Img.row(Img.rows-1) = 0;
	Img.col(0) = 0;
	Img.col(Img.cols-1) = 0;
	Mat ImgDilate;
	int nSegDistPixel = m_dSegDist/m_nGridSize;
	Mat element = getStructuringElement( MORPH_ELLIPSE,
		Size( 2*nSegDistPixel + 1, 2*nSegDistPixel+1 ),
		Point( nSegDistPixel, nSegDistPixel ) );
	dilate(Img, ImgDilate, element);
// 	minMaxLoc(ImgDilate, &dMin, &dMax);
// 	printf("ImgDilate %.2f - %.2f\n", dMin, dMax);
// 	imshow("ImgDilate",ImgDilate);
// 	waitKey();
	vector<Vec4i> hierarchy;
	cv::vector<cv::vector<cv::Point>> contours;
	cv::findContours(ImgDilate, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	Mat ImgDilateId = Mat::zeros(ImgDilate.rows, ImgDilate.cols, CV_16U);
	for (unsigned int i = 0; i < contours.size(); i++)
	{
		drawContours(ImgDilateId, contours, i, CV_RGB(i+1,i+1,i+1), -1);
	}
// 	Mat Locate_;
// 	findNonZero(ImgDilateId==1, Locate_);
// 	minMaxLoc(ImgDilateId, &dMin, &dMax);
// 	printf("ImgDilateId %.2f - %.2f\n", dMin, dMax);
// 	imshow("ImgDilateId",Grey2FateColor(ImgDilateId));
// 	waitKey();
	Mat Img16U = Mat::zeros(ImgDilateId.rows, ImgDilateId.cols, CV_16U);
	Img.convertTo(Img16U, CV_16U);
// 	minMaxLoc(Img16U, &dMin, &dMax);
// 	printf("Img16U %.2f - %.2f\n", dMin, dMax);
// 	imshow("Img16U",Grey2FateColor(Img16U));
// 	waitKey();
// 	findNonZero(Img16U==1, Locate_);
// 	imshow("Img==1",Grey2FateColor(Mat(Img==1)));
// 	imshow("ImgDilateId==1",Grey2FateColor(Mat(ImgDilateId==1)));
// 	imshow("Img16U==1",Grey2FateColor(Mat(Img16U==1)));
	Mat ImgId = ImgDilateId.mul(Img16U);
// 	imshow("ImgId==1",Grey2FateColor(Mat(ImgId==1)));
// 	Mat Locate2;
// 	findNonZero(ImgId==1, Locate2);
// 	imshow("ImgId",Grey2FateColor(ImgId));
// 	waitKey();
	vector<vector<Point2f>> Objects;
	for (unsigned int i = 0; i < contours.size(); i++)
	{
		Mat Locate;
		cv::findNonZero(ImgId==(i+1), Locate);
		vector<Point2f> Obj;
		for (unsigned int j = 0; j < Locate.total(); j++)
		{
			Point PtPixel = Locate.at<Point>(j);
			Point2f PtCar = ImageUV2CarXY(PtPixel);
			Obj.push_back(PtCar);
		}
		Objects.push_back(Obj);
	}

	vector<vector<Point2f>> ObjectsFit;
	ObjectsFit = Objects;
// 	ObjectsFit.resize(Objects.size());
// 	for (unsigned int i = 0; i < Objects.size(); i++)
// 	{
// 		approxPolyDP(Objects[i], ObjectsFit[i], m_dCurveFitEpsilon, false);
// 	}

	vector<Point2f> RefPoints;
	vector<double> Distances;
	vector<Rect> Boxes;
	for (unsigned int i = 0; i < ObjectsFit.size(); i++)
	{
		double dDistMin = DBL_MAX;
		int nInd = -1;
		for (unsigned int j = 0; j < ObjectsFit[i].size(); j++)
		{
			double dDist = sqrt(pow(ObjectsFit[i][j].x,2) + pow(ObjectsFit[i][j].y,2));
			if (dDist <= dDistMin)
			{
				dDistMin = dDist;
				nInd = j;
			}
		}
		RefPoints.push_back(ObjectsFit[i][nInd]);
		Distances.push_back(dDistMin);
		Rect ObjBox =  boundingRect(ObjectsFit[i]);
		Boxes.push_back(ObjBox);
	}

	vector<Point2f> QueryPt;
//	vector<int> QueryId;
	vector<int> MatchesTrainId;
	vector<int> MatchesTrainSubId;
	vector<double> MatchDist;
	for (unsigned int i = 0; i < Esr.size(); i++)
	{
// 		if (!Esr.Objects[i].bValid)
// 		{
// 			QueryPt.push_back(Point2f(0.f,0.f));
// 			MatchesTrainId.push_back(-1);
// 			MatchesTrainSubId.push_back(-1);
// 			MatchDist.push_back(DBL_MAX);
// 			continue;
// 		}
		Point2f ptEsr;
		ptEsr.x = Esr[i].fObjLocX;
		ptEsr.y = Esr[i].fObjLocY;
		QueryPt.push_back(ptEsr);
//		QueryId.push_back(i);
		int nMatchId = -1;
		int nMatchSubId = -1;
		double nMatchDistMin = DBL_MAX;
		for (unsigned int j = 0; j < ObjectsFit.size(); j++)
		{
			for (unsigned int k = 0; k < ObjectsFit[j].size(); k++)
			{
				Point2f ptObj = ObjectsFit[j][k];
				double dDist = sqrt(pow((ptEsr.x - ptObj.x),2) + pow((ptEsr.y - ptObj.y),2));
				if (dDist <= nMatchDistMin)
				{
					nMatchDistMin = dDist;
					nMatchId = j;
					nMatchSubId = k;
				}
			}
		}
		MatchesTrainId.push_back(nMatchId);
		MatchesTrainSubId.push_back(nMatchSubId);
		MatchDist.push_back(nMatchDistMin);
	}

	vector<int> MatchesTrainId_;
	vector<double> MatchDist_;
	for (unsigned int i = 0; i < ObjectsFit.size(); i++)
	{
		int nMatchInd2Esr = -1;
		double dMatchDist = DBL_MAX;
		for (unsigned int j = 0; j < Esr.size(); j++)
		{
			if (MatchesTrainId[j] != i)
			{
				continue;
			}
			if (MatchDist[j] < dMatchDist)
			{
				dMatchDist = MatchDist[j];
				nMatchInd2Esr = j;
			}
		}
		if (dMatchDist >= m_dMatchDist)
			MatchesTrainId_.push_back(-1);
		else
			MatchesTrainId_.push_back(nMatchInd2Esr);
		MatchDist_.push_back(dMatchDist);
	}

	for (unsigned int i = 0; i < ObjectsFit.size(); i++)
	{
		LCM_IBEO_OBJECT IbeoObj;
		IbeoObj.Age = 10;
		IbeoObj.Classification = 1;
		IbeoObj.ContContourPt = ObjectsFit[i].size();
		for (unsigned int j = 0; j < ObjectsFit[i].size(); j++)
		{
			LCM_POINT2D_F pt;
			pt.x = ObjectsFit[i][j].y;
			pt.y = -1.0*ObjectsFit[i][j].x;
			IbeoObj.ContourPts.push_back(pt);
		}
		IbeoObj.ObjBoxCenter.x = Boxes[i].y + Boxes[i].height/2;
		IbeoObj.ObjBoxCenter.y = -1.0*(Boxes[i].x + Boxes[i].width/2);
		IbeoObj.ObjBoxSize.x = Boxes[i].height;
		IbeoObj.ObjBoxSize.y = Boxes[i].width;
		IbeoObj.ObjOrientation = 0;
		IbeoObj.RefPoint.x = RefPoints[i].y;
		IbeoObj.RefPoint.y = -1.0*RefPoints[i].x;
		IbeoObj.PredictAge = 0;
		IbeoObj.ObjExtMeasurement = 1.0;
		IbeoObj.AbsVelocity.x = 0.0;
		IbeoObj.AbsVelocity.y = 0.0;
		if (MatchesTrainId_[i] == -1)
		{
			IbeoObj.Id = 100;
//			IbeoObj.RelativeVelocity.x = 0;
			IbeoObj.RelativeVelocity.x = -1000.0 * sqrt(pow(gps.GPS_VE,2) + pow(gps.GPS_VN,2));
			IbeoObj.RelativeVelocity.y = 0;
		}
		else
		{
			LIDAR_RADAR_OBJECT_INFO& EsrObj = Esr[MatchesTrainId_[i]];
			IbeoObj.Id = MatchesTrainId_[i];
			IbeoObj.RelativeVelocity.x = EsrObj.fObjSizeY;
			IbeoObj.RelativeVelocity.y = -1.0*EsrObj.fObjSizeX;
		}
		out.IbeoObjects.push_back(IbeoObj);
	}
	out.ContObjects = out.IbeoObjects.size();
	out.FrameInd = m_nLogInd;

	//Draw lidar objects
	vector<vector<Point>> ObjectsFitPixel;
	for (unsigned int i = 0; i < ObjectsFit.size(); i++)
	{
		vector<Point> ObjectPixel;
		for (unsigned int j = 0; j < ObjectsFit[i].size(); j++)
		{
			Point PtPixel = CarXY2ImageUV(ObjectsFit[i][j]);
			ObjectPixel.push_back(PtPixel);
		}
		ObjectsFitPixel.push_back(ObjectPixel);
	}

	vector<cv::Rect> BoxesPixel;
	for (unsigned int i = 0; i < Boxes.size(); i++)
	{
		cv::Rect BoxPix;
		Point BoxCenter = CarXY2ImageUV(Point(Boxes[i].x,Boxes[i].y));
		BoxPix.x = BoxCenter.x;
		BoxPix.y = BoxCenter.y;
		BoxPix.width = Boxes[i].width/m_nGridSize;
		BoxPix.height = Boxes[i].height/m_nGridSize;
		BoxesPixel.push_back(BoxPix);
	}

	for (unsigned int i = 0; i < ObjectsFitPixel.size(); i++)
	{
		CvScalar color;
		if (MatchesTrainId_[i] == -1)
			color = CV_RGB(0,0,255);
		else
			color = CV_RGB(255,0,0);
		for (unsigned int j = 1; j < ObjectsFitPixel[i].size(); j++)
		{
			Point& pt0 = ObjectsFitPixel[i][j-1];
			Point& pt1 = ObjectsFitPixel[i][j];
			line(img, pt0, pt1, color);
		}
//		rectangle(img, BoxesPixel[i], color);
	}

	//Draw Esr objects
	for (unsigned int i = 0; i < Esr.size(); i++)
	{
		LIDAR_RADAR_OBJECT_INFO ObjT = Esr[i];
		if (ObjT.bValid == 0)
			continue;
		Point2f ptEsr;
		ptEsr.x = Esr[i].fObjLocX;
		ptEsr.y = Esr[i].fObjLocY;
		Point PtPix = CarXY2ImageUV(ptEsr);
		circle(img, PtPix, 500/m_nGridSize, CV_RGB(0,255,0));

		double dTimeDelta = 1.0;
		double dXDelta = 0.0/*dTimeDelta * ObjT.fObjSizeX*/;
		double dYDelta = dTimeDelta * ObjT.fObjSizeY;
		Point2d ptStart(ptEsr);
		Point2d ptEnd(ptStart.x + dXDelta, ptStart.y + dYDelta);
		Point2i ptStart_ = CarXY2ImageUV(ptStart);
		Point2i ptEnd_ = CarXY2ImageUV(ptEnd);
		arrowedLine(img,ptStart_,ptEnd_,CV_RGB(0,255,0));
	}

	//Draw links
	for (unsigned int i = 0; i < MatchesTrainId_.size(); i++)
	{
		if (MatchesTrainId_[i] == -1)
			continue;
//		Point pt0 = CarXY2ImageUV(RefPoints[i]);
		Point pt0 = ObjectsFitPixel[i][MatchesTrainSubId[MatchesTrainId_[i]]];
		Point2f ptEsr;
		ptEsr.x = Esr[MatchesTrainId_[i]].fObjLocX;
		ptEsr.y = Esr[MatchesTrainId_[i]].fObjLocY;
		Point pt1 = CarXY2ImageUV(ptEsr);
//		line(img, pt0, pt1, CV_RGB(255,255,0));
	}

	DWORD t2 = ::GetTickCount();
	printf("Time use: %d\n", t2-t1);

	return out;
}

Mat CSickLms511DataTransfer::Gridding(vector<cv::Point2f>& Points)
{
	Mat Lidar1 = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8U);
	for (unsigned int i = 0; i < Points.size(); i++)
	{
		Point2d ptImg = CarXY2ImageUV(Points[i]);
		if (ptImg.x >= 0 && ptImg.x < Lidar1.cols && ptImg.y >= 0 && ptImg.y < Lidar1.rows)
		{
			Lidar1.at<UCHAR>(ptImg) = 1;
		}
	}

	return Lidar1;
}

Point2i CSickLms511DataTransfer::CarXY2ImageUV(Point2d PtCar)
{
	return Point2i((m_nGridSide+(PtCar.x + m_nGridSize/2))/m_nGridSize,(m_nGridFront-(PtCar.y + m_nGridSize/2))/m_nGridSize);
}
Point2d CSickLms511DataTransfer::ImageUV2CarXY(Point2i PtImg)
{
	return Point2d(PtImg.x*m_nGridSize - m_nGridSide - m_nGridSize/2, m_nGridFront - PtImg.y*m_nGridSize - m_nGridSize/2);
}

void CSickLms511DataTransfer::RectifyEsrData(LIDAR_RADAR_INFO& Esr)
{
	LIDAR_RADAR_INFO EsrT= Esr;
	double dSinAlpha = sin(m_dEsrHeading/180.0*CV_PI);
	double dCosAlpha = cos(m_dEsrHeading/180.0*CV_PI);
	for (unsigned int i = 0; i < 64; i++)
	{
		EsrT.Objects[i].fObjLocX = dCosAlpha*Esr.Objects[i].fObjLocX - dSinAlpha*Esr.Objects[i].fObjLocY;
		EsrT.Objects[i].fObjLocY = dSinAlpha*Esr.Objects[i].fObjLocX + dCosAlpha*Esr.Objects[i].fObjLocY;
		EsrT.Objects[i].fObjSizeX = dCosAlpha*Esr.Objects[i].fObjSizeX - dSinAlpha*Esr.Objects[i].fObjSizeY;
		EsrT.Objects[i].fObjSizeY = dSinAlpha*Esr.Objects[i].fObjSizeX + dCosAlpha*Esr.Objects[i].fObjSizeY;
	}
	Esr = EsrT;
}

void CSickLms511DataTransfer::DrawCar(cv::Mat& img, int nFront, int nRear, int nSide)
{
	CvScalar color = CV_RGB(100,100,100);
	if (img.data == NULL)
	{
		img = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8UC3);
	}
	img.col((m_nGridSide/m_nGridSize)+1) = color;
	img.row((m_nGridFront/m_nGridSize)+1) = color;

	Point2f Coners[4];
	Coners[0] = Point2f(-1.f*nSide,nFront);
	Coners[1] = Point2f(nSide,nFront);
	Coners[2] = Point2f(nSide,-1.f*nRear);
	Coners[3] = Point2f(-1.f*nSide,-1.f*nRear);

	Point2f Coners_[4];
	for (int i = 0; i < 4; i++)
	{
		Coners_[i] = CarXY2ImageUV(Coners[i]);
	}
	for (int i = 0; i < 4; i++)
	{
		line(img, Coners_[i], Coners_[(i+1)%4], color,1);
	}

	for (int i = -1*m_nGridSide; i < m_nGridSide; i+=10000)
	{
		Point PtImg0 = CarXY2ImageUV(Point2f(i, 0));
		Point PtImg1(PtImg0.x, PtImg0.y+5);
		line(img, PtImg0, PtImg1, color, 1);
	}
	for (int i = -1*m_nGridBack; i < m_nGridFront; i+=10000)
	{
		Point PtImg0 = CarXY2ImageUV(Point2f(0, i));
		Point PtImg1(PtImg0.x+5, PtImg0.y);
		line(img, PtImg0, PtImg1, color, 1);
	}

}

int CSickLms511DataTransfer::InputRadar(vector<LIDAR_RADAR_OBJECT_INFO>& RadarAll, 
	LIDAR_RADAR_INFO& RadarData,
	Point2f T,
	double dHeading)
{
	double dSinAlpha = sin(dHeading/180.0*CV_PI);
	double dCosAlpha = cos(dHeading/180.0*CV_PI);
	unsigned int nContValid = 0;
	for (unsigned int i = 0; i < 64; i++)
	{
		if (!RadarData.Objects[i].bValid)
			continue;
		nContValid++;
		LIDAR_RADAR_OBJECT_INFO ObjT;
		ObjT.nObjectID = RadarData.Objects[i].nObjectID;
		ObjT.bValid = RadarData.Objects[i].bValid;
		ObjT.fObjLocX =  (dCosAlpha*RadarData.Objects[i].fObjLocX -  dSinAlpha*RadarData.Objects[i].fObjLocY)*1000.0 + T.x;
		ObjT.fObjLocY =  (dSinAlpha*RadarData.Objects[i].fObjLocX +  dCosAlpha*RadarData.Objects[i].fObjLocY)*1000.0 + T.y;
		ObjT.fObjSizeX = (dCosAlpha*RadarData.Objects[i].fObjSizeX - dSinAlpha*RadarData.Objects[i].fObjSizeY)*1000.0;
		ObjT.fObjSizeY = (dSinAlpha*RadarData.Objects[i].fObjSizeX + dCosAlpha*RadarData.Objects[i].fObjSizeY)*1000.0;
		RadarAll.push_back(ObjT);
	}

	return nContValid;
}
