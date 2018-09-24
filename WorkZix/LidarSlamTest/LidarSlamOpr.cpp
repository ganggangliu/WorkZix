#include "LidarSlamOpr.h"

CLidarSlamOpr::CLidarSlamOpr()
{
	m_nGridFront = 20000;
	m_nGridBack = 10000;
	m_nGridSide = 20000;
	m_nGridSize = 100;

	m_dGateWidth = 2000.0;
	m_nFilterLen = 10;
	m_dVehicleWidthHalf = 1000.0;
}

CLidarSlamOpr::~CLidarSlamOpr()
{

}

void CLidarSlamOpr::Init(char* pszTickPatten, double dDegreePerTick)
{
	strcpy(m_szTickFile , pszTickPatten);
	m_dDegreePerTick = dDegreePerTick;

	m_dVehicleLen = 3000.0;

	FileStorage fs(m_szTickFile,CV_STORAGE_READ);
	Mat LidarPattan;
	fs["LidarTicks"] >> LidarPattan;
	m_nTickStore.resize(LidarPattan.cols);
	for (int i = 0; i < LidarPattan.cols; i++)
	{
		m_nTickStore[i] = LidarPattan.at<int>(0,i);
	}
	m_dDistAlign.resize(m_nTickStore.size(), 200000.0);
	m_dAngleAlign.resize(m_nTickStore.size());
	fs.release();
	for (int i = 0; i < m_dAngleAlign.size(); i++)
	{
		m_dAngleAlign[i] = m_nTickStore[i] * m_dDegreePerTick;
	}
}

int CLidarSlamOpr::AlignLidarData(double* pDist, double* pAngle, int nDataLen)
{
	

	return 1;
}

int CLidarSlamOpr::AlignLidarDataIbeo(LCM_IBEO_CLOUD_POINTS* pIbeoPoints)
{
	LCM_IBEO_CLOUD_POINTS IbeoPointsT;
	for (int i = 0; i < pIbeoPoints->IbeoPoints.size(); i++)
	{
		if (pIbeoPoints->IbeoPoints[i].layer == 1 && pIbeoPoints->IbeoPoints[i].echo == 0)
		{
			IbeoPointsT.IbeoPoints.push_back(pIbeoPoints->IbeoPoints[i]);
		}
	}

	for (int i = 0; i < m_dDistAlign.size(); i++)
	{
		m_dDistAlign[i] = 200000.0;
	}
	int nFillInd = 0;
	for (int i = 0; i < IbeoPointsT.IbeoPoints.size(); i++)
	{
		int nTick = IbeoPointsT.IbeoPoints[i].angle;
		double nDist = IbeoPointsT.IbeoPoints[i].distance * 10.0;
		for (int j = nFillInd; j < m_nTickStore.size(); j++)
		{
			if (m_nTickStore[j] == nTick)
			{
				m_dAngleAlign[j] = m_nTickStore[j] * m_dDegreePerTick;
				if (nDist >= 200000.0)
				{
					m_dDistAlign[j] = 200000.0;
				}
				else
				{
					m_dDistAlign[j] = nDist;
				}
				nFillInd = j + 1;
				break;
			}
		}
	}

	m_Points = Polar2CartPoints(m_dDistAlign, m_dAngleAlign);

	FilterPoints(m_Points,m_PointsFilter,m_Gates);

	m_imgPoints = DrawPoints(m_Points);

	m_imgPolyArea = DrawPolyArea(m_Points);
	int nVehicleWideHalf = m_dVehicleWidthHalf/m_nGridSize;
	Mat element = getStructuringElement(MORPH_ELLIPSE,Size(nVehicleWideHalf*2+1, nVehicleWideHalf*2+1),
		Point(nVehicleWideHalf,nVehicleWideHalf));
	erode( m_imgPolyArea, m_imgPolyAreaErode, element );
	//imwrite("m_imgPolyAreaErode.bmp",m_imgPolyAreaErode);
	//imwrite("m_imgPolyArea.bmp",m_imgPolyArea);
	dilate( m_imgPoints, m_imgPointsErode, element );

	m_imgPointsFilter = DrawPoints(m_PointsFilter);

	m_imgPolyAreaFilter = DrawPolyArea(m_PointsFilter);

	m_imgGate = DrawGates(m_Gates);

	return 1;
}

Mat CLidarSlamOpr::Polar2CartPoints(vector<double>& dDistAlign, vector<double>& dAngleAlign)
{
	vector<Point3d> VecPt;
	for (int i = 0; i < m_dDistAlign.size(); i++)
	{
		Point3d ptT;
		ptT.x = sin(dAngleAlign[i]/180.f*CV_PI)*dDistAlign[i];
		ptT.y = cos(dAngleAlign[i]/180.f*CV_PI)*dDistAlign[i];
		ptT.z = 0;
		VecPt.push_back(ptT);
	}
	Mat matT = Mat(VecPt.size(),3,CV_64F,VecPt.data());
	return matT.clone();
}

Mat CLidarSlamOpr::DrawPoints(Mat& Points)
{
	Mat out = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8U);
	for (int i = 0; i < Points.rows; i++)
	{
		double X = Points.at<double>(i,0);
		double Z = Points.at<double>(i,1);
		double X_ = X + m_nGridSide;
		double Z_ = Z + m_nGridBack;
		double XX = floor(X_/m_nGridSize);
		double ZZ = floor(Z_/m_nGridSize);
		if (X>=-1*m_nGridSide && X<=m_nGridSide && Z<=m_nGridFront && Z>=-1.f*m_nGridBack)
		{
			out.at<uchar>(ZZ,XX) = 255;
		}
	}
	flip(out,out,0);
	
	return out;
}
Mat CLidarSlamOpr::DrawPolyArea(Mat& Points)
{
	vector<Point2i> VecPtImg;
	for (int i = 0; i < Points.rows; i++)
	{
		double X = Points.at<double>(i,0);
		double Z = Points.at<double>(i,1);
		double X_ = X + m_nGridSide;
		double Z_ = Z + m_nGridBack;
		double XX = floor(X_/m_nGridSize);
		double ZZ = floor(Z_/m_nGridSize);
		VecPtImg.push_back(Point2i(XX,ZZ));
	}
	double X = m_nGridSide;
	double Z = m_nGridBack;
	double XX = floor(X/m_nGridSize);
	double ZZ = floor(Z/m_nGridSize);
	VecPtImg.push_back(Point2i(XX,ZZ));

	Mat out = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8U);
	Point2i* pPt = VecPtImg.data();
	int PtLen = VecPtImg.size();
	fillPoly(out,(const Point2i**)&pPt,(const int*)&PtLen,1,Scalar(255,255,255));
	polylines(out,(const Point2i**)&pPt,(const int*)&PtLen,1,true,Scalar(0,0,0));
	flip(out,out,0);

	return out;
}
Mat CLidarSlamOpr::DrawGates(vector<vector<Point2d>>& Gates)
{
	Mat out = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8U);
	for (int i = 0; i < Gates.size(); i++)
	{
		Point2d pt0 = Gates[i][0];
		Point2d pt1 = Gates[i][1];
		double X = pt0.x;
		double Z = pt0.y;
		double X_ = X + m_nGridSide;
		double Z_ = Z + m_nGridBack;
		double XX = floor(X_/m_nGridSize);
		double ZZ = floor(Z_/m_nGridSize);
		Point2d pt0_(XX,ZZ);
		X = pt1.x;
		Z = pt1.y;
		X_ = X + m_nGridSide;
		Z_ = Z + m_nGridBack;
		XX = floor(X_/m_nGridSize);
		ZZ = floor(Z_/m_nGridSize);
		Point2d pt1_(XX,ZZ);
		line(out,pt0_,pt1_,CV_RGB(255,255,255));
	}
	flip(out,out,0);

	return out;
}

Mat CLidarSlamOpr::GetTrackImage(Mat& Area, double dWheelAngle)
{
	double dCurve = 180.0*m_dVehicleLen/CV_PI/dWheelAngle;
	if (dCurve >= 1e8)
	{
		dCurve = 1e8;
	}
	if (dCurve <= -1e8)
	{
		dCurve = -1e8;
	}
	Mat Track = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8U);
	Point2d PtC(m_nGridSide/m_nGridSize, m_nGridFront/m_nGridSize);
	Track.at<uchar>(PtC) = 255;

	vector<Point2d> TrackPoints;
	TrackPoints.push_back(PtC);
	vector<double> TrackDist;
	TrackDist.push_back(0.0);

	Point2d Anchor((m_nGridSide+dCurve)/m_nGridSize,m_nGridFront/m_nGridSize);
	double dR = abs(dCurve/(double)m_nGridSize);
	int nDirect = 2;
	double dDirect = 0.0;

//	imwrite("Area.bmp",Area);

//	while(IterationTrack(Area,Anchor,dR,Track,PtC,nDirect))
	if (Anchor.x <= 0 ||
		Anchor.y <= 0 ||
		Anchor.x >= Track.cols-1 ||
		Anchor.y >= Track.rows-1)
	{
	}
	else
	{
		Track.at<uchar>(Anchor) = 255;
	}
	while(IterationTrackEx(Area,Anchor,dR,Track,PtC,dDirect))
	{
		//double dDist = sqrt(pow((double)(PtC.x-TrackPoints.back().x),2)+pow((double)(PtC.y-TrackPoints.back().y),2));
		//TrackPoints.push_back(PtC);
		//TrackDist.push_back(TrackDist.back()+dDist);
		cvNamedWindow("Track",1);
		imshow("Track",Track);
		waitKey();
	}

	cout << "Dist:" << TrackDist.back() << endl;

//	imwrite("Track.bmp",Track);

// 	cvNamedWindow("Track",1);
// 	imshow("Track",Mat(Area == 0)+Track);
// 	waitKey();

	return Mat(Area == 0)+Track;
}

Mat CLidarSlamOpr::GetTrackImageEx(Mat& Area, double dWheelAngle, vector<Point2d>& VecTrack)
{
	VecTrack.clear();
	double dCurve = 180.0*m_dVehicleLen/CV_PI/dWheelAngle;
	if (dCurve >= 1e8)
	{
		dCurve = 1e8;
	}
	if (dCurve <= -1e8)
	{
		dCurve = -1e8;
	}
	Mat Track = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8U);
	Point2d PtC(0.0, 0.0);
	Track.at<uchar>(Car2Image(PtC)) = 255;

	vector<Point2d> TrackPoints;
	TrackPoints.push_back(PtC);
	vector<double> TrackDist;
	TrackDist.push_back(0.0);

	Point2d Anchor(dCurve,0.0);
	double dR = dCurve;
	double dDirect = 0.0;

	//	imwrite("Area.bmp",Area);

	//	while(IterationTrack(Area,Anchor,dR,Track,PtC,nDirect))
	Point2i AnchorImg = Car2Image(Anchor);
	if (AnchorImg.x <= 0 ||
		AnchorImg.y <= 0 ||
		AnchorImg.x >= Track.cols-1 ||
		AnchorImg.y >= Track.rows-1)
	{
	}
	else
	{
		Track.at<uchar>(AnchorImg) = 255;
	}
	while(IterationTrackEx(Area,Anchor,dR,Track,PtC,dDirect))
	{
		VecTrack.push_back(PtC);
		//double dDist = sqrt(pow((double)(PtC.x-TrackPoints.back().x),2)+pow((double)(PtC.y-TrackPoints.back().y),2));
		//TrackPoints.push_back(PtC);
		//TrackDist.push_back(TrackDist.back()+dDist);
		//cvNamedWindow("Track",1);
		//imshow("Track",Track);
		//waitKey();
	}

	cout << "Dist:" << TrackDist.back() << endl;

	//	imwrite("Track.bmp",Track);

	// 	cvNamedWindow("Track",1);
	// 	imshow("Track",Mat(Area == 0)+Track);
	// 	waitKey();

	return (Area|Track);
}

Mat CLidarSlamOpr::GetBestTrackEx(Mat& Area, double& dWheelAngle)
{
	vector<vector<Point2d>> VecVecTrack;
	vector<Mat> VecArea_Track;
	vector<int> VecAng;
	vector<int> VecAngTable;
 	int nStartInd = dWheelAngle - 5;
	int nEndInd = dWheelAngle + 5;
	if (nStartInd <= -45)
	{
		nStartInd = -45;
		nEndInd = -35;
	}
	if (nEndInd >= 45)
	{
		nStartInd = 35;
		nEndInd = 45;
	}
	for (int i = nStartInd; i <= nEndInd; i++)
	{
		VecAngTable.push_back(i);
	}
	VecAngTable.push_back(0);
	VecAngTable.push_back(-45);
	VecAngTable.push_back(45);
	for (int i = 0; i < VecAngTable.size(); i++)
	{
		vector<Point2d> VecTrack;
		Mat Area_Track = GetTrackImageEx(Area,VecAngTable[i],VecTrack);
		VecVecTrack.push_back(VecTrack);
		VecArea_Track.push_back(Area_Track);
		VecAng.push_back(VecAngTable[i]);
	}

	int nMaxInd = 0;
	double nMaxCont = INT_MIN;
	for (int i = 0; i < VecVecTrack.size(); i++)
	{
		if ((int)VecVecTrack[i].size() > nMaxCont )
		{
			nMaxCont = (int)VecVecTrack[i].size();
			nMaxInd = i;
		}
	}

	dWheelAngle = VecAng[nMaxInd];

	return VecArea_Track[nMaxInd];
}

//int CLidarSlamOpr::Gridding()
//{
//	vector<Point2i> VecPtImg;
//	m_Lidar = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8U);
//	for (int i = 0; i < m_Points.rows; i++)
//	{
//		double X = m_Points.at<double>(i,0);
//		double Z = m_Points.at<double>(i,1);
//		double X_ = X + m_nGridSide;
//		double Z_ = Z + m_nGridBack;
//		double XX = floor(X_/m_nGridSize);
//		double ZZ = floor(Z_/m_nGridSize);
//		VecPtImg.push_back(Point2i(XX,ZZ));
//		if (X>=-1*m_nGridSide && X<=m_nGridSide && Z<=m_nGridFront && Z>=-1.f*m_nGridBack)
//		{
//			m_Lidar.at<uchar>(ZZ,XX) = 255;
//		}
//	}
//	flip(m_Lidar,m_Lidar,0);
//	//	cvFlip(&CvMat(Lidar1),NULL,0);
//	//	Lidar = Grey2FateColor(Lidar1);
//	double X = m_nGridSide;
//	double Z = m_nGridBack;
//	double XX = floor(X/m_nGridSize);
//	double ZZ = floor(Z/m_nGridSize);
//	VecPtImg.push_back(Point2i(XX,ZZ));
//
//	m_PolyArea = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8U);
//	Point2i* pPt = VecPtImg.data();
//	int PtLen = VecPtImg.size();
//	fillPoly(m_PolyArea,(const Point2i**)&pPt,(const int*)&PtLen,1,Scalar(255,255,255));
//	flip(m_PolyArea,m_PolyArea,0);
//
//	return 1;
//}

int CLidarSlamOpr::FilterPoints(Mat& Points_in, Mat& Points_out, vector<vector<Point2d>>& Gates_o)
{
	Gates_o.clear();
	vector<Point3d> VecPt;
	for (int i = 0; i < Points_in.rows; i++)
	{
		double dMinDist = DBL_MAX;
		int dMinInd = -1;
		int j = i+1;
		for (; (j < i + m_nFilterLen) && (j < Points_in.rows); j++)
		{
			Point2d pt_i(m_Points.at<double>(i,0),m_Points.at<double>(i,1));
			Point2d pt_j(m_Points.at<double>(j,0),m_Points.at<double>(j,1));
			double dDist = sqrt(pow(pt_i.x-pt_j.x,2) + pow(pt_i.y-pt_j.y,2));
			if (dDist <= dMinDist)
			{
				dMinDist = dDist;
				dMinInd = j;
			}
		}
		if (j >= Points_in.rows)
		{
			break;
		}
		Point3d ptT(m_Points.at<double>(i,0),m_Points.at<double>(i,1),0);
		VecPt.push_back(ptT);
		if (dMinDist >= m_dGateWidth)
		{
			vector<Point2d> tt;
			Point2d pt_i(m_Points.at<double>(i,0),m_Points.at<double>(i,1));
			Point2d pt_j(m_Points.at<double>(dMinInd,0),m_Points.at<double>(dMinInd,1));
			tt.push_back(pt_i);
			tt.push_back(pt_j);
			Gates_o.push_back(tt);
		}
		i = dMinInd;
	}

	Mat(VecPt.size(), 3, CV_64F, VecPt.data()).copyTo(Points_out);

	return 1;
}

void CLidarSlamOpr::TickRecord(int nTick)
{
	bool bIsExist = false;
	for (int i = 0; i < m_nTickStore.size(); i++)
	{
		if (m_nTickStore[i] == nTick)
		{
			bIsExist = true;
		}
	}
	if (!bIsExist)
	{
		m_nTickStore.push_back(nTick);
	}
}

void CLidarSlamOpr::TestTick(LCM_IBEO_CLOUD_POINTS* pIbeoPoints)
{
	for (int i = 0; i < pIbeoPoints->IbeoPoints.size(); i++)
	{
		if (pIbeoPoints->IbeoPoints[i].layer == 1)
		{
			TickRecord(int(pIbeoPoints->IbeoPoints[i].angle));
		}
	}
}

void CLidarSlamOpr::SaveTick()
{
	for (int i = 0; i < m_nTickStore.size(); i++)
	{
		for (int j = i; j < m_nTickStore.size(); j++)
		{
			if (m_nTickStore[i]<m_nTickStore[j])
			{
				int temp = m_nTickStore[j];
				m_nTickStore[j] = m_nTickStore[i];
				m_nTickStore[i] = temp;
			}
		}
	}

	Mat aaa = Mat::zeros(1,m_nTickStore.size(),CV_32S);
	for (int i = 0; i < m_nTickStore.size(); i++)
	{
		aaa.at<int>(0,i) = m_nTickStore[i];
	}
	FileStorage fs("LidarTicks.yml",CV_STORAGE_WRITE);
	fs << "LidarTicks" << aaa;
	fs.release();
}

int CLidarSlamOpr::IterationTrack(Mat& Area_i, Point2i Anchor_i, double dDistPix_i, Mat& Tracking_io, Point2i& PtCur_io, int& nDirect_io)
{
//	cout << PtCur_io << endl;
	if (Tracking_io.at<uchar>(PtCur_io.y,PtCur_io.x) == 0 || 
		nDirect_io == 0 ||
		PtCur_io.x <= 0 ||
		PtCur_io.y <= 0 ||
		PtCur_io.x >= Tracking_io.cols-1 ||
		PtCur_io.y >= Tracking_io.rows-1 || 
		Area_i.at<uchar>(PtCur_io.y,PtCur_io.x) == 0 ||
		nDirect_io == 8)
	{
		return 0;
	}

	Vec<double,3> VecDist(-1.0,-1.0,-1.0);
	Vec<int,3> VecLabel(0,0,0);
	Vec<int,3> VecPixU(0,0,0);
	Vec<int,3> VecPixV(0,0,0);
	if (nDirect_io == 1)
	{
		VecLabel = Vec<int,3>(4,1,2);
	}
	else if (nDirect_io == 2)
	{
		VecLabel = Vec<int,3>(1,2,3);
	}
	else if (nDirect_io == 3)
	{
		VecLabel = Vec<int,3>(2,3,6);
	}
	else if (nDirect_io == 6)
	{
		VecLabel = Vec<int,3>(3,6,9);
	}
	else if (nDirect_io == 9)
	{
		VecLabel = Vec<int,3>(6,9,8);
	}
	else if (nDirect_io == 8)
	{
		VecLabel = Vec<int,3>(9,8,7);
	}
	else if (nDirect_io == 7)
	{
		VecLabel = Vec<int,3>(8,7,4);
	}
	else if (nDirect_io == 4)
	{
		VecLabel = Vec<int,3>(7,4,1);
	}
	
	for (int i = 0; i < 3; i++)
	{
		if (VecLabel[i] == 1)
		{
			VecPixU[i] = PtCur_io.x-1;
			VecPixV[i] = PtCur_io.y-1;
		}
		else if (VecLabel[i] == 2)
		{
			VecPixU[i] = PtCur_io.x;
			VecPixV[i] = PtCur_io.y-1;
		}
		else if (VecLabel[i] == 3)
		{
			VecPixU[i] = PtCur_io.x+1;
			VecPixV[i] = PtCur_io.y-1;
		}
		else if (VecLabel[i] == 4)
		{
			VecPixU[i] = PtCur_io.x-1;
			VecPixV[i] = PtCur_io.y;
		}
		else if (VecLabel[i] == 5)
		{
			VecPixU[i] = PtCur_io.x;
			VecPixV[i] = PtCur_io.y;
		}
		else if (VecLabel[i] == 6)
		{
			VecPixU[i] = PtCur_io.x+1;
			VecPixV[i] = PtCur_io.y;
		}
		else if (VecLabel[i] == 7)
		{
			VecPixU[i] = PtCur_io.x-1;
			VecPixV[i] = PtCur_io.y+1;
		}
		else if (VecLabel[i] == 8)
		{
			VecPixU[i] = PtCur_io.x;
			VecPixV[i] = PtCur_io.y+1;
		}
		else if (VecLabel[i] == 9)
		{
			VecPixU[i] = PtCur_io.x+1;
			VecPixV[i] = PtCur_io.y+1;
		}
	}

	double dDistMin = DBL_MAX;
	int dIndMin = -1;
	for (int i = 0; i < 3; i++)
	{
		double dDist = sqrt(pow((double)(VecPixU[i]-Anchor_i.x),2)+pow((double)(VecPixV[i]-Anchor_i.y),2));
		VecDist[i] = abs(dDist - dDistPix_i);
		if (VecDist[i] <= dDistMin)
		{
			dDistMin = VecDist[i];
			dIndMin = i;
		}
	}

	Mat M3_3 = Tracking_io(Range(PtCur_io.y-1,PtCur_io.y+2),Range(PtCur_io.x-1,PtCur_io.x+2));
	int nNewInd = VecLabel[dIndMin];
	int nNewPixU = VecPixU[dIndMin];
	int nNewPixV = VecPixV[dIndMin];
//	cout << (nNewInd-1)/3 << "    " << (nNewInd-1)%3 << endl;
	M3_3.at<uchar>((nNewInd-1)/3,(nNewInd-1)%3) = 255;
	PtCur_io = Point2i(nNewPixU,nNewPixV);
	nDirect_io = nNewInd;

	return 1;
}

int CLidarSlamOpr::IterationTrackEx(Mat& Area_i, Point2d Anchor_i, double dDistPix_i, Mat& Tracking_io, Point2d& PtCur_io, double& dDirect_io)
{
	Point2i PtCur_io_ = Car2Image(PtCur_io);
	if (PtCur_io_.x <= 0 ||
		PtCur_io_.y <= 0 ||
		PtCur_io_.x >= Tracking_io.cols-1 ||
		PtCur_io_.y >= Tracking_io.rows-1)
	{
		return 0;
	}
	if (Tracking_io.at<uchar>(PtCur_io_) == 0 ||
		Area_i.at<uchar>(PtCur_io_) != 0)
	{
		return 0;
	}

	double dAngStep = 500.0/(CV_PI*dDistPix_i)*180.0;

	Point2d pt0(Anchor_i.x,Anchor_i.y);
	Point2d pt1(PtCur_io.x,PtCur_io.y);
	Point2d pt0_pt1 = pt1 - pt0;
	Mat xx = (Mat_<double>(1,1) << pt0_pt1.x);
	Mat yy = (Mat_<double>(1,1) << pt0_pt1.y);
	Mat magnitude_,angle_;
	cartToPolar(xx,yy,magnitude_,angle_,true);
// 	cout << "xx" << xx << endl;
// 	cout << "yy" << yy << endl;
 //	cout << "angle_" << angle_ << endl;
	angle_.at<double>(0,0) -= dAngStep;
	dDirect_io = angle_.at<double>(0,0);

	if (dDistPix_i<0 && dDirect_io>=90.0)
	{
		return 0;
	}
	if (dDistPix_i>=0 && dDirect_io<=90.0)
	{
		return 0;
	}

	polarToCart(magnitude_,angle_,xx,yy,true);

	//cout << "xxt" << xx << endl;
	//cout << "yyt" << yy << endl;
	//cout << "angle_t" << angle_ << endl;

	Point2d pt2(xx.at<double>(0,0),yy.at<double>(0,0));
	pt2 += pt0;

	Point2i pt2_ = Car2Image(pt2);
	if (pt2_.x <= 0 ||
		pt2_.y <= 0 ||
		pt2_.x >= Tracking_io.cols-1 ||
		pt2_.y >= Tracking_io.rows-1)
	{
		return 0;
	}

	PtCur_io = pt2;

	Tracking_io.at<uchar>(pt2_) = 255;

	return 1;
}

Point2i CLidarSlamOpr::Car2Image(Point2d PtCar)
{
	return Point2i((m_nGridSide+(PtCar.x + m_nGridSize/2))/m_nGridSize,(m_nGridFront-(PtCar.y + m_nGridSize/2))/m_nGridSize);
}