#include "LidarDataProcess.h"

CLidarDataProcess::CLidarDataProcess()
{
    m_nGridFront = 30000;
	m_nGridBack = 20000;
	m_nGridSide = 20000;
	m_nGridSize = 100;
}

CLidarDataProcess::~CLidarDataProcess()
{
	
}

void CLidarDataProcess::Init(CLidarDataProcessParam& Param)
{
	m_Param = Param;
}

cv::Mat CLidarDataProcess::Grid(cv::Mat CloudPoint)
{
    cv::Mat out = cv::Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8UC1);
	for (int i = 0; i < CloudPoint.rows; i++)
	{
        cv::Point2d pt = CarXY2ImageUV(cv::Point2d(CloudPoint.at<float>(i,0),CloudPoint.at<float>(i,1)));
        if(pt.x < 0 || pt.x >= out.cols || pt.y < 0 || pt.y >= out.rows)
        {
            continue;
        }
        out.at<uchar>(pt) = 255;
//		circle(out,pt,2,Scalar(255,255,255),-1);
	}

	int nVehicleWideHalf = m_Param.dVehicleWidthHalf/m_nGridSize;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(nVehicleWideHalf*2+1, nVehicleWideHalf*2+1),
        cv::Point(nVehicleWideHalf,nVehicleWideHalf));
    cv::Mat out_;
	dilate( out, out_, element );

	return out_;
}


cv::Point2d CLidarDataProcess::CarXY2ImageUV(cv::Point2d CarXY)
{
    return cv::Point2i((m_nGridSide+(CarXY.x + m_nGridSize/2))/m_nGridSize,(m_nGridFront-(CarXY.y + m_nGridSize/2))/m_nGridSize);;
}

int CLidarDataProcess::GetBestTrackEx(cv::Mat& Area, double& dWheelAngle, std::vector<cv::Point2d>& Track, cv::Mat& TrackImg)
{
    std::vector<std::vector<cv::Point2d> > VecVecTrack;
    std::vector<cv::Mat> VecArea_Track;
    std::vector<int> VecAng;
    std::vector<int> VecAngTable;
    int nSearchAngleStep = 5;
    int nSearchAngleResolution = 10;     //0.1degree
    int nMinAngle = -450;               //0.1degree
    int nMaxAngle = 450;                //0.1degree
    //0.1degree
    int nWheelAngle = round((dWheelAngle*10.0)/nSearchAngleResolution) * nSearchAngleResolution;
    int nStartInd = nWheelAngle - nSearchAngleStep * nSearchAngleResolution;
    int nEndInd = nWheelAngle + nSearchAngleStep * nSearchAngleResolution;
    if (nStartInd <= nMinAngle)
	{
        nStartInd = nMinAngle;
        nEndInd = nMinAngle + nSearchAngleStep * nSearchAngleResolution;
	}
    if (nEndInd >= nMaxAngle)
	{
        nStartInd = nMaxAngle - nSearchAngleStep * nSearchAngleResolution;
        nEndInd = nMaxAngle;
	}
    for (int i = 0; nStartInd+i*nSearchAngleResolution  <= nEndInd; i++)
	{
        VecAngTable.push_back(nStartInd+i*nSearchAngleResolution);
	}
	for (int i = 0; i < VecAngTable.size(); i++)
	{
        std::vector<cv::Point2d> VecTrack;
        cv::Mat Area_Track = GetTrackImageEx(Area,VecAngTable[i]/10.0,VecTrack);
		VecVecTrack.push_back(VecTrack);
		VecArea_Track.push_back(Area_Track);
		VecAng.push_back(VecAngTable[i]);
	}

    std::vector<std::vector<cv::Point2d> > VecVecTrack_;
    std::vector<cv::Mat> VecArea_Track_;
    std::vector<int> VecAng_;
    std::vector<int> VecAngTable_;
    VecAngTable_.push_back(0);
    for (int i = 0; i < VecAngTable_.size(); i++)
    {
        std::vector<cv::Point2d> VecTrack;
        cv::Mat Area_Track = GetTrackImageEx(Area,VecAngTable_[i]/10.0,VecTrack);
        VecVecTrack_.push_back(VecTrack);
        VecArea_Track_.push_back(Area_Track);
        VecAng_.push_back(VecAngTable_[i]);
    }

    int nDrect = 0;
    int nSel = 0;
//    printf("Forward dist:%d\n", VecVecTrack_[0].size()/2);
    if(VecVecTrack_[0].size() >= m_Param.dPathChangeDist*2)
    {
        if(VecAng[nSearchAngleStep] < 0)
        {
            nDrect = 1;
        }
        else if(VecAng[nSearchAngleStep] > 0)
        {
            nDrect = -1;
        }
        else
        {
            nDrect = 0;
        }
        nSel = nSearchAngleStep + GetStableDirect(nDrect);
        Track = VecVecTrack[nSel];
        dWheelAngle = VecAng[nSel]/10.0;
        VecArea_Track[nSel].copyTo(TrackImg);
        return 1;
    }

    int nMaxInd = 0;
    int nMaxCont = INT_MIN;
    for (int i = 0; i < VecVecTrack.size(); i++)
    {
        if ((int)VecVecTrack[i].size() > nMaxCont )
        {
            nMaxCont = (int)VecVecTrack[i].size();
            nMaxInd = i;
        }
    }
//    printf("nMaxInd:%d\n", nMaxInd);

    if (VecVecTrack[nMaxInd].size() >= Track.size() * m_Param.dPathChangeToleratePer)
    {
        if(nMaxInd < nSearchAngleStep)
        {
            nDrect = -1;
        }
        else if(nMaxInd > nSearchAngleStep)
        {
            nDrect = 1;
        }
        else
        {
            nDrect = 0;
        }
        nMaxInd = nSearchAngleStep + GetStableDirect(nDrect);
		Track = VecVecTrack[nMaxInd];
        dWheelAngle = VecAng[nMaxInd]/10.0;
		VecArea_Track[nMaxInd].copyTo(TrackImg);
		return 1;
    }
    else
    {
        nDrect = 0;
        nMaxInd = nSearchAngleStep + GetStableDirect(nDrect);
        Track = VecVecTrack[nMaxInd];
        dWheelAngle = VecAng[nMaxInd]/10.0;
        VecArea_Track[nMaxInd].copyTo(TrackImg);
        return 1;
    }

	return 0;
}

cv::Mat CLidarDataProcess::GetTrackImageEx(cv::Mat& Area, double dWheelAngle, std::vector<cv::Point2d>& VecTrack)
{
	VecTrack.clear();
	double dCurve = 180.0*m_Param.dVehicleLen/CV_PI/dWheelAngle;
	if (dCurve >= 1e8)
	{
		dCurve = 1e8;
	}
	if (dCurve <= -1e8)
	{
		dCurve = -1e8;
	}
    cv::Mat Track = cv::Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8U);
    cv::Point2d PtC(0.0, 0.0);
	Track.at<uchar>(Car2Image(PtC)) = 255;

    std::vector<cv::Point2d> TrackPoints;
	TrackPoints.push_back(PtC);
    std::vector<double> TrackDist;
	TrackDist.push_back(0.0);

    cv::Point2d Anchor(dCurve,0.0);
	double dR = dCurve;
	double dDirect = 0.0;

    cv::Point2i AnchorImg = Car2Image(Anchor);
	if (AnchorImg.x <= 0 ||
		AnchorImg.y <= 0 ||
		AnchorImg.x >= Track.cols-1 ||
		AnchorImg.y >= Track.rows-1)
	{
        int nnnnn = 0;
	}
	else
	{
		Track.at<uchar>(AnchorImg) = 255;
	}
	while(IterationTrackEx(Area,Anchor,dR,Track,PtC,dDirect))
	{
		VecTrack.push_back(PtC);
	}

//    std::cout << "Dist:" << TrackDist.back() << std::endl;

	return (Area|Track);
}

cv::Point2i CLidarDataProcess::Car2Image(cv::Point2d PtCar)
{
    return cv::Point2i((m_nGridSide+(PtCar.x + m_nGridSize/2))/m_nGridSize,(m_nGridFront-(PtCar.y + m_nGridSize/2))/m_nGridSize);
}

int CLidarDataProcess::IterationTrackEx(cv::Mat& Area_i, cv::Point2d Anchor_i, double dDistPix_i, cv::Mat& Tracking_io, cv::Point2d& PtCur_io, double& dDirect_io)
{
    cv::Point2i PtCur_io_ = Car2Image(PtCur_io);
	if (PtCur_io_.x <= 0 ||
		PtCur_io_.y <= 0 ||
		PtCur_io_.x >= Tracking_io.cols-1 ||
		PtCur_io_.y >= Tracking_io.rows-1)
	{
		return 0;
	}
    if (Tracking_io.at<uchar>(PtCur_io_) == 0)
	{
        return 0;
    }
    if (Area_i.at<uchar>(PtCur_io_) != 0)
    {
        return 0;
    }

	double dAngStep = 500.0/(CV_PI*dDistPix_i)*180.0;

    cv::Point2d pt0(Anchor_i.x,Anchor_i.y);
    cv::Point2d pt1(PtCur_io.x,PtCur_io.y);
    cv::Point2d pt0_pt1 = pt1 - pt0;
    cv::Mat xx = (cv::Mat_<double>(1,1) << pt0_pt1.x);
    cv::Mat yy = (cv::Mat_<double>(1,1) << pt0_pt1.y);
    cv::Mat magnitude_,angle_;
	cartToPolar(xx,yy,magnitude_,angle_,true);

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

    cv::Point2d pt2(xx.at<double>(0,0),yy.at<double>(0,0));
	pt2 += pt0;

    cv::Point2i pt2_ = Car2Image(pt2);
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

int CLidarDataProcess::GetStableDirect(int nDirect)
{
    m_StableStatistic.push_front(nDirect);
    while(m_StableStatistic.size() > m_Param.nStableCont)
    {
        m_StableStatistic.pop_back();
    }
    int nAccDecrease = 0;
    int nAccHoldOn = 0;
    int nAccIncrease = 0;
    std::list<int>::const_iterator its = m_StableStatistic.begin();
    std::list<int>::const_iterator ite = m_StableStatistic.end();
    for(; its != ite; its++)
    {
        int nDirectRef = *its;
        if(nDirectRef == -1)
        {
            nAccDecrease++;
        }
        else if(nDirectRef == 0)
        {
            nAccHoldOn++;
        }
        else if(nDirectRef == 1)
        {
            nAccIncrease++;
        }
        else
        {
            printf("Funciton: _GetStableDirect_ Invalid! \n" );
        }
    }
    if(nAccHoldOn >= m_Param.nStableValid)
    {
        return 0;
    }
    if(nAccDecrease >= m_Param.nStableValid)
    {
        return -1;
    }
    if(nAccIncrease >= m_Param.nStableValid)
    {
        return 1;
    }

    return 0;
}
