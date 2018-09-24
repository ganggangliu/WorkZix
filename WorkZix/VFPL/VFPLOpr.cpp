#include "VFPLOpr.h"

CVFPLMapWriter::CVFPLMapWriter()
{
	memset(m_szFilePath,0,sizeof(m_szFilePath));
	m_nMapIndex = 0;
}

CVFPLMapWriter::~CVFPLMapWriter()
{
	m_fsMap.release();
}

void CVFPLMapWriter::Release()
{

}

void CVFPLMapWriter::Init(const char* pszPath, Mat FeatParam)
{
	sprintf(m_szFilePath,"%s\\VFPLMap.yml",pszPath);
	m_fsMap.open(m_szFilePath, CV_STORAGE_WRITE);
	m_fsMap << "FeaturePointsParam" << FeatParam;
}

long CVFPLMapWriter::WriteMap(Mat GpsData, Mat Pose, Mat CloudPts, Mat descriptors, vector<KeyPoint> keypoints)
{
	char szTitle[MAX_PATH] = {0};

	sprintf(szTitle,"GpsData%d",m_nMapIndex);
	m_fsMap << szTitle << GpsData; 

	sprintf(szTitle,"Pose%d",m_nMapIndex);
	m_fsMap << szTitle << Pose; 

	sprintf(szTitle,"CloudPts%d",m_nMapIndex);
	m_fsMap << szTitle << CloudPts;

	sprintf(szTitle,"descriptors%d",m_nMapIndex);
	m_fsMap << szTitle << descriptors;

	sprintf(szTitle,"keypoints%d",m_nMapIndex);
	write(m_fsMap,szTitle,keypoints);

	m_nMapIndex++;

	return m_nMapIndex;
}

CVFPLMapReader::CVFPLMapReader()
{

}

CVFPLMapReader::~CVFPLMapReader()
{

}

long CVFPLMapReader::ReadMap(const char* pszPath, LIBGUIOPENGL_HANDLE hHandle)
{
	sprintf(m_szFilePath,"%s\\VFPLMap.yml",pszPath);
	m_fsMap.open(m_szFilePath, CV_STORAGE_READ);
	if (!m_fsMap.isOpened())
	{
		cout << m_szFilePath << " Open failed!" << endl;
		return 0;
	}

	m_fsMap["FeaturePointsParam"] >> m_FeatPointParam;
	m_VecGpsData.clear();
	m_VecPose.clear();
	m_VecCloudPts.clear();
	m_VecDescriptors.clear();
	m_VecKeyPoints.clear();
	m_VecMileStone.clear();

	m_VecMileStone.push_back(0);
	for (int i = 0; i < 100000; i++)
	{
		char szTitle[MAX_PATH] = {0};
		Mat Temp;

		sprintf(szTitle,"GpsData%d",i);
		m_fsMap[szTitle] >> Temp;
		if (Temp.data == NULL)
			return i;
		m_VecGpsData.push_back(Temp);

		sprintf(szTitle,"Pose%d",i);
		m_fsMap[szTitle] >> Temp;
		if (Temp.data == NULL)
			return i;
		m_VecPose.push_back(Temp);

		sprintf(szTitle,"CloudPts%d",i);
		m_fsMap[szTitle] >> Temp;
		if (Temp.data == NULL)
			return i;
		m_VecCloudPts.push_back(Temp);

		sprintf(szTitle,"descriptors%d",i);
		m_fsMap[szTitle] >> Temp;
		if (Temp.data == NULL)
			return i;
		m_VecDescriptors.push_back(Temp);

		sprintf(szTitle,"keypoints%d",i);
		vector<KeyPoint> temp0;
		read(m_fsMap[szTitle],temp0);
// 		if (Temp.data == NULL)
// 			return i;
		m_VecKeyPoints.push_back(temp0);

		if (i!=0)
		{
			double dxd = m_VecPose[i-1].at<double>(0,3) - m_VecPose[i].at<double>(0,3);
			double dzd = m_VecPose[i-1].at<double>(2,3) - m_VecPose[i].at<double>(2,3);
			double dd = sqrt(pow(dxd,2)+pow(dzd,2));
			m_VecMileStone.push_back(m_VecMileStone.back()+dd);
		}

		if (hHandle != NULL)
		{
			hHandle->AddPoints(m_VecPose.back(), m_VecCloudPts.back());
		}
//		Sleep(10);
	}


	return 0;
}

CVFPLLocator::CVFPLLocator()
{
	m_hHandle = GetLibGuiOpenGL("LibGuiOpenGL_Params_VFPL.xml");
	m_PoseCur = Mat::eye(4,4,CV_64F);
	double m_dTimeBefor = 0;
	double m_dBaseHeading = 0	;
}

CVFPLLocator::~CVFPLLocator()
{

}

long CVFPLLocator::LoadMap(const char* pszPath)
{
	m_VFPLMap.ReadMap(pszPath,m_hHandle);
	cout << "FeatPointParam" << m_VFPLMap.m_FeatPointParam << endl;

	return 1;
}

long CVFPLLocator::InitLocator(CStereoParam ParamSte, CFeatureDetectParam ParamFeatDet)
{
	m_StereoOpr.Init(ParamSte);
	m_FeatPtOpr.Init(ParamFeatDet);
	m_nCurIndex = 0;
	return 1;
}

long CVFPLLocator::Predict(Mat Transition)
{
	if (m_VecPose.size() - m_VecMeasure.size() == 1)
	{
		m_VecMeasure.push_back(Mat());
		m_PoseCur = m_PoseCur*Transition.inv();
		m_VecPose.push_back(m_PoseCur);
		m_VecTrans.push_back(Transition);
		return 1;
	}
	if (m_VecPose.size() - m_VecMeasure.size() == 0)
	{
		m_PoseCur = m_PoseCur*Transition.inv();
		m_VecPose.push_back(m_PoseCur);
		m_VecTrans.push_back(Transition);
		return 1;
	}
	else
	{
		printf("Predict::Size of m_VecPose and m_VecMeasure dismatched!\n");
		getchar();
		return 1;
	}

	return 1;
}

long CVFPLLocator::Predict(double dTime, double dVe, double dVn)
{
	Mat xx = (Mat_<double>(1,1) << dVe);
	Mat yy = (Mat_<double>(1,1) << dVn);
	Mat vv,aa;
	cartToPolar(xx,yy,vv,aa,true);
	double dAngle = 90.f-aa.at<double>(0);
	if (m_VecPose.size() == 0)
	{
		m_dTimeBefor = dTime;
		m_dHeadingBefor = dAngle;
	}
	double dSpeed = sqrt(pow(dVe,2)+pow(dVn,2));
	double dDist = dSpeed*(dTime-m_dTimeBefor)*1000;
	if (m_VecPose.size() == 0)
		m_VecMileRecord.push_back(0);
	else
		m_VecMileRecord.push_back(m_VecMileRecord.back()+dDist);
	Mat rvec = (Mat_<double>(1,3) << 0,(dAngle-m_dHeadingBefor)/180.f*CV_PI,0);
	Mat R = Mat::eye(4,4,CV_64F);
	Mat R0;
	Rodrigues(rvec,R0);
	R0.copyTo(R(Range(0,3),Range(0,3)));
	R.at<double>(2,3) = dDist;
	Predict(R.inv());
//	cout << m_PoseCur << endl;
	m_hHandle->AddPoints(m_PoseCur);

	m_dTimeBefor = dTime;
	m_dHeadingBefor = dAngle;

	return 1;
}

long CVFPLLocator::Correct(Mat Measurment)
{
	if (m_VecPose.size() - m_VecMeasure.size() != 1)
	{
		printf("Correct::Size of m_VecPose and m_VecMeasure dismatched!\n");
		getchar();
		return 1;
	}
	m_VecMeasure.push_back(Measurment);
	Measurment.copyTo(m_PoseCur);
	Measurment.copyTo(m_VecPose.back());

	return 1;
}

Mat CVFPLLocator::Localization(Mat& img, double dTime, double dVe, double dVn, CProjectOpr& ProjMap)
{
	vector<KeyPoint> kpt;
	Mat dscpt;
	m_FeatPtOpr.GetFeatures(img,kpt,dscpt);
	Predict(dTime,dVe,dVn);
	for (int i = m_nCurIndex; i < m_VFPLMap.m_VecMileStone.size(); i++)
	{
		if (m_VFPLMap.m_VecMileStone[i]>=m_VecMileRecord.back())
		{
			m_nCurIndex = i;
			break;
		}
	}

	long nIndStart = m_nCurIndex-5;
	if (nIndStart<=0)
		nIndStart = 0;
	long nIndEnd = m_nCurIndex+5;
	if (nIndEnd>=m_VFPLMap.m_VecPose.size())
		nIndEnd = m_VFPLMap.m_VecPose.size()-1;

	long nMaxPtCont = -1;
	long nIndPtCont = -1;
	Mat mask_;
	vector<DMatch> matches_;
	for (int j = nIndStart; j < nIndEnd; j++)
	{
		vector<DMatch> matches;
		m_FeatPtOpr.KNNMatchCross(dscpt,m_VFPLMap.m_VecDescriptors[j],matches);
		Mat mask;
		PNP(matches,kpt,m_VFPLMap.m_VecCloudPts[j],mask);
		if (mask.rows >= nMaxPtCont)
		{
			nMaxPtCont = mask.rows;
			nIndPtCont = j;
			mask_ = mask;
			matches_ = matches;
		}
	}
	if (nMaxPtCont >= 20)
	{
		m_nCurIndex = nIndPtCont;
		m_VecMileRecord.back() = m_VFPLMap.m_VecMileStone[m_nCurIndex];
	}
	else
	{
		mask_.release();
	}

	Mat imgMap = imread(ProjMap.m_VecImgPathL[m_nCurIndex],0);
	vector<DMatch> matchT;
	for (int i = 0; i < mask_.rows; i++)
	{
		matchT.push_back(matches_[mask_.at<short>(i)]);
	}
	Mat imgout;
	drawMatches(img,kpt,imgMap,m_VFPLMap.m_VecKeyPoints[m_nCurIndex],matchT,imgout);
	
	printf("xxxxxxxxxxxxxxxx%d,%d\n",nMaxPtCont,nIndPtCont);

	return imgout;
}

long CVFPLLocator::Correct(double dLat, double dLon)
{

	return 1;
}

long CVFPLLocator::PNP(vector<DMatch>& match, vector<KeyPoint>& kpt, Mat& Cloud, Mat& valid)
{
	valid.release();
	vector<Point2f> VecPtTest;
	vector<Point3f> VecPtMap;
	for (int ii = 0; ii < match.size(); ii++)
	{
		Point2f pt2d;
		pt2d = kpt[match[ii].queryIdx].pt;
		VecPtTest.push_back(pt2d);
		Point3f ptT;
		ptT.x = Cloud.at<double>(match[ii].trainIdx,0);
		ptT.y = Cloud.at<double>(match[ii].trainIdx,1);
		ptT.z = Cloud.at<double>(match[ii].trainIdx,2);;
		VecPtMap.push_back(ptT);
	}
	Mat matPt0(VecPtTest.size(),2,CV_32FC1,VecPtTest.data());
	Mat matPt1(VecPtMap.size(),3,CV_32FC1,VecPtMap.data());
	Mat R,distCoeffs,rvec,tvec;
	if (VecPtTest.size()>=4)
	{
		solvePnPRansac(matPt1,matPt0,m_StereoOpr.GetCameraMat(),distCoeffs,rvec,tvec,false,100,8.0,100,valid,EPNP);	
	}
	else
	{
		rvec = Mat::eye(3,3,CV_64FC1);
		tvec = Mat::zeros(3,1,CV_64FC1);
		valid = Mat();
	}
// 	vector<DMatch> matchTemp;
// 	for (int ii = 0; ii < valid.rows; ii++)
// 	{
// 		matchTemp.push_back(match[valid.at<ULONG32>(ii,0)]);
// 	}
// 	match = matchTemp;

	return 1;
}

Mat CVFPLLocator::GetDist(double lat0, double long0, double lat1, double long1)
{
	Mat out = Mat::eye(4,4,CV_64F);
	double dZ = 111319.55*(lat1 - lat0)*1000.f;
	double dX = 111319.55*(long1 - long0)*cos((lat0)/180.f*CV_PI)*1000.f;
	out.at<double>(0,3) = dX;
	out.at<double>(2,3) = dZ;

	return out;
}

long CVFPLLocator::AddSinglePoint(Mat GpsData)
{
	Mat Pose = GetDist(
		m_VFPLMap.m_VecGpsData[0].at<double>(0,3),
		m_VFPLMap.m_VecGpsData[0].at<double>(0,4),
		GpsData.at<double>(0,3),
		GpsData.at<double>(0,4));

	m_hHandle->AddPoints(Pose,Mat());

	return 1;
}