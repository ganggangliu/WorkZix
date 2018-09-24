#include "FeatureDetectOpr.h"

tag_FeatureDetectParam::tag_FeatureDetectParam(const char* pszPath, int nImgH, int nImgW)
{
	if (pszPath == NULL)
	{
		sprintf(szPathIni,"%s\\FeatureDetect.ini",pszPath);
	}
	else
	{
		sprintf(szPathIni,"%s\\FeatureDetect.ini",pszPath);
	}
	nImgHeight = nImgH;
	nImgWidth = nImgW;
	dMaxMatchRate = 0.6;
	nMaxStereoDisparityX = 100;
	nMaxStereoDisparityY = 2;
	nMaxFeaturePtDist = 50000;
	nAlgorithmType = FEATURE_TYPE_USE_SURF;
	contrastThreshold = 0.1f;
	edgeThreshold = 30;
	hessianThreshold = 2000;
	nFeatures = 500;
	nAreaUmin = 0;
	nAreaUmax = nImgWidth;
	nAreaVmin = 0;
	nAreaVmax = nImgHeight;
}

long tag_FeatureDetectParam::LoadParam()
{
	// 		nCamCont = GetPrivateProfileIntA("StereoParam","nCamCont",2,psz);
	// 		char szTemp[MAX_PATH] = {0};
	// 		GetPrivateProfileStringA("StereoParam","LeftCamMac","",szTemp,MAX_PATH,psz);
	// 		strcpy(szMacLeft,szTemp);
	// 		GetPrivateProfileStringA("StereoParam","RightCamMac","",szTemp,MAX_PATH,psz);
	// 		strcpy(szMacRight,szTemp);
	// 		nInteTimeL = GetPrivateProfileIntA("StereoParam","ExposTimeLeft",1000,psz);
	return 1;
}

Mat tag_FeatureDetectParam::Param2Mat()
{
	Mat out = Mat::zeros(1,15,CV_64F);
	out.at<double>(0) = nImgHeight;
	out.at<double>(1) = nImgWidth;
	out.at<double>(2) = dMaxMatchRate;
	out.at<double>(3) = nMaxStereoDisparityX;
	out.at<double>(4) = nMaxStereoDisparityY;
	out.at<double>(5) = nMaxFeaturePtDist;
	out.at<double>(6) = nAlgorithmType;
	out.at<double>(7) = contrastThreshold;
	out.at<double>(8) = edgeThreshold;
	out.at<double>(9) =  hessianThreshold;
	out.at<double>(10) = nFeatures;
	out.at<double>(11) = nAreaUmin;
	out.at<double>(12) = nAreaUmax;
	out.at<double>(13) = nAreaVmin;
	out.at<double>(14) = nAreaVmax;

	return out;
}

CFeatureDetectOpr::CFeatureDetectOpr()
{
	return;
}
CFeatureDetectOpr::~CFeatureDetectOpr()
{
	return;
}
long CFeatureDetectOpr::Init(CFeatureDetectParam Param)
{	
	initModule_nonfree();
	m_Param = Param;
	m_nAlgorithm = m_Param.nAlgorithmType;
	m_contrastThreshold = m_Param.contrastThreshold;
	m_edgeThreshold = m_Param.edgeThreshold;
	m_hessianThreshold = m_Param.hessianThreshold;
	m_nFeatures = m_Param.nFeatures;

	if (m_nAlgorithm == FEATURE_TYPE_USE_SIFT)
	{
		m_FeatureOpr = Algorithm::create<Feature2D>("Feature2D.SIFT");
		m_FeatureOpr->set("contrastThreshold", m_contrastThreshold);
		m_FeatureOpr->set("edgeThreshold", m_edgeThreshold);
	}
	else if(m_nAlgorithm == FEATURE_TYPE_USE_SURF)
	{
		m_FeatureOpr = Algorithm::create<Feature2D>("Feature2D.SURF");
		m_FeatureOpr->set("hessianThreshold", m_hessianThreshold);
	}
	else
	{
		m_FeatureOpr = Algorithm::create<Feature2D>("Feature2D.ORB");
		m_FeatureOpr->set("nFeatures", m_nFeatures);
	}

	m_DetAreaMask = Mat::zeros(cvSize(m_Param.nImgWidth,m_Param.nImgHeight),CV_8U);
	m_DetAreaMask(Range(m_Param.nAreaVmin,m_Param.nAreaVmax),
		Range(m_Param.nAreaUmin,m_Param.nAreaUmax)) = 255;

	return 1;
};

long CFeatureDetectOpr::GetFeatures(Mat& img, vector<KeyPoint>& keypoints, Mat& descriptors)
{
	keypoints.clear();
	descriptors.release();
	(*m_FeatureOpr)(img, m_DetAreaMask, keypoints, descriptors);

	return 1;
}

long CFeatureDetectOpr::StereoFeatureMatch(Mat& MatchImage, Mat& image0, Mat& image1, vector<DMatch>& matches, vector<KeyPoint>& keypoints0, vector<KeyPoint>& keypoints1, Mat& descriptors0, Mat& descriptors1)
{
	matches.clear();
// 	(*m_FeatureOpr)((image0),noArray(), keypoints0, descriptors0);
// 	(*m_FeatureOpr)((image1),noArray(), keypoints1, descriptors1);
	GetFeatures(image0,keypoints0,descriptors0);
	GetFeatures(image1,keypoints1,descriptors1);
	vector<DMatch> matchesT;
	KNNMatch(descriptors0,descriptors1,matchesT);

	for (int i = 0; i < matchesT.size(); i++ )
	{
		Point2f ptL,ptR;
		ptL = keypoints0[matchesT[i].queryIdx].pt;
		ptR = keypoints1[matchesT[i].trainIdx].pt;
		if (abs(ptL.x - ptR.x) <= m_Param.nMaxStereoDisparityX && abs(ptL.y - ptR.y) <= m_Param.nMaxStereoDisparityY)
		{
			matchesT[i].imgIdx = ptL.x - ptR.x;
			matches.push_back(matchesT[i]);
		}
	}
	// 	Mat img_featureL,img_featureR;
	// 	drawKeypoints(image0,keypoints0,img_featureL,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	// 	drawKeypoints(image1,keypoints1,img_featureR,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	// 	imwrite("xxxL.bmp",img_featureL);
	// 	imwrite("xxxR.bmp",img_featureR);
	// 	Mat MatchImage;
	drawMatches(image0,keypoints0,image1,keypoints1,matches,MatchImage,CV_RGB(0,255,0)/*Scalar_<double>::all(-1)*/);
	// 	imwrite("xxx.bmp",img_match);
	// 	cvNamedWindow(psz,0);
	// 	imshow(psz,img_match);
	return 1;
}

long CFeatureDetectOpr::StereoFeatureMatchEx(Mat& MatchImage, Mat& image0, Mat& image1, vector<DMatch>& matches, vector<KeyPoint>& keypoints0, vector<KeyPoint>& keypoints1, Mat& descriptors0, Mat& descriptors1)
{
	matches.clear();
// 	(*FeatureOpr)(image0(Range(0,900),Range::all()), noArray(), keypoints0, descriptors0);
// 	(*FeatureOpr)(image1(Range(0,900),Range::all()), noArray(), keypoints1, descriptors1);
	GetFeatures(image0,keypoints0,descriptors0);
	GetFeatures(image1,keypoints1,descriptors1);
	BFMatcher matcher(NORM_L2);
	vector<vector<DMatch>> matchesT;
	matcher.knnMatch(descriptors0,descriptors1,matchesT,2);
	vector<vector<DMatch>>::iterator it;
	for (it = matchesT.begin(); it != matchesT.end(); )
	{
		it->pop_back();
		it++;
	}
	for (int i = 0; i < matchesT.size(); i++ )
	{
		Point2f ptL,ptR;
		ptL = keypoints0[matchesT[i][0].queryIdx].pt;
		ptR = keypoints1[matchesT[i][0].trainIdx].pt;
		if (abs(ptL.x - ptR.x) <= m_Param.nMaxStereoDisparityX && abs(ptL.y - ptR.y) <= m_Param.nMaxStereoDisparityY)
		{
			matchesT[i][0].imgIdx = ptL.x - ptR.x;
			matches.push_back(matchesT[i][0]);
		}

	}
	// 	Mat img_featureL,img_featureR;
	// 	drawKeypoints(image0,keypoints0,img_featureL,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	// 	drawKeypoints(image1,keypoints1,img_featureR,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	// 	imwrite("xxxL.bmp",img_featureL);
	// 	imwrite("xxxR.bmp",img_featureR);
	// 	Mat MatchImage;
	drawMatches(image0,keypoints0,image1,keypoints1,matches,MatchImage,CV_RGB(0,255,0)/*Scalar_<double>::all(-1)*/);
	// 	imwrite("xxx.bmp",img_match);
	// 	cvNamedWindow(psz,0);
	// 	imshow(psz,img_match);
	return 1;
}

long CFeatureDetectOpr::StereoFeatureMatchByLine(Mat& MatchImage, Mat& image0, Mat& image1, vector<DMatch>& matches, vector<KeyPoint>& keypoints0, vector<KeyPoint>& keypoints1, Mat& descriptors0, Mat& descriptors1)
{
	matches.clear();
	GetFeatures(image0,keypoints0,descriptors0);
	GetFeatures(image1,keypoints1,descriptors1);

	vector<DMatch> matchesT;
	for (int i = 0; i < keypoints0.size(); i++)
	{
		for (int j = 0; j < keypoints1.size(); j++)
		{
			if ((keypoints0[i].pt.x - keypoints1[j].pt.x) <= m_Param.nMaxStereoDisparityX
				&& (keypoints0[i].pt.x - keypoints1[j].pt.x) > 0
				&& abs(keypoints0[i].pt.y - keypoints1[j].pt.y) <= m_Param.nMaxStereoDisparityY)
			{
				double dDist = cvNorm(&CvMat(descriptors0.row(i)),&CvMat(descriptors1.row(j)));
				if (dDist <400)
				{
					DMatch matchd;
					matchd.queryIdx = i;
					matchd.trainIdx = j;
					matchesT.push_back(matchd);
				}
			}
		}
	}

	matches = matchesT;
	drawMatches(image0,  keypoints0, image1,  keypoints1,  matches, MatchImage);

	return 1;
}

long CFeatureDetectOpr::KNNMatch(Mat& descriptors0, Mat& descriptors1, vector<DMatch>& matches)
{
	matches.clear();
	BFMatcher matcher(NORM_L2);
	vector<vector<DMatch>> matchesT;
	matcher.knnMatch(descriptors0,descriptors1,matchesT,2);
	vector<vector<DMatch>>::iterator it;
	for (it = matchesT.begin(); it != matchesT.end(); )
	{
		if ((*it)[0].distance > (*it)[1].distance*m_Param.dMaxMatchRate)
			it = matchesT.erase(it);
		else
		{
			it->pop_back();
			it++;
		}
	}
	for (int i = 0; i < matchesT.size(); i++)
	{
		matches.push_back(matchesT[i][0]);
	}

	return 1;
}

long CFeatureDetectOpr::KNNMatchCross(Mat& descriptors0, Mat& descriptors1, vector<DMatch>& matches)
{
	matches.clear();
	BFMatcher matcher(NORM_L2,true);
	vector<vector<DMatch>> matchesT;
	matcher.knnMatch(descriptors0,descriptors1,matchesT,1);
	vector<vector<DMatch>>::iterator it;
	for (it = matchesT.begin(); it != matchesT.end(); )
	{
		if (it->size()>0)
			matches.push_back((*it)[0]);
		it++;
	}

	return 1;
}

long CFeatureDetectOpr::FilterKeyPoints(vector<DMatch>& matches, vector<KeyPoint>& keypoints0, vector<KeyPoint>& keypoints1, Mat& descriptors0, Mat& descriptors1)
{
	vector<DMatch> _matches;
	vector<KeyPoint> _keypoints0;
	vector<KeyPoint> _keypoints1;
	Mat _descriptors0 = Mat::zeros(matches.size(),descriptors0.cols,descriptors0.type());
	Mat _descriptors1 = Mat::zeros(matches.size(),descriptors1.cols,descriptors1.type());
	for (int i = 0; i < matches.size(); i++)
	{
		_matches.push_back(matches[i]);
		_matches[i].queryIdx = i;
		_matches[i].trainIdx = i;
		_keypoints0.push_back(keypoints0[matches[i].queryIdx]);
		_keypoints1.push_back(keypoints1[matches[i].trainIdx]);
		descriptors0.row(matches[i].queryIdx).copyTo(_descriptors0.row(i));
		descriptors1.row(matches[i].trainIdx).copyTo(_descriptors1.row(i));
	}

	matches = _matches;
	keypoints0 = _keypoints0;
	keypoints1 = _keypoints1;
	descriptors0 = _descriptors0;
	descriptors1 = _descriptors1;

	return 1;
}
long CFeatureDetectOpr::IsInVector(vector<DMatch>& matches, int nIndex)
{
	for (int i = 0; i < matches.size(); i++)
	{
		if (matches[i].queryIdx == nIndex)
		{
			return i;
		}
	}

	return -1;
}