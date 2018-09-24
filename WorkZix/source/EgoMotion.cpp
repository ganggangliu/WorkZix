#include "EgoMotion.h"

tag_EgoMotionParam::tag_EgoMotionParam()
{
	nMaxFeatPointDist = 30000;
}

long tag_EgoMotionParam::LoadParam()
{
	return 1;
}

CEgoMotion::CEgoMotion()
{
	return;
}

CEgoMotion::~CEgoMotion()
{
	return;
}

long CEgoMotion::Init(CEgoMotionParam Param, CStereoParam ParamSte, CFeatureDetectParam ParamFeatDef)
{
	m_Param = Param;
	m_StereoOpr.Init(ParamSte);
	m_FeatDetOpr.Init(ParamFeatDef);
	m_nCont = 0;
	return 1;
}

long CEgoMotion::IterationStep(Mat& imageL, Mat& imageR, Mat& rvec, Mat& tvec, Mat& MatchImage, vector<Point3f>& rVecList, vector<Point3f>& tVecList )
{
	rvec.release();
	tvec.release();
	MatchImage.release();
	Mat Temp;
	if (m_nCont == 0)
	{
		m_nCont++;
		imageL.copyTo(m_imageLN);
		imageR.copyTo(m_imageRN);
		m_FeatDetOpr.StereoFeatureMatch(Temp,m_imageLN,m_imageRN,m_matchesN,m_kpLN,m_kpRN,m_dpsLN,m_dpsRN);
		return m_nCont;
	}

	m_nCont++;
	m_imageLN.copyTo(m_imageLP);
	m_imageRN.copyTo(m_imageRP);
	m_matchesP = m_matchesN;
	m_kpLP = m_kpLN;
	m_kpRP = m_kpRN;
	m_dpsLN.copyTo(m_dpsLP);
	m_dpsRN.copyTo(m_dpsRP);
	imageL.copyTo(m_imageLN);
	imageR.copyTo(m_imageRN);
	m_matchesN.clear();
	m_kpLN.clear();
	m_kpRN.clear();
	m_dpsLN.release();
	m_dpsRN.release();
	m_FeatDetOpr.StereoFeatureMatch(Temp,m_imageLN,m_imageRN,m_matchesN,m_kpLN,m_kpRN,m_dpsLN,m_dpsRN);
	vector<DMatch> matchesT,matchesT1,matchesT2;
	m_FeatDetOpr.KNNMatch(m_dpsLN,m_dpsLP,matchesT);
	vector<DMatch>::iterator it;
	for (it = matchesT.begin(); it != matchesT.end(); it++)
	{
		long nP = m_FeatDetOpr.IsInVector(m_matchesP,it->trainIdx);
		long nN = m_FeatDetOpr.IsInVector(m_matchesN,it->queryIdx);
		if ( nP != -1 && nN != -1)
		{
			DMatch matchT;
			matchT.queryIdx = nN;
			matchT.trainIdx = nP;
			matchesT1.push_back(matchT);
		}
	}
	vector<StereoMatchPoint> StePoint;
	StereoMatchPoint ptT;
	vector<Point3f> P;
	vector<Point3f> N;
	vector<Point2f> UndistoredPoints;
	Point3f pt1;
	for (int i = 0; i < matchesT1.size(); i++)
	{
		ptT.ptLP = m_kpLP[m_matchesP[matchesT1[i].trainIdx].queryIdx].pt;
		ptT.ptRP = m_kpRP[m_matchesP[matchesT1[i].trainIdx].trainIdx].pt;
		ptT.ptLN = m_kpLN[m_matchesN[matchesT1[i].queryIdx].queryIdx].pt;
		ptT.ptRN = m_kpRN[m_matchesN[matchesT1[i].queryIdx].trainIdx].pt;
		ptT.ptP3D = m_StereoOpr.SteImgPt2XYZ(ptT.ptLP,ptT.ptRP);
		ptT.ptN3D = m_StereoOpr.SteImgPt2XYZ(ptT.ptLN,ptT.ptRN);
		if (ptT.ptN3D.z <= m_Param.nMaxFeatPointDist)
		{
			StePoint.push_back(ptT);
			pt1.x = ptT.ptP3D.x;
			pt1.y = ptT.ptP3D.y;
			pt1.z = ptT.ptP3D.z;
			P.push_back(pt1);
			pt1.x = ptT.ptN3D.x;
			pt1.y = ptT.ptN3D.y;
			pt1.z = ptT.ptN3D.z;
			N.push_back(pt1);
			UndistoredPoints.push_back(ptT.ptLN);
			matchesT2.push_back(matchesT1[i]);
		}
	}

	Mat matP(P.size(),3,CV_32FC1,P.data());
	Mat matN(N.size(),3,CV_32FC1,N.data());
	// 	saveXYZ1("matP.csv",matP);
	// 	saveXYZ1("matN.csv",matN);
	Mat R,distCoeffs,inliers;
	Mat matUndistortedPoints(UndistoredPoints.size(),2,CV_32FC1,UndistoredPoints.data());
	if (P.size()>=4)
	{
		solvePnPRansac(matP,matUndistortedPoints,m_StereoOpr.GetCameraMat(),distCoeffs,rvec,tvec,false,100,5,100,inliers,EPNP);	
	}
	else
	{
		rvec = Mat::eye(3,3,CV_64FC1);
		tvec = Mat::zeros(3,1,CV_64FC1);
	}
	Rodrigues(rvec,R);
	// 	saveXYZ1("matUndistortedPoints.csv",matUndistortedPoints);
	// 	saveXYZ1("cameraMatrix.csv",m_cameraMatrix);
	// 	saveXYZ1("inliers.csv",inliers);
	// 	saveXYZ1("R.csv",R);
	// 	saveXYZ1("rvec.csv",rvec);
	// 	saveXYZ1("tvec.csv",tvec);

	Mat img_match;
//	drawMatches(m_imageLN,m_kpLN,m_imageLP,m_kpLP,matchesT,img_match);
	DrawOF(m_imageLN,m_kpLN,m_imageLP,m_kpLP,matchesT,img_match,inliers);
	char szDispText[256] = {0};
	// 	sprintf(szDispText,"R:%.2f, %.2f, %.2f  T:%.2f, %.2f, %.2f\n",
	// 		rvec.at<double>(0,0)/3.141592654*180,
	// 		rvec.at<double>(1,0)/3.141592654*180,
	// 		rvec.at<double>(2,0)/3.141592654*180,
	// 		tvec.at<double>(0,0),
	// 		tvec.at<double>(1,0),
	// 		tvec.at<double>(2,0));
	//	putText(img_match,szDispText,cvPoint(100,100),0,1,RGB(0,0,255),2);
	Point3f ptTemp;
	ptTemp.x = rvec.at<double>(0,0)/3.141592654*180;
	ptTemp.y = rvec.at<double>(1,0)/3.141592654*180;
	ptTemp.z = rvec.at<double>(2,0)/3.141592654*180;
	rVecList.push_back(ptTemp);
	sprintf(szDispText,"%.2f",rvec.at<double>(0,0)/3.141592654*180);
	putText(img_match,szDispText,cvPoint(10,100),0,1,RGB(0,0,255),2);
	sprintf(szDispText,"%.2f",rvec.at<double>(1,0)/3.141592654*180);
	putText(img_match,szDispText,cvPoint(10,130),0,1,RGB(0,0,255),2);
	sprintf(szDispText,"%.2f",rvec.at<double>(2,0)/3.141592654*180);
	putText(img_match,szDispText,cvPoint(10,160),0,1,RGB(0,0,255),2);

	ptTemp.x = tvec.at<double>(0,0);
	ptTemp.y = tvec.at<double>(1,0);
	ptTemp.z = tvec.at<double>(2,0);
	tVecList.push_back(ptTemp);
	sprintf(szDispText,"%.2f",tvec.at<double>(0,0));
	putText(img_match,szDispText,cvPoint(10,200),0,1,RGB(0,0,255),2);
	sprintf(szDispText,"%.2f",tvec.at<double>(1,0));
	putText(img_match,szDispText,cvPoint(10,230),0,1,RGB(0,0,255),2);
	sprintf(szDispText,"%.2f",tvec.at<double>(2,0));
	putText(img_match,szDispText,cvPoint(10,260),0,1,RGB(0,0,255),2);

	// 	cvNamedWindow("matchPN",0);
	// 	imshow("matchPN",img_match);
	img_match.copyTo(MatchImage);

	return m_nCont;
}

long CEgoMotion::DrawOF(Mat& img1, vector<KeyPoint>& keypoints1,Mat& img2, vector<KeyPoint>& keypoints2,
	vector<DMatch>& matches1to2, Mat& outImg, Mat Valid)
{
	img1.copyTo(outImg);
	for (int i = 0; i < Valid.rows; i++)
	{
		Point2f pt1,pt2;
		pt1 = keypoints1[matches1to2[Valid.at<long>(i,0)].queryIdx].pt;
		pt2 = keypoints2[matches1to2[Valid.at<long>(i,0)].trainIdx].pt;
		line(outImg,pt1,pt2,CV_RGB(0,255,0),1);
	}

	return 1;
}
