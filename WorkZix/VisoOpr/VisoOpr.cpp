#include "viso_mono.h"
#include "viso_stereo.h"
#include <stdint.h>
#include "VisoOpr.h"
#include "reconstruction.h"
using namespace std;

CVisoOpr::CVisoOpr(CVisoOprParam Param, double dScale, VISO_TYPE type)
{
	m_Param = Param;
	m_Scale = dScale;
	m_Pose = Mat::eye(4,4,CV_64F);
	m_Motion = Mat::eye(4,4,CV_64F);
	if (type == STEREO)
	{
		VisualOdometryStereo::parameters ParamS;
		ParamS.calib.f = Param.f*m_Scale;
		ParamS.calib.cu = Param.cu*m_Scale;
		ParamS.calib.cv = Param.cv*m_Scale;
		ParamS.base = abs(Param.base);
		ParamS.match.match_radius = Param.match_radius*m_Scale;
		m_pVisualOdometry = new VisualOdometryStereo(ParamS);
	}
	else
	{
		VisualOdometryMono::parameters ParamM;
		ParamM.calib.f = Param.f*m_Scale;
		ParamM.calib.cu = Param.cu*m_Scale;
		ParamM.calib.cv = Param.cv*m_Scale;
		ParamM.height = Param.height;
		ParamM.pitch = Param.pitch;
		ParamM.match.match_radius = Param.match_radius*m_Scale;
		m_pVisualOdometry = new VisualOdometryMono(ParamM);
	}
	m_pReconstruction = new Reconstruction();
	((Reconstruction*)m_pReconstruction)->setCalibration(Param.f, Param.cu, Param.cv);

	m_nPtsContCur = 0;

	return ;
}

CVisoOpr::~CVisoOpr()
{
	if (m_pVisualOdometry)
	{
		delete m_pVisualOdometry;
		m_pVisualOdometry = NULL;
	}
}

bool CVisoOpr::process (Mat ImgL,Mat ImgR)
{
	Mat left_img;
	Mat right_img;
	if (ImgL.data == NULL || ImgR.data == NULL)
	{
		m_Motion.release();
		return false;
	}
	if (ImgL.channels() == 3 || ImgR.channels() == 3)
	{
		m_Motion.release();
		return false;
	}
	if (m_Scale != 1.f)
	{
		resize(ImgL,left_img,cvSize(ImgL.cols*m_Scale,ImgL.rows*m_Scale));
		resize(ImgR,right_img,cvSize(ImgR.cols*m_Scale,ImgR.rows*m_Scale));
	}
	else
	{
		left_img = ImgL;
		right_img = ImgR;
	}

	int32_t width  = left_img.cols;
	int32_t height = left_img.rows;
	int32_t dims[] = {width,height,width};

	uint8_t* left_img_data  = left_img.data;
	uint8_t* right_img_data  = right_img.data;

	VisualOdometryStereo* pVisualOdometry = (VisualOdometryStereo*) m_pVisualOdometry;
	Matrix MatTemp;
	if(pVisualOdometry->process(left_img_data,right_img_data,dims))
	{
		MatTemp = pVisualOdometry->getMotion();
		Mat out(4,4,CV_64F);
		MatTemp.getData((FLOAT_M*)out.data);

		if (m_Param.doForceHorizon)
		{
			Mat Motion;
			out.copyTo(Motion);
			Mat vecr;
			Rodrigues(Motion(Range(0,3),Range(0,3)),vecr);
			Mat vecr_ = (Mat_<double>(1,3) << 0,vecr.at<double>(1,0),0);
			Mat R;
			Rodrigues(vecr_,R);
			R.copyTo(Motion(Range(0,3),Range(0,3)));
			Motion.at<double>(1,3) = 0;
			Motion.copyTo(m_Motion);
			m_Pose = m_Pose * m_Motion.inv();
			for (int i = 0; i < m_Motion.rows; i++)
			{
				for (int j = 0; j < m_Motion.cols; j++)
				{
					MatTemp.setVal((m_Motion.at<double>(i,j)),i,j,i,j);
				}
			}
		}
		else
		{
			m_Motion = out;
			m_Pose = m_Pose * m_Motion.inv();
		}

		double num_matches = pVisualOdometry->getNumberOfMatches();
		double num_inliers = pVisualOdometry->getNumberOfInliers();
		cout << ", Matches: " << num_matches;
		cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
	}
	else
	{
		cout << " ... failed!" << endl;
		m_Motion.release();
		return false;
	}
	
	if(m_Param.doReconstruction)
	{	
		long nCont0 = ((Reconstruction*)m_pReconstruction)->getPoints().size();
		((Reconstruction*)m_pReconstruction)->update(pVisualOdometry->getMatches(),MatTemp);
		long nCont1 = ((Reconstruction*)m_pReconstruction)->getPoints().size();
		m_nPtsContCur = nCont1-nCont0;
	}

	return true;
}

long CVisoOpr::DrawOpticFlow(Mat& img, long nType)
{
	if (img.channels() == 1)
	{
		cvtColor(img,img,CV_GRAY2RGB);
	}

	VisualOdometry* pVisualOdometry = (VisualOdometry*) m_pVisualOdometry;
	vector<Matcher::p_match> matches = pVisualOdometry->getMatches();
	vector<int32_t> valid = pVisualOdometry->getInlierIndices();
	long nInd = 0;
	for (int i = 0; i < matches.size(); i++)
	{

		Point2d pt0 = cvPoint(matches[i].u1p/m_Scale,matches[i].v1p/m_Scale);
		Point2d pt1 = cvPoint(matches[i].u1c/m_Scale,matches[i].v1c/m_Scale);
		if (i == valid[nInd] && nInd < valid.size()-1)
		{
			line(img,pt0,pt1,CV_RGB(0,0,255),2);
			nInd++;
		}
		else if (nType == 1)
		{
			line(img,pt0,pt1,CV_RGB(255,0,0),2);
		}
	}

	return 1;
}

bool CVisoOpr::process (Mat ImgL)
{
	Mat left_img;
	if (ImgL.data == NULL)
	{
		return false;
	}
	if (ImgL.channels() == 3)
	{
		cvtColor(ImgL,left_img,COLOR_RGB2GRAY);
	}
	else
	{
		left_img = ImgL;
	}

	int32_t width  = left_img.cols;
	int32_t height = left_img.rows;
	int32_t dims[] = {width,height,width};

	uint8_t* left_img_data  = left_img.data;

	VisualOdometryMono* pVisualOdometry = (VisualOdometryMono*) m_pVisualOdometry;
	Matrix MatTemp;
	if(pVisualOdometry->process(left_img_data,dims))
	{
		MatTemp = pVisualOdometry->getMotion();
		Mat out(4,4,CV_64F);
		MatTemp.getData((FLOAT_M*)out.data);

		if (m_Param.doForceHorizon)
		{
			Mat Motion;
			out.copyTo(Motion);
			Mat vecr;
			Rodrigues(Motion(Range(0,3),Range(0,3)),vecr);
			Mat vecr_ = (Mat_<double>(1,3) << 0,vecr.at<double>(1,0),0);
			Mat R;
			Rodrigues(vecr_,R);
			R.copyTo(Motion(Range(0,3),Range(0,3)));
			Motion.at<double>(1,3) = 0;
			Motion.copyTo(m_Motion);
			m_Pose = m_Pose * m_Motion.inv();
			for (int i = 0; i < m_Motion.rows; i++)
			{
				for (int j = 0; j < m_Motion.cols; j++)
				{
					MatTemp.setVal((m_Motion.at<double>(i,j)),i,j,i,j);
				}
			}
		}
		else
		{
			m_Motion = out;
			m_Pose = m_Pose * m_Motion.inv();
		}

		double num_matches = pVisualOdometry->getNumberOfMatches();
		double num_inliers = pVisualOdometry->getNumberOfInliers();
		cout << ", Matches: " << num_matches;
		cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
	}
	else
	{
		m_Motion.release();
		return false;
	}

	if(m_Param.doReconstruction)
	{	
		long nCont0 = ((Reconstruction*)m_pReconstruction)->getPoints().size();
		((Reconstruction*)m_pReconstruction)->update(pVisualOdometry->getMatches(),MatTemp,1,1);
		long nCont1 = ((Reconstruction*)m_pReconstruction)->getPoints().size();
		m_nPtsContCur = nCont1-nCont0;
		cout << "nCont0" << nCont0 << "    /     " << m_nPtsContCur << endl;
	}

	return true;
}

Mat CVisoOpr::getMotion()
{
	return m_Motion;
}

Mat CVisoOpr::getPose()
{
	return m_Pose;
}

// Mat CVisoOpr::GetMonoCloudPoints()
// {
// 	VisualOdometry* pVisualOdometry = (VisualOdometry*) m_pVisualOdometry;
// 	vector<Matcher::p_match> matches = pVisualOdometry->getMatches();
// 	vector<int32_t> valid = pVisualOdometry->getInlierIndices();
// 	long nInd = 0;
// 	vector<Matcher::p_match> matchesGood,matchesBad;
// 	for (int i = 0; i < matches.size(); i++)
// 	{
// 		Point2d pt0 = cvPoint(matches[i].u1p,matches[i].v1p);
// 		Point2d pt1 = cvPoint(matches[i].u1c,matches[i].v1c);
// 		if (i == valid[nInd] && nInd < valid.size()-1)
// 		{
// 			line(ImgDisp,pt0,pt1,CV_RGB(0,255,0),1);
// 			matchesGood.push_back(matches[i]);
// 			nInd++;
// 		}
// 		else
// 		{
// 			line(ImgDisp,pt0,pt1,CV_RGB(255,0,0),1);
// 		}
// 
// 	}
// }

long CVisoOpr::DrawStereoMatch(Mat& imgMatch, Mat& imgL, Mat& imgR, vector<DMatch>& matchesT, vector<KeyPoint>& kpt0, vector<KeyPoint>& kpt1)
{
	matchesT.clear();
	kpt0.clear();
	kpt1.clear();
	VisualOdometry* pVisualOdometry = (VisualOdometry*) m_pVisualOdometry;
	vector<Matcher::p_match> matches = pVisualOdometry->getMatches();
	for (int i = 0; i < matches.size(); i++)
	{
		KeyPoint kpT;
		DMatch match0;
		kpT.pt = cvPoint(matches[i].u1c,matches[i].v1c);
		kpt0.push_back(kpT);
		kpT.pt = cvPoint(matches[i].u2c,matches[i].v2c);
		kpt1.push_back(kpT);
		match0.queryIdx = i;
		match0.trainIdx = i;
		matchesT.push_back(match0);
	}

	drawMatches(imgL,kpt0,imgR,kpt1,matchesT,imgMatch,CV_RGB(0,255,0));

	return 1;
}

Mat CVisoOpr::GetReconstructionPoints()
{
	vector<Reconstruction::point3d> pt0 = ((Reconstruction*)m_pReconstruction)->getPoints();
	long nPtContTotal = pt0.size();
	Mat pt1(m_nPtsContCur,3,CV_64F);
	for (int i = 0; i < m_nPtsContCur; i++)
	{
		pt1.at<double>(i,0) = pt0[nPtContTotal-1-i].x;
		pt1.at<double>(i,1) = pt0[nPtContTotal-1-i].y;
		pt1.at<double>(i,2) = pt0[nPtContTotal-1-i].z;
	}
	return pt1;
}