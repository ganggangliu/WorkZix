#include "BaslerProcessor.h"
#include "Other.h"

using namespace std;
using namespace cv;
using namespace cv::gpu;

void CBaslerProcessor::Init(CBaslerProParam Param)
{
	m_Param = Param;
	m_MaskRect = Rect(800*m_Param.dIgnorAreaPer, 600*m_Param.dIgnorAreaPer, 
		800*(1-2*m_Param.dIgnorAreaPer), 600*(1-2*m_Param.dIgnorAreaPer));
	m_Mask = Mat::zeros(600, 800, CV_8U);
	m_Mask(m_MaskRect) = 255;
	m_d_Mask.upload(m_Mask);

	CRawDataReaderParam BaslerParam;
	BaslerParam.nImgPattern = CV_BayerBG2GRAY;
	m_Reader.Init(BaslerParam);

    m_pOrbDet = new ORB_GPU(m_Param.nOrbCont, 1.2, 1/*8*/, 31/*31*/,
        1, 2, ORB::FAST_SCORE/*ORB::HARRIS_SCORE*/, 31/*31*/);
	m_pOrbDet->setFastParams(m_Param.dOrbDetThreshold);
}

int CBaslerProcessor::Process(string szProPath)
{
	m_ProjPath = szProPath;
	if (m_ProjPath.back() != '\\' && m_ProjPath.back() != '/')
	{
		m_ProjPath += "/";
	}
	m_LeftPath = m_ProjPath + "left/";
	m_RightPath = m_ProjPath + "right/";
	m_UbloxPath = m_ProjPath + "ublox/";
	m_ResultSavePath = m_ProjPath + "result/";
	int nRt = makeDir(m_ResultSavePath);

	m_UbloxOPr.ReadLog(m_UbloxPath);

	printf("Process left data.......\n");
	ProcessSingle(m_LeftPath, m_ResultSavePath+"left.txt");

	CRawDataReaderParam BaslerParam;
	BaslerParam.nImgPattern = CV_BayerBG2GRAY;
	m_Reader.Init(BaslerParam);

	printf("Process right data.......\n");
	ProcessSingle(m_RightPath, m_ResultSavePath+"right.txt");

	return 1;
}

int CBaslerProcessor::ProcessSingle(std::string szDataPath, std::string szSavePath)
{
 	if (m_Reader.Open(szDataPath) < 0)
 	{
 		return -1;
 	}

	int nInd = 0;
	GpuMat ImageGpu;
	GpuMat d_prePts;
	GpuMat d_preDes;
	vector<GpuMat> d_preDesCollection;
	vector<KeyPoint> preKeyPoint;
	GpuMat d_curPts;
	GpuMat d_curDes;
	vector<KeyPoint> curKeyPoint;

	vector<Mat> VecMat;
	if (m_Param.nRunMode == 0)
	{
		for (unsigned int i = 0; i < m_Param.nPreLoadMaxCont; i++)
		{
			Mat Image;
			if (m_Reader.GetFrameByIndex(i,Image,0,0) < 0)
			{
				break;
			}
			VecMat.push_back(Image);
			if (i%1000 == 0)
			{
				printf("Read frame: %d!\n", i);
			}
		}
		printf("Read finish!\n");
	}

	GpuMat trainIdx, imgIdx, distance ,mask_collection;
	Mat preImage;

    BruteForceMatcher_GPU_base matcher(BruteForceMatcher_GPU_base::HammingDist);
	vector< vector<DMatch> > matches;
	Mat Image;
	vector<Point2f> motionHist;
	motionHist.reserve(1000000);
	vector<DMatch> good_matches, valid_matches;
	vector<Point2f> optStart, optEnd;
	vector<int> optState;
	vector<float> optDist;
	ofstream fs_track;
	fs_track.open(szSavePath.c_str(), ios::out);
	bool bIsRun = true;
	uint64_t t0 = getCurrentTime_ms();
	uint64_t t1 = getCurrentTime_ms();
//	CvVideoWriter* pVidWritor = 0;
//	pVidWritor = cvCreateVideoWriter("out.avi",CV_FOURCC('D', 'I', 'V', 'X'),20,cvSize(1600,600),true);
	while(1)
	{
		if (m_Param.nRunMode >= 1)
		{
			int64_t nRt = m_Reader.GetFrameByIndex(nInd,Image,0,0);
			if (nRt < 0)
			{
				break;
			}
			if (nInd%100 == 0)
			{
				printf("%d\n", nInd);
			}	
		}
		else
		{
			Image = VecMat[nInd];
		}

		ImageGpu.upload(Image);
		(*m_pOrbDet)(ImageGpu, m_d_Mask, d_curPts, d_curDes);
		m_pOrbDet->downloadKeyPoints(d_curPts, curKeyPoint);
		matcher.knnMatch(d_curDes, d_preDes, matches, 1);
		Point2f motion;
		if (motionHist.size() <= 0)
		{
			motion = Point2f(0.f, 0.f);
		}
		else
		{
            motion = getMotion(preKeyPoint, curKeyPoint, matches, optStart, optEnd, optDist, optState, motionHist.back());
		}
		if (motion == BAD_MOTION_)
		{
			break;
			printf("BAD_MOTION_:%d\n",nInd);
			motionHist.push_back(motionHist.back());
			getchar();
		}
		else
		{
			motionHist.push_back(motion);
		}
		fs_track << nInd << "," << m_Reader.GetTimeByInd(nInd) << "," << motion.x << "," << motion.y << "\n";
		fs_track.flush();

		if (m_Param.nRunMode == 2)
		{
			Mat matDisp = drawOrbMatches(Image, optStart, optEnd, optDist, optState, motion);
			char szDisp[1024];
			float dSpeed = sqrt(pow(motion.x,2) + pow(motion.y,2))/3/1000*500;
			float dSpeedUblox = 0.0;
			CUbloxData UbloxData;
			if (m_UbloxOPr.GetDataByTime(m_Reader.GetTimeByInd(nInd), UbloxData) > 0)
			{
				dSpeedUblox = UbloxData.ngSpeed/1000.0;
				sprintf(szDisp, "%06d, %.2f, %.2f, %.2f/%.2fm/s",
					nInd, motion.x, motion.y, dSpeed, dSpeedUblox);
			}
			else
			{
				sprintf(szDisp, "%06d, %.2f, %.2f, %.2f/xxm/s",
					nInd, motion.x, motion.y, dSpeed);
			}
			putText(matDisp, szDisp, Point(0,25), 0, 1, CV_RGB(255,0,0), 2);
			Mat matDisp_ = drawOrbMatchesEx(preImage, Image, optStart, optEnd, optDist, optState, motion);
			putText(matDisp_, szDisp, Point(0,25), 0, 1, CV_RGB(255,0,0), 2);
			cvNamedWindow("Frame0", 0);
			imshow("Frame0", matDisp);
			cvNamedWindow("Frame1", 0);
			imshow("Frame1", matDisp_);
			int nKey = -1;
			if (bIsRun)
			{
				nKey = waitKey(1);
			}
			else
			{
				nKey = waitKey();
			}
			if (nKey == 32)
			{
				bIsRun = !bIsRun;
//				cvReleaseVideoWriter(&pVidWritor);
			}
			Image.copyTo(preImage);
//			cvWriteFrame(pVidWritor,&IplImage(matDisp_)); 
		}

		d_curPts.copyTo(d_prePts);
		d_curDes.copyTo(d_preDes);
		preKeyPoint = curKeyPoint;

		nInd++;
		if (nInd%1000 == 0)
		{
			t0 = t1;
			t1 = getCurrentTime_ms();
			printf("%.2f fps\n", 1000.0/((t1-t0)/1000.0));
		}
	}

	m_Reader.Close();

	return 1;
}

Point2f CBaslerProcessor::getMotion(
	vector<KeyPoint>& pt0,
	vector<KeyPoint>& pt1,
	vector<vector<DMatch> >& matches,
	vector<Point2f>& optFlowStart, 
	vector<Point2f>& optFlowEnd,
	vector<float>& optDist, 
	vector<int>& optFLowStatus,
	Point2f refMotion)
{
	optFlowStart.resize(matches.size());
	optFlowEnd.resize(matches.size());
	optFLowStatus.resize(matches.size(), 0);
	optDist.resize(matches.size(), 0.f);
	if (matches.size() <= 0)
	{
		return BAD_MOTION_;
	}
	for(unsigned int i = 0; i < matches.size(); i++)
	{
		DMatch& match = matches[i][0];
		optFlowStart[i] = pt0[match.trainIdx].pt;
		optFlowEnd[i] = pt1[match.queryIdx].pt;
		optDist[i] = match.distance;
		if (match.distance < m_Param.dMatchThreshold)
		{
			optFLowStatus[i] = 1;
		}
		else
		{
			optFLowStatus[i] = 0;
		}
	}

	Point2f NearVec;
	double dNearDist = DBL_MAX;
	int nNearInd = -1;
	for (unsigned int i = 0; i < optFLowStatus.size(); i++)
	{
		if (optFLowStatus[i] == 0)
		{
			continue;
		}
		Point2f Vec = optFlowEnd[i] - optFlowStart[i];
		double dDist = sqrt(pow(Vec.x-refMotion.x,2) + pow(Vec.y-refMotion.y,2));
		if (dDist <= dNearDist && dDist <= 5.0)	//Critical param
		{
			dNearDist = dDist;
			nNearInd = i;
			NearVec = Vec;
		}
	}

	Point2f out = BAD_MOTION_;
	if (nNearInd == -1)
	{
		Point2f guess = GuessMotion(optFlowStart, optFlowEnd, optFLowStatus);
		if (guess == BAD_MOTION_)
		{
			out = BAD_MOTION_;
		}
		else
		{
			out = guess;
		}
	}
	else
	{
		out = NearVec;
	}
	return out;
}

Point2f CBaslerProcessor::getMotionRansac(
    vector<KeyPoint>& pt0,
    vector<KeyPoint>& pt1,
    vector<vector<DMatch> >& matches,
    vector<Point2f>& optFlowStart,
    vector<Point2f>& optFlowEnd,
    vector<float>& optDist,
    vector<int>& optFLowStatus,
    Point2f refMotion)
{
    optFlowStart.resize(matches.size());
    optFlowEnd.resize(matches.size());
    optFLowStatus.resize(matches.size(), 0);
    optDist.resize(matches.size(), 0.f);
    vector<Point2f> optFlowValid;
    optFlowValid.reserve(matches.size());
    for(unsigned int i = 0; i < matches.size(); i++)
    {
        DMatch& match = matches[i][0];
        optFlowStart[i] = pt0[match.trainIdx].pt;
        optFlowEnd[i] = pt1[match.queryIdx].pt;
        optDist[i] = match.distance;
        if (match.distance < m_Param.dMatchThreshold)
        {
            optFLowStatus[i] = 1;
            optFlowValid.push_back(optFlowEnd[i] - optFlowStart[i]);
        }
        else
        {
            optFLowStatus[i] = 0;
        }
    }

//    data – a set of observed data points
//    model – a model that can be fitted to data points
//    n – the minimum number of data values required to fit the model
//    k – the maximum number of iterations allowed in the algorithm
//    t – a threshold value for determining when a data point fits a model
//    d – the number of close data values required to assert that a model fits well to data

//    iterations = 0
//    bestfit = nul
//    besterr = something really large
    int k = 100;
    int n = 5;
    int d = optFlowValid.size()*0.3;
    int iterations = 0;
    double t = 2.0;
    Point2f bestfit = BAD_MOTION_;
    float besterr = FLT_MAX;
    if (optFlowValid.size() < n)
    {
        return BAD_MOTION_;
    }
    Mat shuffle = Mat::zeros(1, optFlowValid.size(), CV_8U);
    shuffle.colRange(0, n) = Scalar(1);
    while (iterations < k)
    {
//      maybeinliers = n randomly selected values from data
        randShuffle(shuffle);
//      maybemodel = model parameters fitted to maybeinliers
        Point2f maybemodel(0.f, 0.f);
        for (unsigned int i = 0; i < shuffle.cols; i++)
        {
            if (shuffle.at<uchar>(0,i) != 0)
            {
                maybemodel += optFlowValid[i];
            }
        }
        maybemodel = maybemodel*(1.f/n);

//        alsoinliers = empty set
//        for every point in data not in maybeinliers {
//            if point fits maybemodel with an error smaller than t
//                 add point to alsoinliers
//        }
        Mat alsoinliers = Mat::zeros(1, optFlowValid.size(), CV_8U);
        int nInlierCont = 0;
        for (unsigned int i = 0; i < shuffle.cols; i++)
        {
            if (shuffle.at<uchar>(0,i) == 0)
            {
                double dDist = sqrt(pow(maybemodel.x-optFlowValid[i].x,2) +
                        pow(maybemodel.y-optFlowValid[i].y,2));
                if (dDist <= t)
                {
                    alsoinliers.at<uchar>(0,i) = 1;
                    nInlierCont++;
                }
            }
        }

//        if the number of elements in alsoinliers is > d {
//            % this implies that we may have found a good model
//            % now test how good it is
//            bettermodel = model parameters fitted to all points in maybeinliers and alsoinliers
//            thiserr = a measure of how well model fits these points
//            if thiserr < besterr {
//                bestfit = bettermodel
//                besterr = thiserr
//            }
//        }
//        increment iterations
        if (nInlierCont + n >= d)
        {
            Point2f bettermodel(0.f, 0.f);
            int nCont = 0;
            for (unsigned int i = 0; i < shuffle.cols; i++)
            {
                if (shuffle.at<uchar>(0,i) != 0 || alsoinliers.at<uchar>(0,i) != 0)
                {
                    bettermodel += optFlowValid[i];
                    nCont++;
                }
            }
            bettermodel = bettermodel*(1.f/nCont);
            float thiserr = 0.f;
            for (unsigned int i = 0; i < shuffle.cols; i++)
            {
                if (shuffle.at<uchar>(0,i) != 0 || alsoinliers.at<uchar>(0,i) != 0)
                {
                    float fDist = sqrt(pow(bettermodel.x-optFlowValid[i].x,2) +
                            pow(bettermodel.y-optFlowValid[i].y,2));
                    thiserr += pow(fDist,2);
                }
            }
            thiserr = thiserr/nCont;
            if (thiserr < besterr)
            {
                bestfit = bettermodel;
                besterr = thiserr;
            }
        }
        iterations++;
    }

    return bestfit;
}

Point2f CBaslerProcessor::getMotionKeep(
    vector<KeyPoint>& pt0,
    vector<KeyPoint>& pt1,
    vector<vector<DMatch> >& matches,
    vector<Point2f>& optFlowStart,
    vector<Point2f>& optFlowEnd,
    vector<float>& optDist,
    vector<int>& optFLowStatus,
    Point2f refMotion)
{
    float fErrTolerate = 1.f;
    optFlowStart.resize(matches.size());
    optFlowEnd.resize(matches.size());
    optFLowStatus.resize(matches.size(), 0);
    optDist.resize(matches.size(), 0.f);
    vector<Point2f> optFlowValid;
    optFlowValid.reserve(matches.size());
    for(unsigned int i = 0; i < matches.size(); i++)
    {
        DMatch& match = matches[i][0];
        optFlowStart[i] = pt0[match.trainIdx].pt;
        optFlowEnd[i] = pt1[match.queryIdx].pt;
        optDist[i] = match.distance;
        if (match.distance < m_Param.dMatchThreshold)
        {
            optFLowStatus[i] = 1;
            optFlowValid.push_back(optFlowEnd[i] - optFlowStart[i]);
        }
        else
        {
            optFLowStatus[i] = 0;
        }
    }

    if (refMotion == BAD_MOTION_)
    {
        return GuessMotion(optFlowStart, optFlowEnd, optFLowStatus);
    }
    else
    {
        vector<Point2f> NearVec;
        for (unsigned int i = 0; i < optFlowValid.size(); i++)
        {
            double dDist = sqrt(pow(optFlowValid[i].x-refMotion.x,2) +
                                pow(optFlowValid[i].y-refMotion.y,2));
            if (dDist <= fErrTolerate)
            {
                NearVec.push_back(optFlowValid[i]);
            }
        }

        if (NearVec.size() <= 0)
        {
            return BAD_MOTION_;
        }
        Point2f out(0.f, 0.f);
        for (int i = 0; i < NearVec.size(); i++)
        {
            out += NearVec[i];
        }
        out = out*(1.f/NearVec.size());
        return out;
    }
}

Mat CBaslerProcessor::drawOrbMatches(Mat& img, 
	vector<Point2f>& optFlowStart, 
	vector<Point2f>& optFlowEnd,
	vector<float> optDist, 
	vector<int>& optFLowStatus, 
	Point2f motion)
{
	Mat out;
	cvtColor(img, out, COLOR_GRAY2RGB);
	rectangle(out, m_MaskRect, CV_RGB(0,0,255));
	Rect mask_rect_befor(m_MaskRect.x+motion.x, m_MaskRect.y+motion.y, m_MaskRect.width, m_MaskRect.height);
	rectangle(out, mask_rect_befor, CV_RGB(0,255,0));
	for (unsigned int i = 0; i < optFlowEnd.size(); i++)
	{
		circle(out, optFlowEnd[i], 2, CV_RGB(0,0,255), -1);
	}
	for(unsigned int i = 0; i < optFLowStatus.size(); i++)
	{
		if (optFLowStatus[i] == 0)
		{
			continue;
		}
        CvScalar color = CV_RGB((uchar)(optDist[i]/m_Param.dOrbDetThreshold*255),
                                (uchar)((m_Param.dOrbDetThreshold-optDist[i])/m_Param.dOrbDetThreshold*255),
                                0);
		circle(out, optFlowEnd[i], 2, color, -1);
		line(out, optFlowEnd[i], optFlowStart[i], color, 1);
	}

	return out;
}

Mat CBaslerProcessor::drawOrbMatchesEx(Mat& img0, 
	Mat& img1,
	vector<Point2f>& optFlowStart, 
	vector<Point2f>& optFlowEnd,
	vector<float> optDist, 
	vector<int>& optFLowStatus, 
	Point2f motion)
{
	Mat out0, out1, out;
	cvtColor(img0, out0, COLOR_GRAY2RGB);
	cvtColor(img1, out1, COLOR_GRAY2RGB);
	rectangle(out1, m_MaskRect, CV_RGB(0,0,255));
	Rect mask_rect_befor(m_MaskRect.x+motion.x, m_MaskRect.y+motion.y, m_MaskRect.width, m_MaskRect.height);
	rectangle(out1, mask_rect_befor, CV_RGB(0,255,0));
	rectangle(out0, m_MaskRect, CV_RGB(0,255,0));
	Rect mask_rect_after(m_MaskRect.x-motion.x, m_MaskRect.y-motion.y, m_MaskRect.width, m_MaskRect.height);
	rectangle(out0, mask_rect_after, CV_RGB(0,0,255));
	out = Mat::zeros(img1.rows, 2*img1.cols, CV_8UC3);
	if (img0.data == 0 || img1.data == 0)
	{
		return out;
	}
	int nShift = out1.cols;
	out0.copyTo(out(Range(0, out0.rows), Range(0, out0.cols)));
	out1.copyTo(out(Range(0, out1.rows), Range(0+nShift, out1.cols+nShift)));
	for (unsigned int i = 0; i < optFlowStart.size(); i++)
	{
		circle(out, optFlowStart[i], 2, CV_RGB(0,0,255), -1);
	}
	for (unsigned int i = 0; i < optFlowEnd.size(); i++)
	{
		circle(out, optFlowEnd[i]+Point2f(nShift,0), 2, CV_RGB(0,0,255), -1);
	}
	for(unsigned int i = 0; i < optFLowStatus.size(); i++)
	{
		if (optFLowStatus[i] == 0)
		{
			continue;
		}
        CvScalar color = CV_RGB((uchar)(optDist[i]/m_Param.dOrbDetThreshold*255),
                                (uchar)((m_Param.dOrbDetThreshold-optDist[i])/m_Param.dOrbDetThreshold*255),
                                0);
		line(out, optFlowEnd[i]+Point2f(nShift,0), optFlowStart[i], color, 1);
	}

	return out;
}

 Point2f CBaslerProcessor::GuessMotion(
 	vector<Point2f>& optFlowStart, 
 	vector<Point2f>& optFlowEnd,
 	vector<int>& optFLowStatus)
 {
 	vector<Point2f> optFlow;
 	vector<double> optDist;
	//Calculate opt-flow length
 	for (unsigned int i = 0; i < optFLowStatus.size(); i++)
 	{
 		if (optFLowStatus[i] == 1)
 		{
 			Point2f vec = optFlowEnd[i] - optFlowStart[i];
 			double dDist = sqrt(pow(vec.x,2) + pow(vec.y,2));
 			optFlow.push_back(vec);
 			optDist.push_back(dDist);
 		}
 	}
 
	//need 10 opt-flow matches at least
 	if (optDist.size() < 10)	//Critical param
 	{
 		return BAD_MOTION_;
 	}
 
	//Get the middle value of their length
 	Point2f VecMid;
	double dLenMid;
 	int nCont = optDist.size();
 	int nMed = (nCont-1)/2;
 	for (unsigned int i = 0; i < optDist.size(); i++)
 	{
 		for (unsigned int j = i+1; j < optDist.size(); j++)
 		{
 			if (optDist[i] > optDist[j])
 			{
 				double dDistTemp = optDist[i];
 				optDist[i] = optDist[j];
 				optDist[j] = dDistTemp;
 				Point2f dOptTemp = optFlow[i];
 				optFlow[i] = optFlow[j];
 				optFlow[j] = dOptTemp;
 			}
 		}
 		if (i >= nMed)
 		{
 			VecMid = optFlow[i];
			dLenMid = optDist[i];
 			break;
 		}
 	}
 
	//Get the confidence of the middle value
	double nContInRange = 0;
	for (unsigned int i = 0; i < optDist.size(); i++)
	{
		double dDeltaDist = abs(dLenMid - optDist[i]);
		if (dDeltaDist <= dLenMid*0.3)	//Critical param
		{
			nContInRange++;
		}
	}
	double nConf = nContInRange/(double)optDist.size();
	if (nConf >= 0.1)					//Critical param
	{
		return VecMid;
	}
	else
	{
		return BAD_MOTION_;
	}

 }

 Point2f CBaslerProcessor::GuessMotionKmeans(
	 vector<Point2f>& optFlowStart, 
	 vector<Point2f>& optFlowEnd,
	 vector<int>& optFLowStatus)
 {
	 vector<float> VecPoint;
	 VecPoint.reserve(2*optFLowStatus.size());
	 int nValidCont = 0;
	 for (unsigned int i = 0; i < optFLowStatus.size(); i++)
	 {
		 if (optFLowStatus[i] == 1)
		 {
			 Point2f vec = optFlowEnd[i] - optFlowStart[i];
			 VecPoint.push_back(vec.x);
			 VecPoint.push_back(vec.y);
			 nValidCont++;
		 }
	 }

	 if (nValidCont < 10)
	 {
		 return BAD_MOTION_;
	 }

	 Mat points(nValidCont, 1, CV_32FC2, VecPoint.data()), labels, centers;
	 kmeans(points, nValidCont-5, labels,
		 TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0),
		 3, KMEANS_PP_CENTERS, centers);

	 vector<int> VecAcc;
	 VecAcc.resize(centers.rows, 0);
	 for (unsigned int i = 0; i < labels.rows; i++)
	 {
		 VecAcc[labels.at<int>(i)]++;
	 }
	 int nMaxInd = -1;
	 int nMaxCont = -1;
	 for (unsigned int i = 0; i < VecAcc.size(); i++)
	 {
		 if (VecAcc[i] >= nMaxCont)
		 {
			 nMaxCont = VecAcc[i];
			 nMaxInd = i;
		 }
	 }
	 if(nMaxCont < 3)
	 {
		 return BAD_MOTION_;
	 }

	 Point2f pt_out(centers.at<float>(nMaxInd,0), centers.at<float>(nMaxInd,1));

	 return pt_out;
 }
