#include "StereoOpr.h"

tag_StereoParam::tag_StereoParam(const char* pszPath, int nImgH, int nImgW)
{
	if (pszPath == NULL)
	{
		sprintf(szPathIni,"StereoOpr.ini");
		sprintf(szPathIntrinsics,"intrinsics.yml");
		sprintf(szPathExtrinsics,"extrinsics.yml");
	}
	else
	{
		sprintf(szPathIni,"%s\\StereoOpr.ini",pszPath);
		sprintf(szPathIntrinsics,"%s\\intrinsics.yml",pszPath);
		sprintf(szPathExtrinsics,"%s\\extrinsics.yml",pszPath);
	}
	nImgHeight = nImgH;
	nImgWidth = nImgW;
	dScale = 1.f;	
	dRegUpPer = 0.f;	
	dRegDownPer = 1.f;	
	dRegLeftPer = 0.f;	
	dRegRightPer = 1.f;
	nSGBM_numberOfDisparities = 96;
	nSGBM_SADWindowSize = 3;
	nSGBM_cn = 1;
	nSGBM_preFilterCap = 63;
	nSGBM_minDisparity = 0;
	nSGBM_uniquenessRatio = 10;
	nSGBM_speckleWindowSize = 100;
	nSGBM_speckleRange = 32;
	nSGBM_disp12MaxDiff = 1;
	bSGBM_fullDP = true;
	nSGBM_m_scale = 1.f;
	nSGBM_m_LidarWidth = 10000;
	nSGBM_m_LidarDeep = 30000;
	nSGBM_m_LidarGridSize = 100;
	nSGBM_m_LidarHigh = 1000;
}

long tag_StereoParam::LoadParam()
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

CStereoOpr::CStereoOpr()
{
}

CStereoOpr::~CStereoOpr()
{
}

long CStereoOpr::Init(CStereoParam Param)
{	
	m_StereoParam = Param;
	FileStorage fs(m_StereoParam.szPathIntrinsics, CV_STORAGE_READ);
	if(!fs.isOpened())
	{
		printf("Failed to open file %s\n",m_StereoParam.szPathIntrinsics);
		return 0;
	}
	fs["M1"] >> m_M1;
// 	m_M1.at<double>(0,2) = m_M1.at<double>(0,2) - m_StereoParam.nImgWidth*m_StereoParam.dRegLeftPer;
// 	m_M1.at<double>(1,2) = m_M1.at<double>(1,2) - m_StereoParam.nImgHeight*m_StereoParam.dRegUpPer;
 	m_M1 = m_M1 * m_StereoParam.dScale;
	fs["D1"] >> m_D1;
	fs["M2"] >> m_M2;
// 	m_M2.at<double>(0,2) = m_M2.at<double>(0,2) - m_StereoParam.nImgWidth*m_StereoParam.dRegLeftPer;
// 	m_M2.at<double>(1,2) = m_M2.at<double>(1,2) - m_StereoParam.nImgHeight*m_StereoParam.dRegUpPer;
 	m_M2 = m_M2 * m_StereoParam.dScale;
	fs["D2"] >> m_D2;

	Mat mR,mT;
	fs.open(m_StereoParam.szPathExtrinsics, CV_STORAGE_READ);
	if(!fs.isOpened())
	{
		printf("Failed to open file %s\n",m_StereoParam.szPathExtrinsics);
		return 0;
	}
	fs["R"] >> m_R;
	fs["T"] >> m_T;

	m_imgSize.height = m_StereoParam.nImgHeight/**(m_StereoParam.dRegDownPer-m_StereoParam.dRegUpPer)*/*m_StereoParam.dScale;
	m_imgSize.width = m_StereoParam.nImgWidth/**(m_StereoParam.dRegRightPer-m_StereoParam.dRegLeftPer)*/*m_StereoParam.dScale;

	stereoRectify( m_M1, m_D1, m_M2, m_D2, m_imgSize, m_R,m_T,
		m_R1, m_R2, m_P1, m_P2, m_Q, CALIB_ZERO_DISPARITY, -1,
		m_imgSize, &m_ValidRect1, &m_ValidRect2 );

	initUndistortRectifyMap(m_M1, m_D1, m_R1, m_P1, m_imgSize, CV_16SC2, m_map11, m_map12);
	initUndistortRectifyMap(m_M2, m_D2, m_R2, m_P2, m_imgSize, CV_16SC2, m_map21, m_map22);

	//long numberOfDisparities = 96;
	//long SADWindowSize = 3;
	//numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((m_imgSize.width/8) + 15) & -16;
	//int cn = 1;
	//m_sgbm.preFilterCap = 63;
	//m_sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
	//m_sgbm.P1 = 8*cn*m_sgbm.SADWindowSize*m_sgbm.SADWindowSize;
	//m_sgbm.P2 = 32*cn*m_sgbm.SADWindowSize*m_sgbm.SADWindowSize;
	//m_sgbm.minDisparity = 0;
	//m_sgbm.numberOfDisparities = numberOfDisparities;
	//m_sgbm.uniquenessRatio = 10;
	//m_sgbm.speckleWindowSize = 100;
	//m_sgbm.speckleRange = 32;
	//m_sgbm.disp12MaxDiff = 1;
	//m_sgbm.fullDP = true;

	//m_scale = 1.f;
	long numberOfDisparities = m_StereoParam.nSGBM_numberOfDisparities;
	long SADWindowSize = m_StereoParam.nSGBM_SADWindowSize;
	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((m_imgSize.width/8) + 15) & -16;
	int cn = m_StereoParam.nSGBM_cn;
	m_sgbm.preFilterCap = m_StereoParam.nSGBM_preFilterCap;
	m_sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
	m_sgbm.P1 = 8*cn*m_sgbm.SADWindowSize*m_sgbm.SADWindowSize;
	m_sgbm.P2 = 32*cn*m_sgbm.SADWindowSize*m_sgbm.SADWindowSize;
	m_sgbm.minDisparity = m_StereoParam.nSGBM_minDisparity;
	m_sgbm.numberOfDisparities = numberOfDisparities;
	m_sgbm.uniquenessRatio = m_StereoParam.nSGBM_uniquenessRatio;
	m_sgbm.speckleWindowSize = m_StereoParam.nSGBM_speckleWindowSize;
	m_sgbm.speckleRange = m_StereoParam.nSGBM_speckleRange;
	m_sgbm.disp12MaxDiff = m_StereoParam.nSGBM_disp12MaxDiff;
	m_sgbm.fullDP = m_StereoParam.bSGBM_fullDP;

//	m_scale = m_StereoParam.nSGBM_m_scale;

	m_f = m_P1.at<double>(0,0);
	m_TT = m_T.at<double>(0,0);
	m_cx = m_P1.at<double>(0,2)-m_StereoParam.dRegLeftPer*m_imgSize.width;
	m_cy = m_P1.at<double>(1,2)-m_StereoParam.dRegUpPer*m_imgSize.height;;

	float M[9] = {	m_f,	0,	m_cx,
		0,	m_f,	m_cy,
		0,	0,	1};
	Mat MM(3,3,CV_32FC1,M);
	MM.copyTo(m_cameraMatrix);

	std::cout << "scale:\n" << m_StereoParam.dScale << std::endl;
	std::cout << "P:\n" << MM << std::endl;
	std::cout << "m_TT:\n" << m_TT << std::endl;

	//m_dLidarWidth = 10000;
	//m_dLidarDeep = 30000;
	//m_dLidarGridSize = 100;
	//m_dLidarHigh = 1000;
	m_dLidarWidth = m_StereoParam.nSGBM_m_LidarWidth;
	m_dLidarDeep = m_StereoParam.nSGBM_m_LidarDeep;
	m_dLidarGridSize = m_StereoParam.nSGBM_m_LidarGridSize;
	m_dLidarHigh = m_StereoParam.nSGBM_m_LidarHigh;

	return 1;
};

long CStereoOpr::BuildPillar(Mat& ImgL_in, Mat& ImgR_in, Mat& disp_in, Mat& Pillar_out, Mat& PillarImg_out)
{
	Pillar_out.release();
	PillarImg_out.release();
	
//	disp_out = CalDispSGBM(ImgL_in,ImgR_in);

	Mat img1(ImgL_in);
	Mat img1d(&CvMat(img1),1);
	vector<Mat> vecmat(3);
	img1.copyTo(vecmat[0]);
	img1.copyTo(vecmat[1]);
	img1.copyTo(vecmat[2]);
	Mat color;
	merge(vecmat,color);
	Mat Pillar;
	Pillarize(disp_in,Pillar);
	Pillar_out.create(disp_in.rows,disp_in.cols,CV_8UC1);
	cvResize(&CvMat(Pillar),&CvMat(Pillar_out));
	CvScalar green = cvScalar(0,100,0);
	//	cvAddS(&CvMat(img1d),green,&CvMat(img1d),&CvMat(Pillar_out));
	//	img1d.copyTo(PillarImg_out);
	long nnn = color.channels();
	long nnnn = PillarImg_out.channels();
	cvAddS(&CvMat(color),green,&CvMat(color),&CvMat(Pillar_out));
	color.copyTo(PillarImg_out);

	return 1;
}

Mat CStereoOpr::CalDispSGBM(Mat& ImgL_in, Mat& ImgR_in)
{
	Mat disp_out;
	Mat img1(ImgL_in), img2(ImgR_in);
	m_sgbm(img1, img2, disp_out);
	disp_out.convertTo(disp_out,CV_32F);
	return disp_out;
}

long CStereoOpr::MotionDetect(Mat& DispP_in, Mat& DispN_in, Mat& rvec, Mat& tvec, Mat& Motion_out )
{
	// 	saveXYZ1("DispP_in.csv",DispP_in);
	// 	saveXYZ1("DispN_in.csv",DispN_in);
	// 	saveXYZ1("rvec.csv",rvec);
	// 	saveXYZ1("tvec.csv",tvec);
	Mat rvec1,rvec2,tvec1;
	Rodrigues(rvec,rvec1);
	rvec1.convertTo(rvec2,CV_32FC1);
	tvec.convertTo(tvec1,CV_32FC1);
	Mat Disp1(DispP_in.rows, DispP_in.cols, DispP_in.type());
	cvSet(&CvMat(Disp1),cvScalar(-1));
	for (int i = 0; i < DispP_in.rows; i++)
	{
		for (int j = 0; j < DispP_in.cols; j++)
		{
			if (cvGetReal2D(&CvMat(DispP_in),i,j)<=0)
				continue;
			double W = cvGetReal2D(&CvMat(DispP_in),i,j)/16/(-1*m_TT);
			double X = (j - m_cx + 1)/W;
			double Y = (i - m_cy + 1)/W;
			double Z = m_f/W;
			Mat PosP(3,1,CV_32FC1);
			PosP.at<float>(0,0) = X;
			PosP.at<float>(1,0) = Y;
			PosP.at<float>(2,0) = Z;
			Mat PosN(3,1,CV_32FC1);
			cvMatMulAdd(&CvMat(rvec2),&CvMat(PosP),&CvMat(tvec1),&CvMat(PosN));
			double X_ = PosN.at<float>(0,0);
			double Y_ = PosN.at<float>(1,0);
			double Z_ = PosN.at<float>(2,0);
			double W_ = m_f/Z_;
			double P_ = W_*(-1*m_TT)*16.f;
			int i_ = cvRound(Y_*W_+m_cy)-1;
			int j_ = cvRound(X_*W_+m_cx)-1;
			// 			if ii>0&&ii<=r&&jj>0&&jj<=c&&(Disppp>=Disp1(ii,jj))
			// 				Disp1(ii,jj) = Disppp;
			// 			end
			if (i_>=0 && i_<Disp1.rows && j_>=0 && j_<Disp1.cols && P_>=Disp1.at<short>(i_,j_))
			{
				Disp1.at<short>(i_,j_) = P_;
			}
		}
	}


	// 	Disp2 = Disp1;
	// 	for i = 2:r-1
	// 		for j = 2:c-1
	// 			temp = Disp1(i-1:i+1,j-1:j+1);
	// 			if Disp1(i,j) && ~isempty(temp)
	// 				Disp2(i,j) = mean(temp(temp>0));
	// 			end
	// 		end
	// 	end
	Mat Disp2(&CvMat(Disp1),true);
	for (int i = 1; i < Disp1.rows-1; i++)
	{
		for (int j = 1; j < Disp1.cols-1; j++)
		{
			if (Disp1.at<short>(i,j)<0)
			{
				Mat temp = Disp1(Range(i-1,i+2),Range(j-1,j+2));
				//Mat temp1(temp.size(),CV_8UC1);
				//cvThreshold(&CvMat(temp),&CvMat(temp1),0,255,CV_THRESH_BINARY);
				//if (!IsAllZero(temp1))
				//{
				//	double dMax;
				//	cvMinMaxLoc(&CvMat(temp),NULL,&dMax,0,0,&CvMat(temp1));
				//	Disp1.at<short>(i,j) = dMax;
				//}
				double dMax;
				cvMinMaxLoc(&CvMat(temp),NULL,&dMax,0,0);
				if (dMax>=0)
				{
					Disp2.at<short>(i,j) = dMax;
				}	
			}
		}
	}
	// 	out = Disp;
	// 	out(:) = 0;
	// 	for i = 1:r
	// 		for j = 1:c
	// 			if Disp(i,j)>=0 && Disp2(i,j)>=0
	// 				out(i,j) = abs(Disp(i,j)-Disp2(i,j));
	// 			end
	// 		end
	// 	end
	Mat out(DispP_in.rows,DispP_in.cols,CV_32FC1);
	cvSet(&CvMat(out),cvScalar(0));
	for (int i = 0; i < out.rows; i++)
	{
		for (int j = 0; j < out.cols; j++)
		{
			if (cvGetReal2D(&CvMat(DispN_in),i,j)>=0 && cvGetReal2D(&CvMat(Disp2),i,j)>=0)
			{
				//				out(i,j) = abs(Disp(i,j)-Disp2(i,j));
				out.at<float>(i,j) = abs(cvGetReal2D(&CvMat(DispN_in),i,j)-cvGetReal2D(&CvMat(Disp2),i,j));
			}
		}
	}

	// 	out1 = out>=30;
	// 	out2 = imerode(out1,ones(5,5));
	Mat out1(out.size(),CV_8UC1);
	cvThreshold(&CvMat(out),&CvMat(out1),30,255,CV_THRESH_BINARY);
	//	saveXYZ1("out1.csv",out1);
	Mat out2(out.size(),CV_8UC1);
	Mat kenal(5,5,CV_8U);
	cvSet(&CvMat(kenal),cvScalar(255));
	erode(out1,out2,kenal/*,Point(3,3),1*/);
	//	saveXYZ1("out2.csv",out2);

	Motion_out.release();
	out2.copyTo(Motion_out);

	//	saveXYZ1("kenal.csv",kenal);

	return 1;
}

long CStereoOpr::IsAllZero(Mat& mat)
{
	for (int i = 0; i < mat.rows; i++)
	{
		for (int j = 0; j < mat.cols; j++)
		{
			if (cvGetReal2D(&CvMat(mat),i,j) != 0)
			{
				return 0;
			}
		}
	}
	return 1;
}

Mat CStereoOpr::Grey2FateColor(Mat& mat_in)
{
	Mat temp(mat_in.rows,mat_in.cols,CV_8UC1);
	cvNormalize( &CvMat(mat_in), &CvMat(temp), 0, 350, CV_MINMAX );

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

Mat CStereoOpr::GetPillarPoints(Mat& Disp, Mat& Pillar, Mat& Lidar)
{
	vector<Point3f> VecPoint;
	for (int i = 0; i < Disp.rows; i++)
	{
		for (int j = 0; j < Disp.cols; j++)
		{
			if (cvGetReal2D(&CvMat(Pillar),i,j)!=0)
			{
				double W = cvGetReal2D(&CvMat(Disp),i,j)/16/(-1*m_TT);
				double X = (j - m_cx + 1)/W;
				double Y = (i - m_cy + 1)/W;
				double Z = m_f/W;
				Point3f pt(X,Y,Z);
				VecPoint.push_back(pt);
			}
		}
	}

	Mat out(VecPoint.size(),3,CV_32FC1,VecPoint.data());
	Mat out1;
	out.copyTo(out1);

	// 	[r,c] = size(x123123);
	// 	map = zeros(401,201);
	// 	for i = 1:r
	// 		X = x123123(i,1);
	// 		Y = x123123(i,2);
	// 		Z = x123123(i,3);
	// 		if X>=-10000&&X<=10000&&Y>=-3000&&Z<=40000&&Z>=0
	// 			X = X+10000;
	// 			XX = floor(X/100)+1;
	// 			ZZ = floor(Z/100)+1;
	// 			map(ZZ,XX) = -1*Y+4000;
	// 		end
	// 	end
	// 	map = flipud(map);
	Mat Lidar1 = Mat::zeros(m_dLidarDeep/m_dLidarGridSize+1,m_dLidarWidth*2/m_dLidarGridSize+1,CV_32FC1);
	for (int i = 0; i < VecPoint.size(); i++)
	{
		double X = VecPoint[i].x;
		double Y = VecPoint[i].y;
		double Z = VecPoint[i].z;
		if (X>=-1*m_dLidarWidth && X<=m_dLidarWidth && Y>=-1*m_dLidarHigh && Y<=m_dLidarHigh && Z<=m_dLidarDeep && Z>=0)
		{
			X += m_dLidarWidth;
			double XX = floor(X/m_dLidarGridSize);
			double ZZ = floor(Z/m_dLidarGridSize);
			Lidar1.at<float>(ZZ,XX) = -1*Y+4000;
		}
	}
	cvFlip(&CvMat(Lidar1),NULL,0);
	Lidar = Grey2FateColor(Lidar1);

	//draw car head contour
	cvRectangle(&CvMat(Lidar),cvPoint((m_dLidarWidth-700)/m_dLidarGridSize,(m_dLidarDeep-1000)/m_dLidarGridSize),cvPoint((m_dLidarWidth+700)/m_dLidarGridSize,(m_dLidarDeep)/m_dLidarGridSize),cvScalar(255,255,255),1,CV_AA );

	//draw Car oritation line
	cvLine(&CvMat(Lidar),cvPoint(m_dLidarWidth/m_dLidarGridSize,0),cvPoint(m_dLidarWidth/m_dLidarGridSize,m_dLidarDeep/m_dLidarGridSize),cvScalar(0,0,0),1,CV_AA );

	//draw sign angle line
	cvLine(&CvMat(Lidar),cvPoint(0,m_dLidarDeep/m_dLidarGridSize-100),cvPoint(m_dLidarWidth/m_dLidarGridSize,m_dLidarDeep/m_dLidarGridSize),cvScalar(0,0,0),1,CV_AA);
	cvLine(&CvMat(Lidar),cvPoint(2*m_dLidarWidth/m_dLidarGridSize,m_dLidarDeep/m_dLidarGridSize-100),cvPoint(m_dLidarWidth/m_dLidarGridSize,m_dLidarDeep/m_dLidarGridSize),cvScalar(0,0,0),1,CV_AA);


	return out1;
}

void CStereoOpr::Remap(Mat& ImgL_in, Mat& ImgR_in, Mat& ImgL_out, Mat& ImgR_out)
{
	Mat imgL_,imgR_;
	if (m_StereoParam.dScale != 1.f)
	{
		resize(ImgL_in,imgL_,cvSize(ImgL_in.cols*m_StereoParam.dScale,ImgL_in.rows*m_StereoParam.dScale));
		resize(ImgR_in,imgR_,cvSize(ImgR_in.cols*m_StereoParam.dScale,ImgR_in.rows*m_StereoParam.dScale));
	}
	else
	{
		imgL_ = ImgL_in;
		imgR_ = ImgR_in;
	}
	Mat ImgL0,ImgR0;
	remap(imgL_,ImgL0,m_map11,m_map12,INTER_LINEAR);
	remap(imgR_,ImgR0,m_map21,m_map22,INTER_LINEAR);
	ImgL0(Range(ImgL0.rows*m_StereoParam.dRegUpPer,ImgL0.rows*m_StereoParam.dRegDownPer),
		Range(ImgL0.cols*m_StereoParam.dRegLeftPer,ImgL0.cols*m_StereoParam.dRegRightPer)).copyTo(ImgL_out);
	ImgR0(Range(ImgR0.rows*m_StereoParam.dRegUpPer,ImgR0.rows*m_StereoParam.dRegDownPer),
		Range(ImgR0.cols*m_StereoParam.dRegLeftPer,ImgR0.cols*m_StereoParam.dRegRightPer)).copyTo(ImgR_out);
}

long CStereoOpr::Pillarize(Mat& mat_in, Mat& mat_out)
{
	// 	[r,c] = size(disp);
	// 	r_w = 10;
	// 	c_w = 5;
	// 	rr = r/r_w;
	// 	cc = c/c_w;
	// 	des = zeros(rr,cc);
	// 	for i = 1:rr
	// 		for j = 1:cc
	// 			temp = disp((i-1)*r_w+1:(i-1)*r_w+r_w,(j-1)*c_w+1:(j-1)*c_w+c_w);
	// 			des(i,j) = std(temp(:));
	// 		end
	// 	end
	long r = mat_in.rows, c = mat_in.cols;
	long r_w = 10, c_w = 2;
	long rr = r/r_w, cc = c/c_w;
	Mat des(rr,cc,CV_32FC1);
	for (int i = 0; i < rr; i++)
	{
		for (int j = 0; j < cc; j++)
		{
			Mat Temp = mat_in(Range(i*r_w,(i+1)*r_w),Range(j*c_w,(j+1)*c_w));
			CvScalar Std;
			CvScalar Avg;
			cvAvgSdv(&CvMat(Temp),&Avg,&Std);
			cvSetReal2D(&CvMat(des),i,j,Std.val[0]); 
		}
	}
	// 	h = fspecial('gaussian');
	// 	des1 = filter2(h,des);
	// 	des_b1 = (des) < 4;
	// 	des_b2 = (des1) < 4;
	Mat des1,des_b1,des_b2;
	des.copyTo(des1);
	des.copyTo(des_b1);
	des.copyTo(des_b2);
	cvSmooth(&CvMat(des),&CvMat(des1),CV_GAUSSIAN,3,3,0.5);
	cvThreshold(&CvMat(des1),&CvMat(des_b2),6,1,CV_THRESH_BINARY_INV);
	des_b2.row(des_b2.rows-1) = 0;

// 	Mat kenal_ = Mat::ones(3,3,CV_8U);
// 	dilate(des_b2,des_b2,kenal_,Point(1,1),1);
// 	erode(des_b2,des_b2,kenal_,Point(1,1),2);

	des_b2.convertTo(mat_out,CV_8UC1);

	// 	disp1 = (disp)>0;
	// 	disp2 = imresize(disp1,[rr,cc],'nearest');
	// 	disp3 = imerode(disp2,ones(2,2));
	// 	disp4 = disp3&des_b2;

	Mat disp1(mat_in.rows,mat_in.cols,CV_8U);
	cvThreshold(&CvMat(mat_in),&CvMat(disp1),0,1,CV_THRESH_BINARY);
	Mat disp2(rr,cc,CV_8U);
	cvResize(&CvMat(disp1),&CvMat(disp2),CV_INTER_NN);
	Mat disp3(rr,cc,CV_8U);
	Mat kenal;
	kenal.ones(2,2,CV_8U);
	erode(disp2,disp3,kenal,Point(1,1),1);
	Mat disp4(rr,cc,CV_8U);
	cvAnd(&CvMat(mat_out),&CvMat(disp3),&CvMat(disp4));
	disp4.convertTo(mat_out,CV_8U);

	return 1;
}

long CStereoOpr::NPN(vector<KeyPoint>& ptTest, vector<KeyPoint>& pt0, vector<KeyPoint>& pt1, vector<DMatch>& matches, Mat& mask)
{
	vector<Point2f> VecPtTest;
	vector<Point3f> VecPtMap;
	for (int ii = 0; ii < matches.size(); ii++)
	{
		VecPtTest.push_back(ptTest[matches[ii].queryIdx].pt);
		Point3f ptT;
		double W = (pt0[matches[ii].trainIdx].pt.x - pt1[matches[ii].trainIdx].pt.x)/(-1.f*m_TT);
		ptT.x = (pt0[matches[ii].trainIdx].pt.x - m_cx)/W;
		ptT.y = (pt0[matches[ii].trainIdx].pt.y - m_cy)/W;
		ptT.z = m_f/W;
		VecPtMap.push_back(ptT);
	}
	Mat matPt0(VecPtTest.size(),2,CV_32FC1,VecPtTest.data());
	Mat matPt1(VecPtMap.size(),3,CV_32FC1,VecPtMap.data());
	Mat R,distCoeffs,inliers,rvec,tvec;
	if (VecPtTest.size()>=4)
	{
		solvePnPRansac(matPt1,matPt0,m_cameraMatrix,distCoeffs,rvec,tvec,false,100,8.0,100,inliers,EPNP);	
	}
	else
	{
		rvec = Mat::eye(3,3,CV_64FC1);
		tvec = Mat::zeros(3,1,CV_64FC1);
		inliers = Mat::zeros(VecPtTest.size(),1,CV_32FC1);
	}
	vector<DMatch> matchTemp;
	for (int ii = 0; ii < inliers.rows; ii++)
	{
		matchTemp.push_back(matches[inliers.at<ULONG32>(ii,0)]);
	}
	matches = matchTemp;

	// 		cout << "xxxxxxxxxxxxxxxxxxx" << endl;
	// 		cout << inliers.rows << endl;
	// 		cout << rvec/CV_PI*180 << endl;
	// 		cout << tvec << endl;
	// 		cout << "xxxxxxxxxxxxxxxxxxx" << endl;

	return 1;
}

Mat CStereoOpr::CalcCloudPoints(vector<DMatch>& matches, vector<KeyPoint>& keypoints0, vector<KeyPoint>& keypoints1)
{
	Mat out = Mat::zeros(matches.size(),3,CV_64FC1);

	for (int i = 0; i < matches.size(); i++)
	{
		double W = 0.f;
		double X	= 0.f;
		double Y	= 0.f;
		double Z	= 0.f;
		double dDisp = keypoints0[matches[i].queryIdx].pt.x - keypoints1[matches[i].trainIdx].pt.x;
		if (dDisp == 0)
		{
			W = 0.f;
			X = 0.f;
			Y = 0.f;
			Z = 0.f;
		}
		else
		{
			W = dDisp/(-1*m_TT);
			X = (keypoints0[matches[i].queryIdx].pt.x - m_cx)/W;
			Y = (keypoints0[matches[i].queryIdx].pt.y - m_cy)/W;
			Z = m_f/W;
		}
		out.at<double>(i,0) = X;
		out.at<double>(i,1) = Y;
		out.at<double>(i,2) = Z;
	}

	return out;
}

Point3f CStereoOpr::SteImgPt2XYZ(Point2f ptL, Point2f ptR)
{
	Point3f ptout;
	double dDisP = ptL.x - ptR.x;
	double W = dDisP/(-1.f*m_TT);
	ptout.x = (ptL.x - m_cx)/W;
	ptout.y = (ptL.y - m_cy)/W;
	ptout.z = m_f/W;
	return ptout;
}