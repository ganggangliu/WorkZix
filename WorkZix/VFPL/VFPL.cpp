#include "EgoMotion.h"
#include "ProjectOpr.h"
#include "CalibSensor.h"
#include "VisoOpr.h"
#include "LibGuiOpenGL.h"
#include "VFPLOpr.h"

#include "core\core_c.h"

#if _DEBUG
#pragma comment(lib,"StereoOprD.lib")
#pragma comment(lib,"FeatureDetectOprD.lib")
#pragma comment(lib,"EgoMotionD.lib")
#pragma comment(lib,"ProjectOprD.lib")
#pragma comment(lib,"CalibSensorD.lib")
#pragma comment(lib,"VisoOprD.lib")
#else
#pragma comment(lib,"StereoOpr.lib")
#pragma comment(lib,"FeatureDetectOpr.lib")
#pragma comment(lib,"EgoMotion.lib")
#pragma comment(lib,"ProjectOpr.lib")
#pragma comment(lib,"CalibSensor.lib")
#pragma comment(lib,"VisoOpr.lib")
#endif

//制图
int main0(int argc, char* argv[])
{
	LIBGUIOPENGL_HANDLE tLibGuiOpenglHandle = GetLibGuiOpenGL("LibGuiOpenGL_Params_VFPL.xml");
//////////////////////////////////////////////////////////////////////////
/*	string szProjPath = "D:\\测试资料\\data_odometry_gray\\dataset\\sequences\\03_";long nImgHeight = 375;long nImgWidth = 1242;

	CProjectParam ParamProj(szProjPath.c_str());
	strcpy(ParamProj.szImgNameHeadL,"\\image_0\\");
	strcpy(ParamProj.szImgNameHeadR,"\\image_1\\");
	CProjectOpr ProjOpr;
	ProjOpr.Init(ParamProj);

	CFeatureDetectParam ParamFeatDef(szProjPath.c_str(),nImgHeight,nImgWidth);
	CFeatureDetectOpr FeatDetOpr;
	FeatDetOpr.Init(ParamFeatDef);

	CStereoParam ParamSte(szProjPath.c_str(),nImgHeight,nImgWidth);
	CStereoOpr StereoOpr;
	StereoOpr.Init(ParamSte);

	VisualOdometryStereo::parameters param;
	double dScale = 0.5;
	param.calib.f  = StereoOpr.m_f;
	param.calib.cu = StereoOpr.m_cx; 
	param.calib.cv = StereoOpr.m_cy; 
	param.base     = StereoOpr.m_TT; 
	param.match.match_radius = 800;
	param.reweighting = false;
	param.bucket.bucket_height = 20;
	param.bucket.bucket_width = 20;
	param.ransac_iters = 200;
	param.inlier_threshold = 2.0;
	CVisoOpr VisoOpr(param,0.5);*/
//////////////////////////////////////////////////////////////////////////
	long nImgHeight = 1200;
	long nImgWidth = 1600;
	string szProjPath = "D:\\测试资料\\20150709光庭科技园区\\AutoSave20150709145522\\";

	CProjectParam ParamProj(szProjPath.c_str());
	strcpy(ParamProj.szImgNameHeadL,"AutoSaveL");
	strcpy(ParamProj.szImgNameHeadR,"AutoSaveR");
	CProjectOpr ProjOpr;
	ProjOpr.Init(ParamProj);

	CFeatureDetectParam ParamFeatDef;
	ParamFeatDef.nAlgorithmType = 1;
	ParamFeatDef.contrastThreshold = 0.1f;
	ParamFeatDef.edgeThreshold = 30;
	ParamFeatDef.hessianThreshold = 2000;
	ParamFeatDef.nFeatures = 500;
	ParamFeatDef.dMaxMatchRate = 0.8;
	ParamFeatDef.nMaxStereoDisparityX = 200;
	ParamFeatDef.nMaxStereoDisparityY = 20;
	ParamFeatDef.nMaxFeaturePtDist = 50000;
	CFeatureDetectOpr FeatDetOpr;
	FeatDetOpr.Init(ParamFeatDef);

	CStereoParam ParamSte(szProjPath.c_str(),nImgHeight,nImgWidth);
	CStereoOpr StereoOpr;
	StereoOpr.Init(ParamSte);

// 	VisualOdometryStereo::parameters param;
// 	double dScale = 0.5;
// 	param.calib.f  = StereoOpr.m_f;
// 	param.calib.cu = StereoOpr.m_cx; 
// 	param.calib.cv = StereoOpr.m_cy-100; 
// 	param.base     = StereoOpr.m_TT; 
// 	param.match.match_radius = 800;
// 	param.reweighting = false;
// 	param.bucket.bucket_height = 20;
// 	param.bucket.bucket_width = 20;
// 	param.ransac_iters = 200;
// 	param.inlier_threshold = 2.0;
// 	CVisoOpr VisoOpr(param,0.5);

	CVisoOprParam param;
	double dScale = 0.5;
	param.f  = StereoOpr.m_f;
	param.cu = StereoOpr.m_cx; 
	param.cv = StereoOpr.m_cy-100; 
	param.base     = StereoOpr.m_TT; 
	param.match_radius = 800;
	param.reweighting = false;
	param.bucket_height = 20;
	param.bucket_width = 20;
	param.ransac_iters = 200;
	param.inlier_threshold = 2.0;
	CVisoOpr VisoOpr(param,0.5);
//////////////////////////////////////////////////////////////////////////
	CVFPLMapWriter MapWriter;
	MapWriter.Init(szProjPath.c_str(), ParamFeatDef.Param2Mat());
	Mat Img[2];
	while(1)
	{
		Mat GpsData;
		long nFrame = ProjOpr.GetStereoImgByGpsFile(Img,GpsData,0);
		if (Img[0].data == NULL || Img[1].data == NULL)
		{
			break;
		}

		cout << "Frame:" << nFrame << endl;

		Mat imageL1,imageR1;
		StereoOpr.Remap(Img[0],Img[1],imageL1,imageR1);

		Mat imgDispL;
		imageL1.copyTo(imgDispL);
//////////////////////////////////////////////////////////////////////////
		VisoOpr.process(imageL1,imageR1);
		VisoOpr.DrawOpticFlow(imgDispL,1);

		Mat Pose = VisoOpr.getPose();
		cout << Pose << endl;
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
		Mat MatchImage;
		vector<DMatch> matches;
		vector<KeyPoint> keypoints0;
		vector<KeyPoint> keypoints1;
		Mat descriptors0,descriptors1;
		FeatDetOpr.StereoFeatureMatch(MatchImage,imageL1,imageR1,matches,keypoints0,keypoints1,descriptors0,descriptors1);

		FeatDetOpr.FilterKeyPoints(matches,keypoints0,keypoints1,descriptors0,descriptors1);
		printf("Feature points count: %d\n",matches.size());

		Mat CloudPoints = StereoOpr.CalcCloudPoints(matches,keypoints0,keypoints1);

		drawKeypoints(imgDispL,keypoints0,imgDispL);
		Mat MatchImage1;
		drawMatches(imgDispL,keypoints0,imageR1,keypoints1,matches,MatchImage1,CV_RGB(0,255,0));

		tLibGuiOpenglHandle->AddPoints(Pose,CloudPoints);

		MapWriter.WriteMap(GpsData,Pose,CloudPoints,descriptors0,keypoints0);

		cvNamedWindow("123",0);
		imshow("123",MatchImage1);
		waitKey(1);
	}

	MapWriter.Release();
	waitKey();

	return 0;
}

//定位
int main1(int argc, char* argv[])
{
	LIBGUIOPENGL_HANDLE tLibGuiOpenglHandle = GetLibGuiOpenGL();

	string szMapPath = "D:\\测试资料\\20150709光庭科技园区\\AutoSave20150709145522\\";

	long nImgHeight = 1200;
	long nImgWidth = 1600;
	string szProjPath = "D:\\测试资料\\20150709光庭科技园区\\AutoSave20150709150022\\";

	CProjectParam ParamProj(szProjPath.c_str());
	strcpy(ParamProj.szImgNameHeadL,"AutoSaveL");
	strcpy(ParamProj.szImgNameHeadR,"AutoSaveR");
	CProjectOpr ProjOpr;
	ProjOpr.Init(ParamProj);

	CProjectParam ParamProjMap(szMapPath.c_str());
	strcpy(ParamProjMap.szImgNameHeadL,"AutoSaveL");
	strcpy(ParamProjMap.szImgNameHeadR,"AutoSaveR");
	CProjectOpr ProjOprMap;
	ProjOprMap.Init(ParamProjMap);
	//CFeatureDetectParam ParamFeatDef;
	//CFeatureDetectOpr FeatDetOpr;
	//FeatDetOpr.Init(ParamFeatDef);

	//CStereoParam ParamSte(szProjPath.c_str(),nImgHeight,nImgWidth);
	//CStereoOpr StereoOpr;
	//StereoOpr.Init(ParamSte);

	//VisualOdometryStereo::parameters param;
	//double dScale = 0.5;
	//param.calib.f  = StereoOpr.m_f;
	//param.calib.cu = StereoOpr.m_cx; 
	//param.calib.cv = StereoOpr.m_cy-100; 
	//param.base     = StereoOpr.m_TT; 
	//param.match.match_radius = 800;
	//param.reweighting = false;
	//param.bucket.bucket_height = 20;
	//param.bucket.bucket_width = 20;
	//param.ransac_iters = 200;
	//param.inlier_threshold = 2.0;
	//CVisoOpr VisoOpr(param,0.5);

 	CVFPLLocator VFPLLocator;
	VFPLLocator.LoadMap(szMapPath.c_str());
	Mat Img[2];
	double dTimeBefor = 0;
	double dBaseHeading = 0;

	//for (int i = 0; i < 100000; i++)
	//{
	//	Mat GpsData = ProjOpr.GetGpsData().row(i);
	//	double dVe = GpsData.at<double>(6);
	//	double dVn = GpsData.at<double>(7);
	//	double dTime = GpsData.at<double>(1);
	//	VFPLLocator.Predict(dTime,dVe,dVn);
	//	Sleep(100);
	//}

	CFeatureDetectParam ParamFeatDefLoc;
	CStereoParam ParamSteLoc(szProjPath.c_str(),nImgHeight,nImgWidth);
	VFPLLocator.InitLocator(ParamSteLoc,ParamFeatDefLoc);
	for (int i = 0; i < 100000; i++)
	{
		Mat GpsData;
		long nFrame = ProjOpr.GetStereoImgByGpsFile(Img,GpsData,0);
		if (nFrame == -1)
			break;

		double dVe = GpsData.at<double>(6);
		double dVn = GpsData.at<double>(7);
		double dTime = GpsData.at<double>(1);
//		Mat imgMap = imread(ProjOprMap.m_VecImgPathL[nnn],0);
		Mat imgMap = VFPLLocator.Localization(Img[0],dTime,dVe,dVn,ProjOprMap);

		
		cvNamedWindow("123",0);
		imshow("123",imgMap);
		waitKey(1);
	}
	getchar();

	return 1;
}


Mat Grey2FateColor(Mat& mat_in)
{
//	Mat temp = mat_in;
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

//定位1
int main(int argc, char* argv[])
{
	RNG rng;
	Mat test = Mat::zeros(100,100,CV_8U);
	for (int i = 0; i < test.rows; i++)
	{
		for (int j = 0; j < test.cols; j++)
		{
			test.at<UCHAR>(i,j) = rng.uniform(0,122);
		}
	}
	Mat test1 = Grey2FateColor(test);
	cvNamedWindow("123",0);
	imshow("123",test1);
	imwrite("123.bmp",test1);
	waitKey();

	string szMapPath = "D:\\测试资料\\20150709光庭科技园区\\AutoSave20150709145522\\";

	long nImgHeight = 1200;
	long nImgWidth = 1600;
	string szProjPath = "D:\\测试资料\\20150709光庭科技园区\\AutoSave20150709150022\\";

	CProjectParam ParamProj(szProjPath.c_str());
	strcpy(ParamProj.szImgNameHeadL,"AutoSaveL");
	strcpy(ParamProj.szImgNameHeadR,"AutoSaveR");
	CProjectOpr ProjOpr;
	ProjOpr.Init(ParamProj);

	CProjectParam ParamProjMap(szMapPath.c_str());
	strcpy(ParamProjMap.szImgNameHeadL,"AutoSaveL");
	strcpy(ParamProjMap.szImgNameHeadR,"AutoSaveR");
	CProjectOpr ProjOprMap;
	ProjOprMap.Init(ParamProjMap);

	CVFPLLocator VFPLLocator;
	VFPLLocator.LoadMap(szMapPath.c_str());
	Mat Img[2];

// 	CFeatureDetectParam ParamFeatDefLoc;
// 	CStereoParam ParamSteLoc(szProjPath.c_str(),nImgHeight,nImgWidth);
// 	VFPLLocator.InitLocator(ParamSteLoc,ParamFeatDefLoc);
	for (int i = 0; i < 100000; i++)
	{
		Mat GpsData;
		long nFrame = ProjOpr.GetStereoImgByGpsFile(Img,GpsData,0);
		if (nFrame == -1)
			break;

		VFPLLocator.AddSinglePoint(GpsData);

		//cvNamedWindow("123",0);
		//imshow("123",imgMap);
		//waitKey(1);
		Sleep(100);
	}
	getchar();

	return 1;
}
