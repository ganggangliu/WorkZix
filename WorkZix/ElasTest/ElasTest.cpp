#include "StereoOpr.h"
#include "ElasOpr.h"

#if _DEBUG
#pragma comment(lib,"ElasOprD.lib")
#else
#pragma comment(lib,"ElasOpr.lib")
#endif

int main(int argc, char* argv[])
{
	CStereoParam StereoParam;
	strcpy(StereoParam.szPathExtrinsics, "D:\\StereoImg\\extrinsics.yml");
	strcpy(StereoParam.szPathIntrinsics, "D:\\StereoImg\\intrinsics.yml");
	StereoParam.nImgHeight = 1200;
	StereoParam.nImgHeight = 1600;
	StereoParam.dScale = 0.5;

	CStereoOpr StereoOpr;
	StereoOpr.Init(StereoParam);

	CElasOprParam ElasParam;
	ElasParam.nMaxDisp = 120;
	CElasOpr ElasOpr(ElasParam);

	VideoCapture capture0, capture1;
	capture0.open("D:\\StereoImg\\LVSource-1491889524378.avi");
	capture1.open("D:\\StereoImg\\RVSource-1491889524378.avi");

	Mat ImgLeft,ImgRight;
	while(1)
	{
		bool bRt;
		bRt = capture0.read(ImgLeft);
		if (!bRt)
			break;
		bRt = capture1.read(ImgRight);
		if (!bRt)
			break;
		cvtColor(ImgLeft, ImgLeft, CV_RGB2GRAY);
		cvtColor(ImgRight, ImgRight, CV_RGB2GRAY);
		Mat ImgRectL,ImgRectR;
		StereoOpr.Remap(ImgLeft, ImgRight, ImgRectL, ImgRectR);
		ImgRectL = ImgRectL(Range(ImgRectL.rows*0.2, ImgRectL.rows*0.6), Range::all());
		ImgRectR = ImgRectR(Range(ImgRectR.rows*0.2, ImgRectR.rows*0.6), Range::all());
		Mat Deep = ElasOpr.process(ImgRectL, ImgRectR);
		Mat Deep_ = StereoOpr.Grey2FateColor(Deep);
		cvNamedWindow("disparity");
		imshow("disparity", Deep_);
		cvNamedWindow("ori");
		imshow("ori", ImgRectL);
		waitKey(1);
	}

	return 0;
}

