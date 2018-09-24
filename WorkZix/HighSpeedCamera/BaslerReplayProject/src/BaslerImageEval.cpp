#include "RawDataReader.h"
#include "ImageEvaluate.h"

using namespace cv;

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("Path not configured!");
		getchar();
	}
	CImageEvaluateParam ParamEval;
	CImageEvaluate EvalOpr;
	EvalOpr.Init(ParamEval);

	CRawDataReaderParam Param;
	Param.nImgDepth = 1;
	Param.nImgHeight = 600;
	Param.nImgWidth = 800;
	Param.nImgPattern = CV_BayerBG2GRAY;

	CRawDataReader Reader;
	Reader.Init(Param);
	Reader.Open(argv[1]);
	bool bIsRun = false;
	for (uint64 i = 0; i < 1000000; i++)
	{
		Mat Img;
		int nRt = Reader.GetFrameByIndex(i, Img, 0, 0);
		if (nRt < 0)
		{
			break;
		}
		double dEntropy = Reader.GetImageEntropy(Img);
		Mat Disp;
		int nFeatureCont = 0;
		float fAvgRes = 0.f;
		EvalOpr.Evaluate(Img, Disp, nFeatureCont, fAvgRes);
		char szDisp[256] = {0};
		sprintf(szDisp, "Frame:%lld, SMD2:%.2f", i, dEntropy);
		putText(Disp, szDisp, Point(0, Disp.rows-5), 0, 1, CV_RGB(255,0,0), 2);
		cvNamedWindow("disp", 0);
		imshow("disp", Disp);
		int nKey = 0;
		if (bIsRun)
		{
			nKey = waitKey(1);
		}
		else
		{
			nKey = waitKey(0);
		}
		if (nKey == 32)
		{
			bIsRun = !bIsRun;
		}
	}

	getchar();

	return 0;
}

