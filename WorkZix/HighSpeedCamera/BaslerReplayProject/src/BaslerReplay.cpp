#include "LogDisplay.h"

using namespace std;
using namespace cv;


//int main0(int argc, char **argv)
//{
//	int fps = 500;
//	CvVideoWriter* writer =cvCreateVideoWriter("outL.mp4",
//		CV_FOURCC('D', 'I', 'V', '3'),fps,cvSize(800,600),1);

//	CRawDataReaderParam Param;
//	CRawDataReader Rd;
//	Rd.Init(Param);
//	Rd.Open("D:\\data\\CameraVideo\\CameraVideo\\CameraVideo\\left");
//	Mat MatDisp;
//	int64_t nInd = 0;
//	cvNamedWindow("Img", 0);
//	while(1)
//	{
//		int64_t nRt = Rd.GetFrameByIndex(nInd, MatDisp);
//		printf("Frame:%ld\n", nRt);
//		if (nRt < 0)
//		{
//			break;
//		}
//		cvWriteFrame(writer, &IplImage(MatDisp));
//		imshow("Img", MatDisp);
//		waitKey(1);
//		nInd++;
//	}

//	cvReleaseVideoWriter(&writer);

//	getchar();

//	return 0;
//}


int main(int argc, char **argv)
{
	CLogDisplay Ld;
	if (argc < 2)
	{
		printf("Can not find project dir!\n");
		getchar();
		return 0;
	}
	int nRt = Ld.Init(argv[1]);
	if (nRt < 0)
	{
		getchar();
		return 0;
	}

	Ld.Run();

	return 0;
}

