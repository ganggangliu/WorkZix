#include "OpenCVInc.h"

using namespace cv;
using namespace std;

// static void help()
// {
//     cout << "\nThis program demonstrates kmeans clustering.\n"
//             "It generates an image with random points, then assigns a random number of cluster\n"
//             "centers and uses kmeans to move those cluster centers to their representitive location\n"
//             "Call\n"
//             "./kmeans\n" << endl;
// }

int main( int argc, char** argv )
{
	const int MAX_CLUSTERS = 5;
// 	Scalar colorTab[] =
// 	{
// 		Scalar(0, 0, 255),
// 		Scalar(0,255,0),
// 		Scalar(255,100,100),
// 		Scalar(255,0,255),
// 		Scalar(0,255,255)
// 	};
	Scalar* pcolorTab = new Scalar[1000];
	pcolorTab[0] = Scalar(122, 122, 122);
	pcolorTab[1] = Scalar(122, 122, 122);
	pcolorTab[2] = Scalar(255,0,0);
	pcolorTab[3] = Scalar(0,255,0);
	pcolorTab[4] = Scalar(0,0,255);
	RNG rng_color;
	for (unsigned int i = 5; i < 1000; i++)
	{
		pcolorTab[i] = Scalar(255,255,255);
// 		pcolorTab[i] = Scalar((uchar)rng_color.uniform(0,256),
// 			(uchar)rng_color.uniform(0,256), (uchar)rng_color.uniform(0,256));
	}

	Mat img(500, 500, CV_8UC3);
	RNG rng(12345);

	for(;;)
	{
		int nPointCont = 50;
		double dNoiseRate = 0.6;
		Mat points(nPointCont, 1, CV_32FC2), labels;

		Point center;
		center.x = rng.uniform(img.cols/4, img.cols*3/4);
		center.y = rng.uniform(img.rows/4, img.rows*3/4);
		Mat pointChunk = points.rowRange(0, nPointCont*(1-dNoiseRate));
		rng.fill(pointChunk, RNG::NORMAL, Scalar(center.x, center.y), Scalar(img.cols*0.03, img.rows*0.03));

		for (unsigned int i = nPointCont*(1-dNoiseRate); i < points.rows; i++)
		{
			points.at<Vec<float,2>>(i,0) = Vec<float, 2>(rng.uniform(0, img.cols), rng.uniform(0, img.rows));
		}
		Mat centers;

		//////////////////////////////////////////////////////////////////////////

// 		int nSeedCont = 45;
// 		int k, clusterCount = 1/*rng.uniform(2, MAX_CLUSTERS+1)*/;
// 		int i, sampleCount = 50/*rng.uniform(1, 1001)*/;
// 		Mat points(sampleCount, 1, CV_32FC2), labels;
// 
// 		clusterCount = MIN(clusterCount, sampleCount);
// 		Mat centers;
// 
// 		/* generate random sample from multigaussian distribution */
// 		for( k = 0; k < clusterCount; k++ )
// 		{
// 			Point center;
// 			center.x = rng.uniform(0, img.cols);
// 			center.y = rng.uniform(0, img.rows);
// 			Mat pointChunk = points.rowRange(k*sampleCount/clusterCount,
// 				k == clusterCount - 1 ? sampleCount :
// 				(k+1)*sampleCount/clusterCount);
// 			rng.fill(pointChunk, RNG::NORMAL, Scalar(center.x, center.y), Scalar(img.cols*0.03, img.rows*0.03));
// 		}

		randShuffle(points, 1, &rng);

		kmeans(points, nPointCont-20, labels,
			TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0),
			3, KMEANS_PP_CENTERS, centers);

		img = Scalar::all(0);

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
		printf("LableInd:%d, LableAcc:%d\n", nMaxInd, nMaxCont);

		Point2f pt_center(centers.at<float>(nMaxInd,0), centers.at<float>(nMaxInd,1));

		for(int i = 0; i < nPointCont; i++ )
		{
			int clusterIdx = labels.at<int>(i);
			Scalar color = Scalar(122,122,122);
			if (nMaxInd == clusterIdx)
			{
				color = Scalar(0,0,255);
			}
			
			Point ipt = points.at<Point2f>(i);
			circle( img, ipt, 2, color/*pcolorTab[VecAcc[clusterIdx]]*/, -1, 8);
			circle(img , pt_center, 1, CV_RGB(255,255,255), 3, 8);
		}

		imshow("clusters", img);

// 		for (unsigned int i = 0; i < points.rows; i++)
// 		{
// 			std::cout << i << ":" << points.row(i) << ":" << VecAcc[i] 
// 			<< ":" << labels.at<int>(i) << std::endl;
// 		}

		char key = (char)waitKey();
		if( key == 27 || key == 'q' || key == 'Q' ) // 'ESC'
			break;

		
//		std::cout << points << std::endl;
		int iii = 0;
	}

	return 0;
}