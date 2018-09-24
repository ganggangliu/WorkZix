#include "CalibSensor.h"

void saveXYZ1(const char* filename, const Mat& mat)
{
	char szLine[20000] = {0};
	char szWord[100] = {0};
	FILE* fp = fopen(filename, "wt");
	for(int x = 0; x < mat.rows; x++)
	{
		strcpy(szLine,"");
		for(int y = 0; y < mat.cols; y++)
		{
			double point = cvGetReal2D(&CvMat(mat),x,y);
			sprintf(szWord,"%.2f,",point);
			strcat(szLine,szWord);
		}
		fprintf(fp,"%s\n",szLine);
	}
	fclose(fp);
}

CCalibSensor::CCalibSensor()
{
	return;
}

CCamImuCalib::CCamImuCalib(void)
{
	m_nGridSize = 20;
};
CCamImuCalib::~CCamImuCalib(void)
{
};

long CCamImuCalib::CalibByFiles()
{
	FileStorage fs("CameraImuCalib.yml",CV_STORAGE_READ);

	//Param1:Camera Matrix
	if(!fs.isOpened())
	{
		printf("Failed to open file %s\n", "CameraImuCalib.yml");
		waitKey();
		return 0;
	}
	Mat cameraMatrix;
	fs["cameraMatrix"] >> cameraMatrix;
	cout << "cameraMatrix\n" << cameraMatrix << endl;

	Mat matP;
	Mat matUndistortedPoints;
	fs["matP"] >> matP;
	fs["matUndistortedPoints"] >> matUndistortedPoints;
	cout << "matP\n" << matP << endl;
	cout << "matUndistortedPoints\n" << matUndistortedPoints << endl;

	//Param2:Mount offset Tcam2imu
	Mat T0;
	fs["T0"] >> T0;
	cout << "T0\n" << T0 << endl;

	//Param5:IMU data
	Mat ImuData;
	fs["ImuData"] >> ImuData;
	cout << "ImuData\n" << ImuData << endl;

	//Param6:Calib Heading
	Mat CalibHeading;
	fs["CalibHeading"] >> CalibHeading;
	cout << "CalibHeading\n" << CalibHeading << endl;

	//Param7:Ref Hight 2 Ground
	fs["m_RefHight2Ground"] >> m_RefHight2Ground;
	cout << "m_RefHight2Ground\n" << m_RefHight2Ground << endl;

	Mat R,inliers,rvec,tvec, distCoeffs;
	solvePnPRansac(matP,matUndistortedPoints,cameraMatrix,distCoeffs,rvec,tvec,false,100,2,0.99,inliers,ITERATIVE);
	cout << "rvec" << rvec/CV_PI*180 << endl;
	cout << "tvec" << tvec << endl;
	
	double dDeltaRotX = rvec.at<double>(0,0)/CV_PI*180 + ImuData.at<double>(0,PITCH);//pitch
	double dDeltaRotY = ImuData.at<double>(0,HEADING) - CalibHeading.at<double>(0,0) + rvec.at<double>(1,0)/CV_PI*180;//heading
	double dDeltaRotZ = rvec.at<double>(2,0)/CV_PI*180 + ImuData.at<double>(0,ROLL);//roll

	Mat rvec0 = (Mat_<double>(1,3) << dDeltaRotX,dDeltaRotY,dDeltaRotZ);
	Mat tvec0 = T0;

	m_RTImu2Cam.create(4,4,CV_64F);
	m_RTImu2Cam.setTo(0);
	Mat RR;
	Rodrigues(rvec0/180.f*CV_PI,RR);
	RR.copyTo(m_RTImu2Cam(Range(0,3),Range(0,3)));
	Mat ttt = tvec0.t();
	ttt.copyTo(m_RTImu2Cam(Range(0,3),Range(3,4)));
	m_RTImu2Cam.at<double>(3,3) = 1;
	cout << m_RTImu2Cam << endl;

	FileStorage fs_out("extrinsicsCamAndImu.yml",CV_STORAGE_WRITE);
	cout << "rvec0" << rvec0 << endl;
	cout << "tvec0" << tvec0 << endl;
	fs_out << "cameraMatrix" << cameraMatrix;
	fs_out << "rvec0" << rvec0;
	fs_out << "tvec0" << tvec0;
	fs_out << "m_RTImu2Cam" << m_RTImu2Cam;////
	fs_out.release();

	m_cameraMatrix = cameraMatrix;
	m_rvec0 = rvec0;
	m_tvec0 = tvec0;

	return 1;
}

double CCamImuCalib::rad(double d)
{
	return d * CV_PI / 180.0;
}

Mat CCamImuCalib::Gps2Plant(Mat ptLocal, Mat ptTarget)
{
	Mat ptDeltaLocal = Mat::zeros(ptTarget.rows,3,CV_64FC1);
	for (int i = 0; i < ptTarget.rows; i++)
	{
		ptDeltaLocal.at<double>(i,1) = 111319.55*(ptTarget.at<double>(i,0)-ptLocal.at<double>(0,0));
		ptDeltaLocal.at<double>(i,0) = 111319.55*(ptTarget.at<double>(i,1)-ptLocal.at<double>(0,1))*cos(rad(ptLocal.at<double>(0,0)));
		ptDeltaLocal.at<double>(i,2) = (ptTarget.at<double>(i,2) - ptLocal.at<double>(0,2));
	}

	return ptDeltaLocal;
}

//long CCamImuCalib::LoadParams()
//{
//	FileStorage fs_out("extrinsicsCamAndImu.yml",CV_STORAGE_READ);
//	if(!fs_out.isOpened())
//	{
//		printf("Failed to open file %s\n", "extrinsicsCamAndImu.yml");
//		return 0;
//	}
//	fs_out["cameraMatrix"] >> m_cameraMatrix;
//	fs_out["rvec0"] >> m_rvec0;
//	fs_out["tvec0"] >> m_tvec0;
//	fs_out["m_RTImu2Cam"] >> m_RTImu2Cam;
//	cout << "cameraMatrix" << m_cameraMatrix << endl;
//	cout << "rvec0" << m_rvec0 << endl;
//	cout << "tvec0" << m_tvec0 << endl;
//	cout << "m_RTImu2Cam" << m_RTImu2Cam << endl;
//
//	return 1;
//}

Mat CCamImuCalib::LocaleAbsolute(Mat& ImuData, Mat& ptTarget)
{
	Mat rvec = Mat::zeros(1,3,CV_64FC1);
	rvec.at<double>(0,0) = m_rvec0.at<double>(0,0) - ImuData.at<double>(0,PITCH);//pitch
	rvec.at<double>(0,1) = m_rvec0.at<double>(0,HEADING);//heading
	rvec.at<double>(0,2) = m_rvec0.at<double>(0,2) - ImuData.at<double>(0,ROLL);//roll
	//	cout << rvec << endl;
	Mat R0;
	Rodrigues(rvec/180*CV_PI,R0);

	Mat ptLocal = (Mat_<double>(1,3) << ImuData.at<double>(0,LATITUDE),ImuData.at<double>(0,LONGTITUDE),ImuData.at<double>(0,ALTITUDE));
	double dAngle = ImuData.at<double>(0,HEADING)/180*CV_PI;

	Mat R1 = (Mat_<double>(3,3) << cos(dAngle), sin(dAngle), 0, -sin(dAngle), cos(dAngle), 0, 0, 0, 1);
	Mat ptDeltaLocal = Gps2Plant(ptLocal,ptTarget);
//	cout << ptDeltaLocal << endl;
	Mat pt0 = R1.inv()*ptDeltaLocal.t();
//	cout << "R1.inv()" << R1.inv() << endl;
//	saveXYZ1("R1Inv",R1.inv());
	//	cout << "ptDeltaLocal" << ptDeltaLocal << endl;
//		cout << "pt0" << pt0.t() << endl;
	//	saveXYZ1("pt0",pt0.t());

	Mat RT = Mat::zeros(3,4,CV_64FC1);
	RT.col(3) = m_tvec0.t();
	R0.copyTo(RT.colRange(0,3));
	//	cout << "RT" << RT << endl;
	Mat dcameraMatrix;
	m_cameraMatrix.convertTo(dcameraMatrix,CV_64FC1);
	Mat pt11 = Mat::ones(4,pt0.cols,CV_64FC1);
	pt0.row(0).copyTo(pt11.row(0));
	pt0.row(1).copyTo(pt11.row(2));
	pt0.row(2).copyTo(pt11.row(1));
	pt11.row(1) = -1.f*pt11.row(1);
	// 	pt11.row(0).setTo(pt0.row(0));
	// 	pt11.row(1).setTo(pt0.row(2));
	// 	pt11.row(2).setTo(pt0.row(1));
	pt11.row(3).setTo(1);
	//	saveXYZ1("pt11",pt11.t());
	//	Mat pt11_ = Mat::ones(4,1,CV_64FC1);
	// 	pt11.at<double>(0,0) = pt0.at<double>(0,0);
	// 	pt11.at<double>(1,0) = 1.25;
	// 	pt11.at<double>(2,0) = pt0.at<double>(1,0);
	// 	pt11.at<double>(3,0) = 1;
	// 	pt11_.at<double>(0,0) = pt0.at<double>(0,0);
	// 	pt11_.at<double>(1,0) = 1.25-1;
	// 	pt11_.at<double>(2,0) = pt0.at<double>(1,0);
	// 	pt11_.at<double>(3,0) = 1;
// 		cout << "pt11" << pt11 << endl;
// 		cout << "RT" << RT << endl;
// 		cout << "dcameraMatrix" << dcameraMatrix << endl;
// 		saveXYZ1("RT",RT);
	Mat ptCamera = RT*pt11;
	ptCamera.copyTo(m_ptCamera);
//	cout << "ptCamera" << ptCamera << endl;
	Mat pt22 = dcameraMatrix*ptCamera;
//	cout << "pt22" << pt22 << endl;
	Mat pt33 = pt22.t();

	return pt33;
}

Mat CCamImuCalib::LocaleAbsoluteEx(Mat& ImuData, Mat& ptTarget)
{
	Mat ptLocal = (Mat_<double>(1,3) << 
		ImuData.at<double>(0,LATITUDE),
		ImuData.at<double>(0,LONGTITUDE),
		ImuData.at<double>(0,ALTITUDE));
	//////////////////////////////////////////////////////////////////////////
	Mat ptTargetT = ptTarget.t();
	Mat temp = Mat::ones(4,ptTargetT.cols,CV_64F);
	ptTargetT.row(0).copyTo(temp.row(0));
	ptTargetT.row(1).copyTo(temp.row(1));
	ptTargetT.row(2).copyTo(temp.row(2));
	//////////////////////////////////////////////////////////////////////////
//	cout << "ptLocal" << ptLocal << endl;
//	cout << "temp" << temp << endl;
	m_ptCart = WGS48ToCart(ptLocal,temp);
//	cout << "ptCart" << m_ptCart << endl;
	//cout << "ImuData" << ImuData << endl;
	m_ptImu = CartToImu(ImuData,m_ptCart);
//	cout << "ptImu" << m_ptImu << endl;
	//cout << "m_RTImu2Cam" << m_RTImu2Cam << endl;
	m_ptCamera = ImuToCamera(m_RTImu2Cam,m_ptImu);
//	cout << "m_ptCamera" << m_ptCamera << endl;
	//cout << "m_cameraMatrix" << m_cameraMatrix << endl;
	m_ptImage = CameraToImage(m_cameraMatrix,m_ptCamera);
//	cout << "ptImage" << m_ptImage << endl;

	return m_ptImage.t();
}

Mat CCamImuCalib::LocaleRelative(Mat& ptTarget)
{
	Mat R0 = Mat::eye(3,3,CV_64FC1);

	Mat pt0 = ptTarget.t();
	//	cout << "ptDeltaLocal" << ptDeltaLocal << endl;
	//	cout << "pt0" << pt0.t() << endl;
	//	saveXYZ1("pt0",pt0.t());

	Mat RT = Mat::zeros(3,4,CV_64FC1);
	RT.col(3) = m_tvec0.t();
	R0.copyTo(RT.colRange(0,3));
	//	cout << "RT" << RT << endl;
	Mat dcameraMatrix;
	m_cameraMatrix.convertTo(dcameraMatrix,CV_64FC1);
	Mat pt11 = Mat::ones(4,pt0.cols,CV_64FC1);
	pt0.row(0).copyTo(pt11.row(0));
	pt0.row(1).copyTo(pt11.row(2));
	pt0.row(2).copyTo(pt11.row(1));
	// 	pt11.row(0).setTo(pt0.row(0));
	// 	pt11.row(1).setTo(pt0.row(2));
	// 	pt11.row(2).setTo(pt0.row(1));
	pt11.row(3).setTo(1);
	//	saveXYZ1("pt11",pt11.t());
	//	Mat pt11_ = Mat::ones(4,1,CV_64FC1);
	// 	pt11.at<double>(0,0) = pt0.at<double>(0,0);
	// 	pt11.at<double>(1,0) = 1.25;
	// 	pt11.at<double>(2,0) = pt0.at<double>(1,0);
	// 	pt11.at<double>(3,0) = 1;
	// 	pt11_.at<double>(0,0) = pt0.at<double>(0,0);
	// 	pt11_.at<double>(1,0) = 1.25-1;
	// 	pt11_.at<double>(2,0) = pt0.at<double>(1,0);
	// 	pt11_.at<double>(3,0) = 1;
	//	cout << "pt11" << pt11 << endl;
	Mat pt22 = dcameraMatrix*RT*pt11;
	Mat pt33 = pt22.t();

	return pt33;
}

void CCamImuCalib::DrawRefAxis(Mat& image, Mat& ImuData)
{
	Mat ptTarget = (Mat_<double>(4,3) <<	
		ImuData.at<double>(0,LATITUDE),ImuData.at<double>(0,LONGTITUDE),ImuData.at<double>(0,ALTITUDE),
		ImuData.at<double>(0,LATITUDE),ImuData.at<double>(0,LONGTITUDE)+0.00001,ImuData.at<double>(0,ALTITUDE),
		ImuData.at<double>(0,LATITUDE)+0.00001,ImuData.at<double>(0,LONGTITUDE),ImuData.at<double>(0,ALTITUDE),
		ImuData.at<double>(0,LATITUDE),ImuData.at<double>(0,LONGTITUDE),ImuData.at<double>(0,ALTITUDE)+1);
	Mat ptLocal = (Mat_<double>(1,3) << ImuData.at<double>(0,LATITUDE),ImuData.at<double>(0,LONGTITUDE),ImuData.at<double>(0,ALTITUDE));
	//////////////////////////////////////////////////////////////////////////
	Mat ptTargetT = ptTarget.t();
	Mat ptCart = WGS48ToCart(ptLocal,ptTargetT);
	Mat ptImu = CartToImu(ImuData,ptCart);
	Mat ptCamera = ImuToCamera(m_RTImu2Cam,ptImu);
	ptCamera.row(0) = ptCamera.row(0)+3;
	ptCamera.row(1) = ptCamera.row(1)+1;
	ptCamera.row(2) = ptCamera.row(2)+10;
	Mat ptImage = CameraToImage(m_cameraMatrix,ptCamera);
	Mat ptImageT = ptImage.t();
	DrawAxis(image,ptImageT);

	return;
}

Mat CCamImuCalib::WGS48ToCart(Mat ptLocal, Mat ptTarget)
{
	Mat ptDeltaLocal = Mat::ones(4,ptTarget.cols,CV_64FC1);
	for (int i = 0; i < ptDeltaLocal.cols; i++)
	{
		ptDeltaLocal.at<double>(1,i) = 111319.55*(ptTarget.at<double>(0,i)-ptLocal.at<double>(0,0));
		ptDeltaLocal.at<double>(0,i) = 111319.55*(ptTarget.at<double>(1,i)-ptLocal.at<double>(0,1))*cos(rad(ptLocal.at<double>(0,0)));
		ptDeltaLocal.at<double>(2,i) = (ptTarget.at<double>(2,i) - ptLocal.at<double>(0,2));
	}

	return ptDeltaLocal;
}

Mat CCamImuCalib::CartToImu(Mat ImuData, Mat Targets)
{
	double Rx = -1.f*ImuData.at<double>(0,PITCH);
	double Ry = -1.f*ImuData.at<double>(0,ROLL);
	double Rz = /*-1.f**/ImuData.at<double>(0,HEADING);
	Mat Rx_ = Mat::eye(4,4,CV_64F);
	Mat Ry_ = Mat::eye(4,4,CV_64F);
	Mat Rz_ = Mat::eye(4,4,CV_64F);
	Mat temp0,temp1;

	temp0 = (Mat_<double>(1,3) << Rx/180.f*CV_PI, 0, 0);
	Rodrigues(temp0,temp1);
	temp1.copyTo(Rx_(Range(0,3),Range(0,3)));

	temp0 = (Mat_<double>(1,3) << 0, Ry/180.f*CV_PI, 0);
	Rodrigues(temp0,temp1);
	temp1.copyTo(Ry_(Range(0,3),Range(0,3)));

	temp0 = (Mat_<double>(1,3) << 0, 0, Rz/180.f*CV_PI);
	Rodrigues(temp0,temp1);
	temp1.copyTo(Rz_(Range(0,3),Range(0,3)));

	Mat out = Rx_*Ry_*Rz_*Targets;
	return out;
}

Mat CCamImuCalib::ImuToCamera(Mat CalibCam2Imu, Mat Targets)
{
	Mat Tar(Targets.rows,Targets.cols,Targets.type());
	Targets.copyTo(Tar);
	Targets.row(0).copyTo(Tar.row(0));
	Targets.row(1).copyTo(Tar.row(2));
	Targets.row(2).copyTo(Tar.row(1));
	Tar.row(1) = Tar.row(1) * -1.f;
	//cout << Targets << endl;
	//cout << Tar << endl;
	Mat out = CalibCam2Imu*Tar;
	return out;
}

Mat CCamImuCalib::CameraToImage(Mat CamMatrix, Mat targets)
{
	Mat out = CamMatrix*targets.rowRange(0,3);
	return out;
}

long CCamImuCalib::DrawGrid(Mat& image, Mat& ImuData)
{
	Mat ptLatLong;
	vector<Point3d> VecPtLong; 
	Point3d pt3;
	for (int i = -1*m_nGridSize+1; i < m_nGridSize; i++)
	{
		for (int j = -1*m_nGridSize+1; j < m_nGridSize; j++)
		{
			pt3.x = ((long)(ImuData.at<double>(0,LATITUDE)/0.00001))*0.00001+i*0.00001;
			pt3.y = ((long)(ImuData.at<double>(0,LONGTITUDE)/0.00001))*0.00001+j*0.00001;
			pt3.z = ImuData.at<double>(0,ALTITUDE) - m_RefHight2Ground.at<double>(0,0);
			VecPtLong.push_back(pt3);
		}
	}
	Mat ptLong0(VecPtLong.size(),3,CV_64FC1,VecPtLong.data());
	ptLong0.copyTo(ptLatLong);

	Mat ptGridLatLong = LocaleAbsoluteEx(ImuData,ptLatLong);

	for (int i = 0; i < ptGridLatLong.rows; i++)
	{
		if (ptGridLatLong.at<double>(i,2) > 0) 
			circle(image,cvPoint(ptGridLatLong.at<double>(i,0)/ptGridLatLong.at<double>(i,2),ptGridLatLong.at<double>(i,1)/ptGridLatLong.at<double>(i,2)),2,CV_RGB(255,0,0),2);
	}

	return 1;
}

long CCamImuCalib::DrawAxis(Mat& image, Mat& ptTarget)
{
//	cout << ptTarget << endl;
	Mat ptTarget1(&CvMat(ptTarget),true);
	ptTarget1.col(0) = ptTarget1.col(0)/ptTarget1.col(2);
	ptTarget1.col(1) = ptTarget1.col(1)/ptTarget1.col(2);
	ptTarget1.col(2) = ptTarget1.col(2)/ptTarget1.col(2);
//	cout << ptTarget1 << endl;
	circle(image,cvPoint(ptTarget1.at<double>(0,0),ptTarget1.at<double>(0,1)),4,CV_RGB(0,0,0),4);
	circle(image,cvPoint(ptTarget1.at<double>(1,0),ptTarget1.at<double>(1,1)),4,CV_RGB(255,0,0),4);
	circle(image,cvPoint(ptTarget1.at<double>(2,0),ptTarget1.at<double>(2,1)),4,CV_RGB(0,255,0),4);
	circle(image,cvPoint(ptTarget1.at<double>(3,0),ptTarget1.at<double>(3,1)),4,CV_RGB(0,0,255),4);
	line(image,cvPoint(ptTarget1.at<double>(0,0),ptTarget1.at<double>(0,1)),cvPoint(ptTarget1.at<double>(1,0),ptTarget1.at<double>(1,1)),CV_RGB(255,0,0),4);
	putText(image,"E",cvPoint(ptTarget1.at<double>(1,0),ptTarget1.at<double>(1,1)),0,1,CV_RGB(255,0,0),2);
	line(image,cvPoint(ptTarget1.at<double>(0,0),ptTarget1.at<double>(0,1)),cvPoint(ptTarget1.at<double>(2,0),ptTarget1.at<double>(2,1)),CV_RGB(0,255,0),4);
	putText(image,"N",cvPoint(ptTarget1.at<double>(2,0),ptTarget1.at<double>(2,1)),0,1,CV_RGB(0,255,0),2);
	line(image,cvPoint(ptTarget1.at<double>(0,0),ptTarget1.at<double>(0,1)),cvPoint(ptTarget1.at<double>(3,0),ptTarget1.at<double>(3,1)),CV_RGB(0,0,255),4);
	putText(image,"U",cvPoint(ptTarget1.at<double>(3,0),ptTarget1.at<double>(3,1)),0,1,CV_RGB(0,0,255),2);

	return 1;
}