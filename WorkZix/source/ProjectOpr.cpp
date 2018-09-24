#include "ProjectOpr.h"
#include <fstream>
#include <highgui.h>

tag_ProjectParam::tag_ProjectParam(const char* pszPath)
{
	if (pszPath == NULL)
	{
		strcpy(szProjPath,"");
		sprintf(szGpsPath,"%s\\GpsLogAuto.log","");
	}
	else
	{
		strcpy(szProjPath,pszPath);
		sprintf(szGpsPath,"%s\\GpsLogAuto.log",pszPath);
	}
	strcpy(szImgNameHeadL,"L");
	strcpy(szImgNameHeadR,"R");
	strcpy(szImgNameTail,".png");
	
}

long tag_ProjectParam::LoadParam()
{
	return 1;
}

CProjectOpr::CProjectOpr()
{
	m_nReadInd = 0;
	return;
}

CProjectOpr::~CProjectOpr()
{
	return;
}

long CProjectOpr::Init(CProjectParam Param)
{
	m_Param = Param;
	LoadProject();
	return 1;
}

long CProjectOpr::LoadProject()
{
	m_GpsData = LoadGpsData(string(m_Param.szGpsPath));

	for (int i = 0; i < m_GpsData.rows; i++)
	{
		char szPathT[MAX_PATH] = {0};
		sprintf(szPathT,"%s\\AutoSaveL%d.png",m_Param.szProjPath,(int)m_GpsData.row(i).at<double>(0,0));
		m_VecImgPathL.push_back(string(szPathT));
		sprintf(szPathT,"%s\\AutoSaveR%d.png",m_Param.szProjPath,(int)m_GpsData.row(i).at<double>(0,0));
		m_VecImgPathR.push_back(string(szPathT));
	}

	return 1;
}

Mat CProjectOpr::LoadGpsData(string& szFile)
{
	vector<vector<double>> Temp;
	ifstream fs(szFile);            
	for(string s;getline(fs,s);)
	{
		if (s.length() == 0)
			break;
		vector<double> Temp1;
		double Temp2 = atof(s.c_str());
		Temp1.push_back(Temp2);
		const char* pIndex = strchr(s.c_str(),',');
		while(pIndex)
		{
			Temp2 = atof(pIndex+1);
			Temp1.push_back(Temp2);
			pIndex = strchr(pIndex+1,',');
		}
		Temp.push_back(Temp1);
	}
	fs.close();

	if (Temp.size() == 0)
	{
		return Mat();
	}

	Mat GpsOut = Mat::zeros(Temp.size(),Temp[0].size(),CV_64FC1);
	for (int i = 0; i < GpsOut.rows; i ++)
	{
		for (int j = 0; j < GpsOut.cols; j++)
		{
			GpsOut.at<double>(i,j) = Temp[i][j];
		}
	}

	return GpsOut;
}

long CProjectOpr::GetStereoImgByGpsFile(Mat* pSteMat, Mat& GpsData, int Color)
{
	if (m_nReadInd < m_VecImgPathL.size() && m_nReadInd < m_VecImgPathR.size())
	{
		Mat L = imread(m_VecImgPathL[m_nReadInd],Color);
		Mat R = imread(m_VecImgPathR[m_nReadInd],Color);
		if (L.data == NULL && R.data == NULL)
		{
			return -1;
		}
		m_GpsData.row(m_nReadInd).copyTo(GpsData);
		L.copyTo(pSteMat[0]);
		R.copyTo(pSteMat[1]);
		m_nReadInd++;
		return m_nReadInd;
	}
	else
	{
		GpsData.release();
		pSteMat[0].release();
		pSteMat[1].release();
		return -1;
	}
	

	return -1;
}

long CProjectOpr::GetStereoImgByDoc(Mat* pSteMat, int Color)
{
	for (int i = m_nReadInd; i < 1000000; i++)
	{
		char szPathL[MAX_PATH] = {0};
		char szPathR[MAX_PATH] = {0};
		sprintf(szPathL,"%s%s%d%s",m_Param.szProjPath,m_Param.szImgNameHeadL,i,m_Param.szImgNameTail);//读取KOTEI数据集
		sprintf(szPathR,"%s%s%d%s",m_Param.szProjPath,m_Param.szImgNameHeadR,i,m_Param.szImgNameTail);
		Mat L = imread(szPathL,Color);
		Mat R = imread(szPathR,Color);
		if (L.data != NULL && R.data != NULL)
		{
			L.copyTo(pSteMat[0]);
			R.copyTo(pSteMat[1]);
			m_nReadInd = i+1;
			return i;
		}
		sprintf(szPathL,"%s%s%06d%s",m_Param.szProjPath,m_Param.szImgNameHeadL,i,m_Param.szImgNameTail);//读取KITTI数据集
		sprintf(szPathR,"%s%s%06d%s",m_Param.szProjPath,m_Param.szImgNameHeadR,i,m_Param.szImgNameTail);
		L = imread(szPathL,Color);
		R = imread(szPathR,Color);
		if (L.data != NULL && R.data != NULL)
		{
			L.copyTo(pSteMat[0]);
			R.copyTo(pSteMat[1]);
			m_nReadInd = i+1;
			return i;
		}
	}

	return 1;
}

CProjectWriter::CProjectWriter()
{
	memset(m_szAutoSavePath,0,sizeof(m_szAutoSavePath));
	memset(m_szManualSavePath,0,sizeof(m_szManualSavePath));
	memset(m_szGpsLogAuto,0,sizeof(m_szGpsLogAuto));
	memset(m_szGpsLogManual,0,sizeof(m_szGpsLogManual));
	m_bIsCreated = false;
	m_bIsCreatedVideo = false;
	m_nSaveIndAuto = 0;
	m_nSaveIndManual = 0;
	m_fpManual = NULL;
	m_fpAuto = NULL;
}

CProjectWriter::~CProjectWriter()
{
	cvReleaseVideoWriter(&m_writerL);
	cvReleaseVideoWriter(&m_writerR);
}

long CProjectWriter::Init(CProjectParam Param)
{
	m_Param = Param;

	memset(m_szAutoSavePath,0,sizeof(m_szAutoSavePath));
	memset(m_szManualSavePath,0,sizeof(m_szManualSavePath));
	memset(m_szGpsLogAuto,0,sizeof(m_szGpsLogAuto));
	memset(m_szGpsLogManual,0,sizeof(m_szGpsLogManual));
	m_bIsCreated = false;
	m_bIsCreatedVideo = false;
	m_writerL = NULL;
	m_writerR = NULL;
	m_nSaveIndAuto = 0;
	m_nSaveIndManual = 0;
	if (m_fpManual)
	{
		fclose(m_fpManual);
		m_fpManual = NULL;
	}
	if (m_fpAuto)
	{
		fclose(m_fpAuto);
		m_fpAuto = NULL;
	}

	char szFilePath[MAX_PATH] = {0};
	GetModuleFileNameA(NULL,szFilePath,MAX_PATH);
	char* p = strrchr(szFilePath,'\\');
	*p = 0x00;
	strcat(szFilePath,"\\");
	char szTime[MAX_PATH] = {0};
	AddTimeTail(szTime);
	sprintf(m_Param.szProjPath,"%s%s\\",szFilePath,szTime);//创建工程目录
	sprintf(m_szAutoSavePath,"%sAutoSave\\",m_Param.szProjPath);//创建自动保存图片目录
	sprintf(m_szManualSavePath,"%sManualSave\\",m_Param.szProjPath);//创建自动保存图片目录
	sprintf(m_szGpsLogAuto,"%sAutoSave\\GpsLogAuto.log",m_Param.szProjPath);//创建自动保存图片目录
	sprintf(m_szGpsLogManual,"%sManualSave\\GpsLogManual.log",m_Param.szProjPath);//创建自动保存图片目录

	return 1;
}

long CProjectWriter::CreateProject()
{
	m_bIsCreated = true;
	CreateDirectoryA(m_Param.szProjPath,NULL);
	CreateDirectoryA(m_szAutoSavePath,NULL);
	CreateDirectoryA(m_szManualSavePath,NULL);
	m_fpManual = fopen(m_szGpsLogManual,"wt");
	m_fpAuto = fopen(m_szGpsLogAuto,"wt");
	string szPathTemp = m_Param.szProjPath;
	CopyFileA("intrinsics.yml",(szPathTemp+"intrinsics.yml").c_str(),true);
	CopyFileA("extrinsics.yml",(szPathTemp+"extrinsics.yml").c_str(),true);

	return 1;
}

long CProjectWriter::CreateVideo()
{
	m_bIsCreatedVideo = true;
	int isColor = 1;
	int fps = 20; // or 30
	int frameW = 1600; // 744 for firewire cameras
	int frameH = 1200; // 480 for firewire cameras
	m_writerL=cvCreateVideoWriter("outL.avi",CV_FOURCC('D', 'I', 'V', 'X'),fps,cvSize(frameW,frameH),isColor);
	m_writerR=cvCreateVideoWriter("outR.avi",CV_FOURCC('D', 'I', 'V', 'X'),fps,cvSize(frameW,frameH),isColor);

	return 1;
}

long CProjectWriter::SaveImageAuto(long nRecInd, Mat* pImg, LCM_GPS_DATA GpsData)
{
	if (m_bIsCreated)
	{
	}
	else
	{
		CreateProject();
	}
	char szPathName[MAX_PATH] = {0};
	sprintf(szPathName,"%s%s%06d%s",m_szAutoSavePath,m_Param.szImgNameHeadL,m_nSaveIndAuto,m_Param.szImgNameTail);
	imwrite(szPathName,pImg[0]);
	printf("%s\n",szPathName);
	sprintf(szPathName,"%s%s%06d%s",m_szAutoSavePath,m_Param.szImgNameHeadR,m_nSaveIndAuto,m_Param.szImgNameTail);
	imwrite(szPathName,pImg[1]);
	printf("%s\n",szPathName);
	sprintf(szPathName,"%06d,%d,%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.2x,%d\n",
		m_nSaveIndAuto,
		nRecInd,
		GpsData.GPS_TIME,
		GpsData.GPS_HEADING,
		GpsData.GPS_PITCH,
		GpsData.GPS_ROLL,
		GpsData.GPS_LATITUDE,
		GpsData.GPS_LONGITUDE,
		GpsData.GPS_ALTITUDE,
		GpsData.GPS_VE,
		GpsData.GPS_VN,
		GpsData.GPS_VU,
		GpsData.GPS_BASELINE,
		GpsData.GPS_NSV1,
		GpsData.GPS_NSV2,
		GpsData.GPS_STATE,
		GpsData.GPS_DATATYPE);
	printf(szPathName);
	printf("\n");
	fprintf(m_fpAuto,"%s",szPathName);
	fflush(m_fpAuto);

	m_nSaveIndAuto++;

	return 1;
}

long CProjectWriter::SaveImageAutoVideo(long nRecInd, Mat* pImg, LCM_GPS_DATA GpsData)
{
	if (m_bIsCreatedVideo)
	{
	}
	else
	{
		CreateVideo();
	}

	cvWriteFrame(m_writerL,&IplImage(pImg[0])); 
	cvWriteFrame(m_writerR,&IplImage(pImg[1])); 

	return 1;
}

long CProjectWriter::ReleaseVideo()
{
	if (m_writerL)
	{
		cvReleaseVideoWriter(&m_writerL);
		m_writerL = NULL;
	}
	if (m_writerR)
	{
		cvReleaseVideoWriter(&m_writerR);
		m_writerR = NULL;
	}

	return 1;
}

long CProjectWriter::SaveImageManual(long nRecInd, Mat* pImg, LCM_GPS_DATA GpsData)
{
	if (m_bIsCreated)
	{
	}
	else
	{
		CreateProject();
	}
	char szPathName[MAX_PATH] = {0};
	sprintf(szPathName,"%s%s%06d%s",m_szManualSavePath,m_Param.szImgNameHeadL,m_nSaveIndManual,m_Param.szImgNameTail);
	imwrite(szPathName,pImg[0]);
	printf("%s\n",szPathName);
	sprintf(szPathName,"%s%s%06d%s",m_szManualSavePath,m_Param.szImgNameHeadR,m_nSaveIndManual,m_Param.szImgNameTail);
	imwrite(szPathName,pImg[1]);
	printf("%s\n",szPathName);
	sprintf(szPathName,"%06d,%d,%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.2x,%d\n",
		m_nSaveIndManual,
		nRecInd,
		GpsData.GPS_TIME,
		GpsData.GPS_HEADING,
		GpsData.GPS_PITCH,
		GpsData.GPS_ROLL,
		GpsData.GPS_LATITUDE,
		GpsData.GPS_LONGITUDE,
		GpsData.GPS_ALTITUDE,
		GpsData.GPS_VE,
		GpsData.GPS_VN,
		GpsData.GPS_VU,
		GpsData.GPS_BASELINE,
		GpsData.GPS_NSV1,
		GpsData.GPS_NSV2,
		GpsData.GPS_STATE,
		GpsData.GPS_DATATYPE);
	printf(szPathName);
	printf("\n");
	fprintf(m_fpManual,"%s",szPathName);
	fflush(m_fpManual);

	m_nSaveIndManual++;

	return 1;
}

void CProjectWriter::AddTimeTail(char* psz)
{
	time_t t = time(0); 
	char tmp[64]; 
	strftime( tmp, sizeof(tmp), "%Y%m%d%H%M%S",localtime(&t) );
	strcat(psz,tmp);
}

CLidarDataOpr::CLidarDataOpr(char* pszPath)
{
	m_nInd = 0;
	strcpy_s(m_szPath,pszPath);
	strcpy_s(m_szPathRead,pszPath);
	m_fWrite = NULL;
	m_fRead = NULL;
	m_bIsCreateDir = false;
}
CLidarDataOpr::~CLidarDataOpr(void)
{
	if (m_fWrite)
	{
		fclose(m_fWrite);
		m_fWrite = NULL;
	}
	if (m_fRead)
	{
		fclose(m_fRead);
		m_fRead = NULL;
	}
}

long CLidarDataOpr::CreateDir()
{
	m_bIsCreateDir = true;
	AddTimeTail(m_szPath);
	strcat_s(m_szPath,"\\");
	CreateDirectoryA(m_szPath,NULL);

	return 1;
}

long CLidarDataOpr::WriteLog(LCM_IBEO_CLOUD_POINTS* pLidarPoints, LCM_GPS_DATA* pImuInfo)
{
	char szPathCloudPt[MAX_PATH] = {0};
	char szPathImu[MAX_PATH] = {0};
	sprintf_s(szPathCloudPt,"%s%06d.lidar",m_szPath,m_nInd);
	sprintf_s(szPathImu,"%s%06d.imu",m_szPath,m_nInd);
	m_fWrite = fopen(szPathCloudPt,"wt");
	fprintf(m_fWrite,"%d\t%d\t%d\t%d\t%d\t%d\t\n",
		(long)pLidarPoints->ContPoints,
		(long)pLidarPoints->DevInd,
		(long)pLidarPoints->DevType,
		(long)pLidarPoints->FrameInd,
		(long)pLidarPoints->TimeStamp,
		(long)pLidarPoints->IbeoPoints.size());

	for (int i = 0; i < pLidarPoints->IbeoPoints.size(); i++)
	{
		fprintf(m_fWrite,"%d\t%d\t%d\t%.2f\t%.2f\t%.2f\n",
			(long)pLidarPoints->IbeoPoints[i].layer,
			(long)pLidarPoints->IbeoPoints[i].flag,
			(long)pLidarPoints->IbeoPoints[i].echo,
			(double)pLidarPoints->IbeoPoints[i].angle,
			(double)pLidarPoints->IbeoPoints[i].distance,
			(double)pLidarPoints->IbeoPoints[i].epw);
	}
	fclose(m_fWrite);

	m_fWrite = fopen(szPathImu,"wt");
	fprintf(m_fWrite,"%.3f\t%.3f\t%.3f\t%.3f\t%.7f\t%.7f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%d\t%d\t%d\t%d\n",
		pImuInfo->GPS_TIME,
		pImuInfo->GPS_HEADING,
		pImuInfo->GPS_PITCH,
		pImuInfo->GPS_ROLL,
		pImuInfo->GPS_LATITUDE,
		pImuInfo->GPS_LONGITUDE,
		pImuInfo->GPS_ALTITUDE,
		pImuInfo->GPS_VE,
		pImuInfo->GPS_VN,
		pImuInfo->GPS_VU,
		pImuInfo->GPS_BASELINE,
		pImuInfo->GPS_NSV1,
		pImuInfo->GPS_NSV2,
		pImuInfo->GPS_STATE,
		pImuInfo->GPS_DATATYPE);
	fclose(m_fWrite);

	m_nInd++;

	return m_nInd;
}

long CLidarDataOpr::ReadLog(LCM_IBEO_CLOUD_POINTS* pLidarPoints, LCM_GPS_DATA* pImuInfo)
{
	char szPathCloudPt[MAX_PATH] = {0};
	char szPathImu[MAX_PATH] = {0};
	sprintf_s(szPathCloudPt,"%s%06d.lidar",m_szPathRead,m_nInd);
	sprintf_s(szPathImu,"%s%06d.imu",m_szPathRead,m_nInd);
	ifstream fCloudPt(szPathCloudPt);
	ifstream fImu(szPathImu);

	long nLongT;
	double dDoubleT;
	fCloudPt>>nLongT;
	pLidarPoints->ContPoints = nLongT;
	fCloudPt>>nLongT;
	pLidarPoints->DevInd = nLongT;
	fCloudPt>>nLongT;
	pLidarPoints->DevType = nLongT;
	fCloudPt>>nLongT;
	pLidarPoints->FrameInd = nLongT;
	fCloudPt>>nLongT;
	pLidarPoints->TimeStamp = nLongT;
	fCloudPt>>nLongT;
	pLidarPoints->IbeoPoints.resize(nLongT);
	for (int i = 0; i < pLidarPoints->IbeoPoints.size(); i++)
	{
		fCloudPt>>nLongT;
		pLidarPoints->IbeoPoints[i].layer = nLongT;
		fCloudPt>>nLongT;
		pLidarPoints->IbeoPoints[i].flag = nLongT;
		fCloudPt>>nLongT;
		pLidarPoints->IbeoPoints[i].echo = nLongT;
		fCloudPt>>pLidarPoints->IbeoPoints[i].angle;
		fCloudPt>>pLidarPoints->IbeoPoints[i].distance;
		fCloudPt>>pLidarPoints->IbeoPoints[i].epw;
	}
	fCloudPt.close();

	fImu>>pImuInfo->GPS_TIME;
	fImu>>pImuInfo->GPS_HEADING;
	fImu>>pImuInfo->GPS_PITCH;
	fImu>>pImuInfo->GPS_ROLL;
	fImu>>pImuInfo->GPS_LATITUDE;
	fImu>>pImuInfo->GPS_LONGITUDE;
	fImu>>pImuInfo->GPS_ALTITUDE;
	fImu>>pImuInfo->GPS_VE;
	fImu>>pImuInfo->GPS_VN;
	fImu>>pImuInfo->GPS_VU;
	fImu>>pImuInfo->GPS_BASELINE;
	fImu>>pImuInfo->GPS_NSV1;
	fImu>>pImuInfo->GPS_NSV2;
	fImu>>nLongT;
	pImuInfo->GPS_STATE=nLongT;
	fImu>>pImuInfo->GPS_DATATYPE;

	fImu.close();

	m_nInd++;

	return m_nInd;
}

long CLidarDataOpr::WriteLog(LCM_SENSOR_FUSION_PACKAGE* pPackData)
{
	if (m_bIsCreateDir)
	{
	}
	else
	{
		CreateDir();
	}
	char szPath[MAX_PATH] = {0};
	sprintf_s(szPath,"%s%06d.log",m_szPath,m_nInd);
	ofstream fout(szPath, ios::binary);
	int nDataLen = pPackData->getEncodedSize();
	pPackData->encode((void*)m_FileBuffer, 0, PROJECT_MAX_FILE_LEN);
//	fout.write((char*)&nDataLen, sizeof(int));
	fout.write(m_FileBuffer, nDataLen);
	fout.close();

	m_nInd++;

	return m_nInd;
}

long CLidarDataOpr::ReadLog(LCM_SENSOR_FUSION_PACKAGE* pPackData)
{
	char szPath[MAX_PATH] = {0};
	sprintf_s(szPath,"%s%06d.log",m_szPathRead,m_nInd);
	ifstream fin(szPath, ios::binary);
	if (fin.is_open() == false)
	{
		return -1;
	}
	int nDataLen = 0;
//	fin.read((char*)&nDataLen,sizeof(int));
	fin.read((char*)&m_FileBuffer, PROJECT_MAX_FILE_LEN);
	pPackData->decode(m_FileBuffer, 0, PROJECT_MAX_FILE_LEN);
	fin.close();

	m_nInd++;

	return m_nInd;
}

void CLidarDataOpr::AddTimeTail(char* psz)
{
	time_t t = time(0); 
	char tmp[64]; 
	strftime( tmp, sizeof(tmp), "%Y%m%d%H%M%S",localtime(&t) );
	strcat(psz,tmp);
}

long CLidarDataOpr::LcmSendLog(LCM_SENSOR_FUSION_PACKAGE* pPackData)
{
	return 1;
}

void CLidarDataOpr::SetInd(long nInd)
{
	m_nInd = nInd;
}

long CLidarDataOpr::GetInd()
{
	return m_nInd;
}