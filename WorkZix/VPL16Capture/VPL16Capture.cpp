#include "VPL16Opr.h"

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

pcl::visualization::CloudViewer g_viewer("Simple Cloud Viewer");
void DrawLidar(Mat& LidarPoints);

void __stdcall VPL16DataCallBack(Mat* pData, void* pUser)
{	
	Mat CloudPoint = pData->clone();
	
	//FileStorage fs("111.yml",CV_STORAGE_WRITE);
	//fs << "CloudPoint" << CloudPoint;
	//fs.release();
	DrawLidar(CloudPoint);

	return;

}

int main(int argc, char* argv[])
{
	CVPL16Opr VplOpr;
	VplOpr.Init(argv[1]);
	VplOpr.SetCallBack((lpVPL16DataCallBack)VPL16DataCallBack,NULL);
	VplOpr.Start();

	while(!g_viewer.wasStopped())
	{
		;
	}

	Sleep(INFINITE);

	return 0;
}

void DrawLidar(Mat& LidarPoints)
{
// 	Mat CloudPoint;
// 	FileStorage fs("111.yml",CV_STORAGE_READ);
// 	fs["CloudPoint"] >> CloudPoint;
	Mat CloudPoint = LidarPoints;

	pcl::PointCloud<pcl::PointXYZ>::Ptr CloudOri (new pcl::PointCloud<pcl::PointXYZ>);
	CloudOri->points.resize(CloudPoint.rows);
	for (int i = 0; i < CloudPoint.rows; i++)
	{
		CloudOri->points[i].x = CloudPoint.at<double>(i,0);
		CloudOri->points[i].y = CloudPoint.at<double>(i,1);
		CloudOri->points[i].z = CloudPoint.at<double>(i,2);
		CloudOri->points[i].data[3] = 1.0;
	}

	pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data

	pcl::toPCLPointCloud2(*CloudOri,*cloud_blob);

//	std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud_blob);
	sor.setLeafSize (0.1f, 0.1f, 0.1f);
	sor.filter (*cloud_filtered_blob);

	// Convert to the templated PointCloud
	pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

//	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

	// Write the downsampled version to disk
//	pcl::PCDWriter writer;
//	writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.3);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudOriRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
	CloudOriRGB->reserve(cloud_filtered->size());

	int i = 0, nr_points = (int) cloud_filtered->points.size ();
	// While 30% of the original cloud is still there
//	while (cloud_filtered->points.size () > 0.3 * nr_points)
//	for (int i = 0; i < 3; i++)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
//			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
//			break;
		}

		// Extract the inliers
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_p);
//		std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

		std::stringstream ss;
// 		ss << "table_scene_lms400_plane_" << i << ".pcd";
// 		printf("%.5f,%.5f,%.5f,%.5f\n",coefficients->values[0],
// 			coefficients->values[1],
// 			coefficients->values[2],
// 			coefficients->values[3]);
// 		writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

		// Create the filtering object
		extract.setNegative (true);
		extract.filter (*cloud_f);

		cloud_filtered.swap (cloud_f);
//		i++;

// 		float a = coefficients->values[0];
// 		float b = coefficients->values[1];
// 		float c = coefficients->values[2];
// 		float d = coefficients->values[3];
// 		double dRoll = atan(-1.0*a/c)/CV_PI*180.0;
// 		double dPitch = atan(-1.0*b/c)/CV_PI*180.0;
// 		double dHeight = d/c;
// 		if (fabs(dRoll)<=3.0 && fabs(dPitch)<=3.0 && dHeight>=)
// 		{
// 		}

		printf("%d\n",cloud_p->size());
// 		for (int i = 0; i < cloud_p->size(); i++)
// 		{
// 			pcl::PointXYZRGB pt(255,0,0);
// 			memcpy(pt.data, cloud_filtered->at(i).data, 4*sizeof(float));
// 			CloudOriRGB->push_back(pt);
// 		}
	}

	
	

	g_viewer.showCloud (cloud_p);
}