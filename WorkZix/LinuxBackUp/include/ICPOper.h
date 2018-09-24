#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "OpenCVInc.h"

class CICPOpr
{
public:
	CICPOpr();
	~CICPOpr();

    Eigen::Matrix<float,4,4> NDTRegistrate(pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, double& score);

	int AddToMap();

public:
	int m_nCont;
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_cloud_pre;
//	pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> m_icp;
	pcl::ApproximateVoxelGrid<pcl::PointXYZI> m_filter1;
	cv::Mat m_Pose;
	std::vector<cv::Mat> m_VecPose;
};

CICPOpr::CICPOpr(): 
m_cloud_pre(new pcl::PointCloud<pcl::PointXYZI>)
{
	m_nCont = 0;

	m_Pose = cv::Mat::eye(4,4,CV_32F);
}

CICPOpr::~CICPOpr()
{

}

Eigen::Matrix<float,4,4> 
    CICPOpr::NDTRegistrate(pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, double& score)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_(new pcl::PointCloud<pcl::PointXYZI>);
	input_cloud_->reserve(input_cloud->size());
    for (unsigned int i = 0; i < input_cloud->size(); i++)
	{
		const pcl::PointXYZI& pt = input_cloud->at(i);
		if (sqrt(pow(pt.x,2)+pow(pt.y,2)) > 50)
		{
			continue;
		}
		input_cloud_->push_back(pt);
	}

	pcl::ApproximateVoxelGrid<pcl::PointXYZI> filter;
	filter.setLeafSize (0.2, 0.2, 0.2);
	pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
	filter.setInputCloud (input_cloud_);
	filter.filter (*filtered_cloud);
//	(*filtered_cloud) += (*input_cloud);

	if (m_nCont <= 0)
	{
		m_cloud_pre->clear();
		(*m_cloud_pre) += (*filtered_cloud);
		m_nCont++;
		Eigen::Matrix<float,4,4> out;
		m_VecPose.push_back(m_Pose);
		return out;
	}

	pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> m_icp_;
	m_icp_.setInputSource (m_cloud_pre);
	m_icp_.setInputTarget (filtered_cloud);
//    printf("ndt input points cont:%d, output points cont:%d\n",(int)m_cloud_pre->size(),(int)filtered_cloud->size());

//	Eigen::AngleAxisf init_rotation (0, Eigen::Vector3f::UnitZ ());
//	Eigen::Translation3f init_translation (0, 0, 0);
//	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
    pcl::PointCloud<pcl::PointXYZI> output_cloud;
    m_icp_.align (output_cloud/*,init_guess*/);
	bool bIsConvirge = m_icp_.hasConverged();
    score = m_icp_.getFitnessScore();
    printf("hasConverged:%d, score:%.2f\n",(int)bIsConvirge, score);

	Eigen::Matrix<float,4,4> out = m_icp_.getFinalTransformation ();
	cv::Mat RT(4,4,CV_32F,out.data());
	m_Pose = m_Pose*(RT.t().inv());
//	std::cout << "pose" << m_Pose << std::endl;
//	std::cout << "motion" << out << std::endl;
	m_cloud_pre->clear();
	(*m_cloud_pre) += (*filtered_cloud);
	m_nCont++;
//	std::cout << m_Pose << std::endl;
	m_VecPose.push_back(m_Pose.clone());
	return out;
}
